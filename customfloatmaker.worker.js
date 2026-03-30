import ocFactory from "./vendor/opencascade/opencascade.loader.mjs";

const OCCT_BASE_URL = "./vendor/opencascade/";
const PROFILE_SAMPLES = 296;
const CIRCLE_SEGMENTS = 32;
const DEFAULT_LINEAR_DEFLECTION = 0.35;
const DEFAULT_ANGULAR_DEFLECTION = 0.5;
const STL_EXPORT_LINEAR_DEFLECTION = 0.08;
const STL_EXPORT_ANGULAR_DEFLECTION = 0.16;
const STEP_PREVIEW_DELAY_MS = 120;
const LED_SLEEVE_END_CUT_DEG = 45.0;

const state = {
  oc: null,
  ready: false,
  currentShape: null,
  currentMetrics: null,
  latestBuildVersion: 0
};

function post(type, payload = {}, transfer = []) {
  self.postMessage({ type, ...payload }, transfer);
}

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function emitProgress(jobId, message, tone = "normal") {
  post("progress", { jobId, message, tone });
  await sleep(0);
}

function makeBuildCancelledError(jobId) {
  const error = new Error(`Build ${jobId} superseded by newer parameters.`);
  error.name = "BuildCancelledError";
  error.jobId = jobId;
  return error;
}

function throwIfBuildCancelled(jobId) {
  if (jobId == null) {
    return;
  }
  if (Number.isFinite(state.latestBuildVersion) && state.latestBuildVersion > 0 && jobId !== state.latestBuildVersion) {
    throw makeBuildCancelledError(jobId);
  }
}

function callFirstAvailable(target, names, args = []) {
  for (const name of names) {
    if (typeof target[name] === "function") {
      return target[name](...args);
    }
  }
  throw new Error(`Missing method: ${names.join(" / ")}`);
}

function instantiateAvailable(openCascade, names, args = []) {
  for (const name of names) {
    const ctor = openCascade[name];
    if (typeof ctor === "function") {
      return new ctor(...args);
    }
  }
  throw new Error(`Missing constructor: ${names.join(" / ")}`);
}

function callFunctionVariants(fn, variants) {
  const errors = [];
  for (const args of variants) {
    try {
      return fn(...args);
    } catch (err) {
      errors.push(`${args.length} args: ${err.message}`);
    }
  }
  throw new Error(`No working overload. Tried: ${errors.join(" | ")}`);
}

function instantiateAvailableVariants(openCascade, variants) {
  const errors = [];
  for (const variant of variants) {
    const names = variant.names || [];
    const args = variant.args || [];
    for (const name of names) {
      const ctor = openCascade[name];
      if (typeof ctor !== "function") continue;
      try {
        return new ctor(...args);
      } catch (err) {
        errors.push(`${name}: ${err.message}`);
      }
    }
  }
  throw new Error(`Missing working constructor variant. Tried: ${errors.join(" | ")}`);
}

function callFirstAvailableVariant(target, variants) {
  const errors = [];
  for (const variant of variants) {
    const names = variant.names || [];
    const args = variant.args || [];
    for (const name of names) {
      if (typeof target[name] !== "function") continue;
      try {
        return target[name](...args);
      } catch (err) {
        errors.push(`${name}: ${err.message}`);
      }
    }
  }
  throw new Error(`Missing working method variant. Tried: ${errors.join(" | ")}`);
}

function shapeOf(result) {
  return typeof result.Shape === "function" ? result.Shape() : result;
}

function profileRadiusFactor(params, s) {
  s = Math.max(-1.0, Math.min(1.0, s));
  if (params.preset === "tictac") {
    const epsilon = 0.15;
    const rounded = Math.max(0.0, 1.0 - Math.pow(Math.abs(s), 18));
    return Math.pow(rounded, 0.5) * (1.0 - epsilon + epsilon * (1.0 - s * s));
  }
  const core = Math.max(0.0, 1.0 - Math.pow(Math.abs(s), params.q));
  const radius = Math.pow(core, 1.0 / params.p);
  const modifier = 1.0 + params.alpha * s + params.beta * (1.0 - s * s);
  return Math.max(0.0, radius * Math.max(0.0, modifier));
}

function solveReferenceOuterVolume(params, samples = 4096) {
  let total = 0.0;
  const step = 1.0 / samples;
  let previous = 0.0;
  for (let i = 0; i <= samples; i += 1) {
    const t = i / samples;
    const s = 1.0 - 2.0 * t;
    const radius = 0.5 * params.profileThickness * profileRadiusFactor(params, s);
    const area = Math.PI * radius * radius;
    if (i > 0) {
      total += 0.5 * (previous + area) * (params.profileLength * step);
    }
    previous = area;
  }
  return total;
}

function boreArea(params) {
  return Math.PI * Math.pow(params.bore * 0.5, 2);
}

function ledOpenVolume(params) {
  return Math.PI * Math.pow(params.ledD * 0.5, 2) * params.ledLen;
}

function solvedTargetVolume(params) {
  return params.buoyancyG * 1000.0 + ledOpenVolume(params);
}

function solveProfileScale(params) {
  const referenceOuterVolume = solveReferenceOuterVolume(params);
  const targetVolume = solvedTargetVolume(params);
  let low = 0.0;
  let high = Math.max(1.0, Math.cbrt(targetVolume / Math.max(referenceOuterVolume, 1e-9)));
  while (
    referenceOuterVolume * high * high * high
      - boreArea(params) * (params.profileLength * high)
      - targetVolume < 0.0
  ) {
    high *= 2.0;
  }
  for (let i = 0; i < 48; i += 1) {
    const mid = 0.5 * (low + high);
    const netVolume = referenceOuterVolume * mid * mid * mid
      - boreArea(params) * (params.profileLength * mid)
      - targetVolume;
    if (netVolume < 0.0) {
      low = mid;
    } else {
      high = mid;
    }
  }
  return high;
}

function solveOuterDimensions(params) {
  const scale = solveProfileScale(params);
  return {
    scale,
    length: params.profileLength * scale,
    diameter: params.profileThickness * scale
  };
}

function rAtLocal(params, zLocal, length) {
  const scale = length / params.profileLength;
  const halfThickness = 0.5 * params.profileThickness * scale;
  const s = 1.0 - 2.0 * (zLocal / length);
  return profileRadiusFactor(params, s) * halfThickness;
}

function firstLocalZForRadius(params, length, targetRadius) {
  if (targetRadius <= 0.0) return 0.0;
  let lower = 0.0;
  let upper = Math.min(length, 1.0);
  while (upper < length && rAtLocal(params, upper, length) < targetRadius) {
    lower = upper;
    upper = Math.min(length, upper * 2.0);
  }
  if (rAtLocal(params, upper, length) < targetRadius) {
    return length;
  }
  for (let i = 0; i < 24; i += 1) {
    const mid = 0.5 * (lower + upper);
    if (rAtLocal(params, mid, length) < targetRadius) {
      lower = mid;
    } else {
      upper = mid;
    }
  }
  return upper;
}

function rAtLocalOriented(params, zLocal, length, invertOrientation) {
  const sampleLocalZ = invertOrientation ? (length - zLocal) : zLocal;
  return rAtLocal(params, sampleLocalZ, length);
}

function firstLocalZForRadiusOriented(params, length, targetRadius, invertOrientation) {
  if (targetRadius <= 0.0) return 0.0;
  let lower = 0.0;
  let upper = Math.min(length, 1.0);
  while (upper < length && rAtLocalOriented(params, upper, length, invertOrientation) < targetRadius) {
    lower = upper;
    upper = Math.min(length, upper * 2.0);
  }
  if (rAtLocalOriented(params, upper, length, invertOrientation) < targetRadius) {
    return length;
  }
  for (let i = 0; i < 24; i += 1) {
    const mid = 0.5 * (lower + upper);
    if (rAtLocalOriented(params, mid, length, invertOrientation) < targetRadius) {
      lower = mid;
    } else {
      upper = mid;
    }
  }
  return upper;
}

function linearSampleFractions(samples) {
  return Array.from({ length: samples + 1 }, (_, i) => i / samples);
}

function buildOuterProfilePolyline(params, length, bodyZ0, samples = PROFILE_SAMPLES) {
  return linearSampleFractions(samples).map((fraction) => {
    const zLocal = length * fraction;
    return {
      x: Math.max(0.0, rAtLocal(params, zLocal, length)),
      z: bodyZ0 + zLocal
    };
  });
}

function buildInnerOffsetPolyline(params, outerPoints, bodyZ0, length) {
  const minRadius = 0.05;
  const minZ = bodyZ0 + Math.max(params.wallThickness * 0.25, 0.05);
  const maxZ = bodyZ0 + length - Math.max(params.wallThickness * 0.25, 0.05);
  const rawInner = [];

  for (let i = 0; i < outerPoints.length; i += 1) {
    const prev = outerPoints[Math.max(i - 1, 0)];
    const next = outerPoints[Math.min(i + 1, outerPoints.length - 1)];
    const dx = next.x - prev.x;
    const dz = next.z - prev.z;
    const tangentLength = Math.hypot(dx, dz);
    if (tangentLength <= 1.0e-9) {
      continue;
    }

    const inwardNormalX = -dz / tangentLength;
    const inwardNormalZ = dx / tangentLength;
    const outer = outerPoints[i];
    let innerX = outer.x + inwardNormalX * params.wallThickness;
    let innerZ = outer.z + inwardNormalZ * params.wallThickness;

    innerX = Math.min(innerX, outer.x - 0.01);
    if (innerX <= minRadius || innerZ <= minZ || innerZ >= maxZ) {
      continue;
    }
    rawInner.push({ x: innerX, z: innerZ });
  }

  const inner = [];
  let lastZ = -Infinity;
  for (const point of rawInner) {
    if (point.z <= lastZ + 1.0e-3) {
      continue;
    }
    inner.push(point);
    lastZ = point.z;
  }

  if (inner.length < 2) {
    throw new Error("Shell thickness is too large for this profile.");
  }
  return inner;
}

function interpolatePointAtX(a, b, targetX) {
  const dx = b.x - a.x;
  if (Math.abs(dx) <= 1.0e-9) {
    return { x: targetX, z: a.z };
  }
  const t = Math.max(0.0, Math.min(1.0, (targetX - a.x) / dx));
  return {
    x: targetX,
    z: a.z + (b.z - a.z) * t
  };
}

function trimProfileToVerticals(points, topTargetX, bottomTargetX) {
  if (!points || points.length < 2) {
    return points ? points.slice() : [];
  }

  let startPoint = points[0];
  let startIndex = 0;
  for (let i = 1; i < points.length; i += 1) {
    const prev = points[i - 1];
    const curr = points[i];
    if (curr.x >= topTargetX) {
      startPoint = prev.x >= topTargetX ? prev : interpolatePointAtX(prev, curr, topTargetX);
      startIndex = i;
      break;
    }
  }

  let endPoint = points[points.length - 1];
  let endIndex = points.length - 1;
  for (let i = points.length - 1; i > 0; i -= 1) {
    const prev = points[i - 1];
    const curr = points[i];
    if (prev.x >= bottomTargetX) {
      endPoint = curr.x >= bottomTargetX ? curr : interpolatePointAtX(prev, curr, bottomTargetX);
      endIndex = i - 1;
      break;
    }
  }

  const trimmed = [];
  pushUniqueProfilePoint(trimmed, startPoint);
  for (let i = startIndex; i <= endIndex; i += 1) {
    pushUniqueProfilePoint(trimmed, points[i]);
  }
  pushUniqueProfilePoint(trimmed, endPoint);
  return trimmed;
}

function findProfilePointAtX(points, targetX, fromStart = true) {
  if (!points || points.length < 2) {
    return points && points.length ? { x: targetX, z: points[0].z } : { x: targetX, z: 0.0 };
  }

  if (fromStart) {
    for (let i = 1; i < points.length; i += 1) {
      const prev = points[i - 1];
      const curr = points[i];
      if (curr.x >= targetX) {
        return prev.x >= targetX ? { x: prev.x, z: prev.z } : interpolatePointAtX(prev, curr, targetX);
      }
    }
    const last = points[points.length - 1];
    return { x: Math.max(targetX, last.x), z: last.z };
  }

  for (let i = points.length - 1; i > 0; i -= 1) {
    const prev = points[i - 1];
    const curr = points[i];
    if (prev.x >= targetX) {
      return curr.x >= targetX ? { x: curr.x, z: curr.z } : interpolatePointAtX(prev, curr, targetX);
    }
  }

  const first = points[0];
  return { x: Math.max(targetX, first.x), z: first.z };
}

function sliceProfileByZ(points, startPoint, endPoint, tolerance = 1.0e-6) {
  const sliced = [];
  pushUniqueProfilePoint(sliced, startPoint, tolerance);
  for (const point of points) {
    if (point.z > startPoint.z + tolerance && point.z < endPoint.z - tolerance) {
      pushUniqueProfilePoint(sliced, point, tolerance);
    }
  }
  pushUniqueProfilePoint(sliced, endPoint, tolerance);
  return sliced;
}

function normalizeAnglePositive(angle) {
  let normalized = angle % (2.0 * Math.PI);
  if (normalized < 0.0) {
    normalized += 2.0 * Math.PI;
  }
  return normalized;
}

function solveWallFilletOnPolyline(points, wallX, requestedRadius, isTopFillet) {
  if (!points || points.length < 2 || requestedRadius <= 1.0e-6) {
    return null;
  }

  const maxAttempts = 18;
  for (let attempt = 0; attempt <= maxAttempts; attempt += 1) {
    const radius = requestedRadius * (1.0 - attempt / maxAttempts);
    if (radius <= 1.0e-6) {
      break;
    }

    const centerX = wallX + radius;
    const indices = isTopFillet
      ? Array.from({ length: points.length - 1 }, (_, i) => i)
      : Array.from({ length: points.length - 1 }, (_, i) => points.length - 2 - i);

    for (const i of indices) {
      const a = points[i];
      const b = points[i + 1];
      const dx = b.x - a.x;
      const dz = b.z - a.z;
      const segLength = Math.hypot(dx, dz);
      if (segLength <= 1.0e-9 || Math.abs(dx) <= 1.0e-9) {
        continue;
      }

      const normalX = -dz / segLength;
      const normalZ = dx / segLength;
      if (normalX >= -1.0e-6) {
        continue;
      }

      let t = (centerX - (a.x + normalX * radius)) / dx;
      if (t < -1.0e-6 || t > 1.0 + 1.0e-6) {
        continue;
      }
      t = Math.max(0.0, Math.min(1.0, t));

      const tangent = {
        x: a.x + dx * t,
        z: a.z + dz * t
      };
      const center = {
        x: centerX,
        z: tangent.z + normalZ * radius
      };
      const wall = {
        x: wallX,
        z: center.z
      };

      if (isTopFillet && center.z <= tangent.z + 1.0e-6) {
        continue;
      }
      if (!isTopFillet && center.z >= tangent.z - 1.0e-6) {
        continue;
      }

      const tangentAngle = normalizeAnglePositive(Math.atan2(tangent.z - center.z, tangent.x - center.x));
      if (isTopFillet) {
        if (tangentAngle < Math.PI - 1.0e-6) {
          continue;
        }
      } else if (tangentAngle > Math.PI + 1.0e-6) {
        continue;
      }

      return {
        radius,
        tangent,
        center,
        wall,
        tangentAngle
      };
    }
  }

  return null;
}

function makeShelfCornerFillet(wallX, shelfPoint, requestedRadius, isBottomFillet) {
  const horizontalGap = shelfPoint.x - wallX;
  const radius = Math.max(0.0, Math.min(requestedRadius, horizontalGap));
  if (radius <= 1.0e-6) {
    return null;
  }

  if (isBottomFillet) {
    return {
      radius,
      tangent: { x: wallX + radius, z: shelfPoint.z },
      center: { x: wallX + radius, z: shelfPoint.z + radius },
      wall: { x: wallX, z: shelfPoint.z + radius },
      tangentAngle: 1.5 * Math.PI
    };
  }

  return {
    radius,
    tangent: { x: wallX + radius, z: shelfPoint.z },
    center: { x: wallX + radius, z: shelfPoint.z - radius },
    wall: { x: wallX, z: shelfPoint.z - radius },
    tangentAngle: 0.5 * Math.PI
  };
}

function makeProfileWire(oc, points) {
  const polygon = instantiateAvailable(oc, ["BRepBuilderAPI_MakePolygon_1", "BRepBuilderAPI_MakePolygon"], []);
  for (const point of points) {
    callFirstAvailable(polygon, ["Add_1", "Add"], [new oc.gp_Pnt_3(point.x, 0, point.z)]);
  }
  callFirstAvailable(polygon, ["Close"], []);
  return callFirstAvailable(polygon, ["Wire"], []);
}

function makeWireFrom3dPoints(oc, points, close = true) {
  const polygon = instantiateAvailable(oc, ["BRepBuilderAPI_MakePolygon_1", "BRepBuilderAPI_MakePolygon"], []);
  for (const point of points) {
    callFirstAvailable(polygon, ["Add_1", "Add"], [new oc.gp_Pnt_3(point[0], point[1], point[2])]);
  }
  if (close) {
    callFirstAvailable(polygon, ["Close"], []);
  }
  return callFirstAvailable(polygon, ["Wire"], []);
}

function makeFaceFromWire(oc, wire) {
  return shapeOf(
    instantiateAvailableVariants(oc, [
      { names: ["BRepBuilderAPI_MakeFace_16", "BRepBuilderAPI_MakeFace_15", "BRepBuilderAPI_MakeFace_8", "BRepBuilderAPI_MakeFace_6", "BRepBuilderAPI_MakeFace_1", "BRepBuilderAPI_MakeFace"], args: [wire, true] },
      { names: ["BRepBuilderAPI_MakeFace_16", "BRepBuilderAPI_MakeFace_15", "BRepBuilderAPI_MakeFace_8", "BRepBuilderAPI_MakeFace_6", "BRepBuilderAPI_MakeFace_1", "BRepBuilderAPI_MakeFace"], args: [wire] }
    ])
  );
}

function makeFaceFromWires(oc, outerWire, innerWires = []) {
  const maker = instantiateAvailableVariants(oc, [
    {
      names: ["BRepBuilderAPI_MakeFace_16", "BRepBuilderAPI_MakeFace_15", "BRepBuilderAPI_MakeFace_8", "BRepBuilderAPI_MakeFace_6", "BRepBuilderAPI_MakeFace_1", "BRepBuilderAPI_MakeFace"],
      args: [outerWire, true]
    },
    {
      names: ["BRepBuilderAPI_MakeFace_16", "BRepBuilderAPI_MakeFace_15", "BRepBuilderAPI_MakeFace_8", "BRepBuilderAPI_MakeFace_6", "BRepBuilderAPI_MakeFace_1", "BRepBuilderAPI_MakeFace"],
      args: [outerWire]
    }
  ]);
  for (const innerWire of innerWires) {
    callFirstAvailable(maker, ["Add_1", "Add"], [innerWire]);
  }
  return shapeOf(callFirstAvailable(maker, ["Face", "Shape"], []));
}

function makeRevolveProfileFace(oc, profilePoints) {
  if (!profilePoints || profilePoints.length < 2) {
    throw new Error("At least two profile points are required for revolve.");
  }
  const start = profilePoints[0];
  const end = profilePoints[profilePoints.length - 1];
  const closedProfile = [{ x: 0.0, z: start.z }, ...profilePoints, { x: 0.0, z: end.z }];
  const wire = makeProfileWire(oc, closedProfile);
  return makeFaceFromWire(oc, wire);
}

function revolveShape(oc, shape) {
  const axisPoint = new oc.gp_Pnt_3(0, 0, 0);
  const axisDir = instantiateAvailable(oc, ["gp_Dir_4", "gp_Dir_2", "gp_Dir"], [0, 0, 1]);
  const axis = instantiateAvailableVariants(oc, [
    { names: ["gp_Ax1_3", "gp_Ax1_2", "gp_Ax1"], args: [axisPoint, axisDir] }
  ]);
  return shapeOf(
    instantiateAvailableVariants(oc, [
      { names: ["BRepPrimAPI_MakeRevol_1", "BRepPrimAPI_MakeRevol"], args: [shape, axis] },
      { names: ["BRepPrimAPI_MakeRevol_2", "BRepPrimAPI_MakeRevol"], args: [shape, axis, 2.0 * Math.PI] },
      { names: ["BRepPrimAPI_MakeRevol_3", "BRepPrimAPI_MakeRevol"], args: [shape, axis, 2.0 * Math.PI, false] }
    ])
  );
}

function pushUniqueProfilePoint(points, point, tolerance = 1.0e-3) {
  const previous = points[points.length - 1];
  if (
    previous
    && Math.abs(previous.x - point.x) <= tolerance
    && Math.abs(previous.z - point.z) <= tolerance
  ) {
    return;
  }
  points.push(point);
}

function sampleArcPoints(centerX, centerZ, radius, startAngle, endAngle, segments = 8) {
  const pts = [];
  for (let i = 1; i <= segments; i += 1) {
    const t = i / segments;
    const angle = startAngle + (endAngle - startAngle) * t;
    pts.push({
      x: centerX + Math.cos(angle) * radius,
      z: centerZ + Math.sin(angle) * radius
    });
  }
  return pts;
}

function buildInternalCavityProfilePoints(params, length, bodyZ0, postOuterRadius) {
  const outerProfile = buildOuterProfilePolyline(params, length, bodyZ0, PROFILE_SAMPLES);
  const rawInnerProfile = buildInnerOffsetPolyline(params, outerProfile, bodyZ0, length);
  if (rawInnerProfile.length < 2) {
    throw new Error("Inner cavity profile is too short.");
  }

  const rawStart = rawInnerProfile[0];
  const rawEnd = rawInnerProfile[rawInnerProfile.length - 1];
  const verticalSpan = Math.max(0.2, Math.abs(rawEnd.z - rawStart.z));
  const maxInnerX = rawInnerProfile.reduce((maxValue, point) => Math.max(maxValue, point.x), 0.0);
  const maxHorizontalGap = Math.max(0.0, maxInnerX - postOuterRadius);
  let baseRadius = Math.abs(params.postBaseFilletRadius || 0.0);
  let tailRadius = Math.abs(params.postTailFilletRadius || 0.0);
  baseRadius = Math.min(baseRadius, verticalSpan * 0.45);
  tailRadius = Math.min(tailRadius, Math.max(0.0, verticalSpan - baseRadius), verticalSpan * 0.45);
  baseRadius = Math.min(baseRadius, maxHorizontalGap);
  tailRadius = Math.min(tailRadius, maxHorizontalGap);

  const baseShelfPoint = findProfilePointAtX(
    rawInnerProfile,
    postOuterRadius + Math.max(baseRadius, 0.05),
    true
  );
  const tailShelfPoint = findProfilePointAtX(
    rawInnerProfile,
    postOuterRadius + Math.max(tailRadius, 0.05),
    false
  );

  const baseFillet = solveWallFilletOnPolyline(rawInnerProfile, postOuterRadius, baseRadius, true)
    || makeShelfCornerFillet(postOuterRadius, baseShelfPoint, baseRadius, true);
  const tailFillet = solveWallFilletOnPolyline(rawInnerProfile, postOuterRadius, tailRadius, false)
    || makeShelfCornerFillet(postOuterRadius, tailShelfPoint, tailRadius, false);
  baseRadius = baseFillet ? baseFillet.radius : 0.0;
  tailRadius = tailFillet ? tailFillet.radius : 0.0;

  const start = baseFillet ? baseFillet.tangent : findProfilePointAtX(rawInnerProfile, postOuterRadius, true);
  const end = tailFillet ? tailFillet.tangent : findProfilePointAtX(rawInnerProfile, postOuterRadius, false);
  const innerProfile = sliceProfileByZ(rawInnerProfile, start, end);

  const cavityProfile = [];
  const baseFilletPath = [];
  const tailFilletPath = [];
  if (baseFillet) {
    pushUniqueProfilePoint(cavityProfile, baseFillet.wall);
    pushUniqueProfilePoint(baseFilletPath, baseFillet.wall);
    for (const point of sampleArcPoints(baseFillet.center.x, baseFillet.center.z, baseFillet.radius, Math.PI, baseFillet.tangentAngle, 16)) {
      pushUniqueProfilePoint(cavityProfile, point);
      pushUniqueProfilePoint(baseFilletPath, point);
    }
  } else {
    const baseWallPoint = { x: postOuterRadius, z: start.z };
    pushUniqueProfilePoint(cavityProfile, baseWallPoint);
    pushUniqueProfilePoint(baseFilletPath, baseWallPoint);
  }

  pushUniqueProfilePoint(cavityProfile, start);
  pushUniqueProfilePoint(baseFilletPath, start);
  for (let i = 1; i < innerProfile.length - 1; i += 1) {
    pushUniqueProfilePoint(cavityProfile, innerProfile[i]);
  }
  pushUniqueProfilePoint(cavityProfile, end);
  pushUniqueProfilePoint(tailFilletPath, end);

  if (tailFillet) {
    for (const point of sampleArcPoints(tailFillet.center.x, tailFillet.center.z, tailFillet.radius, tailFillet.tangentAngle, Math.PI, 16)) {
      pushUniqueProfilePoint(cavityProfile, point);
      pushUniqueProfilePoint(tailFilletPath, point);
    }
  } else {
    const tailWallPoint = { x: postOuterRadius, z: end.z };
    pushUniqueProfilePoint(cavityProfile, tailWallPoint);
    pushUniqueProfilePoint(tailFilletPath, tailWallPoint);
  }

  return {
    points: cavityProfile,
    baseRadius,
    tailRadius,
    baseFilletPath,
    tailFilletPath
  };
}

function buildInternalCavityWithPostFilletsByRevolve(oc, params, length, bodyZ0, postOuterRadius) {
  const profile = buildInternalCavityProfilePoints(params, length, bodyZ0, postOuterRadius);
  const face = makeRevolveProfileFace(oc, profile.points);
  return {
    shape: revolveShape(oc, face),
    baseRadius: profile.baseRadius,
    tailRadius: profile.tailRadius
  };
}

function buildOuterHullByRevolve(oc, params, length, bodyZ0) {
  const outerProfile = buildOuterProfilePolyline(params, length, bodyZ0, PROFILE_SAMPLES);
  const outerFace = makeRevolveProfileFace(oc, outerProfile);
  return revolveShape(oc, outerFace);
}

function buildShellWithPostFilletsByRevolve(oc, params, length, bodyZ0, postOuterRadius) {
  const boreRadius = Math.max(0.05, params.bore * 0.5);
  const outerProfile = trimProfileToVerticals(
    buildOuterProfilePolyline(params, length, bodyZ0, PROFILE_SAMPLES),
    boreRadius,
    boreRadius
  );
  const cavityProfile = buildInternalCavityProfilePoints(params, length, bodyZ0, postOuterRadius);
  const sectionProfile = [
    { x: boreRadius, z: outerProfile[0].z },
    ...outerProfile,
    { x: boreRadius, z: outerProfile[outerProfile.length - 1].z }
  ];
  const outerWire = makeProfileWire(oc, sectionProfile);
  const cavityWire = makeProfileWire(oc, cavityProfile.points);
  const face = makeFaceFromWires(oc, outerWire, [cavityWire]);
  const boreStartZ = cavityProfile.points[0].z;
  const boreEndZ = cavityProfile.points[cavityProfile.points.length - 1].z;
  return {
    shape: revolveShape(oc, face),
    sectionFace: face,
    baseRadius: cavityProfile.baseRadius,
    tailRadius: cavityProfile.tailRadius,
    cavityProfilePoints: cavityProfile.points.map((point) => ({ ...point })),
    boreStartZ,
    boreEndZ
  };
}

function cross2d(a, b) {
  return a.x * b.z - a.z * b.x;
}

function intersectRayWithSegment2D(origin, direction, a, b, tolerance = 1.0e-6) {
  const segment = { x: b.x - a.x, z: b.z - a.z };
  const denominator = cross2d(direction, segment);
  if (Math.abs(denominator) <= tolerance) {
    return null;
  }
  const diff = { x: a.x - origin.x, z: a.z - origin.z };
  const rayT = cross2d(diff, segment) / denominator;
  const segT = cross2d(diff, direction) / denominator;
  if (rayT < -tolerance || segT < -tolerance || segT > 1.0 + tolerance) {
    return null;
  }
  return {
    t: rayT,
    point: {
      x: origin.x + direction.x * rayT,
      z: origin.z + direction.z * rayT
    }
  };
}

function findRayHitsOnClosedProfile(origin, direction, profilePoints, tolerance = 1.0e-5) {
  if (!profilePoints || profilePoints.length < 2) {
    return [];
  }

  const hits = [];
  for (let i = 0; i < profilePoints.length; i += 1) {
    const a = profilePoints[i];
    const b = profilePoints[(i + 1) % profilePoints.length];
    const hit = intersectRayWithSegment2D(origin, direction, a, b, tolerance);
    if (!hit) {
      continue;
    }
    const duplicate = hits.some((existing) => Math.abs(existing.t - hit.t) <= tolerance);
    if (!duplicate) {
      hits.push(hit);
    }
  }

  hits.sort((left, right) => left.t - right.t);
  return hits;
}

function makeVector(oc, x, y, z) {
  return instantiateAvailable(oc, ["gp_Vec_4", "gp_Vec_2", "gp_Vec_1", "gp_Vec"], [x, y, z]);
}

function extrudeShape(oc, shape, dx, dy, dz) {
  const vec = makeVector(oc, dx, dy, dz);
  return shapeOf(
    instantiateAvailableVariants(oc, [
      { names: ["BRepPrimAPI_MakePrism_1", "BRepPrimAPI_MakePrism"], args: [shape, vec, false, true] },
      { names: ["BRepPrimAPI_MakePrism_2", "BRepPrimAPI_MakePrism"], args: [shape, vec, false, false, true] }
    ])
  );
}

function cross3d(a, b) {
  return {
    x: a.y * b.z - a.z * b.y,
    y: a.z * b.x - a.x * b.z,
    z: a.x * b.y - a.y * b.x
  };
}

function normalize3d(vector) {
  const magnitude = Math.hypot(vector.x, vector.y, vector.z);
  if (magnitude <= 1.0e-9) {
    throw new Error("Vector is too small to normalize.");
  }
  return {
    x: vector.x / magnitude,
    y: vector.y / magnitude,
    z: vector.z / magnitude
  };
}

function fuseShapes(oc, a, b) {
  return shapeOf(
    instantiateAvailableVariants(oc, [
      { names: ["BRepAlgoAPI_Fuse_2", "BRepAlgoAPI_Fuse"], args: [a, b] },
      { names: ["BRepAlgoAPI_Fuse_3"], args: [a, b] }
    ])
  );
}

function cutShapes(oc, a, b) {
  return shapeOf(
    instantiateAvailableVariants(oc, [
      { names: ["BRepAlgoAPI_Cut_2", "BRepAlgoAPI_Cut"], args: [a, b] },
      { names: ["BRepAlgoAPI_Cut_3"], args: [a, b] }
    ])
  );
}

function intersectShapes(oc, a, b) {
  return shapeOf(
    instantiateAvailableVariants(oc, [
      { names: ["BRepAlgoAPI_Common_2", "BRepAlgoAPI_Common"], args: [a, b] },
      { names: ["BRepAlgoAPI_Common_3"], args: [a, b] }
    ])
  );
}

function makeCylinderBetween(oc, start, end, radius) {
  const dx = end[0] - start[0];
  const dy = end[1] - start[1];
  const dz = end[2] - start[2];
  const length = Math.hypot(dx, dy, dz);
  if (length <= 1.0e-6) {
    throw new Error("Cylinder axis is too short.");
  }
  const location = new oc.gp_Pnt_3(start[0], start[1], start[2]);
  const direction = instantiateAvailable(oc, ["gp_Dir_4", "gp_Dir_2"], [dx / length, dy / length, dz / length]);
  const axis = new oc.gp_Ax2_3(location, direction);
  return shapeOf(new oc.BRepPrimAPI_MakeCylinder_3(axis, radius, length));
}

function makeBoxFromCorner(oc, corner, dx, dy, dz) {
  return shapeOf(
    instantiateAvailableVariants(oc, [
      { names: ["BRepPrimAPI_MakeBox_4", "BRepPrimAPI_MakeBox_2", "BRepPrimAPI_MakeBox"], args: [corner, dx, dy, dz] }
    ])
  );
}

function transformShape(oc, shape, trsf) {
  return shapeOf(
    instantiateAvailableVariants(oc, [
      { names: ["BRepBuilderAPI_Transform_2", "BRepBuilderAPI_Transform"], args: [shape, trsf, true] },
      { names: ["BRepBuilderAPI_Transform_1", "BRepBuilderAPI_Transform"], args: [shape, trsf] }
    ])
  );
}

function rotateShapeAroundYAxis(oc, shape, pivotX, pivotY, pivotZ, angleDeg) {
  const trsf = instantiateAvailable(oc, ["gp_Trsf_1", "gp_Trsf"], []);
  const axisPoint = new oc.gp_Pnt_3(pivotX, pivotY, pivotZ);
  const axisDir = instantiateAvailable(oc, ["gp_Dir_4", "gp_Dir_2", "gp_Dir"], [0, 1, 0]);
  const axis = instantiateAvailableVariants(oc, [
    { names: ["gp_Ax1_3", "gp_Ax1_2", "gp_Ax1"], args: [axisPoint, axisDir] }
  ]);
  callFirstAvailableVariant(trsf, [
    { names: ["SetRotation_1", "SetRotation"], args: [axis, angleDeg * Math.PI / 180.0] }
  ]);
  return transformShape(oc, shape, trsf);
}

function rotateShapeAroundXAxis(oc, shape, pivotX, pivotY, pivotZ, angleDeg) {
  const trsf = instantiateAvailable(oc, ["gp_Trsf_1", "gp_Trsf"], []);
  const axisPoint = new oc.gp_Pnt_3(pivotX, pivotY, pivotZ);
  const axisDir = instantiateAvailable(oc, ["gp_Dir_4", "gp_Dir_2", "gp_Dir"], [1, 0, 0]);
  const axis = instantiateAvailableVariants(oc, [
    { names: ["gp_Ax1_3", "gp_Ax1_2", "gp_Ax1"], args: [axisPoint, axisDir] }
  ]);
  callFirstAvailableVariant(trsf, [
    { names: ["SetRotation_1", "SetRotation"], args: [axis, angleDeg * Math.PI / 180.0] }
  ]);
  return transformShape(oc, shape, trsf);
}

function cutCylinderEndAtAngle(oc, cylinder, {
  centerX,
  diameter,
  z0,
  height,
  angleDeg
}) {
  if (angleDeg <= 0.0) {
    return cylinder;
  }

  const cutSpan = Math.max(height, diameter) * 4.0;
  const pivotZ = z0 + height;
  const sideSign = centerX >= 0.0 ? 1.0 : -1.0;
  const pivotX = centerX - sideSign * diameter * 0.5;
  const cutter = makeBoxFromCorner(
    oc,
    new oc.gp_Pnt_3(pivotX - cutSpan * 0.5, -cutSpan * 0.5, pivotZ),
    cutSpan,
    cutSpan,
    cutSpan
  );
  const rotatedCutter = rotateShapeAroundYAxis(
    oc,
    cutter,
    pivotX,
    0.0,
    pivotZ,
    sideSign * angleDeg
  );
  return cutShapes(oc, cylinder, rotatedCutter);
}

function makeVerticalCylinder(oc, centerX, centerY, z0, z1, radius) {
  return makeCylinderBetween(oc, [centerX, centerY, z0], [centerX, centerY, z1], radius);
}

function makeSquareBeam(oc, start, end, size) {
  const dx = end[0] - start[0];
  const dy = end[1] - start[1];
  const dz = end[2] - start[2];
  const length = Math.hypot(dx, dy, dz);
  if (length <= 1.0e-6) {
    throw new Error("Square beam axis is too short.");
  }

  const direction = normalize3d({ x: dx, y: dy, z: dz });
  const reference = Math.abs(direction.z) < 0.9
    ? { x: 0, y: 0, z: 1 }
    : { x: 0, y: 1, z: 0 };
  const xAxis = normalize3d(cross3d(reference, direction));
  const yAxis = normalize3d(cross3d(direction, xAxis));
  const half = size * 0.5;
  const square = [
    [start[0] - xAxis.x * half - yAxis.x * half, start[1] - xAxis.y * half - yAxis.y * half, start[2] - xAxis.z * half - yAxis.z * half],
    [start[0] + xAxis.x * half - yAxis.x * half, start[1] + xAxis.y * half - yAxis.y * half, start[2] + xAxis.z * half - yAxis.z * half],
    [start[0] + xAxis.x * half + yAxis.x * half, start[1] + xAxis.y * half + yAxis.y * half, start[2] + xAxis.z * half + yAxis.z * half],
    [start[0] - xAxis.x * half + yAxis.x * half, start[1] - xAxis.y * half + yAxis.y * half, start[2] - xAxis.z * half + yAxis.z * half]
  ];
  const face = makeFaceFromWire(oc, makeWireFrom3dPoints(oc, square));
  return extrudeShape(oc, face, dx, dy, dz);
}

function buildTripodBeams(oc, params, length, bodyZ0, diameter, baseRadius, baseAnchorZ) {
  const legs = [];
  const addRing = (worldZ, azimuthOffsetDeg) => {
    if (!Number.isFinite(worldZ) || worldZ <= 0.0) return;
    const localZ = Math.max(0.5, Math.min(length - 0.5, worldZ - bodyZ0));
    const attachWorldZ = bodyZ0 + localZ;
    const attachRadius = Math.max(1.0, rAtLocalOriented(params, localZ, length, !!params.invertBase));
    for (const baseAngle of [0, 120, 240]) {
      const angleDeg = params.tripodOffsetDeg + azimuthOffsetDeg + baseAngle;
      const angleRad = angleDeg * Math.PI / 180.0;
      const top = [
        attachRadius * Math.cos(angleRad),
        attachRadius * Math.sin(angleRad),
        attachWorldZ
      ];
      const bottom = [
        (baseRadius - 2.4) * Math.cos(angleRad),
        (baseRadius - 2.4) * Math.sin(angleRad),
        baseAnchorZ
      ];
      legs.push(makeSquareBeam(oc, bottom, top, params.legThickness));
    }
  };

  addRing(params.tripodAttachZ, 0.0);
  if (params.tripodAttachZ2 > 0.0) {
    addRing(params.tripodAttachZ2, 60.0);
  }

  if (!legs.length) {
    return null;
  }

  return legs.reduce((acc, leg) => (acc ? fuseShapes(oc, acc, leg) : leg), null);
}

async function yieldToWorker(jobId, message = null, tone = "normal") {
  throwIfBuildCancelled(jobId);
  if (message) {
    await emitProgress(jobId, message, tone);
  } else {
    await sleep(0);
  }
  throwIfBuildCancelled(jobId);
}

function buildMetrics({ scale, length, diameter, bodyZ0, params, cavityResult, drainActive, buildStage }) {
  return {
    scale,
    length,
    diameter,
    bodyStartZ: bodyZ0,
    targetBuoyantVolumeCm3: params.buoyancyG,
    ledAxisOffset: params.ledCenterOffset,
    drainActive,
    postBaseFilletRadius: cavityResult.baseRadius,
    postTailFilletRadius: cavityResult.tailRadius,
    buildStage
  };
}

async function tessellateShape(
  oc,
  shape,
  jobId = null,
  linearDeflection = DEFAULT_LINEAR_DEFLECTION,
  angularDeflection = DEFAULT_ANGULAR_DEFLECTION
) {
  throwIfBuildCancelled(jobId);
  new oc.BRepMesh_IncrementalMesh_2(shape, linearDeflection, false, angularDeflection, false);
  const positions = [];
  const normals = [];
  const indices = [];
  let vertexOffset = 0;
  let faceCount = 0;
  const explorer = new oc.TopExp_Explorer_1();
  explorer.Init(shape, oc.TopAbs_ShapeEnum.TopAbs_FACE, oc.TopAbs_ShapeEnum.TopAbs_SHAPE);
  for (; explorer.More(); explorer.Next()) {
    faceCount += 1;
    if (jobId !== null && faceCount % 8 === 0) {
      throwIfBuildCancelled(jobId);
      await emitProgress(jobId, "Meshing preview...");
    }
    const face = oc.TopoDS.Face_1(explorer.Current());
    const location = new oc.TopLoc_Location_1();
    const triangulationHandle = callFunctionVariants(
      oc.BRep_Tool.Triangulation.bind(oc.BRep_Tool),
      [
        [face, location],
        [face, location, 0]
      ]
    );
    if (triangulationHandle.IsNull()) {
      continue;
    }
    const triangulation = triangulationHandle.get();
    const transform = location.Transformation();
    const connect = new oc.Poly_Connect_2(triangulationHandle);
    const faceNormals = new oc.TColgp_Array1OfDir_2(1, triangulation.NbNodes());
    oc.StdPrs_ToolTriangulatedShape.Normal(face, connect, faceNormals);

    for (let nodeIndex = 1; nodeIndex <= triangulation.NbNodes(); nodeIndex += 1) {
      const p = triangulation.Node(nodeIndex).Transformed(transform);
      positions.push(p.X(), p.Y(), p.Z());
      const n = faceNormals.Value(nodeIndex).Transformed(transform);
      normals.push(n.X(), n.Y(), n.Z());
    }

    const triangles = triangulation.Triangles();
    const forward = face.Orientation_1() === oc.TopAbs_Orientation.TopAbs_FORWARD;
    for (let triIndex = 1; triIndex <= triangulation.NbTriangles(); triIndex += 1) {
      const tri = triangles.Value(triIndex);
      let n1 = tri.Value(1) - 1;
      let n2 = tri.Value(2) - 1;
      let n3 = tri.Value(3) - 1;
      if (!forward) {
        const tmp = n1;
        n1 = n2;
        n2 = tmp;
      }
      indices.push(vertexOffset + n1, vertexOffset + n2, vertexOffset + n3);
    }

    vertexOffset += triangulation.NbNodes();
  }

  return {
    positions: new Float32Array(positions),
    normals: new Float32Array(normals),
    indices: new Uint32Array(indices)
  };
}

async function emitStage(jobId, shape, metrics) {
  throwIfBuildCancelled(jobId);
  const geometry = await tessellateShape(state.oc, shape, jobId);
  throwIfBuildCancelled(jobId);
  post(
    "stage",
    { jobId, metrics, geometry },
    [geometry.positions.buffer, geometry.normals.buffer, geometry.indices.buffer]
  );
  if (STEP_PREVIEW_DELAY_MS > 0) {
    await sleep(STEP_PREVIEW_DELAY_MS);
  } else {
    await sleep(0);
  }
  throwIfBuildCancelled(jobId);
}

async function buildFloatShape(oc, params, jobId, onStage = null) {
  const { scale, length, diameter } = solveOuterDimensions(params);
  const bodyZ0 = params.noBase ? 0.0 : params.baseThickness + params.floatBaseGap;
  const postOuterRadius = params.bore * 0.5 + params.wallThickness;
  const ledAxis = params.ledCenterOffset;

  await yieldToWorker(jobId, "Building shell profile...");
  const shellResult = buildShellWithPostFilletsByRevolve(oc, params, length, bodyZ0, postOuterRadius);
  const cavityResult = buildInternalCavityWithPostFilletsByRevolve(oc, params, length, bodyZ0, postOuterRadius);
  const outerWorld = buildOuterHullByRevolve(oc, params, length, bodyZ0);
  const innerCavityWorld = cavityResult.shape;
  const boreCore = makeVerticalCylinder(oc, 0, 0, bodyZ0, bodyZ0 + length + 0.8, postOuterRadius);
  const boreSupportTopZ = bodyZ0 + length + 0.8;
  const boreCut = makeVerticalCylinder(oc, 0, 0, bodyZ0 - 1.0, boreSupportTopZ + 1.0, Math.max(0.05, params.bore * 0.5));
  const internalCavity = cutShapes(oc, innerCavityWorld, boreCore);

  await yieldToWorker(jobId, "Revolving shell and post...");
  let body = cutShapes(oc, outerWorld, internalCavity);
  let drainActive = false;

  const metricsFor = (buildStage) => buildMetrics({
    scale,
    length,
    diameter,
    bodyZ0,
    params,
    cavityResult,
    drainActive,
    buildStage
  });

  await yieldToWorker(jobId, "Cutting bore...");
  body = cutShapes(oc, body, boreCut);
  if (onStage) {
    await onStage(body, metricsFor("Shell + Post + Bore"));
  }

  const ledSleeveRaw = makeVerticalCylinder(
    oc,
    ledAxis,
    0,
    bodyZ0,
    bodyZ0 + params.ledLen,
    params.ledD * 0.5 + params.wallThickness
  );
  const ledSleeveCut = cutCylinderEndAtAngle(oc, ledSleeveRaw, {
    centerX: ledAxis,
    diameter: 2.0 * (params.ledD * 0.5 + params.wallThickness),
    z0: bodyZ0,
    height: params.ledLen,
    angleDeg: LED_SLEEVE_END_CUT_DEG
  });
  const ledBayRaw = makeVerticalCylinder(
    oc,
    ledAxis,
    0,
    bodyZ0 - 0.5,
    bodyZ0 + Math.max(params.ledLen + 0.5, params.wallThickness) + 0.5,
    params.ledD * 0.5
  );
  const ledBayCut = cutCylinderEndAtAngle(oc, ledBayRaw, {
    centerX: ledAxis,
    diameter: params.ledD,
    z0: bodyZ0 - 0.5,
    height: Math.max(params.ledLen + 0.5, params.wallThickness) + 1.0,
    angleDeg: LED_SLEEVE_END_CUT_DEG
  });
  const ledSleeve = intersectShapes(oc, ledSleeveCut, outerWorld);
  const ledBay = intersectShapes(oc, ledBayCut, outerWorld);

  await yieldToWorker(jobId, "Adding LED sleeve...");
  try {
    const ledSleeveRing = cutShapes(oc, ledSleeve, ledBay);
    const ledFill = intersectShapes(oc, ledSleeveRing, internalCavity);
    body = fuseShapes(oc, body, ledFill);
    if (onStage) {
      await onStage(body, metricsFor("LED Sleeve"));
    }
  } catch (err) {
    post("progress", { jobId, message: `LED sleeve trim skipped: ${err.message}`, tone: "warn" });
  }

  await yieldToWorker(jobId, "Cutting LED bay...");
  body = cutShapes(oc, body, ledBay);
  if (onStage) {
    await onStage(body, metricsFor("LED Bay"));
  }

  if (!params.noDrain && params.drainHoleDiameter > 0.0) {
    const drainStartLocalZ = Math.min(length - 0.2, Math.max(params.drainHoleDiameter, 0.2));
    const directionSign = ledAxis >= 0.0 ? -1.0 : 1.0;
    const angleRad = params.drainAngleDeg * Math.PI / 180.0;
    const drainStart2d = { x: 0.0, z: bodyZ0 + drainStartLocalZ };
    const drainDirection2d = { x: Math.cos(angleRad), z: Math.sin(angleRad) };
    const drainHits = findRayHitsOnClosedProfile(drainStart2d, drainDirection2d, shellResult.cavityProfilePoints);
    const drainExitHit = drainHits.length >= 2 ? drainHits[1] : (drainHits.length === 1 ? drainHits[0] : null);
    if (!drainExitHit) {
      post("progress", { jobId, message: "Drain hole skipped: no valid cavity exit found.", tone: "warn" });
    } else {
      const drainRadius = params.drainHoleDiameter * 0.5;
      const cavityWallClearance = Math.max(0.05, drainRadius + 0.02);
      const drainLengthTrim = cavityWallClearance + Math.max(0.0, params.wallThickness);
      const drainEndT = Math.max(0.05, drainExitHit.t - drainLengthTrim);
      const drainStart = [0.0, 0.0, drainStart2d.z];
      const drainEnd = [
        directionSign * Math.max(0.0, drainStart2d.x + drainDirection2d.x * drainEndT),
        0.0,
        drainStart2d.z + drainDirection2d.z * drainEndT
      ];
      const drainCut = makeCylinderBetween(oc, drainStart, drainEnd, drainRadius);
      await yieldToWorker(jobId, "Cutting drain hole...");
      body = cutShapes(oc, body, drainCut);
      drainActive = true;
      if (onStage) {
        await onStage(body, metricsFor("Drain Hole"));
      }
    }
  }

  if (params.invertBase) {
    await yieldToWorker(jobId, "Rotating float 180 deg for inverted base...");
    body = rotateShapeAroundXAxis(oc, body, 0.0, 0.0, bodyZ0 + 0.5 * length, 180.0);
    if (onStage) {
      await onStage(body, metricsFor("Float Rotated 180"));
    }
  }

  if (!params.noBase) {
    const supportRadius = postOuterRadius;
    const supportTop = bodyZ0 + firstLocalZForRadiusOriented(params, length, supportRadius, !!params.invertBase);
    const baseRadius = Math.max(diameter * params.baseRadiusFactor, supportRadius + 8.0);
    const basePlate = makeVerticalCylinder(oc, 0, 0, 0.0, params.baseThickness, baseRadius);
    const baseAnchorZ = params.baseThickness;
    const support = makeVerticalCylinder(oc, 0, 0, baseAnchorZ, supportTop, supportRadius);

    await yieldToWorker(jobId, "Adding base support...");
    body = fuseShapes(oc, body, basePlate);
    body = fuseShapes(oc, body, support);

    await yieldToWorker(jobId, "Building tripod...");
    const tripod = buildTripodBeams(oc, params, length, bodyZ0, diameter, baseRadius, baseAnchorZ);
    if (tripod) {
      body = fuseShapes(oc, body, tripod);
    }
    if (onStage) {
      await onStage(body, metricsFor("Base Support + Tripod"));
    }
  }

  return {
    shape: body,
    metrics: metricsFor("Final")
  };
}

async function ensureWorkerReady() {
  if (state.ready && state.oc) {
    return state.oc;
  }
  state.oc = await ocFactory({
    locateFile(file) {
      return `${OCCT_BASE_URL}${file}`;
    }
  });
  state.ready = true;
  return state.oc;
}

function tryReadBinaryFile(oc, filePath) {
  try {
    let data = oc.FS.readFile(filePath, { encoding: "binary" });
    if (!(data instanceof Uint8Array)) {
      data = new Uint8Array(data);
    }
    if (!data.length) {
      return null;
    }
    return data;
  } catch {
    return null;
  }
}

function listDirectorySafe(oc, dirPath) {
  try {
    return oc.FS.readdir(dirPath);
  } catch {
    return [];
  }
}

function buildPathCandidates(name) {
  if (!name) return [];
  const out = [name];
  if (!String(name).startsWith("/")) {
    out.push(`/${name}`);
  }
  return Array.from(new Set(out));
}

function stepHeaderScore(data) {
  if (!(data instanceof Uint8Array) || data.length < 16) {
    return 0;
  }
  const preview = new TextDecoder("utf-8", { fatal: false }).decode(data.subarray(0, Math.min(data.length, 4096))).toUpperCase();
  let score = 0;
  if (preview.includes("ISO-10303-21")) score += 2;
  if (preview.includes("HEADER;")) score += 1;
  if (preview.includes("DATA;")) score += 1;
  if (preview.includes("END-ISO-10303-21")) score += 2;
  return score;
}

function diffNewEntries(beforeEntries, afterEntries) {
  const before = new Set(beforeEntries || []);
  return (afterEntries || []).filter((name) => !before.has(name));
}

function tryFindStepFromEntries(oc, entryNames) {
  let best = null;
  let bestScore = -1;
  for (const entry of entryNames || []) {
    const entryText = String(entry || "");
    if (!entryText || entryText === "." || entryText === "..") {
      continue;
    }
    for (const path of buildPathCandidates(entryText)) {
      const data = tryReadBinaryFile(oc, path);
      if (!data) {
        continue;
      }
      const score = stepHeaderScore(data);
      if (score > bestScore) {
        best = data;
        bestScore = score;
      }
      if (score >= 4) {
        return data;
      }
    }
  }
  return bestScore > 0 ? best : null;
}

function tryWriteStepToPath(oc, writer, filePath) {
  const attempts = [];

  const directPathResult = (() => {
    try {
      return callFirstAvailableVariant(writer, [
        { names: ["Write_1"], args: [filePath] },
        { names: ["Write_2"], args: [filePath] },
        { names: ["Write"], args: [filePath] }
      ]);
    } catch (err) {
      attempts.push(`direct path: ${err.message}`);
      return null;
    }
  })();
  if (directPathResult !== null) {
    return directPathResult;
  }

  let asciiPath = null;
  try {
    asciiPath = instantiateAvailableVariants(oc, [
      {
        names: ["TCollection_AsciiString_2", "TCollection_AsciiString_1", "TCollection_AsciiString"],
        args: [filePath]
      }
    ]);
  } catch (err) {
    attempts.push(`ascii path ctor: ${err.message}`);
  }

  if (asciiPath) {
    try {
      return callFirstAvailableVariant(writer, [
        { names: ["Write_1"], args: [asciiPath] },
        { names: ["Write_2"], args: [asciiPath] },
        { names: ["Write"], args: [asciiPath] }
      ]);
    } catch (err) {
      attempts.push(`ascii path object: ${err.message}`);
    }
  }

  throw new Error(`No working STEP write overload for '${filePath}'. Tried: ${attempts.join(" | ")}`);
}

async function exportCurrentStep(requestId) {
  if (!state.ready || !state.oc || !state.currentShape || !state.currentMetrics) {
    throw new Error("Generate a model before exporting STEP.");
  }

  const oc = state.oc;
  const writer = instantiateAvailable(oc, ["STEPControl_Writer_1", "STEPControl_Writer"], []);
  callFirstAvailableVariant(writer, [
    {
      names: ["Transfer_2", "Transfer_3", "Transfer"],
      args: [state.currentShape, oc.STEPControl_StepModelType.STEPControl_AsIs, true]
    },
    {
      names: ["Transfer_1", "Transfer_2", "Transfer_3", "Transfer"],
      args: [state.currentShape, oc.STEPControl_StepModelType.STEPControl_AsIs]
    },
    {
      names: ["Transfer"],
      args: [state.currentShape]
    }
  ]);

  const stem = `customfloatmaker_${requestId || "export"}`;
  const writeCandidates = [
    `${stem}.step`,
    `/${stem}.step`,
    "customfloatmaker.step",
    "/customfloatmaker.step"
  ];

  for (const filePath of writeCandidates) {
    try {
      const rootBefore = listDirectorySafe(oc, "/");
      try {
        oc.FS.unlink(filePath);
      } catch {}
      const status = tryWriteStepToPath(oc, writer, filePath);
      const data = tryReadBinaryFile(oc, filePath);
      if (data) {
        post("step-export", { requestId, data }, [data.buffer]);
        return;
      }

      const rootAfter = listDirectorySafe(oc, "/");
      const newRootEntries = diffNewEntries(rootBefore, rootAfter);
      const salvaged = tryFindStepFromEntries(oc, newRootEntries);
      if (salvaged) {
        post("step-export", { requestId, data: salvaged }, [salvaged.buffer]);
        return;
      }

      if (typeof status === "number" && status > 1) {
        // Non-success IFSelect status values are >1.
        continue;
      }
    } catch {
      // Try the next candidate path.
    }
  }

  const stepNames = new Set();
  for (const name of listDirectorySafe(oc, "/")) {
    if (String(name).toLowerCase().endsWith(".step")) {
      stepNames.add(`/${name}`);
      stepNames.add(name);
    }
  }
  for (const name of listDirectorySafe(oc, ".")) {
    if (String(name).toLowerCase().endsWith(".step")) {
      stepNames.add(name);
      stepNames.add(`/${name}`);
    }
  }

  for (const filePath of stepNames) {
    const data = tryReadBinaryFile(oc, filePath);
    if (data) {
      post("step-export", { requestId, data }, [data.buffer]);
      return;
    }
  }

  const salvageFromRoot = tryFindStepFromEntries(oc, listDirectorySafe(oc, "/"));
  if (salvageFromRoot) {
    post("step-export", { requestId, data: salvageFromRoot }, [salvageFromRoot.buffer]);
    return;
  }

  const rootListing = listDirectorySafe(oc, "/").join(", ");
  throw new Error(`FS error: STEP file was not found after write. Root entries: ${rootListing}`);
}

function buildBinaryStlFromGeometry(geometry) {
  const positions = geometry.positions;
  const indices = geometry.indices;
  const triangleCount = Math.floor(indices.length / 3);
  const buffer = new ArrayBuffer(84 + triangleCount * 50);
  const bytes = new Uint8Array(buffer);
  const view = new DataView(buffer);

  const headerText = "Custom Float Maker STL";
  for (let i = 0; i < headerText.length && i < 80; i += 1) {
    bytes[i] = headerText.charCodeAt(i) & 0xff;
  }
  view.setUint32(80, triangleCount, true);

  let offset = 84;
  for (let tri = 0; tri < triangleCount; tri += 1) {
    const i0 = indices[tri * 3] * 3;
    const i1 = indices[tri * 3 + 1] * 3;
    const i2 = indices[tri * 3 + 2] * 3;

    const x0 = positions[i0];
    const y0 = positions[i0 + 1];
    const z0 = positions[i0 + 2];
    const x1 = positions[i1];
    const y1 = positions[i1 + 1];
    const z1 = positions[i1 + 2];
    const x2 = positions[i2];
    const y2 = positions[i2 + 1];
    const z2 = positions[i2 + 2];

    const ux = x1 - x0;
    const uy = y1 - y0;
    const uz = z1 - z0;
    const vx = x2 - x0;
    const vy = y2 - y0;
    const vz = z2 - z0;

    let nx = uy * vz - uz * vy;
    let ny = uz * vx - ux * vz;
    let nz = ux * vy - uy * vx;
    const nLen = Math.hypot(nx, ny, nz);
    if (nLen > 1.0e-12) {
      nx /= nLen;
      ny /= nLen;
      nz /= nLen;
    } else {
      nx = 0.0;
      ny = 0.0;
      nz = 0.0;
    }

    view.setFloat32(offset, nx, true);
    view.setFloat32(offset + 4, ny, true);
    view.setFloat32(offset + 8, nz, true);

    view.setFloat32(offset + 12, x0, true);
    view.setFloat32(offset + 16, y0, true);
    view.setFloat32(offset + 20, z0, true);
    view.setFloat32(offset + 24, x1, true);
    view.setFloat32(offset + 28, y1, true);
    view.setFloat32(offset + 32, z1, true);
    view.setFloat32(offset + 36, x2, true);
    view.setFloat32(offset + 40, y2, true);
    view.setFloat32(offset + 44, z2, true);

    view.setUint16(offset + 48, 0, true);
    offset += 50;
  }

  return bytes;
}

async function exportCurrentStl(requestId) {
  if (!state.ready || !state.oc || !state.currentShape || !state.currentMetrics) {
    throw new Error("Generate a model before exporting STL.");
  }

  const geometry = await tessellateShape(
    state.oc,
    state.currentShape,
    null,
    STL_EXPORT_LINEAR_DEFLECTION,
    STL_EXPORT_ANGULAR_DEFLECTION
  );
  const data = buildBinaryStlFromGeometry(geometry);
  post("stl-export", { requestId, data }, [data.buffer]);
}

async function handleGenerate(jobId, params) {
  throwIfBuildCancelled(jobId);
  const oc = await ensureWorkerReady();
  throwIfBuildCancelled(jobId);
  await emitProgress(jobId, "Generating model...");
  const result = await buildFloatShape(oc, params, jobId, async (shape, metrics) => {
    await emitStage(jobId, shape, metrics);
  });
  throwIfBuildCancelled(jobId);
  state.currentShape = result.shape;
  state.currentMetrics = result.metrics;
  const finalGeometry = await tessellateShape(oc, result.shape, jobId);
  throwIfBuildCancelled(jobId);
  post(
    "done",
    { jobId, metrics: result.metrics, geometry: finalGeometry },
    [finalGeometry.positions.buffer, finalGeometry.normals.buffer, finalGeometry.indices.buffer]
  );
}

state.queue = Promise.resolve();

self.addEventListener("message", (event) => {
  const message = event.data || {};
  if (message.type === "invalidate-build") {
    const nextVersion = Number(message.buildVersion);
    if (Number.isFinite(nextVersion)) {
      state.latestBuildVersion = nextVersion;
    }
    return;
  }
  if (message.type === "generate") {
    const nextVersion = Number(message.jobId);
    if (Number.isFinite(nextVersion)) {
      state.latestBuildVersion = nextVersion;
    }
  }
  state.queue = state.queue
    .then(async () => {
      if (message.type === "init") {
        await ensureWorkerReady();
        post("ready");
        return;
      }
      if (message.type === "generate") {
        await handleGenerate(message.jobId, message.params);
        return;
      }
      if (message.type === "export-step") {
        await exportCurrentStep(message.requestId);
        return;
      }
      if (message.type === "export-stl") {
        await exportCurrentStl(message.requestId);
        return;
      }
      throw new Error(`Unknown worker message: ${message.type}`);
    })
    .catch((err) => {
      if (err?.name === "BuildCancelledError") {
        return;
      }
      console.error(err);
      post("error", {
        jobId: message.jobId ?? null,
        requestId: message.requestId ?? null,
        message: err.message,
        stack: err.stack || ""
      });
    });
});
