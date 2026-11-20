// Delta transfer robot: a three-arm rotary delta (Clavel type, parallelogram forearms) hanging in its portal frame.
// Our own generic model, sized as an 8 kg-class delta: base joints on R = 0.200 m, upper arms 0.380 m, forearms
// 0.900 m, effector joints on r = 0.060 m, arms at 90/210/330 deg about the vertical. No vendor model or CAD.
//
// Frame: part-local metres, origin on the belt top under the robot, +Y up, +X across the belts, +Z up the belts.
// Every tick the behavior solves the inverse kinematics for the TCP (elbow-out branch) and poses every moving link
// with setSubPose: upper arms turn about their motor axes, each forearm is two parallel rods from the elbow bar to
// the effector bar, and the effector, tool and pad only translate (the parallelograms keep it level).
//
// PLC handshake (the same as the DuoPod part, so one program drives either robot): an INT command picks a row of the
// point table (1 ready, 2 pick, 3 place), a rising Execute starts the move, Done reads true while idle on a point.
// Moves between points at different places run as a goal-post: up to the travel height, across in a straight line,
// down, overlapped with minimum-jerk profiles. The vacuum is latched only on arrival (never mid-stroke), VacuumOk
// reports a carried load. Enable low holds the robot where it is (the motion clock stops). The TCP is reported in mm.

// ---------------------------------------------------------------- geometry
const R = 0.200, r = 0.060, L1 = 0.380, L2 = 0.900, ZM = 1.200, TOOL = 0.100, W = 0.030
const PHI = [90, 210, 330].map((d) => d * Math.PI / 180)
const U = PHI.map((f) => [Math.cos(f), 0, Math.sin(f)])     // radial direction of each arm
const V = PHI.map((f) => [-Math.sin(f), 0, Math.cos(f)])    // its motor axis (horizontal, tangent)

const add = (a, b, k = 1) => [a[0] + k * b[0], a[1] + k * b[1], a[2] + k * b[2]]
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
const unit = (a) => { const n = Math.hypot(a[0], a[1], a[2]); return [a[0] / n, a[1] / n, a[2] / n] }
// rotation whose local X, Y, Z axes are the given orthonormal vectors -> quaternion [x, y, z, w]
function quatOf(X, Y, Z) {
  const m00 = X[0], m11 = Y[1], m22 = Z[2], tr = m00 + m11 + m22
  if (tr > 0) { const s = Math.sqrt(tr + 1) * 2; return [(Y[2] - Z[1]) / s, (Z[0] - X[2]) / s, (X[1] - Y[0]) / s, s / 4] }
  if (m00 > m11 && m00 > m22) { const s = Math.sqrt(1 + m00 - m11 - m22) * 2; return [s / 4, (Y[0] + X[1]) / s, (Z[0] + X[2]) / s, (Y[2] - Z[1]) / s] }
  if (m11 > m22) { const s = Math.sqrt(1 + m11 - m00 - m22) * 2; return [(Y[0] + X[1]) / s, s / 4, (Z[1] + Y[2]) / s, (Z[0] - X[2]) / s] }
  const s = Math.sqrt(1 + m22 - m00 - m11) * 2; return [(Z[0] + X[2]) / s, (Z[1] + Y[2]) / s, s / 4, (X[1] - Y[0]) / s]
}
// a basis with local Y along d (cylinders stand on their local Y)
function alongY(d, hint) { const Y = unit(d), X = unit(cross(Y, hint)), Z = cross(X, Y); return quatOf(X, Y, Z) }

// joint angles (rad, 0 = upper arm horizontal, + = down) for the TCP [x, height, z]; null outside the workspace
function ik(tcp) {
  const P = [tcp[0], tcp[1] + TOOL, tcp[2]], th = []
  for (let i = 0; i < 3; i++) {
    const D = add(add(P, U[i], r), add([0, ZM, 0], U[i], R), -1)
    const a = dot(D, U[i]), b = D[1], c = dot(D, V[i])
    const k = (L1 * L1 + a * a + b * b + c * c - L2 * L2) / (2 * L1), rho = Math.hypot(a, b)
    if (Math.abs(k) > rho) return null
    th.push(-Math.acos(k / rho) - Math.atan2(b, a))         // elbow-out branch
  }
  return th
}

// every moving link's pose for a TCP and its joint angles
function poses(tcp, th) {
  const P = [tcp[0], tcp[1] + TOOL, tcp[2]], out = {}
  for (let i = 0; i < 3; i++) {
    const B = add([0, ZM, 0], U[i], R)
    const c = Math.cos(th[i]), X = [c * U[i][0], -Math.sin(th[i]), c * U[i][2]]   // along the upper arm
    const E = add(B, X, L1), J = add(P, U[i], r)
    out[`upper${i}`] = { position: add(B, X, L1 / 2), quat: quatOf(X, cross(V[i], X), V[i]) }
    out[`elbow${i}`] = { position: E, quat: quatOf(U[i], [0, 1, 0], V[i]) }
    out[`ebar${i}`] = { position: J, quat: quatOf(U[i], [0, 1, 0], V[i]) }
    for (const [s, n] of [[-1, 'a'], [1, 'b']]) {
      const e = add(E, V[i], s * W), j = add(J, V[i], s * W)
      out[`rod${i}${n}`] = { position: add(e, add(j, e, -1), 0.5), quat: alongY(add(e, j, -1), V[i]) }
    }
  }
  const at = (dy) => ({ position: [tcp[0], tcp[1] + dy, tcp[2]] })
  out.effector = at(TOOL)
  out.shaft = at(0.055)
  out.pad = at(0.0125)
  out.vacLed = { position: [tcp[0] + 0.045, tcp[1] + 0.05, tcp[2] + 0.045] }
  out.gripper = at(0)
  return out
}

// ---------------------------------------------------------------- motion
const ease = (s) => s * s * s * (10 + s * (6 * s - 15))
// pieces [t0, t1, from, to] per axis; between pieces an axis holds its value
function axisAt(v0, pieces, t) {
  let v = v0
  for (const [a, b, f, to] of pieces) {
    if (t <= a) return v
    v = t >= b ? to : f + (to - f) * ease((t - a) / (b - a))
    if (t < b) return v
  }
  return v
}
// goal-post plan from p0 to p1 ([x, height, z]) through `travel`: h runs 0 -> 1 along the straight horizontal line
function plan(p0, p1, travel, k) {
  const T = (v) => v * k
  const dist = Math.hypot(p1[0] - p0[0], p1[2] - p0[2])
  if (dist < 1e-3) {
    const dz = Math.abs(p1[1] - p0[1]), dur = T(Math.max(0.12, 0.24 * dz / 0.158))
    return { T: dur, hs: [], ys: [[0, dur, p0[1], p1[1]]], p0, p1 }
  }
  const up = p0[1] < travel - 1e-3, down = p1[1] < travel - 1e-3
  const h0 = up ? T(0.16) : 0, hDur = T(0.30 * Math.max(0.6, dist / 0.4))
  const ys = []
  if (up) ys.push([0, T(0.30), p0[1], travel])
  let end = h0 + hDur
  if (down) { const d0 = h0 + hDur - T(0.14); ys.push([d0, d0 + T(0.30), up ? travel : p0[1], p1[1]]); end = d0 + T(0.30) }
  return { T: end, hs: [[h0, h0 + hDur, 0, 1]], ys, p0, p1 }
}
function planAt(m, t) {
  const h = axisAt(0, m.hs, t)
  return [m.p0[0] + (m.p1[0] - m.p0[0]) * h, axisAt(m.p0[1], m.ys, t), m.p0[2] + (m.p1[2] - m.p0[2]) * h]
}

// ---------------------------------------------------------------- state (kept between ticks, one instance each)
let tcp = null, move = null, clock = 0, target = 0, vac = false

definePart({
  params: {
    commandTag: { type: 'tag', dir: 'in', stType: 'INT', default: 'RobotCmd', label: 'Command tag (point row)', group: 'Comms',
      values: [{ value: 1, label: 'ready' }, { value: 2, label: 'pick' }, { value: 3, label: 'place' }],
      desc: 'Row of the point table to move to: 1 ready, 2 pick, 3 place. Set it before the Execute edge.' },
    executeTag: { type: 'tag', dir: 'in', stType: 'BOOL', default: 'RobotExecute', label: 'Execute tag (rising edge)', group: 'Comms',
      desc: 'A rising edge starts the move to the commanded point.' },
    doneTag: { type: 'tag', dir: 'out', stType: 'BOOL', default: 'RobotDone', label: 'Done tag', group: 'Comms',
      desc: 'True while the robot stands on a point and no move is running.' },
    busyTag: { type: 'tag', dir: 'out', stType: 'BOOL', default: 'RobotBusy', label: 'Busy tag', group: 'Comms',
      desc: 'True while a move runs (also while it is held by Enable).' },
    pointTag: { type: 'tag', dir: 'out', stType: 'INT', default: 'RobotPoint', label: 'At point tag', group: 'Comms',
      desc: 'The point row the robot stands on (0 while moving).' },
    vacuumTag: { type: 'tag', dir: 'in', stType: 'BOOL', default: 'Vacuum', label: 'Vacuum tag', group: 'Comms',
      desc: 'Vacuum on/off. Latched only while the robot stands still on a point, as a real cup seals or releases.' },
    vacOkTag: { type: 'tag', dir: 'out', stType: 'BOOL', default: 'VacuumOk', label: 'Vacuum OK tag', group: 'Comms',
      desc: 'True while the gripper actually carries a load (the vacuum switch of the ejector).' },
    enableTag: { type: 'tag', dir: 'in', stType: 'BOOL', default: 'RobotEnable', label: 'Enable tag', group: 'Comms',
      desc: 'Safety enable: low stops the motion where it is until it returns. Always on until a program drives it.' },
    xTag: { type: 'tag', dir: 'out', stType: 'REAL', default: 'RobotX', label: 'TCP x tag (mm)', group: 'Comms',
      desc: 'Actual TCP across the belts, mm (0 = robot centre).' },
    zTag: { type: 'tag', dir: 'out', stType: 'REAL', default: 'RobotZ', label: 'TCP height tag (mm)', group: 'Comms',
      desc: 'Actual TCP height above the belt top, mm (the face of the vacuum pad).' },
    points: { type: 'numberList', arity: 3, columns: ['x mm', 'height mm', 'along mm'], label: 'Points (row 1 ready, 2 pick, 3 place)', group: 'Motion',
      default: [[0, 306, 0], [200, 280, 0], [200, 122, 0], [-200, 122, 0]],
      desc: 'TCP points, mm: x across the belts, height above the belt top, along the belts (+ = upstream). Row 0 is the rest pose.' },
    travel: { type: 'number', default: 280, min: 200, max: 320, step: 1, label: 'Travel height (mm)', group: 'Motion',
      desc: 'TCP height for moves across: the carton bottom then clears the guides by 100 mm.' },
    timeScale: { type: 'number', default: 1, min: 0.5, max: 3, step: 0.05, label: 'Motion time scale', group: 'Motion',
      desc: 'Stretches every move; 1 = peak about 2.5 m/s and 2.8 g on the transfer stroke.' },
  },
  grippers: ['gripper'],
  lamps: ['vacuum'],
  step(ctx) {
    const P = ctx.params
    const pts = P.points.length ? P.points : [[0, 306, 0]]
    const pt = (i) => { const q = pts[Math.max(0, Math.min(pts.length - 1, i))]; return [q[0] / 1000, q[1] / 1000, (q[2] ?? 0) / 1000] }
    if (!tcp) { tcp = pt(0); target = 0 }
    const enabled = ctx.tagDriven(P.enableTag) ? ctx.readTag(P.enableTag) : true

    // start a move on the Execute edge (a target outside the workspace is refused)
    if (ctx.rising(P.executeTag)) {
      const row = Math.round(ctx.readNumber(P.commandTag))
      if (row >= 0 && row < pts.length && ik(pt(row))) {
        move = plan(tcp, pt(row), P.travel / 1000, P.timeScale); clock = 0; target = row
      }
    }
    if (move && enabled) {
      clock += ctx.dt
      const next = clock >= move.T ? pt(target) : planAt(move, clock)
      if (ik(next)) tcp = next
      if (clock >= move.T) move = null
    }
    const idle = !move

    // vacuum: latched on arrival only
    if (idle) vac = ctx.readTag(P.vacuumTag)
    ctx.grips.gripper = vac
    const holding = ctx.measured.grips.gripper
    ctx.lamps.vacuum = vac

    const th = ik(tcp)
    if (th) for (const [n, p] of Object.entries(poses(tcp, th))) ctx.setSubPose(n, p)

    // feedback
    ctx.writeTag(P.doneTag, idle)
    ctx.writeTag(P.busyTag, !idle)
    ctx.writeNumber(P.pointTag, idle ? target : 0)
    ctx.writeTag(P.vacOkTag, holding)
    ctx.writeNumber(P.xTag, Math.round(tcp[0] * 10000) / 10)
    ctx.writeNumber(P.zTag, Math.round(tcp[1] * 10000) / 10)
    ctx.display('state', !enabled ? 'HOLD' : idle ? `AT ${target}` : 'MOVING')
  },
})
