// Kinematics check of the delta part: on a grid over the working volume and along every planned move of the cycle,
// the posed links must close the loop (both ends of every forearm rod on its ball studs, the upper arm on its
// motor axis) and the joints stay inside their range. Exits 1 on any failure.   Usage: node tools/check_kin.mjs
import { readFileSync } from 'node:fs'
import { dirname, join } from 'node:path'
import { fileURLToPath } from 'node:url'

const src = readFileSync(join(dirname(fileURLToPath(import.meta.url)), '..', 'parts/delta.transfer-robot/behavior.js'), 'utf8')
let spec = null
const k = new Function('definePart', `${src}\nreturn { ik, poses, plan, planAt, add, R, r, L1, L2, ZM, U, V, W }`)((s) => { spec = s })
// rotate v by the unit quaternion q = [x, y, z, w]
function rot(q, v) {
  const [x, y, z, w] = q, t = [2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])]
  return [v[0] + w * t[0] + y * t[2] - z * t[1], v[1] + w * t[1] + z * t[0] - x * t[2], v[2] + w * t[2] + x * t[1] - y * t[0]]
}
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]], len = (a) => Math.hypot(...a)
const LO = -20 * Math.PI / 180, HI = 80 * Math.PI / 180  // joint range checked (rad)
let n = 0, worst = 0, minTh = 9, maxTh = -9
const fail = []
function check(tcp, where) {
  const th = k.ik(tcp)
  if (!th) { fail.push(`${where}: outside the workspace ${tcp.map((v) => (v * 1000).toFixed(0))}`); return }
  const p = k.poses(tcp, th)
  for (let i = 0; i < 3; i++) {
    minTh = Math.min(minTh, th[i]); maxTh = Math.max(maxTh, th[i])
    if (th[i] < LO || th[i] > HI) fail.push(`${where}: arm ${i} at ${(th[i] * 180 / Math.PI).toFixed(1)} deg`)
    // the posed links, as drawn: the upper arm's ball studs (its frame: motor axis, X along the arm, Z the axis),
    // each rod's two ends (centre +- L2/2 along its Y) and the effector's ball studs must coincide
    const up = p[`upper${i}`], B = k.add([0, k.ZM, 0], k.U[i], k.R)
    worst = Math.max(worst, len(sub(up.position, B)))
    const tip = k.add(up.position, rot(up.quat, [1, 0, 0]), k.L1)
    const Pc = p.effector.position
    for (const [s, n] of [[-1, 'a'], [1, 'b']]) {
      const rd = p[`rod${i}${n}`], ax = rot(rd.quat, [0, 1, 0])
      const top = k.add(rd.position, ax, -k.L2 / 2), bot = k.add(rd.position, ax, k.L2 / 2)
      const ballUp = k.add(tip, rot(up.quat, [0, 0, 1]), s * k.W)
      const ballEff = k.add(k.add(Pc, k.U[i], k.r), k.V[i], s * k.W)
      worst = Math.max(worst, len(sub(top, ballUp)), len(sub(bot, ballEff)))
    }
  }
  n++
}
// working volume: the two belts (x -0.35..0.35), +-0.2 m along them, from the carton top to above the travel height
for (let x = -0.35; x <= 0.351; x += 0.05) for (let z = -0.2; z <= 0.201; z += 0.05) for (let y = 0.10; y <= 0.321; y += 0.02) check([x, y, z], 'grid')
// every move of the cycle, sampled at 1 ms
const pts = spec.params.points.default.map((q) => q.map((v) => v / 1000))
const travel = spec.params.travel.default / 1000
const moves = [[0, 1], [1, 2], [2, 3], [3, 1], [2, 1], [3, 2]]
for (const [a, b] of moves) {
  const m = k.plan(pts[a], pts[b], travel, 1)
  for (let t = 0; t <= m.T; t += 0.001) check(k.planAt(m, t), `move ${a}->${b} t=${t.toFixed(3)}`)
}
console.log(`${n} poses, loop closure worst ${(worst * 1e9).toFixed(3)} nm, joints ${(minTh * 180 / Math.PI).toFixed(1)}..${(maxTh * 180 / Math.PI).toFixed(1)} deg`)
if (fail.length) { console.log(fail.slice(0, 10).join('\n')); console.log(`✗ ${fail.length} failure(s)`); process.exit(1) }
if (worst > 1e-9) { console.log('✗ loop does not close'); process.exit(1) }
console.log('✓ kinematics')
