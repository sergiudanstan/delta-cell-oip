// Kinematics check of the delta part: on a grid over the working volume and along every planned move of the cycle,
// the inverse kinematics must close the loop (each forearm rod exactly L2 long, the upper arm exactly L1 from its
// motor axis) and stay inside the joint range. Exits 1 on any failure.   Usage: node tools/check_kin.mjs
import { readFileSync } from 'node:fs'
import { dirname, join } from 'node:path'
import { fileURLToPath } from 'node:url'

const src = readFileSync(join(dirname(fileURLToPath(import.meta.url)), '..', 'parts/delta.transfer-robot/behavior.js'), 'utf8')
let spec = null
const k = new Function('definePart', `${src}\nreturn { ik, poses, plan, planAt, add, R, L1, L2, ZM, U, W }`)((s) => { spec = s })
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
    const B = k.add([0, k.ZM, 0], k.U[i], k.R), E = p[`elbow${i}`].position
    worst = Math.max(worst, Math.abs(len(sub(E, B)) - k.L1))
    // rod ends: the rod is a cylinder of height L2 centred on its position along its local Y
    const P = [tcp[0], tcp[1] + 0.1, tcp[2]], J = k.add(P, k.U[i], 0.06)
    worst = Math.max(worst, Math.abs(len(sub(E, J)) - k.L2))
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
