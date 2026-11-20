// Build parts/delta.transfer-robot/manifest.json: the rig of our own generic delta, all primitives (no mesh, no CAD).
// The rest pose of every moving link comes from the behavior's own kinematics (poses(ik(rest point))), so the static
// editor view and the first tick agree.   Usage: node tools/build_part.mjs
import { readFileSync, writeFileSync } from 'node:fs'
import { dirname, join } from 'node:path'
import { fileURLToPath } from 'node:url'

const dir = join(dirname(fileURLToPath(import.meta.url)), '..', 'parts', 'delta.transfer-robot')
const src = readFileSync(join(dir, 'behavior.js'), 'utf8')
let spec = null
const k = new Function('definePart', `${src}\nreturn { poses, ik, alongY, add, R, ZM, U, V }`)((s) => { spec = s })
const rest = spec.params.points.default[0].map((v) => v / 1000)
const P = k.poses(rest, k.ik(rest))

const hex = (h) => [1, 3, 5].map((i) => +(parseInt(h.slice(i, i + 2), 16) / 255).toFixed(3))
const ALU = { color: hex('#b8bcc2'), metalness: 0.75, roughness: 0.38 }
const WHITE = { color: hex('#ecebe6'), metalness: 0.1, roughness: 0.45 }
const DARK = { color: hex('#2b2d30'), metalness: 0.3, roughness: 0.5 }
const box = (name, size, offset, look, extra = {}) => ({ name, shape: { kind: 'box', size }, offset, ...look, collision: 'none', ...extra })
const cyl = (name, radius, height, offset, look, extra = {}) => ({ name, shape: { kind: 'cylinder', radius, height }, offset, ...look, collision: 'none', ...extra })
const posed = (link) => { const p = P[link.name]; return { ...link, offset: p.position, ...(p.quat ? { quat: p.quat } : {}) } }

// portal: four posts, a ring of beams and two carriers the base hangs from (origin on the belt top, floor at -0.85)
const PX = 0.80, PZ = 0.72, BM = 0.08, FLOOR = -0.85, TOPY = 1.40
const links = []
for (const sx of [-1, 1]) for (const sz of [-1, 1]) links.push(box(`post_${sx > 0 ? 'p' : 'm'}${sz > 0 ? 'p' : 'm'}`, [BM, TOPY + BM / 2 - FLOOR, BM], [sx * PX, (TOPY + BM / 2 + FLOOR) / 2, sz * PZ], ALU))
for (const sz of [-1, 1]) links.push(box(`beamX_${sz > 0 ? 'p' : 'm'}`, [2 * PX + BM, BM, BM], [0, TOPY, sz * PZ], ALU))
for (const sx of [-1, 1]) links.push(box(`beamZ_${sx > 0 ? 'p' : 'm'}`, [BM, BM, 2 * PZ - BM], [sx * PX, TOPY, 0], ALU))
for (const sx of [-1, 1]) links.push(box(`carrier_${sx > 0 ? 'p' : 'm'}`, [BM, BM, 2 * PZ - BM], [sx * 0.30, TOPY, 0], ALU))
// base housing and the three drives (gearbox along each motor axis)
links.push(cyl('base', 0.30, 0.16, [0, k.ZM + 0.08, 0], WHITE))
for (let i = 0; i < 3; i++) {
  const B = k.add([0, k.ZM, 0], k.U[i], k.R)
  links.push({ ...cyl(`drive${i}`, 0.06, 0.20, B, ALU), quat: k.alongY(k.V[i], [0, 1, 0]) })
}
// moving links, posed by the behavior every tick
for (let i = 0; i < 3; i++) {
  links.push(posed(box(`upper${i}`, [0.42, 0.055, 0.05], [0, 0, 0], WHITE)))
  links.push(posed(box(`elbow${i}`, [0.035, 0.035, 0.095], [0, 0, 0], ALU)))
  links.push(posed(box(`ebar${i}`, [0.03, 0.03, 0.09], [0, 0, 0], ALU)))
  for (const n of ['a', 'b']) links.push(posed(cyl(`rod${i}${n}`, 0.009, 0.90, [0, 0, 0], DARK)))
}
links.push(posed(cyl('effector', 0.085, 0.035, [0, 0, 0], ALU)))
links.push(posed(cyl('shaft', 0.022, 0.09, [0, 0, 0], ALU)))
links.push(posed(box('pad', [0.16, 0.025, 0.22], [0, 0, 0], { color: hex('#1f4e8c'), metalness: 0.05, roughness: 0.8 })))
links.push(posed(box('vacLed', [0.012, 0.012, 0.012], [0, 0, 0], {}, { lamps: [{ name: 'vacuum', emissive: [0.15, 1.0, 0.35], offColor: [0.02, 0.06, 0.03] }] })))
// TCP = face of the vacuum pad; the pickup volume is the pad and reaches the carton centre 60 mm below
links.push(posed({ name: 'gripper', shape: { kind: 'frame' }, offset: [0, 0, 0], grip: { zone: { size: [0.16, 0.16, 0.22], offset: [0, -0.06, 0] } } }))

const manifest = {
  id: 'delta.transfer-robot',
  name: 'Delta transfer robot (3-arm, 8 kg class)',
  category: 'Equipment',
  geometry: { kind: 'rig', links, joints: [], bounds: { length: 2 * PX + BM, width: 2 * PZ + BM, top: TOPY + BM / 2, bottom: FLOOR } },
}
writeFileSync(join(dir, 'manifest.json'), JSON.stringify(manifest, null, 1) + '\n')
console.log(join(dir, 'manifest.json'), `${links.length} links`)
