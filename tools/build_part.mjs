// Build parts/delta.transfer-robot/manifest.json: the portal frame as boxes, the robot as mesh links of our own
// procedural model (tools/model.mjs, welded + Draco) embedded as base64, so the part is one self-contained folder.
// The rest pose of every moving link comes from the behavior's own kinematics (poses(ik(rest point))), so the static
// editor view and the first tick agree.   Usage: tools/make_part.sh (model -> Draco -> this -> check-part)
import { readFileSync, writeFileSync } from 'node:fs'
import { dirname, join } from 'node:path'
import { fileURLToPath } from 'node:url'

const root = join(dirname(fileURLToPath(import.meta.url)), '..')
const dir = join(root, 'parts', 'delta.transfer-robot')
const src = readFileSync(join(dir, 'behavior.js'), 'utf8')
let spec = null
const k = new Function('definePart', `${src}\nreturn { poses, ik }`)((s) => { spec = s })
const rest = spec.params.points.default[0].map((v) => v / 1000)
const P = k.poses(rest, k.ik(rest))
const glb = readFileSync(join(root, '_build', 'delta_draco.glb')).toString('base64')

const hex = (h) => [1, 3, 5].map((i) => +(parseInt(h.slice(i, i + 2), 16) / 255).toFixed(3))
const ALU = { color: hex('#b8bcc2'), metalness: 0.75, roughness: 0.38 }
const box = (name, size, offset, look, extra = {}) => ({ name, shape: { kind: 'box', size }, offset, ...look, collision: 'none', ...extra })
const mesh = (name, node) => ({ name, shape: { kind: 'mesh', asset: 'model', node }, offset: [0, 0, 0] })
const posed = (link) => { const p = P[link.name]; return { ...link, offset: p.position, ...(p.quat ? { quat: p.quat } : {}) } }

// portal: four posts, a ring of beams and two carriers the base hangs from (origin on the belt top, floor at -0.85)
const PX = 0.80, PZ = 0.72, BM = 0.08, FLOOR = -0.85, TOPY = 1.40
const links = []
links.push(mesh('frame', 'frame'))                     // the portal: 80 x 80 slot profiles, brackets, levelling feet
// the robot: base housing and drives (static), moving links posed by the behavior every tick
links.push(mesh('base', 'base'))
for (let i = 0; i < 3; i++) {
  links.push(posed(mesh(`upper${i}`, 'upper')))
  for (const n of ['a', 'b']) links.push(posed(mesh(`rod${i}${n}`, 'rod')))
  for (const n of ['t', 'b']) links.push(posed(mesh(`spring${i}${n}`, 'spring')))
}
links.push(posed(mesh('effector', 'effector')))
links.push(posed(box('vacLed', [0.008, 0.008, 0.008], [0, 0, 0], {}, { lamps: [{ name: 'vacuum', emissive: [0.15, 1.0, 0.35], offColor: [0.02, 0.06, 0.03] }] })))
// TCP = face of the vacuum pad; the pickup volume is the pad and reaches the carton centre 60 mm below
links.push(posed({ name: 'gripper', shape: { kind: 'frame' }, offset: [0, 0, 0], grip: { zone: { size: [0.16, 0.16, 0.22], offset: [0, -0.06, 0] } } }))

const manifest = {
  id: 'delta.transfer-robot',
  name: 'Delta transfer robot (3-arm, 8 kg class)',
  category: 'Equipment',
  assets: [{ id: 'model', mime: 'model/gltf-binary', data: glb }],
  geometry: { kind: 'rig', links, joints: [], bounds: { length: 2 * PX + 0.16, width: 2 * PZ + 0.16, top: TOPY + BM / 2, bottom: FLOOR } },
}
writeFileSync(join(dir, 'manifest.json'), JSON.stringify(manifest, null, 1) + '\n')
console.log(join(dir, 'manifest.json'), `${links.length} links`, (JSON.stringify(manifest).length / 1e6).toFixed(2), 'MB')
