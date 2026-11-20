// Our own procedural model of the delta (no vendor model or CAD): one GLB node per link frame, used by
// tools/build_part.mjs. Dimensions come from the behavior (R, L1, L2, ZM, TOOL, W, r), so the model and the kinematics
// cannot drift apart. Generic 8 kg-class delta: a cast base housing with three drive units (gearbox + servo motor)
// hanging under it, tapered cast upper arms with a lightening slot and ball studs on a cross shaft, carbon forearm
// pairs with socket cups and two tension springs each, a triangular effector with ball studs, a vacuum tool with four
// bellows cups.
//   node tools/model.mjs   -> _build/delta.glb
import { mkdirSync, readFileSync, writeFileSync } from 'node:fs'
import { homedir } from 'node:os'
import { dirname, join } from 'node:path'
import { fileURLToPath, pathToFileURL } from 'node:url'

const root = join(dirname(fileURLToPath(import.meta.url)), '..')
const oip = process.env.OIP ?? join(homedir(), 'code/_vendor/open-industry-project')
const nm = (p) => pathToFileURL(join(oip, 'node_modules', p)).href
const THREE = await import(nm('three/build/three.module.js'))
const { GLTFExporter } = await import(nm('three/examples/jsm/exporters/GLTFExporter.js'))
const { mergeGeometries } = await import(nm('three/examples/jsm/utils/BufferGeometryUtils.js'))
// GLTFExporter reads its binary blob through FileReader, which node lacks
globalThis.FileReader ??= class { readAsArrayBuffer(b) { b.arrayBuffer().then((r) => { this.result = r; this.onloadend?.() }) } readAsDataURL(b) { b.arrayBuffer().then((r) => { this.result = `data:${b.type};base64,${Buffer.from(r).toString('base64')}`; this.onloadend?.() }) } }

const src = readFileSync(join(root, 'parts/delta.transfer-robot/behavior.js'), 'utf8')
const K = new Function('definePart', `${src}\nreturn { R, r, L1, L2, ZM, TOOL, W, SPRING, PHI, U, V }`)(() => {})
const { R, r, L1, L2, ZM, W, PHI } = K

// ---------------------------------------------------------------- materials
const mat = (name, color, metalness, roughness) => Object.assign(new THREE.MeshStandardMaterial({ color, metalness, roughness }), { name })
const M = {
  cast: mat('cast_white', 0xe9e8e3, 0.05, 0.42),      // painted cast aluminium (housing, upper arms)
  alu: mat('anodised', 0xb4b9bf, 0.8, 0.32),          // machined / anodised aluminium
  steel: mat('steel', 0xd2d5d9, 1.0, 0.22),           // ball studs, shafts, springs
  motor: mat('motor_black', 0x1c1e21, 0.35, 0.45),    // servo motors
  carbon: mat('carbon', 0x18191b, 0.25, 0.3),         // forearm tubes
  cup: mat('socket_white', 0xf2f1ec, 0.0, 0.55),      // polymer socket cups
  rubber: mat('rubber', 0x101112, 0.0, 0.85),         // bellows cups
  plate: mat('tool_plate', 0x3b4048, 0.6, 0.4),       // tool plate
  cable: mat('cable', 0x2a2c30, 0.0, 0.7),
  profile: mat('profile', 0xc9cdd2, 0.72, 0.34),    // natural anodised extrusion
  endcap: mat('end_cap', 0x2b2d31, 0.0, 0.6),
  bracket: mat('bracket', 0xbfc3c8, 0.6, 0.45),     // cast angle brackets
}

// ---------------------------------------------------------------- geometry helpers (metres, Y up)
const parts = new Map() // node name -> material -> geometries
const put = (node, m, g) => { if (!parts.has(node)) parts.set(node, new Map()); const n = parts.get(node); if (!n.has(m)) n.set(m, []); n.get(m).push(g.index ? g.toNonIndexed() : g) }
const place = (g, pos = [0, 0, 0], quat = null) => { if (quat) g.applyQuaternion(quat); g.translate(...pos); return g }
const qAxis = (from, to) => new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(...from).normalize(), new THREE.Vector3(...to).normalize())
// cylinder between two points
function rod(a, b, rad, seg = 24, rad2 = rad) {
  const d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], L = Math.hypot(...d)
  return place(new THREE.CylinderGeometry(rad2, rad, L, seg), [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2], qAxis([0, 1, 0], d))
}
const sphere = (c, rad) => place(new THREE.SphereGeometry(rad, 24, 16), c)
const box = (c, s) => place(new THREE.BoxGeometry(...s), c)
function rbox(c, s, rad = 0.004) { // box with rounded edges (extruded rounded rectangle)
  const g = new THREE.ExtrudeGeometry(roundRect(s[0] - 2 * rad, s[1] - 2 * rad, rad * 1.5), { depth: s[2] - 2 * rad, bevelEnabled: true, bevelSize: rad, bevelThickness: rad, bevelSegments: 3, curveSegments: 6 })
  g.translate(0, 0, -(s[2] - 2 * rad) / 2); return place(g, c)
}
function roundRect(w, h, rad) {
  const s = new THREE.Shape(), x = -w / 2, y = -h / 2
  s.moveTo(x + rad, y); s.lineTo(x + w - rad, y); s.quadraticCurveTo(x + w, y, x + w, y + rad); s.lineTo(x + w, y + h - rad)
  s.quadraticCurveTo(x + w, y + h, x + w - rad, y + h); s.lineTo(x + rad, y + h); s.quadraticCurveTo(x, y + h, x, y + h - rad)
  s.lineTo(x, y + rad); s.quadraticCurveTo(x, y, x + rad, y); return s
}
// rounded triangle in the horizontal plane, corners toward the angles `at` (rad, from +X toward +Z), extruded upward
function roundTri(circR, cornerR, at, y0, h, bevel = 0.006) {
  const vs = at.map((a) => [circR * Math.cos(a), -circR * Math.sin(a)]) // shape (x, y) -> world (x, -y) after rotateX
  const s = new THREE.Shape()
  for (let k = 0; k < 3; k++) {
    const p = vs[k], prev = vs[(k + 2) % 3], next = vs[(k + 1) % 3]
    const toward = (q, t) => { const dx = q[0] - p[0], dy = q[1] - p[1], n = Math.hypot(dx, dy); return [p[0] + dx / n * t, p[1] + dy / n * t] }
    const a = toward(prev, cornerR), b = toward(next, cornerR)
    k ? s.lineTo(...a) : s.moveTo(...a)
    s.quadraticCurveTo(p[0], p[1], ...b)
  }
  const g = new THREE.ExtrudeGeometry(s, { depth: h - 2 * bevel, bevelEnabled: true, bevelSize: bevel, bevelThickness: bevel, bevelSegments: 3, curveSegments: 16 })
  g.rotateX(-Math.PI / 2); g.translate(0, y0 + bevel, 0); return g
}
// helical tension spring along local Z, centred, with a hook loop at each end
function spring(len, coilR, wire, turns) {
  class Helix extends THREE.Curve { getPoint(t) { const a = t * turns * 2 * Math.PI; return new THREE.Vector3(coilR * Math.cos(a), coilR * Math.sin(a), (t - 0.5) * len) } }
  return new THREE.TubeGeometry(new Helix(), turns * 16, wire, 6, false)
}
const deg = Math.PI / 180

// ---------------------------------------------------------------- base (static, part frame)
{
  const lobes = PHI.map((f) => f - 35 * deg) // the housing lobes cover the drive units
  put('base', M.alu, roundTri(0.375, 0.13, lobes, ZM + 0.138, 0.022, 0.004))          // mounting plate (bolted to the carriers)
  put('base', M.cast, roundTri(0.355, 0.13, lobes, ZM + 0.062, 0.076, 0.012))         // cast housing
  put('base', M.cast, place(new THREE.SphereGeometry(0.13, 40, 12, 0, 2 * Math.PI, Math.PI / 2, Math.PI / 2), [0, ZM + 0.062, 0])) // bottom dome
  for (let i = 0; i < 3; i++) {
    const u = K.U[i], v = K.V[i], B = [R * u[0], ZM, R * u[2]]
    const at = (t, dy = 0) => [B[0] + t * v[0], B[1] + dy, B[2] + t * v[2]]
    put('base', M.alu, rod(at(-0.030), at(-0.045), 0.070, 40))                         // gearbox output flange
    put('base', M.alu, rod(at(-0.045), at(-0.150), 0.062, 40))                         // gearbox
    put('base', M.alu, rod(at(-0.150), at(-0.162), 0.056, 40))                         // motor flange
    put('base', M.motor, rod(at(-0.162), at(-0.300), 0.050, 40))                       // servo motor
    put('base', M.motor, rod(at(-0.300), at(-0.312), 0.044, 40))                       // rear cap
    put('base', M.alu, rbox(at(-0.10, 0.060), [0.10, 0.012, 0.12], 0.003)) // saddle under the housing
    put('base', M.motor, rbox(at(-0.235, 0.058), [0.05, 0.03, 0.045], 0.004))          // connector box
    put('base', M.cable, rod(at(-0.235, 0.07), [B[0] * 0.55 - 0.2 * v[0], ZM + 0.09, B[2] * 0.55 - 0.2 * v[2]], 0.009, 10))
  }
}

// ---------------------------------------------------------------- upper arm (frame on the motor axis: X along the arm, Z the motor axis)
{
  const a = 0.045, b = L1 - 0.03, h0 = 0.036, h1 = 0.021
  const s = new THREE.Shape()
  s.moveTo(a, -h0); s.lineTo(b, -h1); s.lineTo(b, h1); s.lineTo(a, h0); s.lineTo(a, -h0)
  const hole = new THREE.Path(), x0 = 0.11, x1 = L1 - 0.085, k0 = 0.017, k1 = 0.008
  hole.moveTo(x0, -k0); hole.lineTo(x1, -k1); hole.quadraticCurveTo(x1 + k1, 0, x1, k1); hole.lineTo(x0, k0); hole.quadraticCurveTo(x0 - k0, 0, x0, -k0)
  s.holes.push(hole)
  const T = 0.034
  const g = new THREE.ExtrudeGeometry(s, { depth: T - 0.008, bevelEnabled: true, bevelSize: 0.004, bevelThickness: 0.004, bevelSegments: 3, curveSegments: 12 })
  g.translate(0, 0, -(T - 0.008) / 2)
  put('upper', M.cast, g)
  put('upper', M.cast, rod([0, 0, -0.032], [0, 0, 0.032], 0.052, 40))                 // hub on the gearbox flange
  put('upper', M.alu, rod([0, 0, 0.032], [0, 0, 0.040], 0.040, 40))                   // clamp cap
  for (let k = 0; k < 6; k++) { const t = k * Math.PI / 3; put('upper', M.steel, rod([0.032 * Math.cos(t), 0.032 * Math.sin(t), 0.040], [0.032 * Math.cos(t), 0.032 * Math.sin(t), 0.046], 0.005, 8)) }
  put('upper', M.cast, rod([L1, 0, -0.022], [L1, 0, 0.022], 0.026, 32))               // tip boss
  put('upper', M.steel, rod([L1, 0, -W - 0.004], [L1, 0, W + 0.004], 0.008, 16))     // cross shaft
  for (const z of [-W, W]) put('upper', M.steel, sphere([L1, 0, z], 0.0135))         // ball studs
}

// ---------------------------------------------------------------- forearm rod (frame on its centre, Y along it, ball centres at +-L2/2)
{
  const e = L2 / 2
  put('rod', M.carbon, rod([0, -(e - 0.05), 0], [0, e - 0.05, 0], 0.0105, 20))
  for (const s of [-1, 1]) {
    put('rod', M.cup, rod([0, s * (e - 0.058), 0], [0, s * (e - 0.016), 0], 0.0125, 20, 0.0155)) // cup neck, widening to the socket
    put('rod', M.cup, sphere([0, s * e, 0], 0.0185))                                 // socket around the ball
    put('rod', M.steel, place(new THREE.TorusGeometry(0.0115, 0.0016, 6, 20), [0, s * (e - K.SPRING), 0], qAxis([0, 0, 1], [0, 1, 0]))) // spring collar
  }
}

// ---------------------------------------------------------------- spring between the rods of a pair (frame centred, Z across)
put('spring', M.steel, spring(2 * W - 0.022, 0.0045, 0.0011, 9))
for (const s of [-1, 1]) put('spring', M.steel, place(new THREE.TorusGeometry(0.0045, 0.0011, 6, 12), [0, 0, s * (W - 0.011)], qAxis([0, 0, 1], [1, 0, 0])))

// ---------------------------------------------------------------- effector + vacuum tool (frame on the plate centre; TCP TOOL below)
{
  const T = K.TOOL
  put('effector', M.alu, roundTri(0.112, 0.032, PHI, -0.011, 0.022, 0.003))
  for (let i = 0; i < 3; i++) {
    const u = K.U[i], v = K.V[i], c = [r * u[0], 0, r * u[2]]
    const at = (t) => [c[0] + t * v[0], 0, c[2] + t * v[2]]
    put('effector', M.steel, rod(at(-W - 0.004), at(W + 0.004), 0.008, 16))
    for (const t of [-W, W]) put('effector', M.steel, sphere(at(t), 0.0135))
  }
  put('effector', M.alu, rod([0, -0.011, 0], [0, -0.023, 0], 0.046, 40))               // tool flange
  put('effector', M.alu, rod([0, -0.023, 0], [0, -0.045, 0], 0.020, 24))               // adapter
  put('effector', M.alu, rbox([0, -0.0575, 0], [0.07, 0.025, 0.07], 0.004))            // vacuum manifold
  put('effector', M.cable, rod([0.035, -0.052, 0.0], [0.07, -0.02, 0.05], 0.006, 10))  // vacuum hose stub
  put('effector', M.plate, rbox([0, -0.075, 0], [0.15, 0.010, 0.21], 0.003))           // tool plate
  for (const x of [-0.05, 0.05]) for (const z of [-0.075, 0.075]) {
    put('effector', M.alu, rod([x, -0.080, z], [x, -0.085, z], 0.009, 16))
    put('effector', M.rubber, rod([x, -0.085, z], [x, -0.089, z], 0.012, 20, 0.016))  // bellows: two folds and the lip
    put('effector', M.rubber, rod([x, -0.089, z], [x, -0.092, z], 0.016, 20, 0.012))
    put('effector', M.rubber, rod([x, -0.092, z], [x, -0.095, z], 0.012, 20, 0.017))
    put('effector', M.rubber, rod([x, -0.095, z], [x, -T, z], 0.017, 20, 0.021))
  }
}


// ---------------------------------------------------------------- portal frame (static, part frame): 80 x 80 slot profiles
{
  const PX = 0.80, PZ = 0.72, BM = 0.08, FLOOR = -0.85, TOPY = 1.40, h = BM / 2
  const FOOT = 0.075                                   // levelling foot + base plate under each post
  // 80 x 80 profile section: rounded square, a T-slot (10 mm opening) in the middle of each face, centre bore 14.5 mm
  const section = (() => {
    const s = new THREE.Shape(), o = 0.005, n = 0.0045, u = 0.011, d1 = 0.006, d2 = 0.017, c = 0.0015
    // walk one face from its left corner to the next, in a local frame where the face is y = h, x from -h to +h
    const face = [[-h + c, h], [-o, h], [-o, h - d1], [-u, h - d1], [-u + 0.004, h - d2], [u - 0.004, h - d2], [u, h - d1], [o, h - d1], [o, h], [h - c, h]]
    const pts = []
    for (let k = 0; k < 4; k++) { const a = -k * Math.PI / 2, ca = Math.cos(a), sa = Math.sin(a); for (const [x, y] of face) pts.push([x * ca - y * sa, x * sa + y * ca]) }
    s.moveTo(...pts[0]); for (const p of pts.slice(1)) s.lineTo(...p); s.closePath()
    const bore = new THREE.Path(); bore.absarc(0, 0, 0.00725, 0, 2 * Math.PI, true); s.holes.push(bore)
    for (const [x, y] of [[1, 1], [-1, 1], [-1, -1], [1, -1]]) { const p = new THREE.Path(); p.absarc(x * 0.025, y * 0.025, 0.0035, 0, 2 * Math.PI, true); s.holes.push(p) }
    return s
  })()
  // a profile from a to b (axis-aligned), with a dark end cap on each free end
  const profile = (a, b, caps = [true, true]) => {
    const d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], L = Math.hypot(...d)
    const g = new THREE.ExtrudeGeometry(section, { depth: L, bevelEnabled: false, curveSegments: 6 })
    g.translate(0, 0, -L / 2)
    put('frame', M.profile, place(g, [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2], qAxis([0, 0, 1], d)))
    const dir = d.map((x) => x / L)
    caps.forEach((on, k) => { if (!on) return; const e = k ? b : a, sgn = k ? 1 : -1
      put('frame', M.endcap, place(new THREE.BoxGeometry(BM - 0.002, BM - 0.002, 0.004), e.map((x, j) => x + sgn * dir[j] * 0.002), qAxis([0, 0, 1], d))) })
  }
  // cast corner bracket in the inside corner of two members: two legs of 70 mm and a diagonal rib, `w` wide
  const bracket = (corner, e1, e2, w = 0.04) => {
    const s = new THREE.Shape(), L = 0.07, t = 0.008
    s.moveTo(0, 0); s.lineTo(L, 0); s.lineTo(L, t); s.lineTo(t + 0.012, t); s.lineTo(t, t + 0.012); s.lineTo(t, L); s.lineTo(0, L); s.closePath()
    const g = new THREE.ExtrudeGeometry(s, { depth: w, bevelEnabled: false }); g.translate(0, 0, -w / 2)
    const rib = new THREE.Shape(); rib.moveTo(t, L - 0.006); rib.lineTo(L - 0.006, t); rib.lineTo(t, t); rib.closePath()
    const gr = new THREE.ExtrudeGeometry(rib, { depth: 0.006, bevelEnabled: false }); gr.translate(0, 0, -0.003)
    const X = new THREE.Vector3(...e1), Y = new THREE.Vector3(...e2), Z = new THREE.Vector3().crossVectors(X, Y)
    const q = new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(X, Y, Z))
    put('frame', M.bracket, place(g, corner, q)); put('frame', M.bracket, place(gr, corner, q))
  }
  const y0 = FLOOR + FOOT, yTop = TOPY + h
  for (const sx of [-1, 1]) for (const sz of [-1, 1]) {
    const x = sx * PX, z = sz * PZ
    profile([x, y0, z], [x, yTop, z], [false, true])                                  // post
    put('frame', M.alu, box([x, y0 - 0.006, z], [0.12, 0.012, 0.12]))                  // base plate
    for (const [dx, dz] of [[1, 1], [-1, 1], [-1, -1], [1, -1]]) put('frame', M.steel, rod([x + dx * 0.045, y0, z + dz * 0.045], [x + dx * 0.045, y0 + 0.004, z + dz * 0.045], 0.006, 10)) // bolt heads
    put('frame', M.steel, rod([x, FLOOR + 0.014, z], [x, y0 - 0.012, z], 0.008, 12))  // levelling screw
    put('frame', M.steel, rod([x, FLOOR + 0.030, z], [x, FLOOR + 0.040, z], 0.013, 6)) // lock nut (hex)
    put('frame', M.alu, rod([x, FLOOR + 0.004, z], [x, FLOOR + 0.014, z], 0.040, 32, 0.034)) // swivel pad
    put('frame', M.rubber, rod([x, FLOOR, z], [x, FLOOR + 0.004, z], 0.040, 32))
    // brackets under the top beams, against the inner faces of the post
    bracket([x - sx * h, TOPY - h, z], [-sx, 0, 0], [0, -1, 0])
    bracket([x, TOPY - h, z - sz * h], [0, 0, -sz], [0, -1, 0])
  }
  for (const sz of [-1, 1]) profile([-PX + h, TOPY, sz * PZ], [PX - h, TOPY, sz * PZ], [false, false])   // beams along X
  for (const sx of [-1, 1]) profile([sx * PX, TOPY, -PZ + h], [sx * PX, TOPY, PZ - h], [false, false])   // beams along Z
  for (const sx of [-1, 1]) {
    profile([sx * 0.22, TOPY, -PZ + h], [sx * 0.22, TOPY, PZ - h], [false, false])                   // carriers for the robot
    for (const sz of [-1, 1]) for (const k of [-1, 1]) bracket([sx * 0.22 + k * h, TOPY - h, sz * (PZ - h)], [k, 0, 0], [0, -1, 0], 0.04)
    for (const z of [-0.16, 0.16]) for (const dx of [-0.02, 0.02]) put('frame', M.steel, rod([sx * 0.22 + dx, TOPY + h, z], [sx * 0.22 + dx, TOPY + h + 0.006, z], 0.007, 6)) // bolts through the carrier
  }
}

// ---------------------------------------------------------------- export
const scene = new THREE.Scene()
for (const [name, byMat] of parts) {
  const node = new THREE.Group(); node.name = name; scene.add(node)
  for (const [m, gs] of byMat) {
    const g = mergeGeometries(gs.map((x) => { for (const k of Object.keys(x.attributes)) if (!['position', 'normal', 'uv'].includes(k)) x.deleteAttribute(k); return x }))
    const mesh = new THREE.Mesh(g, m); mesh.name = `${name}_${m.name}`; node.add(mesh)
  }
}
const glb = await new GLTFExporter().parseAsync(scene, { binary: true })
const out = join(root, '_build', 'delta.glb')
mkdirSync(dirname(out), { recursive: true })
writeFileSync(out, Buffer.from(glb))
console.log(out, (glb.byteLength / 1e6).toFixed(2), 'MB', [...parts.keys()].join(' '))
