#!/usr/bin/env python3
"""Write variants/single-boom/viewer/index.html from the twin-boom viewer.

    python3 airframe/tools/make_single_viewer.py

The 3D viewer, the assembly animation and the process and DFMA sections are the
twin's; this swaps the labels, views, text and the side layout for the single boom.
"""
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
src = (HERE / "viewer" / "index.html").read_text()


def sub(old, new, count=1):
    global src
    assert src.count(old) >= 1, f"not found: {old[:70]!r}"
    src = src.replace(old, new) if count == 0 else src.replace(old, new, count)


sub("<title>Kipinä 485</title>", "<title>Kipinä single boom</title>")
sub("Twin-boom micro FPV pusher · 3D printed · parametric CadQuery",
    "Single-boom micro FPV pusher · 3D printed · parametric CadQuery")
sub('<h1>Kipinä <span id="h1-span">485</span></h1>', '<h1>Kipinä <span id="h1-span">485</span> single boom</h1>')

# views and their buttons
sub('''          <button id="v-pod" data-view="pod" aria-pressed="false">Pod</button>
          <button id="v-nose" data-view="nose" aria-pressed="false">Nose</button>
          <button id="v-prop" data-view="prop" aria-pressed="false">Prop</button>
          <button id="v-bay" data-view="bay" aria-pressed="false">Bay</button>
          <button id="v-esc" data-view="esc" aria-pressed="false">ESC</button>
          <button id="v-servo" data-view="servo" aria-pressed="false">Servo</button>
          <button id="v-tail" data-view="tail" aria-pressed="false">Tail</button>''',
    '''          <button id="v-fuselage" data-view="fuselage" aria-pressed="false">Fuselage</button>
          <button id="v-nose" data-view="nose" aria-pressed="false">Nose</button>
          <button id="v-bay" data-view="bay" aria-pressed="false">Bay</button>
          <button id="v-servos" data-view="servos" aria-pressed="false">Servos</button>
          <button id="v-esc" data-view="esc" aria-pressed="false">ESC</button>
          <button id="v-ail" data-view="ail" aria-pressed="false">Aileron</button>
          <button id="v-tail" data-view="tail" aria-pressed="false">Tail</button>
          <button id="v-motor" data-view="motor" aria-pressed="false">Motor</button>''')
sub('''  pod: { dir: [-0.8, 0.75, 0.95], fit: ["pod_front", "pod_rear", "motor", "wing_centre"] },
  nose: { dir: [-0.85, -0.3, 0.75], fit: ["camera", "vtx"], pad: 3.0 },
  prop: { dir: [0.95, 0.35, 0.55], fit: ["prop", "motor", "servo_elev", "servo_rud"], pad: 1.25 },
  bay: { dir: [-0.3, 1, 0.5], fit: ["camera", "vtx", "fc", "receiver"], pad: 1.2, hide: ["lid"] },
  esc: { dir: [-0.45, 1, 0.55], fit: ["esc", "motor"], pad: 1.6, hide: ["lid", ...WING, "servo_elev", "pushrod_elev"] },
  servo: { dir: [-0.35, -0.8, -0.55], fit: ["servo_ail_R", "horn_ail_R", "pushrod_ail_R"], pad: 1.5 },
  tail: { dir: [0.75, -0.45, 0.6], fit: ["rudder_R", "rudder_L", "bellcrank", "tail_mount", "fin_R", "fin_L"], pad: 1.05 },''',
    '''  fuselage: { dir: [-0.8, 0.75, 0.95], fit: ["fuselage_front", "fuselage_rear", "wing_centre"] },
  nose: { dir: [-0.85, -0.3, 0.75], fit: ["camera", "vtx"], pad: 3.0 },
  bay: { dir: [-0.3, 1, 0.5], fit: ["camera", "vtx", "fc", "receiver"], pad: 1.2, hide: ["lid"] },
  servos: { dir: [-0.2, 1, 0.45], fit: ["servo_elev", "servo_rud"], pad: 1.6, hide: ["lid", ...WING] },
  esc: { dir: [0.2, 0.3, 1], fit: ["esc", "servo_rud"], pad: 1.5, hide: [...WING] },
  ail: { dir: [-0.35, -0.8, -0.55], fit: ["servo_ail_R", "horn_ail_R", "pushrod_ail_R"], pad: 1.5, hide: ["fairing_ail_R"] },
  tail: { dir: [0.75, 0.45, 0.6], fit: ["rudder", "fin", "tail_mount", "stab", "elevator"], pad: 1.1 },
  motor: { dir: [0.9, 0.35, -0.5], fit: ["motor", "motor_mount", "prop", "tail_mount"], pad: 1.4 },''')
sub('''const WING = ["wing_centre", "wing_R", "wing_L", "aileron_R", "aileron_L", "spar_main", "spar_rear", "servo_ail_R",
              "servo_ail_L", "servo_rud", "horn_ail_R", "horn_ail_L", "pushrod_ail_R", "pushrod_ail_L", "pushrod_rud"];''',
    '''const WING = ["wing_centre", "wing_R", "wing_L", "aileron_R", "aileron_L", "spar_main", "spar_rear", "servo_ail_R",
              "servo_ail_L", "horn_ail_R", "horn_ail_L", "pushrod_ail_R", "pushrod_ail_L", "fairing_ail_R", "fairing_ail_L"];''')

# labels and bought parts
sub('''const LABEL = {
  pod_front: "Pod, front half", pod_rear: "Pod, rear half", lid: "Hatch lid", wing_centre: "Wing centre",''',
    '''const LABEL = {
  fuselage_front: "Fuselage, front", fuselage_rear: "Fuselage, rear", boom: "Boom, 10×9 carbon",
  fin: "Fin", rudder: "Rudder", motor_mount: "Motor cap", fairing_ail_R: "Servo fairing R", fairing_ail_L: "Servo fairing L",
  pod_front: "Pod, front half", pod_rear: "Pod, rear half", lid: "Hatch lid", wing_centre: "Wing centre",''')
sub('''"camera", "vtx", "antenna", "tube_L", "tube_R", "spar_main", "spar_rear"];''',
    '''"camera", "vtx", "antenna", "boom", "spar_main", "spar_rear"];''')

# specs, lede, kit
sub('''  ["Tail booms", `2 × ${spec.tube.replace("x", "×")} mm, ${spec.tube_len} long, ${spec.tube_spacing} apart`],
  ["Prop to boom", `${f1(spec.tube_spacing / 2 - spec.prop_r - 3)} mm clear`],''',
    '''  ["Boom", `1 × 10×9 mm roll-wrapped carbon, ${spec.tube_len} long`],
  ["Motor", `on the boom's end, ${Math.round(spec.x_prop)} mm from the nose`],''')
sub('''$("min-span").textContent = `${spec.min_span} mm`;
$("build-span").textContent = `${spec.span} mm`;
$("no-rud").textContent = spec.no_rudder_span;
''', "")
sub('''document.title = `Kipinä ${spec.span}`;''', '''document.title = `Kipinä ${spec.span} single boom`;''')
sub('''$("lede").textContent = `A ${spec.span} mm twin-boom pusher. The camera sits in the nose of a short pod, the motor on its back wall, and the prop turns between two plain 6 mm carbon booms, ${spec.tube_spacing} mm apart, that carry an H-tail with twin rudders. Every other part prints on a Bambu Lab A1 mini, four SG90 servos give roll, pitch and yaw, and ${spec.span} mm is the smallest span that still hand-launches below ${spec.stall_limit} m/s.`;
$("why-span").textContent = spec.span;
$("kit-tubes").textContent = `2 × carbon tube ${spec.tube.replace("x", "×")} mm, ${spec.tube_len} mm long, the tail booms (6×0.5 aluminium also fits, +6 g)`;''',
    '''$("lede").textContent = `The ${spec.span} mm Kipinä on one carbon boom. The camera sits in the nose, the fuselage holds the battery, FC, ESC and the tail servos, and a 10 mm boom carries the tail and, on its end, the motor and pusher prop. Every printed part fits a Bambu Lab A1 mini, and four SG90 servos give roll, pitch and yaw.`;
$("kit-tubes").textContent = `1 × roll-wrapped carbon tube 10×9 mm, ${spec.tube_len} mm long (the boom), and 2/1 mm PTFE tube for the pushrod sleeves`;''')
sub('''<p class="lede" id="lede">A 485 mm twin-boom pusher. The camera sits in the nose of a short pod, the motor on its back wall, and the prop turns between two plain 6 mm carbon booms that carry an H-tail with twin rudders. Every other part prints on a Bambu Lab A1 mini, and four SG90 servos give roll, pitch and yaw.</p>''',
    '''<p class="lede" id="lede">The 485 mm Kipinä on one carbon boom, with the motor on the boom's end.</p>''')
sub('''        <li id="kit-tubes">2 × carbon tube 6×5 mm (6×0.5 aluminium also fits, +6 g)</li>''',
    '''        <li id="kit-tubes">1 × roll-wrapped carbon tube 10×9 mm (the boom)</li>''')
sub('''        <li>1 mm carbon rod (2 × ~265 mm pushrods, taped to the booms as guides) and 0.8 mm wire for the aileron links and rudder joiners, 2 micro control horns, M2 screws for the motor, FC and bellcrank</li>''',
    '''        <li>1 mm carbon rod (2 pushrods, about 350 mm, in PTFE sleeves inside the boom) and 0.8 mm wire for the aileron links, 2 micro control horns, M2 screws for the motor and FC</li>''')
sub("airframe/design.py</code>, <code>build.py</code> and <code>process.py</code>",
    "airframe/design.py</code>, <code>build.py --variant single-boom</code> and <code>process.py --variant single-boom</code>")
sub("branch claude/cad-software-connection-l8ls4m · airframe/</footer>",
    "branch claude/cad-software-connection-l8ls4m · airframe/variants/single-boom/</footer>")

# drop the span sweep (it belongs to the twin's sizing) and redraw the side layout for one boom
a = src.index('  <section class="two">\n    <div class="prose">\n      <h2>Why <span id="why-span">')
b = src.index("  <section>\n    <h2>Balance</h2>")
src = src[:a] + src[b:]
sub("Distances from the nose of the pod. The battery slides in the pod to set the CG",
    "Distances from the nose. The battery slides in the fuselage to set the CG")
sub("drawChart();\ndrawBalance();", "drawBalance();")
sub("  drawChart(); drawBalance(); drawGraph();", "  drawBalance(); drawGraph();")
sub('''  const end = spec.x_stab + spec.c_h;''', '''  const end = Math.max(spec.x_stab + spec.c_h, spec.x_prop + 10);''')
sub('''  // booms, pod, wing, tail, battery''', '''  // boom, fuselage, wing, tail, battery''')
sub('''  g.append(svg("rect", { x: x(spec.x_stab), y: 78, width: x(spec.x_stab + spec.c_h) - x(spec.x_stab), height: 4, fill: ink }));
  g.append(svg("rect", { x: x(spec.x_stab + 2), y: 44, width: x(spec.x_stab + spec.c_h) - x(spec.x_stab + 2), height: 34, fill: "none", stroke: ink, "stroke-width": 1.2 }));''',
    '''  g.append(svg("rect", { x: x(spec.x_stab), y: 96, width: x(spec.x_stab + spec.c_h) - x(spec.x_stab), height: 4, fill: ink }));
  g.append(svg("path", { d: `M${x(spec.x_stab + 4)} 83 L${x(spec.x_stab + 18)} 40 L${x(spec.x_stab + spec.c_h)} 40 L${x(spec.x_stab + spec.c_h)} 83 Z`,
                         fill: "none", stroke: ink, "stroke-width": 1.2 }));''')
sub('''  g.append(svg("text", { x: x(spec.x_stab - 4), y: 38, "text-anchor": "end", "font-size": 11, fill: muted }, "H-tail"));
  g.append(svg("text", { x: x((spec.x_prop + spec.x_stab) / 2 + 10), y: 78, "text-anchor": "middle", "font-size": 11, fill: muted },
               `two 6 mm booms, ${spec.tube_spacing} mm apart`));''',
    '''  g.append(svg("text", { x: x(spec.x_stab - 4), y: 44, "text-anchor": "end", "font-size": 11, fill: muted }, "fin"));
  g.append(svg("text", { x: x((spec.pod_len + spec.x_stab) / 2), y: 78, "text-anchor": "middle", "font-size": 11, fill: muted },
               "one 10 mm boom"));''')
sub('''"pusher prop"));''', '''"motor, prop"));''')

# let the screenshot tool stop the draw loop once a view has settled
sub("""renderer.setAnimationLoop((now) => {
  if (params.has("record")) return;""", """renderer.setAnimationLoop((now) => {
  if (params.has("record") || window.__freeze) return;""")

out = HERE / "variants" / "single-boom" / "viewer" / "index.html"
out.write_text(src)
print("wrote", out)
