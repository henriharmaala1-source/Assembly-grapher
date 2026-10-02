// THE CROSSHAIR LAW, TWICE: ScriptMode::crosshair_ (C++, what flies) and the
// copy in control_playground.html (JavaScript, what previews it). Both are
// pinned to the same three cases -- this file checks the JavaScript one;
// onboard/test/test_script.cpp checks the C++ one with the same numbers.
const fs = require("fs");
const path = require("path");
const page = fs.readFileSync(path.join(__dirname, "..", "control_playground.html"), "utf8");
const a = page.indexOf("// LAW-BEGIN"), b = page.indexOf("// LAW-END");
if (a < 0 || b < 0) { console.log("FAIL: no LAW-BEGIN/LAW-END in the page"); process.exit(1); }
const law = new Function(page.slice(a, b) + "\nreturn { crosshairCmd, LAW };")();
// x, y (-1..1, + right/up), throttle -> yaw, pitch, throttle; 640 px wide,
// 480 tall, 10 m up, camera 30 deg down, 60 deg FoV.
const cases = [
  [0.0, 0.0, 0.5, 0.0, 0.4330, -0.6667],
  [0.0, 0.5, 0.5, 0.0, 0.4761, -0.4072],
  [0.5, -0.5, 0.8, 0.2147, 0.2745, -1.0000],
];
let fails = 0;
for (const [x, y, t, yw, p, th] of cases) {
  const c = law.crosshairCmd(x * 320, -y * 240, 640, t, 10, law.LAW);
  const ok = Math.abs(c.yaw - yw) < 2e-3 && Math.abs(c.pitch - p) < 2e-3 && Math.abs(c.throttle - th) < 2e-3;
  console.log(`  crosshair ${x} ${y} throttle ${t}: ${ok ? "ok  " : "FAIL"} [${c.yaw.toFixed(4)} ${c.pitch.toFixed(4)} ${c.throttle.toFixed(4)}]`);
  if (!ok) fails++;
}
console.log(fails ? `${fails} FAILED` : "the JavaScript law matches the C++ one");
process.exit(fails ? 1 : 0);
