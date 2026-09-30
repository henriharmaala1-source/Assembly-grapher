// Screenshots of the as-built against optimized page (airframe/variants/index.html).
//
//   npm install three@0.169.0 playwright   (or use a global playwright)
//   node airframe/tools/shoot-variants.mjs           # PNGs into airframe/variants/figures/
//   node airframe/tools/shoot-variants.mjs --dark    # the dark theme too
//   node airframe/tools/shoot-variants.mjs --page    # and the whole page
//   node airframe/tools/shoot-variants.mjs --three path/to/node_modules/three
//
// Serves airframe/ on a local port (the page reaches ../viewer/airframe.glb),
// waits for the charts and both 3D models, and saves one PNG per figure plus
// the 3D view from a few of the change buttons. three.js is served from a
// local copy so no network access is needed.
import { createServer } from "node:http";
import { readFile, mkdir } from "node:fs/promises";
import { createRequire } from "node:module";
import path from "node:path";
import { fileURLToPath } from "node:url";

const here = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(here, "..");
const out = path.join(root, "variants", "figures");
const arg = (k, d) => { const i = process.argv.indexOf(k); return i > 0 ? process.argv[i + 1] : d; };
const threeDir = path.resolve(arg("--three", "node_modules/three"));
const require = createRequire(import.meta.url);
let chromium;
try { ({ chromium } = require("playwright")); }
catch { ({ chromium } = require(path.join(process.execPath, "../../lib/node_modules/playwright"))); }

const types = { ".html": "text/html", ".json": "application/json", ".glb": "model/gltf-binary" };
const server = createServer(async (req, res) => {
  const file = path.join(root, decodeURIComponent(req.url.split("?")[0]));
  try {
    const body = await readFile(file);
    res.writeHead(200, { "content-type": types[path.extname(file)] || "application/octet-stream" });
    res.end(body);
  } catch { res.writeHead(404); res.end(); }
});
await new Promise((r) => server.listen(0, r));
const base = `http://127.0.0.1:${server.address().port}/variants/index.html`;
await mkdir(out, { recursive: true });

const browser = await chromium.launch({ args: ["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"] });
const themes = process.argv.includes("--dark") ? ["light", "dark"] : ["light"];
async function open(theme, scale) {
  const page = await browser.newPage({ viewport: { width: 1240, height: 900 }, deviceScaleFactor: scale, colorScheme: theme });
  await page.route("https://cdn.jsdelivr.net/npm/three@0.169.0/**", async (route) => {
    const rel = new URL(route.request().url()).pathname.replace("/npm/three@0.169.0/", "");
    route.fulfill({ body: await readFile(path.join(threeDir, rel)), contentType: "text/javascript" });
  });
  await page.route(/fonts\.(googleapis|gstatic)\.com/, (route) => route.abort());
  page.on("pageerror", (e) => console.error("page error:", e.message));
  await page.goto(base);
  await page.waitForFunction(() => window.__chartsReady === true && window.__modelsReady === true, null, { timeout: 60000 });
  await page.waitForTimeout(1500);                     // first frames
  return page;
}
// The 3D stage is shot as a clip of the viewport once the draw loop is still:
// software GL is slow enough that Playwright's element-stability wait gives up.
async function shootStage(page, file) {
  await page.evaluate(() => document.getElementById("stage").scrollIntoView({ block: "center" }));
  await page.waitForFunction(() => window.__still === true, null, { timeout: 60000 });
  await page.waitForTimeout(300);
  await page.screenshot({ path: path.join(out, file), clip: await page.locator("#stage").boundingBox(), timeout: 90000 });
}
for (const theme of themes) {
  const suffix = theme === "dark" ? "-dark" : "";
  let page = await open(theme, 2);
  if (process.argv.includes("--page")) await page.screenshot({ path: path.join(out, `page${suffix}.png`), fullPage: true, timeout: 120000 });
  await shootStage(page, `models${suffix}.png`);
  for (const id of ["summary", "f-mass", "f-cg", "f-split", "f-power", "f-sw-time", "f-sw-stall", "f-sw-climb", "f-sw-long"]) {
    await page.locator(`#${id}`).screenshot({ path: path.join(out, `${id.replace(/^f-/, "")}${suffix}.png`) });
  }
  await page.close();
  // The close-ups at 1x: every camera move redraws the whole canvas in software.
  page = await open(theme, 1);
  for (const view of ["side", "top", "under"]) {
    await page.locator(`.seg button[data-view="${view}"]`).click();
    await page.waitForTimeout(900);
    await shootStage(page, `models-${view}${suffix}.png`);
  }
  const changes = page.locator(".change");
  for (const [i, name] of [[0, "airfoil"], [1, "hood"], [2, "boattail"], [4, "fairings"]]) {
    await changes.nth(i).click();
    await page.waitForTimeout(900);                    // the camera move
    await shootStage(page, `models-${name}${suffix}.png`);
  }
  console.log("wrote", theme);
  await page.close();
}
await browser.close();
server.close();
