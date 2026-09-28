#!/usr/bin/env node
// Render preview PNGs from viewer/index.html in headless Chromium.
//
//   npm install three@0.169.0 playwright   (or use a global playwright)
//   node airframe/tools/render.mjs [--three node_modules/three]
//
// three.js is served from a local copy so no network access is needed.
import { createServer } from "node:http";
import { readFile, mkdir } from "node:fs/promises";
import { createRequire } from "node:module";
import path from "node:path";
import { fileURLToPath } from "node:url";

const here = path.dirname(fileURLToPath(import.meta.url));
const viewer = path.resolve(here, "../viewer");
const out = path.resolve(here, "../preview");
const arg = (k, d) => { const i = process.argv.indexOf(k); return i > 0 ? process.argv[i + 1] : d; };
const threeDir = path.resolve(arg("--three", "node_modules/three"));

const require = createRequire(import.meta.url);
let chromium;
try { ({ chromium } = require("playwright")); }
catch { ({ chromium } = require(path.join(process.execPath, "../../lib/node_modules/playwright"))); }

const types = { ".html": "text/html", ".json": "application/json", ".glb": "model/gltf-binary", ".js": "text/javascript" };
const server = createServer(async (req, res) => {
  try {
    const file = path.join(viewer, decodeURIComponent(new URL(req.url, "http://x").pathname).replace(/^\/$/, "/index.html"));
    res.writeHead(200, { "content-type": types[path.extname(file)] ?? "application/octet-stream" });
    res.end(await readFile(file));
  } catch { res.writeHead(404); res.end(); }
}).listen(0);
const base = `http://127.0.0.1:${server.address().port}/`;

const browser = await chromium.launch({ args: ["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"] });
async function page(w, h) {
  const p = await browser.newPage({ viewport: { width: w, height: h }, deviceScaleFactor: 1 });
  await p.route("https://cdn.jsdelivr.net/npm/three@0.169.0/**", async (route) => {
    const rel = new URL(route.request().url()).pathname.replace("/npm/three@0.169.0/", "");
    route.fulfill({ body: await readFile(path.join(threeDir, rel)), contentType: "text/javascript" });
  });
  await p.route(/fonts\.(googleapis|gstatic)\.com/, (route) => route.abort());
  p.on("pageerror", (e) => console.error("page error:", e.message));
  return p;
}

await mkdir(out, { recursive: true });
const shots = [
  ["hero", "view=quarter", 1600, 1000],
  ["top", "view=top", 1400, 1000],
  ["side", "view=side", 1600, 700],
  ["front", "view=front", 1400, 700],
  ["under", "view=under", 1400, 900],
  ["cutaway", "view=side&cut=1", 1600, 700],
];
for (const [name, q, w, h] of shots) {
  const p = await page(w, h);
  await p.goto(`${base}?shot=1&${q}`);
  await p.waitForFunction(() => window.__ready === true, null, { timeout: 60000 });
  await p.waitForTimeout(800);
  await p.screenshot({ path: path.join(out, `${name}.png`) });
  await p.close();
  console.log("wrote", name);
}
if (process.argv.includes("--page")) {
  const p = await page(1280, 900);
  await p.goto(base);
  await p.waitForFunction(() => window.__ready === true, null, { timeout: 60000 });
  await p.waitForTimeout(800);
  await p.screenshot({ path: path.join(out, "page.png"), fullPage: true });
  console.log("wrote page");
}
await browser.close();
server.close();
