#!/usr/bin/env node
// Render preview PNGs from viewer/index.html in headless Chromium.
//
//   npm install three@0.169.0 playwright   (or use a global playwright)
//   node airframe/tools/render.mjs [--three node_modules/three] [--page]
//   node airframe/tools/render.mjs --video [--ffmpeg path/to/ffmpeg] [--fps 30]
//
// --video steps the viewer's assembly animation frame by frame and encodes
// preview/assembly.mp4 (H.264) with ffmpeg.
//
// three.js is served from a local copy so no network access is needed.
import { createServer } from "node:http";
import { readFile, mkdir, mkdtemp, rm } from "node:fs/promises";
import { spawnSync } from "node:child_process";
import os from "node:os";
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
if (process.argv.includes("--video")) {
  const fps = Number(arg("--fps", "30"));
  const p = await page(1280, 720);
  await p.goto(`${base}?shot=1&record=1`);
  await p.waitForFunction(() => window.__ready === true, null, { timeout: 60000 });
  await p.waitForTimeout(500);
  const duration = await p.evaluate(() => window.__anim.duration);
  const tmp = await mkdtemp(path.join(os.tmpdir(), "frames-"));
  const n = Math.ceil(duration * fps);
  for (let i = 0; i <= n; i++) {
    await p.evaluate((t) => window.__anim.seek(t), i / fps);
    await p.screenshot({ path: path.join(tmp, `f${String(i).padStart(5, "0")}.png`) });
    if (i % (fps * 10) === 0) console.log(`frame ${i} / ${n}`);
  }
  await p.close();
  const mp4 = path.join(out, "assembly.mp4");
  const res = spawnSync(arg("--ffmpeg", "ffmpeg"), ["-y", "-loglevel", "error", "-framerate", String(fps),
    "-i", path.join(tmp, "f%05d.png"), "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "24",
    "-preset", "slow", "-movflags", "+faststart", mp4], { stdio: "inherit" });
  if (res.status !== 0) throw new Error("ffmpeg failed");
  await rm(tmp, { recursive: true, force: true });
  console.log("wrote", mp4, `${duration.toFixed(1)} s`);
  await browser.close();
  server.close();
  process.exit(0);
}
const shots = [
  ["hero", "view=quarter", 1600, 1000],
  ["top", "view=top", 1400, 1000],
  ["side", "view=side", 1600, 700],
  ["front", "view=front", 1400, 700],
  ["under", "view=under", 1400, 900],
  ["cutaway", "view=side&cut=1", 1600, 700],
  ["pod", "view=pod&cut=1", 1400, 900],
  ["nose", "view=nose", 1200, 800],
  ["bay", "view=bay", 1400, 900],
  ["esc", "view=esc", 1300, 800],
  ["servo", "view=servo", 1200, 800],
  ["tail", "view=tail", 1400, 900],
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
