#!/usr/bin/env node
// Render preview PNGs of viewer/index.html in headless Chromium.
//   node trackx/tools/render.mjs --three /path/to/node_modules/three [name ...]
// three.js is served from a local copy so no network is needed.
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
const only = process.argv.slice(2).filter((a, i, all) => !a.startsWith("--") && all[i - 1] !== "--three");

const require = createRequire(import.meta.url);
let chromium;
try { ({ chromium } = require("playwright")); }
catch { ({ chromium } = require(path.join(process.execPath, "../../lib/node_modules/playwright"))); }

// the artifact host wraps a page in this skeleton
const wrap = (html) => `<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover"><style>:root{color-scheme:light;padding:env(safe-area-inset-top,0px) 0 env(safe-area-inset-bottom,0px)}body{margin:0;font:14px system-ui,sans-serif;background:#fafaf8}img{max-width:100%}[hidden]{display:none!important}</style></head><body>${html}</body></html>`;
const types = { ".html": "text/html", ".json": "application/json", ".glb": "model/gltf-binary" };
const server = createServer(async (req, res) => {
  try {
    const p = decodeURIComponent(new URL(req.url, "http://x").pathname).replace(/^\/$/, "/index.html");
    const file = path.join(viewer, p);
    let body = await readFile(file);
    if (p === "/index.html") body = Buffer.from(wrap(body.toString()));
    res.writeHead(200, { "content-type": types[path.extname(file)] ?? "application/octet-stream" });
    res.end(body);
  } catch { res.writeHead(404); res.end(); }
}).listen(0);
const base = `http://127.0.0.1:${server.address().port}/`;

const browser = await chromium.launch({ args: ["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"] });
async function page(w, h, scheme = "light") {
  const p = await browser.newPage({ viewport: { width: w, height: h }, deviceScaleFactor: 1, colorScheme: scheme });
  await p.route("https://cdn.jsdelivr.net/npm/three@0.169.0/**", async (route) => {
    const rel = new URL(route.request().url()).pathname.replace("/npm/three@0.169.0/", "");
    route.fulfill({ body: await readFile(path.join(threeDir, rel)), contentType: "text/javascript" });
  });
  await p.route(/fonts\.(googleapis|gstatic)\.com/, (route) => route.abort());
  p.on("pageerror", (e) => console.error("page error:", e.message));
  return p;
}

const shots = [
  ["hero", "view=quarter", 1600, 900],
  ["side", "view=side", 1600, 700],
  ["front", "view=front", 1300, 800],
  ["rear", "view=rear", 1300, 800],
  ["top", "view=top", 1600, 800],
  ["under", "view=under", 1600, 800],
  ["hull", "view=quarter&gear=0", 1600, 900],
];
await mkdir(out, { recursive: true });
for (const [name, q, w, h] of shots) {
  if (only.length && !only.includes(name)) continue;
  const p = await page(w, h);
  await p.goto(`${base}?shot=1&${q}`);
  await p.waitForFunction(() => window.__ready === true, null, { timeout: 90000 });
  await p.waitForTimeout(600);
  await p.screenshot({ path: path.join(out, `${name}.png`) });
  await p.close();
  console.log("wrote", name);
}
if (!only.length || only.includes("page")) {
  const p = await page(1280, 900);
  await p.goto(base);
  await p.waitForFunction(() => window.__ready === true, null, { timeout: 90000 });
  await p.waitForTimeout(600);
  await p.screenshot({ path: path.join(out, "page.png"), fullPage: true });
  console.log("wrote page");
}
await browser.close();
server.close();
