// screenshots of the whole page at desktop and phone widths, light and dark
import { createServer } from "node:http";
import { readFile } from "node:fs/promises";
import { createRequire } from "node:module";
import path from "node:path";
import { fileURLToPath } from "node:url";
const here = path.dirname(fileURLToPath(import.meta.url));
const viewer = path.resolve(here, "../viewer"), out = path.resolve(here, "../preview");
const threeDir = path.resolve(process.argv[2]);
const require = createRequire(import.meta.url);
let chromium;
try { ({ chromium } = require("playwright")); } catch { ({ chromium } = require(path.join(process.execPath, "../../lib/node_modules/playwright"))); }
const wrap = (html) => `<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover"><style>:root{color-scheme:light;padding:env(safe-area-inset-top,0px) 0 env(safe-area-inset-bottom,0px)}body{margin:0;font:14px system-ui,sans-serif;background:#fafaf8}img{max-width:100%}[hidden]{display:none!important}</style></head><body>${html}</body></html>`;
const types = { ".html": "text/html", ".json": "application/json", ".glb": "model/gltf-binary" };
const server = createServer(async (req, res) => {
  try {
    const p = decodeURIComponent(new URL(req.url, "http://x").pathname).replace(/^\/$/, "/index.html");
    let body = await readFile(path.join(viewer, p));
    if (p === "/index.html") body = Buffer.from(wrap(body.toString()));
    res.writeHead(200, { "content-type": types[path.extname(p)] ?? "application/octet-stream" }); res.end(body);
  } catch { res.writeHead(404); res.end(); }
}).listen(0);
const base = `http://127.0.0.1:${server.address().port}/`;
const browser = await chromium.launch({ args: ["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"] });
for (const [name, w, h, scheme] of [["page-desktop", 1280, 900, "light"], ["page-phone-dark", 400, 800, "dark"]]) {
  const p = await browser.newPage({ viewport: { width: w, height: h }, deviceScaleFactor: 1, colorScheme: scheme });
  await p.route("https://cdn.jsdelivr.net/npm/three@0.169.0/**", async (route) => {
    const rel = new URL(route.request().url()).pathname.replace("/npm/three@0.169.0/", "");
    route.fulfill({ body: await readFile(path.join(threeDir, rel)), contentType: "text/javascript" });
  });
  await p.route(/fonts\.(googleapis|gstatic)\.com/, (r) => r.abort());
  p.on("pageerror", (e) => console.error("page error:", e.message));
  await p.goto(base);
  await p.waitForFunction(() => window.__ready === true, null, { timeout: 90000 });
  await p.waitForTimeout(800);
  const overflow = await p.evaluate(() => document.documentElement.scrollWidth - document.documentElement.clientWidth);
  console.log(name, "horizontal overflow px:", overflow);
  await p.screenshot({ path: path.join(out, `${name}.png`), fullPage: true });
  await p.close();
}
await browser.close(); server.close();
