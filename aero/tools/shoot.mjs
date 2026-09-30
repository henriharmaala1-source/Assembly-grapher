// Screenshots of the efficiency study page (aero/study/index.html) for the README.
//
//   npm install playwright   (or use a global playwright)
//   node aero/tools/shoot.mjs            # PNGs into aero/study/figures/
//   node aero/tools/shoot.mjs --dark     # the dark theme too
//   node aero/tools/shoot.mjs --page     # and the whole page
//
// Serves aero/study/ on a local port, waits for the charts, and saves one PNG
// per chart, the cross-checks too when crosscheck.json is there (and the whole page with --page).
import { createServer } from "node:http";
import { readFile, mkdir } from "node:fs/promises";
import { createRequire } from "node:module";
import path from "node:path";
import { fileURLToPath } from "node:url";

const here = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(here, "../study");
const out = path.join(root, "figures");
const require = createRequire(import.meta.url);
let chromium;
try { ({ chromium } = require("playwright")); }
catch { ({ chromium } = require(path.join(process.execPath, "../../lib/node_modules/playwright"))); }

const types = { ".html": "text/html", ".json": "application/json", ".dat": "text/plain", ".md": "text/plain" };
const server = createServer(async (req, res) => {
  const file = path.join(root, decodeURIComponent(req.url.split("?")[0]).replace(/^\/$/, "/index.html"));
  try {
    const body = await readFile(file);
    res.writeHead(200, { "content-type": types[path.extname(file)] || "application/octet-stream" });
    res.end(body);
  } catch { res.writeHead(404); res.end(); }
});
await new Promise((r) => server.listen(0, r));
const base = `http://127.0.0.1:${server.address().port}/`;
await mkdir(out, { recursive: true });

const browser = await chromium.launch();
const themes = process.argv.includes("--dark") ? ["light", "dark"] : ["light"];
for (const theme of themes) {
  const page = await browser.newPage({ viewport: { width: 1200, height: 900 }, deviceScaleFactor: 2, colorScheme: theme });
  page.on("pageerror", (e) => console.error("page error:", e.message));
  await page.goto(base);
  await page.waitForFunction(() => window.__ready === true, null, { timeout: 30000 });
  await page.waitForTimeout(600);                     // web fonts
  const suffix = theme === "dark" ? "-dark" : "";
  if (process.argv.includes("--page")) await page.screenshot({ path: path.join(out, `page${suffix}.png`), fullPage: true });
  const ids = ["summary", "f-shape", "f-points", "f-polar", "f-stall", "f-cp", "f-power", "f-breakdown"];
  if (await page.locator("#crosscheck").isVisible())                 // when crosscheck.json is there
    ids.push("xc-tools", "f-xc-drag", "f-xc-stall", "f-xc-ncrit", "f-xc-cp-base", "f-xc-cp-opt", "f-xc-plane");
  for (const id of ids) {
    await page.locator(`#${id}`).screenshot({ path: path.join(out, `${id.replace(/^f-/, "")}${suffix}.png`) });
  }
  console.log("wrote", theme);
  await page.close();
}
await browser.close();
server.close();
