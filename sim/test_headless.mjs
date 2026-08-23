// Headless test driver for the ballpark 3D simulator.
// Requires a headless chromium with CDP, e.g.:
//   ~/Library/Caches/ms-playwright/chromium_headless_shell-*/chrome-headless-shell-*/chrome-headless-shell \
//     --remote-debugging-port=9333 --no-sandbox --use-angle=swiftshader --enable-unsafe-swiftshader about:blank
// and the sim served locally:  cd sim && python3 -m http.server 8765
// Usage: SIM_QS='?test=pin&seed=7' node test_headless.mjs
// minimal CDP driver: run a ballpark sim test URL in headless chromium
const URL_BASE = 'http://localhost:8765/sim3d.html';
const qs = process.env.SIM_QS || '?test=pin&seed=7';
const target = process.env.TIMEOUT_MS ? +process.env.TIMEOUT_MS : 120000;

const list = await (await fetch('http://127.0.0.1:9333/json/list')).json();
const page = list.find(t => t.type === 'page');
const ws = new WebSocket(page.webSocketDebuggerUrl);
let id = 0; const pending = new Map();
const send = (method, params = {}) => new Promise(res => {
  const i = ++id; pending.set(i, res); ws.send(JSON.stringify({id: i, method, params}));
});
ws.onmessage = e => {
  const m = JSON.parse(e.data);
  if (m.id && pending.has(m.id)) { pending.get(m.id)(m.result); pending.delete(m.id); }
};
await new Promise(r => ws.onopen = r);
await send('Page.enable');
await send('Runtime.enable');
await send('Page.navigate', {url: URL_BASE + qs});
const t0 = Date.now();
while (Date.now() - t0 < target) {
  await new Promise(r => setTimeout(r, 3000));
  const ev = await send('Runtime.evaluate', {expression: 'document.title', returnByValue: true});
  const title = ev.result.value;
  process.stdout.write('[' + Math.round((Date.now()-t0)/1000) + 's] ' + title + '\n');
  if (/GRID3D|SINGLE3D|PAGEERR/.test(title)) {
    const out = await send('Runtime.evaluate', {expression:
      'document.getElementById("batchout") ? document.getElementById("batchout").textContent.split("\\n").filter(l=>l.startsWith("✓")||l.startsWith("✗")).join("\\n") : ""',
      returnByValue: true});
    if (out.result.value) process.stdout.write('POSES:\n' + out.result.value + '\n');
    break;
  }
}
ws.close();
process.exit(0);
