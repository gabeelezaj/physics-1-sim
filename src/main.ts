import 'katex/dist/katex.min.css';
import './style.css';
import { emptyScene } from './model/types';
import { PRESETS } from './presets';
import { App } from './ui/app';
import { Editor } from './ui/editor';
import { PropsPanel } from './ui/props';

const $ = <T extends HTMLElement = HTMLElement>(id: string) => document.getElementById(id) as T;

const canvas = $<HTMLCanvasElement>('canvas');
const app = new App(canvas);
const editor = new Editor(app, canvas);
new PropsPanel(app, $('props'), editor);

// ------------------------------------------------------------------ toolbar
const playBtn = $<HTMLButtonElement>('btn-play');
const timeEl = $('time');
const undoBtn = $<HTMLButtonElement>('btn-undo');
const redoBtn = $<HTMLButtonElement>('btn-redo');
const gOn = $<HTMLInputElement>('gravity-on');
const gVal = $<HTMLInputElement>('gravity-g');
const banner = $('mode-banner');
const warnEl = $('warnings');

playBtn.addEventListener('click', () => app.toggle());
$('btn-step').addEventListener('click', () => app.step(0.01));
$('btn-reset').addEventListener('click', () => app.reset());
$('btn-undo').addEventListener('click', () => app.undo());
$('btn-redo').addEventListener('click', () => app.redo());

const ts = $<HTMLInputElement>('time-scale');
const tsVal = $('time-scale-val');
const applyScale = () => {
  app.timeScale = Math.pow(2, Number(ts.value));
  tsVal.textContent = `${app.timeScale.toFixed(2)}×`;
};
ts.addEventListener('input', applyScale);
applyScale();

gOn.addEventListener('change', () => app.commit((s) => (s.gravity.on = gOn.checked)));
gVal.addEventListener('change', () => {
  const g = Number(gVal.value);
  if (Number.isFinite(g)) app.commit((s) => (s.gravity.g = g));
});
$<HTMLInputElement>('snap').addEventListener('change', (e) => (app.snap = (e.target as HTMLInputElement).checked));
$<HTMLInputElement>('show-fbd').addEventListener('change', (e) => { app.showFBD = (e.target as HTMLInputElement).checked; app.redraw(); });
$<HTMLInputElement>('show-labels').addEventListener('change', (e) => { app.showLabels = (e.target as HTMLInputElement).checked; app.redraw(); });

const presetSel = $<HTMLSelectElement>('preset-select');
for (const p of PRESETS) {
  const o = document.createElement('option');
  o.value = p.id;
  o.textContent = p.title;
  presetSel.appendChild(o);
}
presetSel.addEventListener('change', () => {
  const p = PRESETS.find((x) => x.id === presetSel.value);
  presetSel.value = '';
  if (!p) return;
  app.setScene(p.build());
  fitView();
});

$('btn-new').addEventListener('click', () => app.setScene(emptyScene()));
$('btn-save').addEventListener('click', () => {
  const blob = new Blob([JSON.stringify(app.scene, null, 2)], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `${(app.scene.title || 'mechanism').replace(/[^\w.-]+/g, '_')}.json`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
});
const fileInput = $<HTMLInputElement>('file-input');
$('btn-load').addEventListener('click', () => fileInput.click());
fileInput.addEventListener('change', async () => {
  const f = fileInput.files?.[0];
  fileInput.value = '';
  if (!f) return;
  try {
    const s = JSON.parse(await f.text());
    if (!s || !Array.isArray(s.elements)) throw new Error('not a scene file');
    app.setScene({ ...emptyScene(), ...s });
    fitView();
  } catch (err) {
    alert(`Could not load scene: ${(err as Error).message}`);
  }
});

function syncToolbar(): void {
  playBtn.textContent = app.running ? '❚❚ Pause' : '▶ Play';
  timeEl.textContent = `t = ${app.sim.t.toFixed(3)} s`;
  undoBtn.disabled = !app.history.canUndo;
  redoBtn.disabled = !app.history.canRedo;
  gOn.checked = app.scene.gravity.on;
  if (document.activeElement !== gVal) gVal.value = String(app.scene.gravity.g);
  if (app.modeAnim) {
    const m = app.modeAnim;
    banner.hidden = false;
    banner.innerHTML = `Animating mode ${m.index + 1} · ω = ${m.lin.omega[m.index].toPrecision(4)} rad/s (slowed for viewing) <button id="stop-mode">Stop</button>`;
    $('stop-mode').addEventListener('click', () => app.stopModeAnim());
  } else banner.hidden = true;
  warnEl.innerHTML = '';
  for (const w of app.warnings) {
    const d = document.createElement('div');
    d.textContent = w;
    warnEl.appendChild(d);
  }
}
app.onState(syncToolbar);
app.onSceneChange(syncToolbar);
app.onTick(() => (timeEl.textContent = `t = ${app.sim.t.toFixed(3)} s`));

/** Frame the scene's bounding box. */
function fitView(): void {
  const xs: number[] = [], ys: number[] = [];
  for (const e of app.scene.elements) {
    if ('x' in e) {
      const r = e.kind === 'ground' || e.kind === 'rect' ? Math.max(e.w, e.h) / 2 : e.kind === 'pulley' ? e.r : 0.1;
      xs.push(e.x - r, e.x + r);
      ys.push(e.y - r, e.y + r);
    }
  }
  if (!xs.length) return;
  const r = app.renderer;
  const w = Math.max(...xs) - Math.min(...xs), h = Math.max(...ys) - Math.min(...ys);
  r.view.cx = (Math.max(...xs) + Math.min(...xs)) / 2;
  r.view.cy = (Math.max(...ys) + Math.min(...ys)) / 2;
  r.view.scale = Math.min(400, 0.8 * Math.min(r.w / Math.max(w, 0.5), r.h / Math.max(h, 0.5)));
  app.redraw();
}

const resize = () => { app.renderer.resize(); app.redraw(); };
new ResizeObserver(resize).observe(canvas);
resize();
syncToolbar();
if (app.scene.elements.length) fitView();
app.start();

// handy for debugging from the console
(window as any).app = app;
