import 'katex/dist/katex.min.css';
import './style.css';
import { emptyScene } from './model/types';
import { PRESETS } from './presets';
import { AnalysisPanel } from './ui/analysisPanel';
import { App } from './ui/app';
import { Editor } from './ui/editor';
import { PropsPanel } from './ui/props';

const $ = <T extends HTMLElement = HTMLElement>(id: string) => document.getElementById(id) as T;

const canvas = $<HTMLCanvasElement>('canvas');
const app = new App(canvas);
const editor = new Editor(app, canvas);
new PropsPanel(app, $('props'), editor);
new AnalysisPanel(app);

// ------------------------------------------------------------------ toolbar
const playBtn = $<HTMLButtonElement>('btn-play');
const timeEl = $('time');
const rateEl = $('rate');
const undoBtn = $<HTMLButtonElement>('btn-undo');
const redoBtn = $<HTMLButtonElement>('btn-redo');
const gVal = $<HTMLInputElement>('gravity-g');
const banner = $('mode-banner');
const warnEl = $('warnings');
const emptyEl = $('empty-state');

const PLAY_ICON = '<svg viewBox="0 0 24 24"><path d="M7 4.5v15l12-7.5z" fill="currentColor"/></svg><span>Play</span>';
const PAUSE_ICON = '<svg viewBox="0 0 24 24"><rect x="6" y="5" width="4" height="14" fill="currentColor"/><rect x="14" y="5" width="4" height="14" fill="currentColor"/></svg><span>Pause</span>';

playBtn.addEventListener('click', () => app.toggle());
$('btn-step').addEventListener('click', () => app.step(0.01));
$('btn-reset').addEventListener('click', () => app.reset());
undoBtn.addEventListener('click', () => app.undo());
redoBtn.addEventListener('click', () => app.redo());

// speed presets
const SPEEDS = [0.1, 0.25, 0.5, 1, 2, 5, 10];
const seg = $('speed-seg');
const SPEED_KEY = 'mechanism-sandbox:speed';
function setSpeed(v: number): void {
  app.timeScale = v;
  for (const b of seg.querySelectorAll<HTMLButtonElement>('button')) b.setAttribute('aria-checked', String(Number(b.dataset.v) === v));
  try { localStorage.setItem(SPEED_KEY, String(v)); } catch { /* storage unavailable */ }
}
for (const v of SPEEDS) {
  const b = document.createElement('button');
  b.dataset.v = String(v);
  b.setAttribute('role', 'radio');
  b.textContent = `${v}×`;
  b.title = v < 1 ? `Slow motion: 1 s of simulation takes ${1 / v} s` : v > 1 ? `${v}× faster than real time` : 'Real time';
  b.addEventListener('click', () => setSpeed(v));
  seg.appendChild(b);
}
setSpeed((() => {
  try { const v = Number(localStorage.getItem(SPEED_KEY)); return SPEEDS.includes(v) ? v : 1; } catch { return 1; }
})());

// toggles
function bindToggle(id: string, get: () => boolean, set: (v: boolean) => void): () => void {
  const b = $<HTMLButtonElement>(id);
  const sync = () => b.setAttribute('aria-pressed', String(get()));
  b.addEventListener('click', () => { set(!get()); sync(); app.redraw(); });
  sync();
  return sync;
}
const syncGravity = bindToggle('tg-gravity', () => app.scene.gravity.on, (v) => app.commit((s) => (s.gravity.on = v)));
const syncContact = bindToggle('tg-contact', () => app.scene.settings.contact !== false, (v) => app.commit((s) => (s.settings.contact = v)));
bindToggle('tg-snap', () => app.snap, (v) => (app.snap = v));
bindToggle('tg-fbd', () => app.showFBD, (v) => (app.showFBD = v));
bindToggle('tg-labels', () => app.showLabels, (v) => (app.showLabels = v));
bindToggle('tg-trails', () => app.showTrails, (v) => (app.showTrails = v));
bindToggle('tg-vel', () => app.showVelocity, (v) => (app.showVelocity = v));
gVal.addEventListener('change', () => {
  const g = Number(gVal.value);
  if (Number.isFinite(g)) app.commit((s) => (s.gravity.g = g));
});

// scene menu: new / open / save / presets
const sceneBtn = $<HTMLButtonElement>('btn-scene');
const sceneMenu = $('scene-menu');
const emptyPresets = emptyEl.querySelector('.empty-presets')!;
function setMenu(open: boolean): void {
  sceneMenu.hidden = !open;
  sceneBtn.setAttribute('aria-expanded', String(open));
}
sceneBtn.addEventListener('click', (ev) => { ev.stopPropagation(); setMenu(sceneMenu.hidden === true); });
document.addEventListener('click', (ev) => { if (!sceneMenu.contains(ev.target as Node)) setMenu(false); });
document.addEventListener('keydown', (ev) => { if (ev.key === 'Escape') setMenu(false); });

function loadPreset(id: string): void {
  const p = PRESETS.find((x) => x.id === id);
  if (!p) return;
  app.setScene(p.build());
  fitView();
}
for (const p of PRESETS) {
  for (const host of [$('menu-presets'), emptyPresets]) {
    const b = document.createElement('button');
    b.textContent = p.title;
    b.setAttribute('role', 'menuitem');
    b.addEventListener('click', () => { setMenu(false); loadPreset(p.id); });
    host.appendChild(b);
  }
}

const fileInput = $<HTMLInputElement>('file-input');
function saveScene(): void {
  const blob = new Blob([JSON.stringify(app.scene, null, 2)], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `${(app.scene.title || 'mechanism').replace(/[^\w.-]+/g, '_')}.json`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}
sceneMenu.querySelectorAll<HTMLButtonElement>('[data-act]').forEach((b) => b.addEventListener('click', () => {
  setMenu(false);
  if (b.dataset.act === 'new') app.setScene(emptyScene());
  else if (b.dataset.act === 'open') fileInput.click();
  else saveScene();
}));
fileInput.addEventListener('change', async () => {
  const f = fileInput.files?.[0];
  fileInput.value = '';
  if (!f) return;
  try {
    const s = JSON.parse(await f.text());
    if (!s || !Array.isArray(s.elements)) throw new Error('not a scene file');
    const base = emptyScene();
    app.setScene({ ...base, ...s, settings: { ...base.settings, ...s.settings } });
    fitView();
  } catch (err) {
    alert(`Could not load scene: ${(err as Error).message}`);
  }
});

// view menu (toggles stay open so several can be flipped)
const viewBtn = $<HTMLButtonElement>('btn-view');
const viewMenu = $('view-menu');
viewBtn.addEventListener('click', (ev) => {
  ev.stopPropagation();
  viewMenu.hidden = !viewMenu.hidden;
  viewBtn.setAttribute('aria-expanded', String(!viewMenu.hidden));
});
viewMenu.addEventListener('click', (ev) => ev.stopPropagation());
document.addEventListener('click', () => { viewMenu.hidden = true; viewBtn.setAttribute('aria-expanded', 'false'); });

// resizable side panel
const side = $('side');
const SIDE_KEY = 'mechanism-sandbox:side-width';
try { const w = Number(localStorage.getItem(SIDE_KEY)); if (w > 0) side.style.width = `${w}px`; } catch { /* storage unavailable */ }
$('side-resizer').addEventListener('pointerdown', (ev) => {
  const el = ev.currentTarget as HTMLElement;
  el.setPointerCapture(ev.pointerId);
  el.classList.add('dragging');
  const startX = ev.clientX, startW = side.getBoundingClientRect().width;
  const move = (e: PointerEvent) => {
    const w = Math.min(window.innerWidth * 0.65, Math.max(280, startW - (e.clientX - startX)));
    side.style.width = `${w}px`;
  };
  const up = () => {
    el.removeEventListener('pointermove', move);
    el.removeEventListener('pointerup', up);
    el.classList.remove('dragging');
    try { localStorage.setItem(SIDE_KEY, String(side.getBoundingClientRect().width)); } catch { /* storage unavailable */ }
  };
  el.addEventListener('pointermove', move);
  el.addEventListener('pointerup', up);
});

// ------------------------------------------------------------------ status
function syncTime(): void {
  timeEl.textContent = `${app.sim.t.toFixed(3)} s`;
  const r = app.achievedScale;
  if (app.running && r !== null && r < app.timeScale * 0.85) {
    rateEl.textContent = `CPU-limited: ${r < 1 ? r.toFixed(2) : r.toFixed(1)}×`;
    rateEl.title = 'This scene is too heavy to simulate at the chosen speed; it is running as fast as it can.';
  } else rateEl.textContent = '';
}

function syncToolbar(): void {
  playBtn.innerHTML = app.running ? PAUSE_ICON : PLAY_ICON;
  playBtn.classList.toggle('running', app.running);
  syncTime();
  undoBtn.disabled = !app.history.canUndo;
  redoBtn.disabled = !app.history.canRedo;
  syncGravity();
  syncContact();
  if (document.activeElement !== gVal) gVal.value = String(app.scene.gravity.g);
  gVal.disabled = !app.scene.gravity.on;
  if (app.modeAnim) {
    const m = app.modeAnim;
    banner.hidden = false;
    banner.innerHTML = `Mode ${m.index + 1} · ω = ${m.lin.omega[m.index].toPrecision(4)} rad/s · slowed for viewing <button id="stop-mode">Stop</button>`;
    $('stop-mode').addEventListener('click', () => app.stopModeAnim());
  } else banner.hidden = true;
  warnEl.innerHTML = '';
  for (const w of app.warnings) {
    const d = document.createElement('div');
    d.textContent = w;
    warnEl.appendChild(d);
  }
  emptyEl.hidden = app.scene.elements.length > 0 || editor.tool !== 'select';
}
app.onState(syncToolbar);
app.onSceneChange(syncToolbar);
app.onTick(syncTime);
editor.onToolChange = syncToolbar;

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
  r.view.scale = Math.min(400, 0.82 * Math.min(r.w / Math.max(w, 0.5), (r.h - 60) / Math.max(h, 0.5)));
  app.redraw();
}
$('btn-fit').addEventListener('click', () => fitView());
window.addEventListener('keydown', (ev) => {
  const t = ev.target as HTMLElement;
  if (t && (t.tagName === 'INPUT' || t.tagName === 'SELECT' || t.tagName === 'TEXTAREA')) return;
  if ((ev.key === 'f' || ev.key === 'F') && !ev.metaKey && !ev.ctrlKey) fitView();
});

const resize = () => { app.renderer.resize(); app.redraw(); };
new ResizeObserver(resize).observe(canvas);
resize();
syncToolbar();
if (app.scene.elements.length) fitView();
app.start();

// handy for debugging from the console
(window as any).app = app;
(window as any).editor = editor;
