// Keyboard and mouse reference, opened with "?" or the toolbar button.

import { ALL } from './palette';

const GENERAL: [string, string][] = [
  ['Space', 'Play / pause'],
  ['R', 'Reset to t = 0'],
  ['F', 'Frame the whole scene'],
  ['⌘/Ctrl Z', 'Undo'],
  ['⌘/Ctrl ⇧ Z', 'Redo'],
  ['⌘/Ctrl D', 'Duplicate selection'],
  ['Delete', 'Delete selection'],
  ['[  ]', 'Rotate selection 15°'],
  ['Arrows', 'Nudge 5 cm (⇧ 1 cm)'],
  ['Esc', 'Cancel / back to Select'],
  ['?', 'This help'],
];

const MOUSE: [string, string][] = [
  ['Drag body', 'Move it (paused at t = 0)'],
  ['Drag body while running', 'Grab and throw it'],
  ['Drag empty space', 'Pan (also right / middle drag)'],
  ['Scroll', 'Zoom'],
  ['⇧ click', 'Add to selection'],
  ['Hold ⇧ while placing', 'Turn off snapping'],
  ['Right click', 'Leave the current tool'],
];

export function initHelp(button: HTMLElement): void {
  const el = document.createElement('div');
  el.id = 'help';
  el.hidden = true;
  const rows = (list: [string, string][]) => list.map(([k, v]) => `<kbd>${k}</kbd><span>${v}</span>`).join('');
  el.innerHTML = `<div class="help-card" role="dialog" aria-label="Shortcuts">
    <header><h2>Shortcuts</h2><button class="icon" data-close title="Close (Esc)">✕</button></header>
    <div class="help-cols">
      <section><h3>General</h3><div class="help-grid">${rows(GENERAL)}</div>
        <h3>Mouse</h3><div class="help-grid">${rows(MOUSE)}</div></section>
      <section><h3>Tools</h3><div class="help-grid">${rows(ALL.map((t) => [t.key.toUpperCase(), t.label]))}</div></section>
    </div>
  </div>`;
  document.body.appendChild(el);
  const set = (open: boolean) => { el.hidden = !open; };
  button.addEventListener('click', (ev) => { ev.stopPropagation(); set(el.hidden === true); });
  el.addEventListener('click', (ev) => { if (ev.target === el || (ev.target as HTMLElement).closest('[data-close]')) set(false); });
  window.addEventListener('keydown', (ev) => {
    const t = ev.target as HTMLElement;
    if (t && (t.tagName === 'INPUT' || t.tagName === 'SELECT' || t.tagName === 'TEXTAREA')) return;
    if (ev.key === '?') { ev.preventDefault(); set(el.hidden === true); }
    else if (ev.key === 'Escape' && !el.hidden) set(false);
  });
}
