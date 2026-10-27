// Element palette: textbook-style icons, click to arm a tool or drag onto the canvas.

export type Tool =
  | 'select' | 'ground' | 'ramp' | 'curve' | 'point' | 'rect' | 'pulley' | 'wheel' | 'force'
  | 'pin' | 'rod' | 'slider' | 'spring' | 'tspring' | 'rope';

interface Item {
  tool: Tool;
  key: string;
  label: string;
  icon: string;
  help: string;
}

const s = (body: string) =>
  `<svg viewBox="0 0 26 26" fill="none" stroke="#111" stroke-width="1.4" stroke-linecap="round" stroke-linejoin="round">${body}</svg>`;

const hatch = (x: number, y: number, w: number) =>
  Array.from({ length: Math.floor(w / 3) }, (_, i) => `<line x1="${x + i * 3}" y1="${y + 4}" x2="${x + i * 3 + 3}" y2="${y}" stroke-width="0.8"/>`).join('');

const GROUPS: [string, Item[]][] = [
  ['Select', [
    { tool: 'select', key: 'v', label: 'Select / move', icon: s('<path d="M7 4 L7 20 L11 16 L14 22 L16.5 21 L13.5 15 L19 15 Z" fill="#fff"/>'), help: 'Click to select, drag bodies to move. Drag empty space to pan, scroll to zoom.' },
  ]],
  ['Bodies', [
    { tool: 'ground', key: 'g', label: 'Ground', icon: s(`<line x1="3" y1="12" x2="23" y2="12" stroke-width="2"/>${hatch(3, 12, 20)}`), help: 'Click two points to draw a floor, wall or ceiling between them.' },
    { tool: 'ramp', key: 'w', label: 'Ramp / wedge', icon: s(`<path d="M3 20 L23 20 L23 7 Z" fill="#f3f4f6"/><line x1="3" y1="20" x2="23" y2="7" stroke-width="2"/>`), help: 'Click the bottom then the top of the slope to draw an incline. Tick "movable" to make it a sliding wedge.' },
    { tool: 'curve', key: 'u', label: 'Curved ramp', icon: s(`<path d="M3 20 L23 20 L23 5 Q23 20 3 20 Z" fill="#f3f4f6"/><path d="M3 20 Q23 20 23 5" stroke-width="2"/>`), help: 'Click the bottom then the top: a quarter-pipe curving from flat to steep. Switch it to a hump in its properties.' },
    { tool: 'point', key: 'm', label: 'Point mass', icon: s('<circle cx="13" cy="13" r="6" fill="#c4c4c4"/>'), help: 'Click to place a point mass.' },
    { tool: 'rect', key: 'b', label: 'Rigid body', icon: s('<rect x="4" y="9" width="18" height="9" fill="#dcdcdc"/><circle cx="13" cy="13.5" r="2" fill="#111"/>'), help: 'Click to place a rectangular rigid body (bar, cart, block).' },
    { tool: 'wheel', key: 'o', label: 'Wheel / ball', icon: s('<circle cx="13" cy="13" r="8" fill="#dcdcdc"/><circle cx="13" cy="13" r="1.6" fill="#111"/><line x1="13" y1="13" x2="13" y2="5"/>'), help: 'Click to place a free rolling disc. Give the ground some friction μ and it rolls instead of sliding.' },
    { tool: 'pulley', key: 'p', label: 'Pulley', icon: s('<circle cx="13" cy="14" r="7.5" fill="#dcdcdc"/><circle cx="13" cy="14" r="1.6" fill="#fff"/><line x1="13" y1="2" x2="13" y2="12"/>'), help: 'Click to place a pulley (fixed axle by default).' },
  ]],
  ['Joints', [
    { tool: 'pin', key: 'j', label: 'Pin joint', icon: s('<path d="M13 9 L7 19 L19 19 Z" fill="#fff"/><circle cx="13" cy="9" r="2.6" fill="#fff"/>'), help: 'Click a body: it is pinned to the body beneath, or to the ground.' },
    { tool: 'rod', key: 'l', label: 'Massless rod', icon: s('<line x1="5" y1="20" x2="21" y2="6" stroke-width="3"/><circle cx="5" cy="20" r="1.8" fill="#111"/><circle cx="21" cy="6" r="1.8" fill="#111"/>'), help: 'Click two points to connect them with a rigid massless rod.' },
    { tool: 'slider', key: 't', label: 'Slider / track', icon: s('<rect x="7" y="7" width="12" height="8" fill="#dcdcdc"/><circle cx="10" cy="17.5" r="2" fill="#fff"/><circle cx="16" cy="17.5" r="2" fill="#fff"/><line x1="2" y1="20" x2="24" y2="20" stroke-dasharray="2 2"/>'), help: 'Click on a body to put it on a straight track through that point.' },
  ]],
  ['Loads', [
    { tool: 'force', key: 'a', label: 'Applied force', icon: s('<rect x="13" y="8" width="10" height="10" fill="#dcdcdc"/><line x1="2" y1="13" x2="11" y2="13" stroke="#9f1239" stroke-width="2"/><path d="M13 13 L9 10.5 L9 15.5 Z" fill="#9f1239" stroke="#9f1239"/>'), help: 'Click a body to apply a constant force (and/or torque). Set its components in the properties panel.' },
  ]],
  ['Springs', [
    { tool: 'spring', key: 's', label: 'Linear spring', icon: s('<path d="M2 13h3c1.5 0 1.5-5 3-5s.5 10 2.5 10S11 8 13 8s.5 10 2.5 10S16 8 18 8s1.5 5 3 5h3"/>'), help: 'Click two points to attach a spring (with optional damping).' },
    { tool: 'tspring', key: 'q', label: 'Torsional spring', icon: s('<path d="M13 13 m0 -2 a2 2 0 1 1 -2 2 a4 4 0 0 1 4 -4 a6 6 0 0 1 6 6 a8 8 0 0 1 -8 8"/><circle cx="13" cy="13" r="1.5" fill="#fff"/>'), help: 'Click a pin joint to add a torsional spring.' },
  ]],
  ['Ropes', [
    { tool: 'rope', key: 'c', label: 'Rope', icon: s('<circle cx="13" cy="9" r="5" fill="#dcdcdc"/><path d="M8 9 L8 22 M18 9 L18 18"/><path d="M8 9 A5 5 0 0 1 18 9" stroke="#3b2f22" stroke-width="1.8"/>'), help: 'Click the first end, then each pulley in order, then the other end.' },
  ]],
];

export const ALL = GROUPS.flatMap(([, items]) => items);
export const TOOL_HELP = Object.fromEntries(ALL.map((i) => [i.tool, i.help])) as Record<Tool, string>;
export const TOOL_LABEL = Object.fromEntries(ALL.map((i) => [i.tool, i.label])) as Record<Tool, string>;
/** Single-key shortcuts (lower case) → tool. */
export const TOOL_KEYS: Record<string, Tool> = Object.fromEntries(ALL.map((i) => [i.key, i.tool]));

export function buildPalette(root: HTMLElement, onPick: (t: Tool) => void): (t: Tool) => void {
  root.innerHTML = '';
  const buttons = new Map<Tool, HTMLButtonElement>();
  for (const [title, items] of GROUPS) {
    const h = document.createElement('h4');
    h.textContent = title;
    root.appendChild(h);
    for (const it of items) {
      const b = document.createElement('button');
      b.className = 'tool';
      b.innerHTML = `${it.icon}<span>${it.label}</span><kbd>${it.key.toUpperCase()}</kbd>`;
      b.title = `${it.label} (${it.key.toUpperCase()}): ${it.help}`;
      b.draggable = it.tool !== 'select';
      b.addEventListener('click', () => onPick(it.tool));
      b.addEventListener('dragstart', (ev) => {
        ev.dataTransfer?.setData('text/x-tool', it.tool);
        ev.dataTransfer?.setData('text/plain', it.tool);
        if (ev.dataTransfer) ev.dataTransfer.effectAllowed = 'copy';
      });
      root.appendChild(b);
      buttons.set(it.tool, b);
    }
  }
  const help = document.createElement('div');
  help.className = 'palette-help';
  help.innerHTML = `
    <div><kbd>Space</kbd> play / pause</div>
    <div><kbd>R</kbd> reset</div>
    <div><kbd>Del</kbd> delete</div>
    <div><kbd>[</kbd> <kbd>]</kbd> rotate</div>
    <div><kbd>←↑→↓</kbd> nudge</div>
    <div><kbd>⌘Z</kbd> undo · <kbd>⌘D</kbd> duplicate</div>
    <div><kbd>F</kbd> fit view</div>
    <div class="muted-line">Drag empty space to pan, scroll to zoom. Right-click leaves a tool.</div>`;
  root.appendChild(help);
  return (active: Tool) => {
    for (const [t, b] of buttons) b.classList.toggle('active', t === active);
  };
}
