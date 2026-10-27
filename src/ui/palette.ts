// Element palette: textbook-style icons, click to arm a tool or drag onto the canvas.

export type Tool =
  | 'select' | 'ground' | 'point' | 'rect' | 'pulley'
  | 'pin' | 'rod' | 'slider' | 'spring' | 'tspring' | 'rope';

interface Item {
  tool: Tool;
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
    { tool: 'select', label: 'Select / move', icon: s('<path d="M7 4 L7 20 L11 16 L14 22 L16.5 21 L13.5 15 L19 15 Z" fill="#fff"/>'), help: 'Click to select, drag bodies to move. Drag empty space to pan, scroll to zoom.' },
  ]],
  ['Bodies', [
    { tool: 'ground', label: 'Ground', icon: s(`<line x1="3" y1="12" x2="23" y2="12" stroke-width="2"/>${hatch(3, 12, 20)}`), help: 'Click to place a fixed ground/wall. Rotate it in the properties panel.' },
    { tool: 'point', label: 'Point mass', icon: s('<circle cx="13" cy="13" r="6" fill="#c4c4c4"/>'), help: 'Click to place a point mass.' },
    { tool: 'rect', label: 'Rigid body', icon: s('<rect x="4" y="9" width="18" height="9" fill="#dcdcdc"/><circle cx="13" cy="13.5" r="2" fill="#111"/>'), help: 'Click to place a rectangular rigid body (bar, cart, block).' },
    { tool: 'pulley', label: 'Pulley', icon: s('<circle cx="13" cy="14" r="7.5" fill="#dcdcdc"/><circle cx="13" cy="14" r="1.6" fill="#fff"/><line x1="13" y1="2" x2="13" y2="12"/>'), help: 'Click to place a pulley (fixed axle by default).' },
  ]],
  ['Joints', [
    { tool: 'pin', label: 'Pin joint', icon: s('<path d="M13 9 L7 19 L19 19 Z" fill="#fff"/><circle cx="13" cy="9" r="2.6" fill="#fff"/>'), help: 'Click on a body: pins it to the body beneath, or to the ground.' },
    { tool: 'rod', label: 'Massless rod', icon: s('<line x1="5" y1="20" x2="21" y2="6" stroke-width="3"/><circle cx="5" cy="20" r="1.8" fill="#111"/><circle cx="21" cy="6" r="1.8" fill="#111"/>'), help: 'Click two points to connect them with a rigid massless rod.' },
    { tool: 'slider', label: 'Slider / track', icon: s('<rect x="7" y="7" width="12" height="8" fill="#dcdcdc"/><circle cx="10" cy="17.5" r="2" fill="#fff"/><circle cx="16" cy="17.5" r="2" fill="#fff"/><line x1="2" y1="20" x2="24" y2="20" stroke-dasharray="2 2"/>'), help: 'Click on a body to put it on a straight track through that point.' },
  ]],
  ['Springs', [
    { tool: 'spring', label: 'Linear spring', icon: s('<polyline points="2,13 6,13 8,8 11,18 14,8 17,18 20,13 24,13"/>'), help: 'Click two points to attach a spring (with optional damping).' },
    { tool: 'tspring', label: 'Torsional spring', icon: s('<path d="M13 13 m0 -2 a2 2 0 1 1 -2 2 a4 4 0 0 1 4 -4 a6 6 0 0 1 6 6 a8 8 0 0 1 -8 8"/><circle cx="13" cy="13" r="1.5" fill="#fff"/>'), help: 'Click a pin joint to add a torsional spring.' },
  ]],
  ['Ropes', [
    { tool: 'rope', label: 'Rope', icon: s('<circle cx="13" cy="9" r="5" fill="#dcdcdc"/><path d="M8 9 L8 22 M18 9 L18 18"/><path d="M8 9 A5 5 0 0 1 18 9" stroke="#3b2f22" stroke-width="1.8"/>'), help: 'Click the first end, then each pulley in order, then the other end.' },
  ]],
];

export const TOOL_HELP: Record<Tool, string> = Object.fromEntries(
  GROUPS.flatMap(([, items]) => items.map((i) => [i.tool, i.help])),
) as Record<Tool, string>;

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
      b.innerHTML = `${it.icon}<span>${it.label}</span>`;
      b.title = it.help;
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
  help.innerHTML = 'Drag an item onto the canvas or click it, then click in the scene.<br><br><b>Del</b> delete · <b>Esc</b> cancel<br><b>Space</b> play/pause · <b>R</b> reset<br><b>[ ]</b> rotate · <b>arrows</b> nudge<br><b>Alt-drag</b> or drag empty space to pan';
  root.appendChild(help);
  return (active: Tool) => {
    for (const [t, b] of buttons) b.classList.toggle('active', t === active);
  };
}
