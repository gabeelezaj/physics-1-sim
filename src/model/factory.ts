// Element constructors with sensible defaults. Scripts (tests, presets) and the
// editor both build scenes through these.

import type {
  Attach, El, GroundEl, PinEl, PointEl, PulleyEl, RectEl, RodEl, RopeEl, RopeRoute,
  Scene, SliderEl, SpringEl, TSpringEl, Vec2,
} from './types';

let counter = 0;
export function newId(prefix: string): string {
  counter++;
  return `${prefix}${Date.now().toString(36).slice(-4)}${counter.toString(36)}`;
}

const PREFIX: Record<El['kind'], string> = {
  ground: 'G', point: 'm', rect: 'B', pulley: 'P', rod: 'R', pin: 'O',
  slider: 'S', spring: 'k', tspring: 'kt', rope: 'T',
};

/** Next free display name like "m3" for a kind in a scene. */
export function nextName(scene: Scene, kind: El['kind']): string {
  const p = PREFIX[kind];
  const used = new Set(scene.elements.map((e) => e.name));
  for (let i = 1; ; i++) if (!used.has(`${p}${i}`)) return `${p}${i}`;
}

export const at = (body: string | null, x: number, y: number): Attach => ({ body, p: [x, y] });

export function ground(name: string, x: number, y: number, o: Partial<GroundEl> = {}): GroundEl {
  return { id: newId('g'), kind: 'ground', name, x, y, theta: 0, w: 2, h: 0.12, ...o };
}

export function point(name: string, x: number, y: number, m: number, o: Partial<PointEl> = {}): PointEl {
  return { id: newId('p'), kind: 'point', name, x, y, vx: 0, vy: 0, m, radius: 0.06, ...o };
}

export function rect(name: string, x: number, y: number, m: number, w: number, h: number, o: Partial<RectEl> = {}): RectEl {
  return { id: newId('b'), kind: 'rect', name, x, y, theta: 0, vx: 0, vy: 0, omega: 0, m, w, h, I: null, ...o };
}

export function pulley(name: string, x: number, y: number, m: number, r: number, o: Partial<PulleyEl> = {}): PulleyEl {
  return { id: newId('u'), kind: 'pulley', name, x, y, theta: 0, vx: 0, vy: 0, omega: 0, m, r, I: null, fixedAxle: true, ...o };
}

export function rod(name: string, a: Attach, b: Attach, length: number): RodEl {
  return { id: newId('r'), kind: 'rod', name, a, b, length };
}

export function pin(name: string, a: Attach, b: Attach): PinEl {
  return { id: newId('o'), kind: 'pin', name, a, b };
}

export function slider(name: string, a: Attach, origin: Vec2, angle: number, o: Partial<SliderEl> = {}): SliderEl {
  return { id: newId('s'), kind: 'slider', name, a, origin, angle, mu: 0, lockRotation: true, extent: 1.5, ...o };
}

export function spring(name: string, a: Attach, b: Attach, k: number, L0: number, c = 0): SpringEl {
  return { id: newId('k'), kind: 'spring', name, a, b, k, L0, c };
}

export function tspring(name: string, pinId: string, k: number, theta0: number, c = 0): TSpringEl {
  return { id: newId('t'), kind: 'tspring', name, pin: pinId, k, theta0, c };
}

export function rope(name: string, a: Attach, b: Attach, route: RopeRoute[], length: number): RopeEl {
  return { id: newId('T'), kind: 'rope', name, a, b, route, length };
}
