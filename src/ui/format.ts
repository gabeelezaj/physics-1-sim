// Human-friendly number formatting for the UI.

const PREFIX: [number, string][] = [[1e9, 'G'], [1e6, 'M'], [1e3, 'k'], [1, ''], [1e-3, 'm'], [1e-6, 'µ']];

/** 3 significant figures with an SI prefix: 154000 N → "154 kN". */
export function si(v: number, unit: string, sig = 3): string {
  if (!Number.isFinite(v)) return `— ${unit}`;
  const a = Math.abs(v);
  if (a < 1e-9) return `0 ${unit}`;
  for (const [f, p] of PREFIX) {
    if (a >= f * 0.9995) return `${Number((v / f).toPrecision(sig))} ${p}${unit}`;
  }
  return `${v.toExponential(sig - 1)} ${unit}`;
}

/** Plain number with `sig` significant figures, tiny values shown as 0. */
export function num(v: number, sig = 4): string {
  if (!Number.isFinite(v)) return '—';
  if (Math.abs(v) < 1e-12) return '0';
  return Number(v.toPrecision(sig)).toString();
}

export const deg = (rad: number) => `${num((rad * 180) / Math.PI, 4)}°`;
