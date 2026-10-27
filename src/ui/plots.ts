// Minimal time-series plot on a canvas: auto-scaled y, nice ticks, light grid.

export interface Series {
  label: string;
  color: string;
  data: ArrayLike<number>;
}

export const PALETTE = ['#2563eb', '#dc2626', '#059669', '#d97706', '#7c3aed', '#0891b2', '#db2777', '#4b5563'];

function niceStep(range: number, target: number): number {
  const raw = range / Math.max(1, target);
  const p = 10 ** Math.floor(Math.log10(raw || 1));
  const f = raw / p;
  return (f < 1.5 ? 1 : f < 3.5 ? 2 : f < 7.5 ? 5 : 10) * p;
}

const tickLabel = (v: number, step: number) => {
  if (Math.abs(v) < step * 1e-6) return '0';
  if (step < 1e-3 || step >= 1e5) return v.toExponential(1);
  const d = Math.max(0, -Math.floor(Math.log10(step) + 1e-9));
  return Math.abs(v) < step * 1e-6 ? '0' : v.toFixed(Math.min(6, d));
};

export class Plot {
  ctx: CanvasRenderingContext2D;
  constructor(public canvas: HTMLCanvasElement, public yLabel = '') {
    this.ctx = canvas.getContext('2d')!;
  }

  draw(t: ArrayLike<number>, series: Series[]): void {
    const c = this.canvas;
    const dpr = window.devicePixelRatio || 1;
    const W = c.clientWidth, H = c.clientHeight;
    if (!W || !H) return;
    if (c.width !== Math.round(W * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(H * dpr); }
    const ctx = this.ctx;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = '#fff';
    ctx.fillRect(0, 0, W, H);
    const L = 52, R = 8, T = 8, B = 22;
    const pw = W - L - R, ph = H - T - B;
    const n = t.length;
    const t0 = n ? t[0] : 0;
    const t1 = n ? Math.max(t[n - 1], t0 + 1) : 1;
    let ymin = Infinity, ymax = -Infinity;
    for (const s of series) for (let i = 0; i < s.data.length; i++) {
      const v = s.data[i];
      if (v < ymin) ymin = v;
      if (v > ymax) ymax = v;
    }
    if (!Number.isFinite(ymin)) { ymin = -1; ymax = 1; }
    const span = ymax - ymin;
    const pad = span > 1e-12 ? span * 0.08 : Math.max(Math.abs(ymax) * 0.1, 1e-3);
    ymin -= pad; ymax += pad;
    const X = (v: number) => L + ((v - t0) / (t1 - t0)) * pw;
    const Y = (v: number) => T + (1 - (v - ymin) / (ymax - ymin)) * ph;

    ctx.font = '10px ui-sans-serif, system-ui, sans-serif';
    ctx.fillStyle = '#667085';
    ctx.strokeStyle = '#eef0f3';
    ctx.lineWidth = 1;
    const ys = niceStep(ymax - ymin, 4);
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (let v = Math.ceil(ymin / ys) * ys; v <= ymax; v += ys) {
      const y = Math.round(Y(v)) + 0.5;
      ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(L + pw, y); ctx.stroke();
      ctx.fillText(tickLabel(v, ys), L - 5, y);
    }
    const xs = niceStep(t1 - t0, 5);
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    for (let v = Math.ceil(t0 / xs) * xs; v <= t1; v += xs) {
      const x = Math.round(X(v)) + 0.5;
      ctx.beginPath(); ctx.moveTo(x, T); ctx.lineTo(x, T + ph); ctx.stroke();
      ctx.fillText(tickLabel(v, xs), x, T + ph + 5);
    }
    ctx.textAlign = 'right';
    ctx.fillText('t [s]', L + pw, T + ph + 5 + 0);
    ctx.strokeStyle = '#cfd4dc';
    ctx.strokeRect(L + 0.5, T + 0.5, pw, ph);
    if (this.yLabel) {
      ctx.save();
      ctx.translate(11, T + ph / 2);
      ctx.rotate(-Math.PI / 2);
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(this.yLabel, 0, 0);
      ctx.restore();
    }

    if (n < 2) {
      ctx.fillStyle = '#98a2b3';
      ctx.font = '12px ui-sans-serif, system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText('Press Play to record', L + pw / 2, T + ph / 2);
      return;
    }
    ctx.save();
    ctx.beginPath();
    ctx.rect(L, T, pw, ph);
    ctx.clip();
    // decimate to about one point per pixel column
    const stride = Math.max(1, Math.floor(n / (pw * 2)));
    for (const s of series) {
      ctx.strokeStyle = s.color;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      for (let i = 0; i < n; i += stride) {
        const x = X(t[i]), y = Y(s.data[i]);
        i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
      }
      if (n) ctx.lineTo(X(t[n - 1]), Y(s.data[n - 1]));
      ctx.stroke();
    }
    ctx.restore();
  }
}
