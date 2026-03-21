#!/usr/bin/env node
// Generates sensor_fusion_preview.gif
// Usage: node scripts/sensor-fusion-preview.js
// Output: sensor_fusion_preview.gif (800x450 16:9, ~9s loop at 20fps)

const { createCanvas } = require('canvas');
const GifEncoder = require('gif-encoder-2');
const fs = require('fs');
const path = require('path');

const W = 800, H = 450;
const FPS = 20;
const LOOP_T = 9;
const TOTAL_FRAMES = Math.ceil(FPS * LOOP_T);

const TREE_SIG  = 0.055;
const ALT_M     = 150;
const TREE_H_M  = 42;
const GPS_SIG   = 2;
const LAS_SIG   = 12;
const MAX_HIST  = 90;
const READ_DT   = 0.10;
const TREES = [
  {nx:0.40,rh:0.65},{nx:0.46,rh:0.82},{nx:0.52,rh:0.95},
  {nx:0.57,rh:1.00},{nx:0.62,rh:0.88},{nx:0.67,rh:0.72},{nx:0.73,rh:0.55}
];

const canvas = createCanvas(W, H);
const ctx    = canvas.getContext('2d');

let lcgState = 42;
function lcgRand() {
  lcgState = (lcgState * 1664525 + 1013904223) >>> 0;
  return lcgState / 0xffffffff;
}
function gauss(s, u, v) {
  return s * Math.sqrt(-2 * Math.log(Math.max(u, 1e-10))) * Math.cos(2 * Math.PI * v);
}
function treeH(nx) {
  let best = 0;
  for (const t of TREES) {
    const d = nx - t.nx;
    const h = TREE_H_M * t.rh * Math.exp(-d * d / (TREE_SIG * TREE_SIG));
    if (h > best) best = h;
  }
  return best;
}

// Pre-generate readings deterministically
const readings = [];
{
  let lastRead = -999;
  const gH = [], lH = [];
  for (let frame = 0; frame < TOTAL_FRAMES; frame++) {
    const t = frame / FPS;
    const rawT = t % LOOP_T;
    const apxN = (rawT / LOOP_T) * 1.3 - 0.15;
    if (t - lastRead >= READ_DT) {
      lastRead = t;
      if (rawT < READ_DT * 2) { gH.length = 0; lH.length = 0; }
      const th = treeH(apxN);
      gH.push(ALT_M + gauss(GPS_SIG, lcgRand(), lcgRand()));
      lH.push(ALT_M - th + gauss(LAS_SIG, lcgRand(), lcgRand()));
      if (gH.length > MAX_HIST) gH.shift();
      if (lH.length > MAX_HIST) lH.shift();
    }
    readings.push({ gpsH: [...gH], lasH: [...lH] });
  }
}

// ── Tree helpers ──────────────────────────────────────────────────────────────

function treeTopY(x, groundY) {
  let best = groundY;
  for (const tr of TREES) {
    const cx = tr.nx * W, tH = H * 0.21 * tr.rh;
    const trunkH = tH * 0.30, cH = tH * 0.80, cW = W * 0.036;
    const cbY = groundY - trunkH, ctY = groundY - trunkH - cH;
    const dx = Math.abs(x - cx);
    if (dx < cW) { const sy = ctY + (cbY - ctY) * (dx / cW); if (sy < best) best = sy; }
  }
  return best;
}

function drawForest(groundY) {
  for (const tr of TREES) {
    const cx = tr.nx * W, tH = H * 0.21 * tr.rh;
    const trunkH = tH * 0.30, trunkW = W * 0.007;
    const cH = tH * 0.80, cW = W * 0.036;
    ctx.fillStyle = '#5a3820';
    ctx.fillRect(cx - trunkW, groundY - trunkH, trunkW * 2, trunkH);
    ctx.fillStyle = 'rgba(30,80,44,0.92)';
    ctx.beginPath();
    ctx.moveTo(cx, groundY - trunkH - cH);
    ctx.lineTo(cx + cW, groundY - trunkH);
    ctx.lineTo(cx - cW, groundY - trunkH);
    ctx.closePath(); ctx.fill();
    ctx.strokeStyle = 'rgba(62,207,142,0.15)'; ctx.lineWidth = 0.7; ctx.stroke();
  }
}

// ── Plane ─────────────────────────────────────────────────────────────────────

function drawPlane(px, py) {
  const s = H * 0.038;
  const f1 = '#4a9eff', f2 = 'rgba(74,158,255,0.75)', ol = 'rgba(200,230,255,0.2)';
  ctx.save(); ctx.translate(px, py);
  // wing
  ctx.fillStyle = f2; ctx.strokeStyle = ol; ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(s*0.6,s*0.6); ctx.lineTo(-s*0.3,s*0.65);
  ctx.bezierCurveTo(-s*1.2,s*0.9,-s*2.2,s*2.8,-s*2.0,s*3.2);
  ctx.bezierCurveTo(-s*1.8,s*3.5,-s*0.8,s*3.5,-s*0.4,s*3.1);
  ctx.bezierCurveTo(s*0.1,s*2.5,s*0.5,s*1.5,s*0.5,s*0.9);
  ctx.closePath(); ctx.fill(); ctx.stroke();
  // wheel fairing
  ctx.beginPath(); ctx.ellipse(-s*0.8,s*3.25,s*0.52,s*0.20,0,0,Math.PI*2); ctx.fill(); ctx.stroke();
  // fuselage
  ctx.fillStyle = f1; ctx.strokeStyle = ol; ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(s*3.0,-s*0.35);
  ctx.bezierCurveTo(s*3.7,-s*0.35,s*3.7,s*0.65,s*3.0,s*0.65);
  ctx.bezierCurveTo(s*2.0,s*0.75,s*0.8,s*0.75,-s*0.5,s*0.65);
  ctx.bezierCurveTo(-s*1.5,s*0.55,-s*2.5,s*0.35,-s*3.0,s*0.15);
  ctx.bezierCurveTo(-s*3.5,0,-s*3.5,-s*0.6,-s*3.0,-s*0.65);
  ctx.bezierCurveTo(-s*2.0,-s*0.95,-s*0.5,-s*1.05,s*1.0,-s*1.0);
  ctx.bezierCurveTo(s*2.0,-s*0.95,s*2.8,-s*0.6,s*3.0,-s*0.35);
  ctx.closePath(); ctx.fill(); ctx.stroke();
  // vertical tail fin
  ctx.fillStyle = f2;
  ctx.beginPath();
  ctx.moveTo(-s*2.0,-s*0.65);
  ctx.bezierCurveTo(-s*2.2,-s*1.5,-s*2.6,-s*2.1,-s*2.9,-s*2.0);
  ctx.bezierCurveTo(-s*3.1,-s*1.8,-s*3.1,-s*1.0,-s*3.0,-s*0.65);
  ctx.closePath(); ctx.fill(); ctx.stroke();
  // propeller hub
  ctx.fillStyle = f1; ctx.strokeStyle = ol; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.arc(s*3.5,s*0.15,s*0.22,0,Math.PI*2); ctx.fill(); ctx.stroke();
  // propeller blades
  ctx.fillStyle = f2; ctx.strokeStyle = ol; ctx.lineWidth = 1;
  ctx.save(); ctx.translate(s*3.5, s*0.15);
  ctx.beginPath(); ctx.ellipse(0,-s*1.05,s*0.20,s*0.95,0.15,0,Math.PI*2); ctx.fill(); ctx.stroke();
  ctx.beginPath(); ctx.ellipse(0,s*1.05,s*0.20,s*0.95,-0.15,0,Math.PI*2); ctx.fill(); ctx.stroke();
  ctx.restore();
  // GPS antenna (yellow)
  ctx.fillStyle = 'rgba(240,200,0,0.95)';
  ctx.fillRect(s*0.2,-s*1.05,s*0.16,s*0.65);
  ctx.beginPath(); ctx.arc(s*0.28,-s*1.05,s*0.22,Math.PI,0); ctx.fill();
  // laser emitter (red)
  ctx.fillStyle = 'rgba(232,69,69,0.9)';
  ctx.beginPath();
  ctx.moveTo(-s*0.55,s*0.65); ctx.lineTo(s*0.10,s*1.45); ctx.lineTo(s*0.75,s*0.65);
  ctx.closePath(); ctx.fill();
  ctx.restore();
}

// ── Beams ─────────────────────────────────────────────────────────────────────

function drawBeams(px, py, groundY) {
  const N = 7, spread = W * 0.09;
  for (let b = 0; b < N; b++) {
    const endX = px + (b / (N - 1) - 0.5) * 2 * spread;
    const endY = treeTopY(endX, groundY);
    const grad = ctx.createLinearGradient(px, py + H * 0.040, endX, endY);
    grad.addColorStop(0, 'rgba(232,69,69,0.55)');
    grad.addColorStop(0.7, 'rgba(232,69,69,0.15)');
    grad.addColorStop(1, 'rgba(232,69,69,0.0)');
    ctx.save(); ctx.setLineDash([5, 5]);
    ctx.strokeStyle = grad; ctx.lineWidth = (b === 3) ? 1.8 : 0.9;
    ctx.beginPath(); ctx.moveTo(px, py + H * 0.040); ctx.lineTo(endX, endY); ctx.stroke();
    ctx.restore();
    ctx.beginPath(); ctx.arc(endX, endY, 2.5, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(232,69,69,0.5)'; ctx.fill();
  }
}

// ── Graph ─────────────────────────────────────────────────────────────────────

function drawGraph(gx, gy, gw, gh, hist, label, color, yLo, yHi) {
  ctx.fillStyle = 'rgba(8,10,15,0.88)';
  ctx.beginPath(); ctx.rect(gx, gy, gw, gh); ctx.fill();
  ctx.strokeStyle = 'rgba(42,55,75,0.9)'; ctx.lineWidth = 1; ctx.stroke();
  const mx = 14, my = 32, mr = 10, mb = 18;
  const pw = gw - mx - mr, ph = gh - my - mb;
  ctx.strokeStyle = 'rgba(42,55,75,0.4)'; ctx.lineWidth = 0.5;
  for (let g = 0; g <= 3; g++) {
    const gy2 = gy + my + (g / 3) * ph;
    ctx.beginPath(); ctx.moveTo(gx + mx, gy2); ctx.lineTo(gx + mx + pw, gy2); ctx.stroke();
  }
  ctx.font = 'bold 16px monospace';
  ctx.fillStyle = color; ctx.textAlign = 'left'; ctx.textBaseline = 'top';
  ctx.fillText(label, gx + mx, gy + 7);
  ctx.font = '13px monospace';
  ctx.fillStyle = 'rgba(255,255,255,0.2)'; ctx.textAlign = 'right';
  ctx.fillText(yHi + 'm', gx + mx - 2, gy + my);
  ctx.fillText(yLo + 'm', gx + mx - 2, gy + my + ph - 4);
  if (hist.length < 2) return;
  ctx.strokeStyle = color; ctx.lineWidth = 1.5;
  ctx.beginPath();
  for (let i = 0; i < hist.length; i++) {
    const hx2 = gx + mx + (i / (MAX_HIST - 1)) * pw;
    let hy2 = gy + my + ph - ((hist[i] - yLo) / (yHi - yLo)) * ph;
    hy2 = Math.max(gy + my, Math.min(gy + my + ph, hy2));
    if (i === 0) ctx.moveTo(hx2, hy2); else ctx.lineTo(hx2, hy2);
  }
  ctx.stroke();
  ctx.fillStyle = color;
  for (let i = 0; i < hist.length; i++) {
    const hx2 = gx + mx + (i / (MAX_HIST - 1)) * pw;
    let hy2 = gy + my + ph - ((hist[i] - yLo) / (yHi - yLo)) * ph;
    hy2 = Math.max(gy + my, Math.min(gy + my + ph, hy2));
    ctx.beginPath(); ctx.arc(hx2, hy2, 1.8, 0, Math.PI * 2); ctx.fill();
  }
  if (hist.length > 0) {
    ctx.font = 'bold 14px monospace';
    ctx.fillStyle = 'rgba(255,255,255,0.55)'; ctx.textAlign = 'right'; ctx.textBaseline = 'top';
    ctx.fillText(hist[hist.length-1].toFixed(0) + 'm', gx + gw - 6, gy + 7);
  }
  ctx.textBaseline = 'alphabetic';
}

function drawArrow(x1, y1, x2, y2, col) {
  const dx = x2 - x1, dy = y2 - y1, len = Math.hypot(dx, dy);
  if (len < 4) return;
  const ux = dx / len, uy = dy / len;
  ctx.save(); ctx.setLineDash([4, 4]);
  ctx.strokeStyle = col; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
  ctx.restore();
  ctx.fillStyle = col;
  ctx.beginPath();
  ctx.moveTo(x2, y2);
  ctx.lineTo(x2 - ux*7 + uy*3.5, y2 - uy*7 - ux*3.5);
  ctx.lineTo(x2 - ux*7 - uy*3.5, y2 - uy*7 + ux*3.5);
  ctx.closePath(); ctx.fill();
}

// ── Main draw loop ────────────────────────────────────────────────────────────

function drawFrame(frame) {
  const t = frame / FPS;
  const rawT = t % LOOP_T;
  const worldX = rawT / LOOP_T;
  const apx = (worldX * 1.3 - 0.15) * W;
  const groundY = H * 0.73;
  const apy = groundY - (ALT_M / (ALT_M + 30)) * (groundY - H * 0.10);

  const { gpsH, lasH } = readings[frame];

  ctx.clearRect(0, 0, W, H);

  const sky = ctx.createLinearGradient(0, 0, 0, groundY);
  sky.addColorStop(0, '#040710'); sky.addColorStop(1, '#0a1422');
  ctx.fillStyle = sky; ctx.fillRect(0, 0, W, groundY + 2);

  ctx.fillStyle = '#0d1218'; ctx.fillRect(0, groundY, W, H - groundY);
  ctx.strokeStyle = 'rgba(30,42,56,0.5)'; ctx.lineWidth = 1;
  const gsp = W * 0.055, goff = (t * gsp * 0.35) % gsp;
  for (let gx2 = -gsp + goff; gx2 < W + gsp; gx2 += gsp) {
    ctx.beginPath(); ctx.moveTo(gx2, groundY); ctx.lineTo(gx2 - 22, H); ctx.stroke();
  }
  ctx.strokeStyle = '#222a36'; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(0, groundY); ctx.lineTo(W, groundY); ctx.stroke();
  ctx.font = '14px monospace';
  ctx.fillStyle = 'rgba(74,158,255,0.3)'; ctx.textAlign = 'left'; ctx.textBaseline = 'alphabetic';
  ctx.fillText('sea level (0 m)', 8, groundY - 6);

  ctx.save(); ctx.setLineDash([6, 8]);
  ctx.strokeStyle = 'rgba(74,158,255,0.08)'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(0, apy); ctx.lineTo(W, apy); ctx.stroke();
  ctx.restore();

  drawForest(groundY);
  if (apx > -60 && apx < W + 60) drawBeams(apx, apy, groundY);
  if (apx > -80 && apx < W + 80) drawPlane(apx, apy);

  // Corner sensor graphs
  const GW = W * 0.27, GH = H * 0.30;
  const yLo = 80, yHi = 190;
  const gpsGX = W * 0.015, gpsGY = H * 0.035;
  const lasGX = W * 0.985 - GW, lasGY = H * 0.035;
  drawGraph(gpsGX, gpsGY, GW, GH, gpsH, 'GPS altitude', 'rgba(240,144,64,0.85)', yLo, yHi);
  drawGraph(lasGX, lasGY, GW, GH, lasH, 'Laser altimeter', 'rgba(232,69,69,0.85)', yLo, yHi);

  if (apx > W * 0.15 && apx < W * 0.85) {
    drawArrow(gpsGX + GW, gpsGY + GH * 0.5, apx + H*0.014, apy - H*0.083,
              'rgba(240,144,64,0.55)');
    drawArrow(lasGX, lasGY + GH * 0.5, apx + H*0.006, apy + H*0.040,
              'rgba(232,69,69,0.45)');
  }

  if (apx > W * 0.05 && apx < W * 0.90) {
    const lineX = apx - H * 0.06;
    ctx.save(); ctx.setLineDash([3, 3]);
    ctx.strokeStyle = 'rgba(240,192,64,0.25)'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(lineX, apy); ctx.lineTo(lineX, groundY); ctx.stroke();
    ctx.restore();
    ctx.font = 'bold 16px monospace';
    ctx.fillStyle = 'rgba(240,192,64,0.5)'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(ALT_M + 'm', lineX - 22, (apy + groundY) / 2);
    ctx.textBaseline = 'alphabetic';
  }

  ctx.textBaseline = 'alphabetic';
}

// ── GIF encoding ──────────────────────────────────────────────────────────────

const encoder = new GifEncoder(W, H, 'neuquant', true);
encoder.setDelay(Math.round(1000 / FPS));
encoder.setRepeat(0);
encoder.setQuality(10);

const outPath = path.join(__dirname, '..', 'sensor_fusion_preview.gif');
const stream  = fs.createWriteStream(outPath);
encoder.createReadStream().pipe(stream);
encoder.start();

console.log(`Rendering ${TOTAL_FRAMES} frames at ${FPS}fps (${LOOP_T}s)...`);

for (let frame = 0; frame < TOTAL_FRAMES; frame++) {
  drawFrame(frame);
  encoder.addFrame(ctx);
  if (frame % FPS === 0) process.stdout.write(`  ${(frame / FPS).toFixed(1)}s / ${LOOP_T}s\r`);
}

encoder.finish();
stream.on('finish', () => {
  const size = (fs.statSync(outPath).size / 1024).toFixed(0);
  console.log(`\nDone! ${outPath} (${size} KB)`);
});
