#!/usr/bin/env node
// Generates drone_interception_preview.gif from the latency simulator animation.
// Usage: node scripts/drone-interception-preview.js
// Output: drone_interception_preview.gif (800x450, ~5s loop at 20fps)

const { createCanvas } = require('canvas');
const GifEncoder = require('gif-encoder-2');
const fs = require('fs');
const path = require('path');

const W = 800, H = 450;
const FPS = 20;
const DURATION_SEC = 5; // one clean loop
const TOTAL_FRAMES = FPS * DURATION_SEC;

// Fixed simulation params (default slider values)
const SPEED_MS  = 20;   // m/s
const LAT_MS    = 100;  // ms
const GAP_M     = SPEED_MS * LAT_MS / 1000; // 2.0 m

const canvas = createCanvas(W, H);
const ctx    = canvas.getContext('2d');

// Polyfill roundRect if needed (node-canvas may lack it on older builds)
if (!ctx.roundRect) {
  ctx.roundRect = function(x, y, w, h, r) {
    this.beginPath();
    this.moveTo(x + r, y);
    this.lineTo(x + w - r, y);
    this.quadraticCurveTo(x + w, y, x + w, y + r);
    this.lineTo(x + w, y + h - r);
    this.quadraticCurveTo(x + w, y + h, x + w - r, y + h);
    this.lineTo(x + r, y + h);
    this.quadraticCurveTo(x, y + h, x, y + h - r);
    this.lineTo(x, y + r);
    this.quadraticCurveTo(x, y, x + r, y);
    this.closePath();
  };
}

const encoder = new GifEncoder(W, H, 'neuquant', true);
encoder.setDelay(Math.round(1000 / FPS));
encoder.setRepeat(0); // loop forever
encoder.setQuality(10);

const outPath = path.join(__dirname, '..', 'drone_interception_preview.gif');
const stream  = fs.createWriteStream(outPath);
encoder.createReadStream().pipe(stream);
encoder.start();

// Packet state (shared across frames, mirrors the browser animation)
let packets = [], lastPkt = 0;

function drawFrame(t) {
  ctx.clearRect(0, 0, W, H);

  const dt      = 1 / FPS;
  const groundY = H * 0.62, skyY = H * 0.22;
  const radarX  = W * 0.5,  radarY = groundY + 12;
  const intBaseX = W * 0.18, intBaseY = groundY;

  const m2px      = (W * 0.45) / 15;
  const gapPx     = GAP_M * m2px;
  const vspd      = W / 7;
  const totalTravel = W * 0.7 + gapPx + 60;
  const rawX      = (t * vspd) % totalTravel;
  const actualX   = W * 0.85 - rawX;
  const percX     = actualX + gapPx;

  // Sky background
  ctx.fillStyle = '#0a0c10';
  ctx.fillRect(0, 0, W, groundY);

  // Ground
  ctx.fillStyle = '#111318';
  ctx.fillRect(0, groundY, W, H - groundY);
  ctx.strokeStyle = '#252a35'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(0, groundY); ctx.lineTo(W, groundY); ctx.stroke();
  ctx.strokeStyle = 'rgba(42,48,64,0.4)'; ctx.lineWidth = 0.7;
  for (let i = 0; i < W + 40; i += 10) {
    ctx.beginPath(); ctx.moveTo(i, groundY); ctx.lineTo(i - 14, groundY + 14); ctx.stroke();
  }

  // Radar
  const rW = 20, rH = 28;
  ctx.fillStyle = '#1e2230'; ctx.strokeStyle = '#3a4258'; ctx.lineWidth = 1.5;
  ctx.roundRect(radarX - rW / 2, radarY - rH, rW, rH, 3);
  ctx.fill(); ctx.stroke();
  ctx.strokeStyle = '#4a5570'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(radarX, radarY - rH); ctx.lineTo(radarX, radarY - rH - 12); ctx.stroke();
  ctx.beginPath(); ctx.arc(radarX, radarY - rH - 12, 8, Math.PI * 1.15, Math.PI * 1.85);
  ctx.strokeStyle = '#5a6580'; ctx.lineWidth = 2; ctx.stroke();
  ctx.font = '10px monospace'; ctx.fillStyle = 'rgba(255,255,255,0.35)'; ctx.textAlign = 'center';
  ctx.fillText('RADAR', radarX, radarY + 16);

  // Interceptor
  const iW = 32, iH = 14;
  ctx.fillStyle = 'rgba(74,158,255,0.15)'; ctx.strokeStyle = 'rgba(74,158,255,0.5)'; ctx.lineWidth = 1.5;
  ctx.roundRect(intBaseX - iW / 2, intBaseY - iH - 2, iW, iH, 3);
  ctx.fill(); ctx.stroke();
  const ps = 6 * Math.sin(t * 12);
  ctx.strokeStyle = 'rgba(74,158,255,0.4)'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(intBaseX - 10 - ps, intBaseY - iH - 4); ctx.lineTo(intBaseX - 10 + ps, intBaseY - iH - 4); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(intBaseX + 10 - ps, intBaseY - iH - 4); ctx.lineTo(intBaseX + 10 + ps, intBaseY - iH - 4); ctx.stroke();
  ctx.font = '10px monospace'; ctx.fillStyle = 'rgba(74,158,255,0.6)'; ctx.textAlign = 'center';
  ctx.fillText('INTERCEPTOR', intBaseX, intBaseY + 16);

  // Packets
  if (t - lastPkt > 0.5) { lastPkt = t; packets.push({ born: t, progress: 0 }); }
  const radarTop  = { x: radarX, y: radarY - rH - 20 };
  const percVisible = percX > 0 && percX < W;
  const percPt    = { x: percVisible ? percX : radarX, y: percVisible ? skyY : radarTop.y - 30 };
  const intPt     = { x: intBaseX, y: intBaseY - iH - 2 };

  for (let i = packets.length - 1; i >= 0; i--) {
    const p = packets[i];
    p.progress += 2.2 * dt;
    let px, py, op;
    if (p.progress < 1) {
      const f = p.progress;
      px = radarTop.x + (percPt.x - radarTop.x) * f;
      py = radarTop.y + (percPt.y - radarTop.y) * f;
      op = 0.7;
    } else if (p.progress < 2) {
      const f = p.progress - 1;
      px = percPt.x + (radarTop.x - percPt.x) * f;
      py = percPt.y + (radarTop.y - percPt.y) * f;
      op = 0.6;
    } else if (p.progress < 3) {
      const f = p.progress - 2;
      px = radarTop.x + (intPt.x - radarTop.x) * f;
      py = radarTop.y + (intPt.y - radarTop.y) * f;
      op = 0.5;
    } else {
      packets.splice(i, 1);
      continue;
    }
    ctx.beginPath(); ctx.arc(px, py, 3, 0, Math.PI * 2);
    ctx.fillStyle = `rgba(240,192,64,${op})`; ctx.fill();
    ctx.beginPath(); ctx.arc(px, py, 7, 0, Math.PI * 2);
    ctx.fillStyle = `rgba(240,192,64,${op * 0.15})`; ctx.fill();
  }

  // Dashed data lines
  if (percVisible) {
    ctx.strokeStyle = 'rgba(240,192,64,0.15)'; ctx.lineWidth = 1; ctx.setLineDash([4, 6]);
    ctx.beginPath(); ctx.moveTo(radarTop.x, radarTop.y); ctx.lineTo(percX, skyY); ctx.stroke(); ctx.setLineDash([]);
  }
  ctx.strokeStyle = 'rgba(240,192,64,0.12)'; ctx.lineWidth = 1; ctx.setLineDash([4, 6]);
  ctx.beginPath(); ctx.moveTo(radarTop.x, radarTop.y); ctx.lineTo(intBaseX, intBaseY - iH - 2); ctx.stroke(); ctx.setLineDash([]);

  // Gap annotation
  const bothVis = actualX > 20 && actualX < W - 20 && percX > 20 && percX < W - 20;
  if (bothVis && gapPx > 6) {
    ctx.fillStyle = 'rgba(240,192,64,0.04)'; ctx.fillRect(actualX, skyY - 28, gapPx, 56);
    const aY = skyY - 36;
    ctx.strokeStyle = 'rgba(240,192,64,0.5)'; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(actualX + 2, aY); ctx.lineTo(percX - 2, aY); ctx.stroke();
    ctx.fillStyle = 'rgba(240,192,64,0.5)';
    ctx.beginPath(); ctx.moveTo(actualX + 2, aY); ctx.lineTo(actualX + 8, aY - 3); ctx.lineTo(actualX + 8, aY + 3); ctx.fill();
    ctx.beginPath(); ctx.moveTo(percX - 2, aY); ctx.lineTo(percX - 8, aY - 3); ctx.lineTo(percX - 8, aY + 3); ctx.fill();
    ctx.font = 'bold 13px monospace'; ctx.fillStyle = GAP_M > 5 ? '#e84545' : GAP_M > 2 ? '#f0c040' : '#3ecf8e';
    ctx.textAlign = 'center';
    ctx.fillText(GAP_M.toFixed(1) + ' m', (actualX + percX) / 2, aY - 10);
    ctx.textAlign = 'left';
  }

  // Perceived dot
  if (percX > -20 && percX < W + 20) {
    ctx.beginPath(); ctx.arc(percX, skyY, 14, 0, Math.PI * 2);
    ctx.strokeStyle = 'rgba(74,158,255,0.25)'; ctx.lineWidth = 1.5; ctx.setLineDash([3, 3]); ctx.stroke(); ctx.setLineDash([]);
    ctx.beginPath(); ctx.arc(percX, skyY, 5, 0, Math.PI * 2); ctx.fillStyle = 'rgba(74,158,255,0.4)'; ctx.fill();
    ctx.font = '10px monospace'; ctx.fillStyle = 'rgba(74,158,255,0.55)'; ctx.textAlign = 'center';
    ctx.fillText('perceived', percX, skyY + 26); ctx.textAlign = 'left';
  }

  // Actual dot
  if (actualX > -20 && actualX < W + 20) {
    ctx.beginPath(); ctx.arc(actualX, skyY, 14, 0, Math.PI * 2); ctx.fillStyle = 'rgba(232,69,69,0.1)'; ctx.fill();
    ctx.strokeStyle = '#e84545'; ctx.lineWidth = 2; ctx.stroke();
    ctx.beginPath(); ctx.arc(actualX, skyY, 5, 0, Math.PI * 2); ctx.fillStyle = '#e84545'; ctx.fill();
    ctx.strokeStyle = 'rgba(232,69,69,0.5)'; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(actualX - 8, skyY); ctx.lineTo(actualX - 22, skyY); ctx.stroke();
    ctx.fillStyle = 'rgba(232,69,69,0.5)';
    ctx.beginPath(); ctx.moveTo(actualX - 22, skyY); ctx.lineTo(actualX - 17, skyY - 3); ctx.lineTo(actualX - 17, skyY + 3); ctx.fill();
    ctx.font = '10px monospace'; ctx.fillStyle = 'rgba(232,69,69,0.75)'; ctx.textAlign = 'center';
    ctx.fillText('actual', actualX, skyY + 26); ctx.textAlign = 'left';
  }
}

console.log(`Rendering ${TOTAL_FRAMES} frames at ${FPS}fps (${DURATION_SEC}s)...`);

for (let frame = 0; frame < TOTAL_FRAMES; frame++) {
  const t = frame / FPS;
  drawFrame(t);
  encoder.addFrame(ctx);
  if (frame % FPS === 0) process.stdout.write(`  ${t.toFixed(1)}s / ${DURATION_SEC}s\r`);
}

encoder.finish();
stream.on('finish', () => {
  const size = (fs.statSync(outPath).size / 1024).toFixed(0);
  console.log(`\nDone! ${outPath} (${size} KB)`);
});
