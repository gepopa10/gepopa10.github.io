#!/usr/bin/env node
// Generates path_collision_preview.gif from the convex polygon edge-by-edge animation.
// Usage: node scripts/path-collision-preview.js
// Output: path_collision_preview.gif (600x450 4:3, ~6.5s loop at 15fps)

const { createCanvas } = require('canvas');
const GifEncoder = require('gif-encoder-2');
const fs = require('fs');
const path = require('path');

const W = 600, H = 450; // 4:3 matches the .sq canvas
const FPS = 15;

// Animation timing (matches the blog animation)
const EDGE_DUR = 0.7;
const PAUSE    = 0.3;
const STEP_T   = EDGE_DUR + PAUSE; // 1.0s per edge
const FINAL_PAUSE = 1.5;
const N_EDGES  = 5;
const TOTAL_CYCLE = N_EDGES * STEP_T + FINAL_PAUSE; // 6.5s

const TOTAL_FRAMES = Math.ceil(FPS * TOTAL_CYCLE);

// Fixed point P inside the polygon (centre of pentagon)
const pX = 0.5, pY = 0.48;

const canvas = createCanvas(W, H);
const ctx    = canvas.getContext('2d');

const LABELS = ['A', 'B', 'C', 'D', 'E'];

function getVerts() {
  const cx = W * 0.5, cy = H * 0.48, r = Math.min(W, H) * 0.28;
  const angles = [
    -Math.PI/2 - Math.PI*2/5,
    -Math.PI/2,
    -Math.PI/2 + Math.PI*2/5,
    -Math.PI/2 + 2*Math.PI*2/5,
    -Math.PI/2 + 3*Math.PI*2/5,
  ];
  return angles.map(a => [cx + Math.cos(a) * r, cy + Math.sin(a) * r]);
}

function cross2(ax, ay, bx, by) { return ax * by - ay * bx; }

function drawArrow(x1, y1, x2, y2, color, lw) {
  const dx = x2-x1, dy = y2-y1, len = Math.hypot(dx, dy);
  if (len < 1) return;
  const ux = dx/len, uy = dy/len;
  ctx.strokeStyle = color; ctx.lineWidth = lw;
  ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.moveTo(x2, y2);
  ctx.lineTo(x2 - ux*8 - uy*3.5, y2 - uy*8 + ux*3.5);
  ctx.lineTo(x2 - ux*8 + uy*3.5, y2 - uy*8 - ux*3.5);
  ctx.fill();
}

function drawFrame(t) {
  ctx.clearRect(0, 0, W, H);

  const verts  = getVerts();
  const nE     = verts.length;
  const px     = pX * W, py = pY * H;

  // Compute cross products for all edges
  const crosses = [], signs = [];
  for (let i = 0; i < nE; i++) {
    const j = (i + 1) % nE;
    const abx = verts[j][0] - verts[i][0], aby = verts[j][1] - verts[i][1];
    const apx = px - verts[i][0],          apy = py - verts[i][1];
    const c = cross2(abx, aby, apx, apy);
    crosses.push(c);
    signs.push(c >= 0 ? '+' : '\u2212');
  }

  let allPos = true, allNeg = true;
  for (const c of crosses) { if (c < 0) allPos = false; if (c > 0) allNeg = false; }
  const allIn = allPos || allNeg;
  const refPositive = crosses[0] >= 0;

  // Animation state
  const cycleT    = t % TOTAL_CYCLE;
  const curStep   = Math.floor(cycleT / STEP_T);
  const stepProg  = (cycleT - curStep * STEP_T) / STEP_T;
  const finalPhase = curStep >= nE;
  const edgesDone  = finalPhase ? nE : curStep;

  // Background
  ctx.fillStyle = '#0a0c10';
  ctx.fillRect(0, 0, W, H);

  // Polygon fill
  ctx.fillStyle = (finalPhase && allIn) ? 'rgba(232,69,69,0.06)' : 'rgba(255,255,255,0.04)';
  ctx.beginPath();
  ctx.moveTo(verts[0][0], verts[0][1]);
  for (let i = 1; i < nE; i++) ctx.lineTo(verts[i][0], verts[i][1]);
  ctx.closePath();
  ctx.fill();

  // Edges
  for (let idx = 0; idx < nE; idx++) {
    const v1 = verts[idx], v2 = verts[(idx + 1) % nE];
    const isActive   = !finalPhase && idx === curStep;
    const isDone     = idx < edgesDone;
    const sameSign   = (crosses[idx] >= 0) === refPositive;

    let col, alpha, lw;
    if (isActive)          { col = sameSign ? '#3ecf8e' : '#e84545'; alpha = 1;   lw = 3;   }
    else if (isDone || finalPhase) { col = sameSign ? '#3ecf8e' : '#e84545'; alpha = 0.4; lw = 2; }
    else                   { col = 'rgba(255,255,255,0.35)'; alpha = 1; lw = 1.5; }

    ctx.strokeStyle = col; ctx.globalAlpha = alpha; ctx.lineWidth = lw;
    ctx.beginPath(); ctx.moveTo(v1[0], v1[1]); ctx.lineTo(v2[0], v2[1]); ctx.stroke();
    ctx.globalAlpha = 1;

    // Vertex dot and label
    ctx.beginPath(); ctx.arc(v1[0], v1[1], 4, 0, Math.PI*2);
    ctx.fillStyle = 'rgba(255,255,255,0.6)'; ctx.fill();
    const vcx = W*0.5, vcy = H*0.48;
    const ddx = v1[0]-vcx, ddy = v1[1]-vcy, ddl = Math.hypot(ddx, ddy);
    const lx = v1[0] + (ddx/ddl)*18, ly = v1[1] + (ddy/ddl)*18;
    ctx.font = 'bold 12px monospace';
    ctx.fillStyle = 'rgba(255,255,255,0.6)'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(LABELS[idx], lx, ly);
    ctx.textBaseline = 'alphabetic';

    // Sign result on completed edges
    if (isDone || (isActive && stepProg > 0.6)) {
      const mx = (v1[0]+v2[0])/2, my = (v1[1]+v2[1])/2;
      const nx = -(v2[1]-v1[1]), ny = v2[0]-v1[0], nl = Math.hypot(nx, ny);
      const ox = mx + (nx/nl)*24, oy = my + (ny/nl)*24;
      ctx.font = 'bold 14px monospace'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillStyle = sameSign ? 'rgba(62,207,142,0.8)' : 'rgba(232,69,69,0.8)';
      ctx.fillText(signs[idx], ox, oy);
      ctx.textBaseline = 'alphabetic';
    }

    // Animated vectors for active edge
    if (isActive) {
      const vp = Math.min(1, stepProg / 0.55);
      const ep = vp*vp*(3-2*vp);
      const abEX = v1[0] + (v2[0]-v1[0])*ep, abEY = v1[1] + (v2[1]-v1[1])*ep;
      drawArrow(v1[0], v1[1], abEX, abEY, 'rgba(74,158,255,0.7)', 2);
      const acEX = v1[0] + (px-v1[0])*ep, acEY = v1[1] + (py-v1[1])*ep;
      drawArrow(v1[0], v1[1], acEX, acEY, 'rgba(240,192,64,0.7)', 2);

      if (vp > 0.3) {
        ctx.font = '10px monospace'; ctx.textAlign = 'center'; ctx.globalAlpha = vp;
        const edgeLbl = LABELS[idx] + LABELS[(idx+1)%nE];
        const ptLbl   = LABELS[idx] + 'P';
        const abMx = (v1[0]+abEX)/2, abMy = (v1[1]+abEY)/2;
        const enx = -(v2[1]-v1[1]), eny = v2[0]-v1[0], enl = Math.hypot(enx, eny);
        ctx.fillStyle = 'rgba(74,158,255,0.7)';
        ctx.fillText(edgeLbl, abMx-(enx/enl)*14, abMy-(eny/enl)*14);
        const acMx = (v1[0]+acEX)/2, acMy = (v1[1]+acEY)/2;
        ctx.fillStyle = 'rgba(240,192,64,0.7)';
        ctx.fillText(ptLbl, acMx+14, acMy-6);
        ctx.globalAlpha = 1;
      }
    }
  }

  // Point P
  const pulse = 0.5 + 0.5 * Math.sin(t * 3);
  ctx.beginPath(); ctx.arc(px, py, 10+pulse*2, 0, Math.PI*2);
  ctx.fillStyle = allIn ? 'rgba(232,69,69,0.1)' : 'rgba(74,158,255,0.07)'; ctx.fill();
  ctx.beginPath(); ctx.arc(px, py, 6, 0, Math.PI*2);
  ctx.fillStyle = allIn ? '#e84545' : '#4a9eff'; ctx.fill();
  ctx.font = 'bold 12px monospace'; ctx.fillStyle = allIn ? '#e84545' : '#4a9eff';
  ctx.textAlign = 'center';
  ctx.fillText('P', px, py-14);

  // Final phase conclusion
  if (finalPhase) {
    const signStr = signs.join(' , ');
    ctx.textAlign = 'center';
    ctx.font = 'bold 16px monospace'; ctx.fillStyle = '#ffffff';
    ctx.fillText(allIn ? 'same sign \u2192 inside' : 'different signs \u2192 outside', W/2, H-42);
    ctx.font = 'bold 12px monospace'; ctx.fillStyle = 'rgba(255,255,255,0.6)';
    ctx.fillText('signs: [ ' + signStr + ' ]', W/2, H-22);
  }
}

const encoder = new GifEncoder(W, H, 'neuquant', true);
encoder.setDelay(Math.round(1000 / FPS));
encoder.setRepeat(0);
encoder.setQuality(10);

const outPath = path.join(__dirname, '..', 'path_collision_preview.gif');
const stream  = fs.createWriteStream(outPath);
encoder.createReadStream().pipe(stream);
encoder.start();

console.log(`Rendering ${TOTAL_FRAMES} frames at ${FPS}fps (${TOTAL_CYCLE.toFixed(1)}s)...`);

for (let frame = 0; frame < TOTAL_FRAMES; frame++) {
  const t = frame / FPS;
  drawFrame(t);
  encoder.addFrame(ctx);
  if (frame % FPS === 0) process.stdout.write(`  ${t.toFixed(1)}s / ${TOTAL_CYCLE.toFixed(1)}s\r`);
}

encoder.finish();
stream.on('finish', () => {
  const size = (fs.statSync(outPath).size / 1024).toFixed(0);
  console.log(`\nDone! ${outPath} (${size} KB)`);
});
