#!/usr/bin/env node
// Generates ik_five_ways_preview.gif — exact replica of the blog comparison animation
// (section 8, 3×2 grid) with auto-cycling targets, no method titles.
// Usage: node scripts/ik-five-ways-preview.js
// Output: ik_five_ways_preview.gif (800×454, ~12s loop at 15fps)

const { createCanvas } = require('canvas');
const GifEncoder = require('gif-encoder-2');
const fs = require('fs');
const path = require('path');

/* ── Layout — mirrors the blog's 3×2 grid ── */
const FPS           = 15;
const DURATION_SEC  = 10;
const TOTAL_FRAMES  = FPS * DURATION_SEC; // 150

const GRID_W        = 800;
const CELL_W        = Math.floor((GRID_W - 2) / 3);   // 266 px
const CELL_H        = Math.floor(CELL_W * 0.85);       // 226 px
const W             = GRID_W;
const H             = CELL_H * 2 + 1;                  // 453 px

/* ── Arm constants (identical to blog section 8) ── */
const L1 = 0.15, L2 = 0.10;
const PAD = 30;
const START_T1 = 7  * Math.PI / 180;
const START_T2 = 96 * Math.PI / 180;
const ERR_THRESH = 0.001;
const MAX_ITER   = 200;

/* Shared sc / orig / w2s — all relative to cell-local (0,0) */
const sc = Math.min(CELL_W, CELL_H) * 3.2;
function origXY() { return [PAD, CELL_H - PAD]; }
function w2s(wx, wy) {
  const [ox, oy] = origXY();
  return [ox + wx * sc, oy - wy * sc];
}

/* ── Canvas ── */
const canvas = createCanvas(W, H);
const ctx    = canvas.getContext('2d');

/* ── FK / IK ── */
function fk(t1, t2) {
  return { x: L1*Math.cos(t1)+L2*Math.cos(t1+t2), y: L1*Math.sin(t1)+L2*Math.sin(t1+t2) };
}
function elbowW(t1) { return { x: L1*Math.cos(t1), y: L1*Math.sin(t1) }; }
function solveIK(x, y) {
  let cosB = (L1*L1+L2*L2-x*x-y*y) / (2*L1*L2);
  cosB = Math.max(-1, Math.min(1, cosB));
  const t2 = Math.PI - Math.acos(cosB);
  const t1 = Math.atan2(y, x) - Math.atan2(L2*Math.sin(t2), L1+L2*Math.cos(t2));
  return { t1, t2 };
}

/* ── Solvers (exact copies from blog) ── */
let tgtX = 0, tgtY = 0, hasTarget = false;

function jacobian(t1, t2) {
  const s1=Math.sin(t1),c1=Math.cos(t1),s12=Math.sin(t1+t2),c12=Math.cos(t1+t2);
  return { a:-L1*s1-L2*s12, b:-L2*s12, c:L1*c1+L2*c12, d:L2*c12 };
}

function stepJacobian(s) {
  if (s.done) return;
  const tip = fk(s.t1, s.t2);
  s.err = Math.hypot(tip.x-tgtX, tip.y-tgtY);
  if (s.err < ERR_THRESH) { s.done = true; return; }
  const ex = tgtX-tip.x, ey = tgtY-tip.y;
  const J = jacobian(s.t1, s.t2);
  const det = J.a*J.d - J.b*J.c;
  if (Math.abs(det) < 1e-8) { s.done = true; return; }
  const id = 1/det;
  s.t1 += (J.d*id)*ex + (-J.b*id)*ey;
  s.t2 += (-J.c*id)*ex + (J.a*id)*ey;
  s.iter++;
  if (s.iter >= MAX_ITER) s.done = true;
}

function stepCCD(s) {
  if (s.done) return;
  let tip = fk(s.t1, s.t2);
  s.err = Math.hypot(tip.x-tgtX, tip.y-tgtY);
  if (s.err < ERR_THRESH) { s.done = true; s.ccdVis = null; return; }
  const joint = (s.iter % 2 === 0) ? 2 : 1;
  if (joint === 2) {
    const e = elbowW(s.t1);
    const ax=tip.x-e.x, ay=tip.y-e.y, bx=tgtX-e.x, by=tgtY-e.y;
    s.ccdVis = { px:e.x, py:e.y, tipX:tip.x, tipY:tip.y, joint:2 };
    s.t2 += Math.atan2(ax*by-ay*bx, ax*bx+ay*by);
  } else {
    tip = fk(s.t1, s.t2);
    const ax=tip.x, ay=tip.y, bx=tgtX, by=tgtY;
    s.ccdVis = { px:0, py:0, tipX:tip.x, tipY:tip.y, joint:1 };
    s.t1 += Math.atan2(ax*by-ay*bx, ax*bx+ay*by);
  }
  s.iter++;
  tip = fk(s.t1, s.t2);
  s.err = Math.hypot(tip.x-tgtX, tip.y-tgtY);
  if (s.err < ERR_THRESH) { s.done = true; s.ccdVis = null; }
  if (s.iter >= MAX_ITER) { s.done = true; s.ccdVis = null; }
}

function stepToward(ax, ay, rx, ry, len) {
  const dx=rx-ax, dy=ry-ay, d=Math.hypot(dx,dy);
  if (d < 1e-10) return { x:ax+len, y:ay };
  return { x:ax+dx/d*len, y:ay+dy/d*len };
}
function stepFABRIK(s) {
  if (s.done) return;
  if (s.fabrikSub === undefined) s.fabrikSub = 0;
  if (!s._p0) {
    const c1=Math.cos(s.t1), sn1=Math.sin(s.t1), c12=Math.cos(s.t1+s.t2), s12=Math.sin(s.t1+s.t2);
    s._p0={x:0,y:0}; s._p1={x:L1*c1,y:L1*sn1}; s._p2={x:s._p1.x+L2*c12,y:s._p1.y+L2*s12};
  }
  const p0=s._p0, p1=s._p1, p2=s._p2;

  if (s.fabrikSub === 0) {
    s.err = Math.hypot(p2.x-tgtX, p2.y-tgtY);
    if (s.err < ERR_THRESH) { s.done=true; s.fabrikVis=null; return; }
    const op1={x:p1.x,y:p1.y};
    const newP2={x:tgtX,y:tgtY};
    const newP1=stepToward(newP2.x,newP2.y,op1.x,op1.y,L2);
    s.fabrikVis={ phase:'fwd1', dashA:op1, dashB:{x:tgtX,y:tgtY},
      l1a:{x:p0.x,y:p0.y},l1b:op1,l1bright:false,
      l2a:newP1,l2b:newP2,l2bright:true, gapA:op1,gapB:newP1 };
    s._p1=newP1; s._p2=newP2; s._oldP0={x:p0.x,y:p0.y};
    s.fabrikSub=1;
  } else if (s.fabrikSub === 1) {
    const oldP0=s._oldP0;
    const newP0=stepToward(p1.x,p1.y,oldP0.x,oldP0.y,L1);
    s.fabrikVis={ phase:'fwd2', dashA:p1, dashB:oldP0,
      l1a:newP0,l1b:{x:p1.x,y:p1.y},l1bright:true,
      l2a:{x:p1.x,y:p1.y},l2b:{x:p2.x,y:p2.y},l2bright:false,
      driftFrom:{x:0,y:0},driftTo:newP0 };
    s._p0=newP0; s.fabrikSub=2;
  } else if (s.fabrikSub === 2) {
    const fwdP1={x:p1.x,y:p1.y};
    const newP0={x:0,y:0};
    const newP1=stepToward(newP0.x,newP0.y,fwdP1.x,fwdP1.y,L1);
    s.fabrikVis={ phase:'bwd1', dashA:newP0, dashB:fwdP1,
      l1a:newP0,l1b:newP1,l1bright:true,
      l2a:fwdP1,l2b:{x:p2.x,y:p2.y},l2bright:false, gapA:fwdP1,gapB:newP1 };
    s._p0=newP0; s._p1=newP1; s.fabrikSub=3;
  } else {
    const newP2=stepToward(p1.x,p1.y,tgtX,tgtY,L2);
    s.fabrikVis={ phase:'bwd2', dashA:{x:p1.x,y:p1.y}, dashB:{x:tgtX,y:tgtY},
      l1a:{x:p0.x,y:p0.y},l1b:{x:p1.x,y:p1.y},l1bright:false,
      l2a:{x:p1.x,y:p1.y},l2b:newP2,l2bright:true, gapA:newP2,gapB:{x:tgtX,y:tgtY} };
    s._p2=newP2;
    s.t1=Math.atan2(p1.y-p0.y, p1.x-p0.x);
    s.t2=Math.atan2(newP2.y-p1.y, newP2.x-p1.x)-s.t1;
    s.iter++;
    s.err=Math.hypot(newP2.x-tgtX, newP2.y-tgtY);
    if (s.err < ERR_THRESH) { s.done=true; s.fabrikVis=null; }
    if (s.iter >= MAX_ITER) { s.done=true; s.fabrikVis=null; }
    s.fabrikSub=0;
  }
}

/* ── Learning-Based: dense pre-sampled data ── */
const mlSamples = [];
(function() {
  const STEP = 0.005;
  for (let gx=0.02; gx<=L1+L2-0.01; gx+=STEP) {
    for (let gy=0.02; gy<=L1+L2-0.01; gy+=STEP) {
      const d = Math.hypot(gx, gy);
      if (d > L1+L2-0.005 || d < Math.abs(L1-L2)+0.005) continue;
      const ik = solveIK(gx, gy);
      const tip = fk(ik.t1, ik.t2);
      mlSamples.push({ tx:tip.x, ty:tip.y, t1:ik.t1, t2:ik.t2 });
    }
  }
})();
function solveML() {
  const s = solvers[4];
  if (!mlSamples.length) { s.done=true; s.err=999; return; }
  let wSum=0, wt1=0, wt2=0;
  for (const m of mlSamples) {
    const dx=m.tx-tgtX, dy=m.ty-tgtY, w=1/(dx*dx+dy*dy+1e-8);
    wSum+=w; wt1+=w*m.t1; wt2+=w*m.t2;
  }
  s.t1=wt1/wSum; s.t2=wt2/wSum;
  const tip=fk(s.t1,s.t2);
  s.err=Math.hypot(tip.x-tgtX,tip.y-tgtY);
  s.iter=1; s.done=true;
}
function solveAnalytical() {
  const s=solvers[0];
  const ik=solveIK(tgtX,tgtY);
  s.t1=ik.t1; s.t2=ik.t2;
  const tip=fk(s.t1,s.t2);
  s.err=Math.hypot(tip.x-tgtX,tip.y-tgtY);
  s.iter=1; s.done=true;
}

/* ── Solver state ── */
let solvers = [];
function initSolvers() {
  solvers = [];
  for (let i = 0; i < 5; i++)
    solvers.push({ t1:START_T1, t2:START_T2, iter:0, done:false, err:999,
                   ccdVis:null, fabrikVis:null, fabrikSub:undefined, _p0:null });
}
initSolvers();

/* ── Auto-cycle targets ── */
const TARGETS = [
  { x:0.15, y:0.15 },
  { x:0.08, y:0.21 },
  { x:0.20, y:0.07 },
  { x:0.05, y:0.18 },
  { x:0.18, y:0.13 },
];
let tgtIdx = 0;

function setTarget(tx, ty) {
  tgtX=tx; tgtY=Math.max(0.01,ty); hasTarget=true;
  if (Math.hypot(tgtX,tgtY) > L1+L2-0.001) { hasTarget=false; return; }
  initSolvers();
  solveAnalytical();
  solveML();
}

/* ── Drawing helpers (exact copy of blog) ── */
function drawLink(cx, x1, y1, x2, y2, w, color) {
  const dx=x2-x1, dy=y2-y1, len=Math.hypot(dx,dy); if(len<1) return;
  const ux=dx/len, uy=dy/len, nx=-uy*w/2, ny=ux*w/2;
  cx.fillStyle=color; cx.beginPath();
  cx.moveTo(x1+nx,y1+ny); cx.lineTo(x2+nx,y2+ny);
  cx.lineTo(x2-nx,y2-ny); cx.lineTo(x1-nx,y1-ny);
  cx.closePath(); cx.fill();
  cx.beginPath(); cx.arc(x1,y1,w/2,0,Math.PI*2); cx.fill();
  cx.beginPath(); cx.arc(x2,y2,w/2,0,Math.PI*2); cx.fill();
}
function drawJoint(cx, x, y, r) {
  cx.beginPath(); cx.arc(x,y,r,0,Math.PI*2);
  cx.fillStyle='#1a1e26'; cx.fill();
  cx.strokeStyle='rgba(255,255,255,0.5)'; cx.lineWidth=2; cx.stroke();
}
function drawArm(cx, t1, t2, aL1, aL2) {
  const e=elbowW(t1), tip=fk(t1,t2);
  const os=w2s(0,0), es=w2s(e.x,e.y), ts=w2s(tip.x,tip.y);
  drawLink(cx,os[0],os[1],es[0],es[1],18,`rgba(74,158,255,${aL1})`);
  drawLink(cx,es[0],es[1],ts[0],ts[1],14,`rgba(62,207,142,${aL2})`);
  drawJoint(cx,os[0],os[1],9); drawJoint(cx,es[0],es[1],7);
  cx.beginPath(); cx.arc(ts[0],ts[1],4,0,Math.PI*2);
  cx.fillStyle=`rgba(240,192,64,${aL2})`; cx.fill();
}
function drawTarget(cx, t) {
  if (!hasTarget) return;
  const ts=w2s(tgtX,tgtY); const pulse=0.5+0.5*Math.sin(t*3);
  cx.beginPath(); cx.arc(ts[0],ts[1],8+pulse*2,0,Math.PI*2);
  cx.fillStyle='rgba(232,69,69,0.12)'; cx.fill();
  cx.beginPath(); cx.arc(ts[0],ts[1],5,0,Math.PI*2);
  cx.fillStyle='#e84545'; cx.fill();
}
function drawReach(cx) {
  const o=origXY();
  cx.save(); cx.setLineDash([4,4]);
  cx.strokeStyle='rgba(255,255,255,0.06)'; cx.lineWidth=1;
  cx.beginPath(); cx.arc(o[0],o[1],(L1+L2)*sc,0,Math.PI*2); cx.stroke();
  cx.restore();
}
function drawGrid(cx) {
  const o=origXY();
  cx.strokeStyle='rgba(42,48,64,0.25)'; cx.lineWidth=0.5;
  for (let gx=0; gx<CELL_W; gx+=30) { cx.beginPath(); cx.moveTo(gx,0); cx.lineTo(gx,CELL_H); cx.stroke(); }
  for (let gy=0; gy<CELL_H; gy+=30) { cx.beginPath(); cx.moveTo(0,gy); cx.lineTo(CELL_W,gy); cx.stroke(); }
  cx.strokeStyle='rgba(255,255,255,0.15)'; cx.lineWidth=1;
  cx.beginPath(); cx.moveTo(o[0],0); cx.lineTo(o[0],CELL_H); cx.stroke();
  cx.beginPath(); cx.moveTo(0,o[1]); cx.lineTo(CELL_W,o[1]); cx.stroke();
}
function drawArrow(cx, x1, y1, x2, y2, color) {
  const dx=x2-x1, dy=y2-y1, len=Math.hypot(dx,dy); if(len<2) return;
  const ux=dx/len, uy=dy/len;
  cx.strokeStyle=color; cx.lineWidth=1.5;
  cx.beginPath(); cx.moveTo(x1,y1); cx.lineTo(x2,y2); cx.stroke();
  cx.fillStyle=color; cx.beginPath();
  cx.moveTo(x2,y2); cx.lineTo(x2-ux*6+uy*3,y2-uy*6-ux*3);
  cx.lineTo(x2-ux*6-uy*3,y2-uy*6+ux*3); cx.closePath(); cx.fill();
}
function drawCCDVis(cx, s) {
  if (!s.ccdVis || s.done) return;
  const v=s.ccdVis;
  const ps=w2s(v.px,v.py), oldTs=w2s(v.tipX,v.tipY), tgs=w2s(tgtX,tgtY);
  cx.save(); cx.setLineDash([4,4]);
  cx.strokeStyle='rgba(240,192,64,0.4)'; cx.lineWidth=1.5;
  cx.beginPath(); cx.moveTo(ps[0],ps[1]); cx.lineTo(oldTs[0],oldTs[1]); cx.stroke();
  cx.restore();
  drawArrow(cx,ps[0],ps[1],tgs[0],tgs[1],'rgba(232,69,69,0.6)');
  const r=20;
  const a1=Math.atan2(oldTs[1]-ps[1],oldTs[0]-ps[0]);
  const a2=Math.atan2(tgs[1]-ps[1],tgs[0]-ps[0]);
  cx.strokeStyle='rgba(255,255,255,0.35)'; cx.lineWidth=1.5;
  cx.beginPath(); cx.arc(ps[0],ps[1],r,a1,a2,a1>a2); cx.stroke();
}
function drawFABRIKVis(cx, s) {
  if (!s.fabrikVis) return false;
  const v=s.fabrikVis; const l1w=18, l2w=14;
  const da=w2s(v.dashA.x,v.dashA.y), db=w2s(v.dashB.x,v.dashB.y);
  cx.save(); cx.setLineDash([5,5]);
  cx.strokeStyle='rgba(255,255,255,0.25)'; cx.lineWidth=1.5;
  cx.beginPath(); cx.moveTo(da[0],da[1]); cx.lineTo(db[0],db[1]); cx.stroke();
  cx.restore();
  const l1as=w2s(v.l1a.x,v.l1a.y), l1bs=w2s(v.l1b.x,v.l1b.y);
  drawLink(cx,l1as[0],l1as[1],l1bs[0],l1bs[1],l1w,`rgba(74,158,255,${v.l1bright?0.75:0.18})`);
  const l2as=w2s(v.l2a.x,v.l2a.y), l2bs=w2s(v.l2b.x,v.l2b.y);
  drawLink(cx,l2as[0],l2as[1],l2bs[0],l2bs[1],l2w,`rgba(62,207,142,${v.l2bright?0.7:0.15})`);
  drawJoint(cx,l1as[0],l1as[1],9); drawJoint(cx,l1bs[0],l1bs[1],7);
  cx.beginPath(); cx.arc(l2bs[0],l2bs[1],4,0,Math.PI*2);
  cx.fillStyle='rgba(240,192,64,0.8)'; cx.fill();
  if (v.gapA && v.gapB) {
    const ga=w2s(v.gapA.x,v.gapA.y), gb=w2s(v.gapB.x,v.gapB.y);
    const gd=Math.hypot(ga[0]-gb[0],ga[1]-gb[1]);
    if (gd > 2) {
      cx.save(); cx.setLineDash([3,3]);
      cx.strokeStyle='rgba(240,192,64,0.5)'; cx.lineWidth=1.5;
      cx.beginPath(); cx.moveTo(ga[0],ga[1]); cx.lineTo(gb[0],gb[1]); cx.stroke();
      cx.restore();
    }
  }
  if (v.driftFrom && v.driftTo) {
    const df=w2s(v.driftFrom.x,v.driftFrom.y), dt=w2s(v.driftTo.x,v.driftTo.y);
    if (Math.hypot(df[0]-dt[0],df[1]-dt[1]) > 2) {
      cx.save(); cx.setLineDash([3,3]);
      cx.strokeStyle='rgba(232,69,69,0.5)'; cx.lineWidth=1.5;
      cx.beginPath(); cx.moveTo(df[0],df[1]); cx.lineTo(dt[0],dt[1]); cx.stroke();
      cx.restore();
    }
  }
  const phaseName = v.phase==='fwd1'?'fwd · link 2':v.phase==='fwd2'?'fwd · link 1':
                    v.phase==='bwd1'?'bwd · link 1':'bwd · link 2';
  cx.font='bold 10px "Space Mono",monospace'; cx.fillStyle='rgba(255,255,255,0.5)'; cx.textAlign='center';
  cx.fillText(phaseName, CELL_W/2, 26);
  return true;
}

/* ── Draw one cell (applies a canvas transform to offset it) ── */
function drawCell(ci, t) {
  const col = ci % 3;
  const row = Math.floor(ci / 3);
  const offX = col * (CELL_W + 1);
  const offY = row * (CELL_H + 1);

  ctx.save();
  ctx.translate(offX, offY);
  /* Clip to cell */
  ctx.beginPath(); ctx.rect(0, 0, CELL_W, CELL_H); ctx.clip();

  ctx.fillStyle = '#0a0c10'; ctx.fillRect(0, 0, CELL_W, CELL_H);
  drawGrid(ctx);
  drawReach(ctx);

  if (ci < 5) {
    /* Ghost start pose */
    if (hasTarget) drawArm(ctx, START_T1, START_T2, 0.12, 0.10);

    const s = solvers[ci];

    /* FABRIK custom vis */
    let drewCustom = false;
    if (ci === 3 && hasTarget && !s.done && s.fabrikVis)
      drewCustom = drawFABRIKVis(ctx, s);

    if (hasTarget && !drewCustom) drawArm(ctx, s.t1, s.t2, 0.85, 0.8);

    if (ci === 2 && hasTarget && !s.done) drawCCDVis(ctx, s);

    drawTarget(ctx, t);

    /* Status */
    if (hasTarget && s.iter > 0) {
      const itxt = s.iter===1 ? '1 step' : s.iter+' iterations';
      if (s.done && s.err < ERR_THRESH) {
        ctx.font='bold 12px "Space Mono",monospace'; ctx.fillStyle='#3ecf8e'; ctx.textAlign='center';
        ctx.fillText('\u2713 '+itxt, CELL_W/2, CELL_H-24);
        ctx.font='11px "Space Mono",monospace'; ctx.fillStyle='rgba(255,255,255,0.4)';
        ctx.fillText('err: '+s.err.toFixed(6), CELL_W/2, CELL_H-10);
      } else if (s.done) {
        ctx.font='bold 12px "Space Mono",monospace'; ctx.fillStyle='#f0c040'; ctx.textAlign='center';
        ctx.fillText(itxt, CELL_W/2, CELL_H-24);
        ctx.font='11px "Space Mono",monospace'; ctx.fillStyle='rgba(255,255,255,0.4)';
        ctx.fillText('err: '+s.err.toFixed(6), CELL_W/2, CELL_H-10);
      } else {
        ctx.font='11px "Space Mono",monospace'; ctx.fillStyle='rgba(255,255,255,0.35)'; ctx.textAlign='center';
        ctx.fillText('iter: '+s.iter, CELL_W/2, CELL_H-24);
        ctx.font='11px "Space Mono",monospace'; ctx.fillStyle='rgba(255,255,255,0.25)';
        ctx.fillText('err: '+s.err.toFixed(6), CELL_W/2, CELL_H-10);
      }
    }
  } else {
    /* Cell 5 — summary (no method names, just iter+err per solver) */
    if (!hasTarget) {
      const blink = 0.5+0.5*Math.sin(t*4);
      ctx.font='bold 13px "Space Mono",monospace';
      ctx.fillStyle=`rgba(255,255,255,${(0.3+blink*0.3).toFixed(2)})`;
      ctx.textAlign='center'; ctx.textBaseline='middle';
      ctx.fillText('setting target\u2026', CELL_W/2, CELL_H/2);
    } else {
      const icons = ['\u2460','\u2461','\u2462','\u2463','\u2464']; // ①②③④⑤
      ctx.font='bold 11px "Space Mono",monospace'; ctx.textAlign='left'; ctx.textBaseline='alphabetic';
      const y0=28, dy=22;
      for (let k=0; k<5; k++) {
        const sv=solvers[k];
        const col=(sv.done&&sv.err<ERR_THRESH)?'#3ecf8e':(sv.done?'#f0c040':'rgba(255,255,255,0.35)');
        ctx.fillStyle=col;
        const it2=sv.iter===0?'\u2026':sv.iter===1?'1 step':sv.iter+' iter';
        ctx.fillText(icons[k], 12, y0+k*dy);
        ctx.textAlign='right';
        ctx.fillText(it2, CELL_W-12, y0+k*dy);
        ctx.textAlign='left';
        ctx.font='10px "Space Mono",monospace'; ctx.fillStyle='rgba(255,255,255,0.35)';
        ctx.fillText('err: '+sv.err.toFixed(5), 12, y0+k*dy+12);
        ctx.font='bold 11px "Space Mono",monospace';
      }
      ctx.font='10px "Space Mono",monospace'; ctx.fillStyle='rgba(255,255,255,0.25)'; ctx.textAlign='center';
      ctx.fillText('\u03B5 = '+ERR_THRESH, CELL_W/2, CELL_H-10);
    }
  }

  ctx.restore();
}

/* ── GIF encoder ── */
const encoder = new GifEncoder(W, H, 'neuquant', true);
encoder.setDelay(Math.round(1000 / FPS));
encoder.setRepeat(0);
encoder.setQuality(10);

const outPath = path.join(__dirname, '..', 'ik_five_ways_preview.gif');
const stream  = fs.createWriteStream(outPath);
encoder.createReadStream().pipe(stream);
encoder.start();

/* ── Simulation state ── */
const STEP_FRAMES  = 1;   // one solver step every frame — continuous motion
const HOLD_FRAMES  = 18;  // hold after convergence (~1.2s)

let framesSinceStep = 0;
let doneFrame       = -1;  // frame at which all iterative solvers finished

/* Set first target at frame 3 */
console.log(`Rendering ${TOTAL_FRAMES} frames at ${FPS}fps (${DURATION_SEC}s)...`);
console.log(`Grid: ${W}×${H}, cells: ${CELL_W}×${CELL_H}, ${mlSamples.length} ML samples.`);

for (let frame = 0; frame < TOTAL_FRAMES; frame++) {
  const t = frame / FPS;

  /* Auto-set target at start or after hold */
  if (frame === 0 || (doneFrame >= 0 && frame === doneFrame + HOLD_FRAMES)) {
    setTarget(TARGETS[tgtIdx].x, TARGETS[tgtIdx].y);
    tgtIdx = (tgtIdx + 1) % TARGETS.length;
    framesSinceStep = 0;
    doneFrame = -1;
  }

  /* Step iterative solvers */
  if (hasTarget && doneFrame < 0) {
    framesSinceStep++;
    if (framesSinceStep >= STEP_FRAMES) {
      framesSinceStep = 0;
      stepJacobian(solvers[1]);
      stepCCD(solvers[2]);
      stepFABRIK(solvers[3]);
    }
    /* Check if all done */
    if (solvers[1].done && solvers[2].done && solvers[3].done)
      doneFrame = frame;
  }

  /* Draw background + separators */
  ctx.fillStyle = '#0a0c10';
  ctx.fillRect(0, 0, W, H);
  ctx.fillStyle = '#1e2330';
  for (let c=1; c<3; c++) ctx.fillRect(c*(CELL_W+1)-1, 0, 1, H);
  ctx.fillRect(0, CELL_H, W, 1);

  /* Draw all 6 cells */
  for (let ci = 0; ci < 6; ci++) drawCell(ci, t);

  encoder.addFrame(ctx);
  if (frame % FPS === 0) process.stdout.write(`  ${t.toFixed(1)}s / ${DURATION_SEC}s\r`);
}

encoder.finish();
stream.on('finish', () => {
  const size = (fs.statSync(outPath).size / 1024).toFixed(0);
  console.log(`\nDone! ${outPath} (${size} KB)`);
});
