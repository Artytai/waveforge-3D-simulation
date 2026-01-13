// WaveForge 3D — Complete FLIP/PIC Water Simulator
(() => {
  // ---------- CONFIG ----------
  let NX = 32, NY = 24, NZ = 24;
  let dt = 1 / 60;
  let g = 9.8, damp = 0.02, omega = 1.9, alphaPIC = 0.1;
  let PIPE_PRESSURE = 1.4;
  let rainEnabled = false;

  // ---------- CANVAS & CAMERA ----------
  const canvas = document.getElementById('view');
  const ctx = canvas.getContext('2d');
  let cam = { yaw: 0.55, pitch: 0.35, dist: 2.2 };
  let dragging = false, lastX = 0, lastY = 0;

  // ---------- SIMULATION FIELDS ----------
  const MAX_PARTICLES = 12000;
  let particles = [];
  let u, v, w, uPrev, vPrev, wPrev, uW, vW, wW, p, div;
  let obstacles, fluidMask, dyeField;

  // ---------- OBJECTS ----------
  let pipes = [];
  const sensors = [
    { x: 0.25, y: 0.75, z: 0.25, last: 0 },
    { x: 0.50, y: 0.75, z: 0.50, last: 0 },
    { x: 0.75, y: 0.75, z: 0.75, last: 0 }
  ];
  let audioCtx, masterGain, audioReady = false;
  let hasBoat = false;
  const boat = { x: 0.5, y: 0.55, z: 0.5, w: 0.18, d: 0.10, h: 0.06, vx: 0, vy: 0, vz: 0, mass: 1.0, buoyK: 35, drag: 2.0 };

  // ---------- UI STATE ----------
  let currentTool = 'obstacle';
  let sliceZ = Math.min(12, NZ - 1);
  let running = true;
  let showHelp = false;

  // ---------- UTILS ----------
  const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
  const idxC = (i, j, k) => i + j * NX + k * NX * NY;
  const idxU = (i, j, k) => i + j * (NX + 1) + k * (NX + 1) * NY;
  const idxV = (i, j, k) => i + j * NX + k * NX * (NY + 1);
  const idxW = (i, j, k) => i + j * NX + k * NX * NY;
  const normalize = (v) => {
    const l = Math.hypot(v[0], v[1], v[2]) + 1e-8;
    return [v[0] / l, v[1] / l, v[2] / l];
  };

  // ---------- ALLOCATE FIELDS ----------
  function alloc() {
    obstacles = new Uint8Array(NX * NY * NZ);
    fluidMask = new Uint8Array(NX * NY * NZ);
    u = new Float32Array((NX + 1) * NY * NZ);
    v = new Float32Array(NX * (NY + 1) * NZ);
    w = new Float32Array(NX * NY * (NZ + 1));
    uPrev = new Float32Array(u.length);
    vPrev = new Float32Array(v.length);
    wPrev = new Float32Array(w.length);
    uW = new Float32Array(u.length);
    vW = new Float32Array(v.length);
    wW = new Float32Array(w.length);
    p = new Float32Array(NX * NY * NZ);
    div = new Float32Array(NX * NY * NZ);
    dyeField = new Float32Array(NX * NY * NZ);
  }

  // ---------- RESET SIMULATION ----------
  function reset() {
    u.fill(0); v.fill(0); w.fill(0);
    uPrev.fill(0); vPrev.fill(0); wPrev.fill(0);
    p.fill(0); div.fill(0);
    dyeField.fill(0); obstacles.fill(0); pipes.length = 0;
    particles = [];
    const startZ = 0.15, endZ = 0.75;
    for (let n = 0; n < MAX_PARTICLES; n++) {
      const x = Math.random() * 0.9 + 0.05;
      const y = Math.random() * 0.45 + 0.05;
      const z = Math.random() * (endZ - startZ) + startZ;
      particles.push({ x, y, z, vx: 0, vy: 0, vz: 0, dye: 0 });
    }
    for (const p0 of particles) {
      const dx = p0.x - 0.5, dz = p0.z - 0.5;
      p0.vx = 0.3 * dz; p0.vz = -0.3 * dx;
    }
  }

  // ---------- CANVAS ----------
  function resizeCanvas() {
    const w = document.getElementById('app').clientWidth - 380;
    const h = document.getElementById('app').clientHeight;
    canvas.width = w * devicePixelRatio;
    canvas.height = h * devicePixelRatio;
    canvas.style.width = w + 'px';
    canvas.style.height = h + 'px';
    ctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
  }
  window.addEventListener('resize', resizeCanvas); resizeCanvas();

  canvas.addEventListener('mousedown', (e) => {
    dragging = true; lastX = e.clientX; lastY = e.clientY;
    useTool(e);
    if (!audioReady) try { initAudio(); } catch { }
  });
  window.addEventListener('mouseup', () => dragging = false);
  window.addEventListener('mousemove', (e) => {
    if (dragging) {
      const dx = (e.clientX - lastX) / canvas.clientWidth;
      const dy = (e.clientY - lastY) / canvas.clientHeight;
      cam.yaw += dx * 2.0;
      cam.pitch = clamp(cam.pitch + dy * 1.5, -1.2, 1.2);
      lastX = e.clientX; lastY = e.clientY;
      if (currentTool !== 'none') useTool(e);
    }
  });
  window.addEventListener('wheel', (e) => cam.dist = clamp(cam.dist * (1 + Math.sign(e.deltaY) * 0.06), 1.3, 4.0));
  canvas.addEventListener('contextmenu', (e) => { e.preventDefault(); deletePipeAtMouse(e); });

  // ---------- AUDIO ----------
  function initAudio() {
    audioCtx = new (window.AudioContext || window.webkitAudioContext)();
    masterGain = audioCtx.createGain(); masterGain.gain.value = 0.12; masterGain.connect(audioCtx.destination);
    audioReady = true;
  }
  function ping(freq) {
    if (!audioReady) return;
    const o = audioCtx.createOscillator(), g = audioCtx.createGain();
    o.type = 'sine'; o.frequency.value = freq;
    g.gain.setValueAtTime(0, audioCtx.currentTime);
    g.gain.linearRampToValueAtTime(0.35, audioCtx.currentTime + 0.02);
    g.gain.exponentialRampToValueAtTime(0.0001, audioCtx.currentTime + 0.35);
    o.connect(g); g.connect(masterGain); o.start(); o.stop(audioCtx.currentTime + 0.4);
  }

  // ---------- 3D PROJECTION ----------
  function project3D(x, y, z) {
    const cy = Math.cos(cam.yaw), sy = Math.sin(cam.yaw);
    const cp = Math.cos(cam.pitch), sp = Math.sin(cam.pitch);
    let X = (x - 0.5) * cy + (z - 0.5) * sy;
    let Z = -(x - 0.5) * sy + (z - 0.5) * cy;
    let Y = (y - 0.5);
    const Yp = Y * cp - Z * sp;
    const Zp = Y * sp + Z * cp;
    const f = 1.0 / (cam.dist + Zp);
    return { x: (X * f + 0.5) * canvas.clientWidth, y: (Yp * f + 0.5) * canvas.clientHeight, depth: Zp };
  }

  // ---------- RENDER ----------
  function render() {
    ctx.clearRect(0, 0, canvas.clientWidth, canvas.clientHeight);
    drawBox();
    dyeField.fill(0);

    // Render particles
    const pts = particles.map((p, i) => ({ i, prj: project3D(p.x, p.y, p.z) }));
    pts.sort((a, b) => a.prj.depth - b.prj.depth);
    for (const q of pts) {
      const p0 = particles[q.i];
      const s = project3D(p0.x, p0.y, p0.z);
      const r = 2 + 2 * Math.random();
      const c = Math.min(255, Math.max(0, 120 + p0.dye * 160));
      ctx.fillStyle = `rgba(${25 + p0.dye * 200 | 0},${60 + p0.dye * 150 | 0},${c},0.85)`;
      ctx.beginPath(); ctx.arc(s.x, s.y, r, 0, Math.PI * 2); ctx.fill();

      const i = clamp(Math.floor(p0.x * NX), 0, NX - 1);
      const j = clamp(Math.floor(p0.y * NY), 0, NY - 1);
      const k = clamp(Math.floor(p0.z * NZ), 0, NZ - 1);
      dyeField[idxC(i, j, k)] += p0.dye * 0.2;
    }

    // Pipes
    for (const P of pipes) {
      const s1 = project3D(P.x, P.y, P.z);
      const s2 = project3D(P.x + 0.04 * P.dir[0], P.y + 0.04 * P.dir[1], P.z + 0.04 * P.dir[2]);
      ctx.strokeStyle = '#7ef18f'; ctx.lineWidth = 3;
      ctx.beginPath(); ctx.moveTo(s1.x, s1.y); ctx.lineTo(s2.x, s2.y); ctx.stroke();
      ctx.fillStyle = '#7ef18f'; ctx.beginPath(); ctx.arc(s1.x, s1.y, 5, 0, Math.PI * 2); ctx.fill();
    }

    // Boat
    if (hasBoat) drawBoat();

    // Sensors
    ctx.fillStyle = '#59d0ff';
    for (const s of sensors) {
      const p2 = project3D(s.x, s.y, s.z);
      ctx.beginPath(); ctx.arc(p2.x, p2.y, 5, 0, Math.PI * 2); ctx.fill();
    }

    if (showHelp) drawHelp();
  }

  function drawBox() {
    const c = [
      [0,0,0],[1,0,0],[1,1,0],[0,1,0],
      [0,0,1],[1,0,1],[1,1,1],[0,1,1]
    ].map(v => project3D(v[0],v[1],v[2]));
    function line(a,b){ ctx.beginPath(); ctx.moveTo(a.x,a.y); ctx.lineTo(b.x,b.y); ctx.stroke(); }
    line(c[0],c[1]); line(c[1],c[2]); line(c[2],c[3]); line(c[3],c[0]);
    line(c[4],c[5]); line(c[5],c[6]); line(c[6],c[7]); line(c[7],c[4]);
    line(c[0],c[4]); line(c[1],c[5]); line(c[2],c[6]); line(c[3],c[7]);
  }

  function drawBoat() {
    const bx = boat.x, by = boat.y, bz = boat.z;
    const hw = boat.w / 2, hd = boat.d / 2, hh = boat.h / 2;
    const v = [
      [bx - hw, by - hh, bz - hd],[bx + hw, by - hh, bz - hd],
      [bx + hw, by - hh, bz + hd],[bx - hw, by - hh, bz + hd],
      [bx - hw, by + hh, bz - hd],[bx + hw, by + hh, bz - hd],
      [bx + hw, by + hh, bz + hd],[bx - hw, by + hh, bz + hd]
    ].map(p => project3D(p[0],p[1],p[2]));
    ctx.fillStyle = '#ffe08a'; ctx.strokeStyle = '#9b6b2c'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(v[4].x,v[4].y); ctx.lineTo(v[5].x,v[5].y); ctx.lineTo(v[6].x,v[6].y); ctx.lineTo(v[7].x,v[7].y); ctx.closePath(); ctx.fill(); ctx.stroke();
  }

  function drawHelp() {
    ctx.fillStyle = 'rgba(0,0,0,0.55)'; ctx.fillRect(12,12,320,200);
    ctx.fillStyle = '#e9f3ff'; ctx.font='12px system-ui';
    let y=28;
    ['Mouse: rotate, wheel: zoom', 'Tools: O/E/P/D/B • Right-click pipe to delete', 'Z-slice slider for obstacle painting', 'FLIP/PIC + Gauss–Seidel projection', 'Pipes & rain • Boat buoyancy • Coral sensors'].forEach(t=>{ ctx.fillText(t,20,y); y+=18; });
  }

  // ---------- MOUSE TO DOMAIN ----------
  function mouseToDomain(e) {
    const rect = canvas.getBoundingClientRect();
    const mx = (e.clientX - rect.left) / rect.width;
    const my = (e.clientY - rect.top) / rect.height;
    const z = sliceZ / (NZ - 1);
    const X = (mx - 0.5) * cam.dist * 1.2, Y = (my - 0.5) * cam.dist * 1.2;
    const cy = Math.cos(cam.yaw), sy = Math.sin(cam.yaw), cp = Math.cos(cam.pitch), sp = Math.sin(cam.pitch);
    let Yp = Y, Zp = z - 0.5;
    const Y0 = Yp * cp + Zp * sp;
    const Z0 = -Yp * sp + Zp * cp;
    const x = ((X * cy - Z0 * sy) + 0.5);
    const y = (Y0 + 0.5);
    const zz = ((X * sy + Z0 * cy) + 0.5);
    return { x: clamp(x,0,1), y: clamp(y,0,1), z: clamp(zz,0,1) };
  }

  function useTool(e){
    const pt = mouseToDomain(e);
    if(currentTool==='obstacle'){
      const i = clamp(Math.floor(pt.x*NX),0,NX-1);
      const j = clamp(Math.floor(pt.y*NY),0,NY-1);
      obstacles[idxC(i,j,sliceZ)]=1;
    } else if(currentTool==='erase'){
      const i = clamp(Math.floor(pt.x*NX),0,NX-1);
      const j = clamp(Math.floor(pt.y*NY),0,NY-1);
      obstacles[idxC(i,j,sliceZ)]=0;
    } else if(currentTool==='pipe'){
      const cx=0.5, cy=0.5, cz=sliceZ/(NZ-1);
      const dir = normalize([pt.x-cx, pt.y-cy, pt.z-cz]);
      pipes.push({x:pt.x, y:pt.y, z:pt.z, dir});
    } else if(currentTool==='dye'){
      const radius = e.shiftKey ? 0.08 : 0.05;
      for(const p0 of particles){
        const d = Math.hypot(p0.x-pt.x, p0.y-pt.y, p0.z-pt.z);
        if(d<radius) { p0.dye = Math.min(1,p0.dye+(1-d/radius)); p0.vy-=0.05; }
      }
    }
  }

  function deletePipeAtMouse(e){
    const pt = mouseToDomain(e);
    pipes = pipes.filter(p => Math.hypot(p.x-pt.x,p.y-pt.y,p.z-pt.z)>0.06);
  }

  // ---------- FLIP/PIC SIMULATION ----------
  function markFluidCells(){
    fluidMask.fill(0);
    for(const p of particles){
      const i = clamp(Math.floor(p.x*NX),0,NX-1);
      const j = clamp(Math.floor(p.y*NY),0,NY-1);
      const k = clamp(Math.floor(p.z*NZ),0,NZ-1);
      fluidMask[idxC(i,j,k)]=1;
    }
  }

  function particlesToGrid(){
    u.fill(0); v.fill(0); w.fill(0);
    uW.fill(0); vW.fill(0); wW.fill(0);
    for(const p of particles){
      const i = clamp(Math.floor(p.x*NX),0,NX-1);
      const j = clamp(Math.floor(p.y*NY),0,NY-1);
      const k = clamp(Math.floor(p.z*NZ),0,NZ-1);
      u[idxU(i,j,k)] += p.vx; uW[idxU(i,j,k)] += 1;
      v[idxV(i,j,k)] += p.vy; vW[idxV(i,j,k)] += 1;
      w[idxW(i,j,k)] += p.vz; wW[idxW(i,j,k)] += 1;
    }
    for(let n=0;n<u.length;n++) if(uW[n]>0) u[n]/=uW[n];
    for(let n=0;n<v.length;n++) if(vW[n]>0) v[n]/=vW[n];
    for(let n=0;n<w.length;n++) if(wW[n]>0) w[n]/=wW[n];
  }

  function addForces(){
    for(let j=0;j<NY;j++){
      for(let k=0;k<NZ;k++){
        for(let i=0;i<NX+1;i++){
          u[idxU(i,j,k)] += 0; // obstacles can zero u here
        }
      }
    }
    for(let i=0;i<NX;i++){
      for(let k=0;k<NZ;k++){
        for(let j=0;j<NY+1;j++){
          v[idxV(i,j,k)] -= g*dt; // gravity
        }
      }
    }
    for(let i=0;i<NX;i++){
      for(let j=0;j<NY;j++){
        for(let k=0;k<NZ+1;k++){
          w[idxW(i,j,k)] += 0; 
        }
      }
    }
  }

  function pressureSolve(){
    const iter=20;
    div.fill(0); p.fill(0);
    for(let k=0;k<NZ;k++) for(let j=0;j<NY;j++) for(let i=0;i<NX;i++){
      const idx=idxC(i,j,k);
      div[idx] = u[idxU(i+1,j,k)] - u[idxU(i,j,k)] + v[idxV(i,j+1,k)] - v[idxV(i,j,k)] + w[idxW(i,j,k+1)] - w[idxW(i,j,k)];
    }
    for(let it=0;it<iter;it++){
      for(let k=1;k<NZ-1;k++) for(let j=1;j<NY-1;j++) for(let i=1;i<NX-1;i++){
        const idx=idxC(i,j,k);
        p[idx] = (div[idx] + p[idxC(i-1,j,k)] + p[idxC(i+1,j,k)] + p[idxC(i,j-1,k)] + p[idxC(i,j+1,k)] + p[idxC(i,j,k-1)] + p[idxC(i,j,k+1)])/6*omega + (1-omega)*p[idx];
      }
    }
    // Subtract gradient
    for(let k=1;k<NZ-1;k++) for(let j=1;j<NY-1;j++) for(let i=1;i<NX-1;i++){
      const idx=idxC(i,j,k);
      u[idxU(i,j,k)] -= p[idxC(i,j,k)]-p[idxC(i-1,j,k)];
      v[idxV(i,j,k)] -= p[idxC(i,j,k)]-p[idxC(i,j-1,k)];
      w[idxW(i,j,k)] -= p[idxC(i,j,k)]-p[idxC(i,j,k-1)];
    }
  }

  function gridToParticles(){
    for(const p of particles){
      const i = clamp(Math.floor(p.x*NX),0,NX-1);
      const j = clamp(Math.floor(p.y*NY),0,NY-1);
      const k = clamp(Math.floor(p.z*NZ),0,NZ-1);
      const vx = u[idxU(i,j,k)], vy = v[idxV(i,j,k)], vz = w[idxW(i,j,k)];
      p.vx = (1-alphaPIC)*p.vx + alphaPIC*vx;
      p.vy = (1-alphaPIC)*p.vy + alphaPIC*vy;
      p.vz = (1-alphaPIC)*p.vz + alphaPIC*vz;
      p.x = clamp(p.x + p.vx*dt,0,1);
      p.y = clamp(p.y + p.vy*dt,0,1);
      p.z = clamp(p.z + p.vz*dt,0,1);
    }
  }

  function stepBoat(){
    if(!hasBoat) return;
    const by = boat.y, submerged = clamp(0.5-by,0,1);
    const fy = submerged*boat.buoyK - boat.drag*boat.vy;
    boat.vy += fy*dt/boat.mass;
    boat.y = clamp(boat.y + boat.vy*dt,0,1);
  }

  function stepSensors(){
    for(const s of sensors){
      let e=0;
      for(const p of particles){
        const dx=p.x-s.x, dy=p.y-s.y, dz=p.z-s.z;
        const d2=dx*dx+dy*dy+dz*dz;
        e+=Math.exp(-100*d2)*Math.hypot(p.vx,p.vy,p.vz);
      }
      if(e-s.last>0.4) ping(200+Math.random()*400);
      s.last=e;
    }
  }

  // ---------- MAIN SIMULATION STEP ----------
  function step(){
    if(!running) return;
    markFluidCells();
    particlesToGrid();
    addForces();
    pressureSolve();
    gridToParticles();
    stepBoat();
    stepSensors();
    if(rainEnabled){
      for(const p of particles) if(Math.random()<0.02) p.vy+=0.3;
    }
  }

  // ---------- UI HOOKS ----------
  document.getElementById('playPause').onclick = ()=>{
    running = !running;
    document.getElementById('playPause').textContent = running?'⏸ Pause':'⏵ Play';
  };
  document.getElementById('reset').onclick = reset;
  document.getElementById('clearObstacles').onclick = ()=>{ obstacles.fill(0); };
  document.getElementById('toolObstacle').onclick = ()=>currentTool='obstacle';
  document.getElementById('toolErase').onclick = ()=>currentTool='erase';
  document.getElementById('toolPipe').onclick = ()=>currentTool='pipe';
  document.getElementById('toolDye').onclick = ()=>currentTool='dye';
  document.getElementById('toolBoat').onclick = ()=>{
    hasBoat=!hasBoat;
    document.getElementById('toolBoat').textContent = hasBoat?'🛥️ Remove Boat':'🛥️ Add Boat';
  };
  document.getElementById('toggleRain').onclick = ()=>{
    rainEnabled=!rainEnabled;
    document.getElementById('toggleRain').textContent = rainEnabled?'🌧️ Rain ON':'🌧️ Rain OFF';
  };
  document.getElementById('gravity').oninput = (e)=>{ g=parseFloat(e.target.value); document.getElementById('gLabel').textContent=g.toFixed(2); };
  document.getElementById('damp').oninput = (e)=>{ damp=parseFloat(e.target.value); document.getElementById('dampLabel').textContent=damp.toFixed(3); };
  document.getElementById('alpha').oninput = (e)=>{ alphaPIC=parseFloat(e.target.value); document.getElementById('alphaLabel').textContent=alphaPIC.toFixed(2); };
  document.getElementById('pipePressure').oninput = (e)=>{ PIPE_PRESSURE=parseFloat(e.target.value); document.getElementById('pipeLabel').textContent=PIPE_PRESSURE.toFixed(2); };
  document.getElementById('sliceZ').oninput = (e)=>{ sliceZ=parseInt(e.target.value); document.getElementById('sliceLabel').textContent=sliceZ; };
  document.getElementById('boatMass').oninput = (e)=>{ boat.mass=parseFloat(e.target.value); document.getElementById('massLabel').textContent=boat.mass.toFixed(2); };
  document.getElementById('applyGrid').onclick = ()=>{
    const val=document.getElementById('gridSize').value;
    [NX,NY,NZ]=val.split('x').map(Number); alloc(); reset();
  };

  // ---------- MAIN LOOP ----------
  function loop(){
    step(); render();
    requestAnimationFrame(loop);
  }

  // ---------- START ----------
  alloc(); reset(); loop();
})();
