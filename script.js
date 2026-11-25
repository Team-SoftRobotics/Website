/**
 * ===================================================================
 * SCPA | Soft Continuum Proprioceptive Arm
 * Interactive Digital Twin, Kinematics & UI Controller
 *
 * Palette-aligned:
 * - Primary Blue:     #5289AD
 * - Deep Navy:        #243C4C
 * - Muted Blue-Gray:  #698696
 * - Soft Gray:        #ACBCBF
 * - Off-White:        #F4FCFB
 * ===================================================================
 */

// -------------------------------------------------------------------
// 1. Dynamic Footer Year & Theme Guard
// -------------------------------------------------------------------
const currentYearEl = document.getElementById('current-year');
if (currentYearEl) {
  currentYearEl.textContent = new Date().getFullYear();
}

const themeToggleBtn = document.getElementById('theme-toggle');
if (themeToggleBtn) {
  const themeIcon = themeToggleBtn.querySelector('.theme-icon');
  function getSavedTheme() {
    const saved = localStorage.getItem('scpa-theme');
    if (saved) return saved;
    return window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
  }
  function applyTheme(theme) {
    document.documentElement.setAttribute('data-theme', theme);
    localStorage.setItem('scpa-theme', theme);
    if (themeIcon) themeIcon.textContent = theme === 'dark' ? '☀️' : '🌙';
  }
  applyTheme(getSavedTheme());
  themeToggleBtn.addEventListener('click', () => {
    const current = document.documentElement.getAttribute('data-theme') || 'dark';
    applyTheme(current === 'dark' ? 'light' : 'dark');
  });
}

// ===================================================================
// 2. DOC-OC CONTINUUM ROBOT KINEMATICS & MUJOCO PHYSICS ENGINE
// Direct Port of: https://github.com/Team-SoftRobotics/Simulations
// ===================================================================

const DOCOC_PARAMS = {
  // Geometry (meters) - exactly matching robot_params.yaml & scene.xml
  L: 0.096,                          // Arm centerline length (96.0 mm)
  r: 0.00676,                        // Tendon routing radius (6.76 mm)
  discCount: 11,                     // 11 precision discs
  discDiameter: 0.020,               // 20.0 mm diameter (10.0 mm radius)
  discThickness: 0.004,              // 4.0 mm thickness
  discPitch: 0.0096,                 // 9.6 mm disc pitch
  centralWireCount: 7,               // 7 central Nitinol core wires
  centralWireBundleRadius: 0.00075,  // 0.75 mm bundle radius
  
  // Motor & Drive specifications
  spoolRadius: 0.005,                // 5.0 mm effective spool radius
  stepAngleDeg: 0.18,                // 0.18 deg motor step angle
  microstepsPerPulse: 16,            // 16 microsteps per command pulse (from main.py)
  repeatPeriodMs: 30,                // 30 ms key repeat period (33.3 Hz repeat rate)
  maxCommandStepsX: 450,             // Max command steps X
  maxCommandStepsY: 450,             // Max command steps Y

  // Nitinol Mechanics
  nitinolFreeLength: 0.0959804,      // Free reference length (m)
  warningStrain: 0.07,               // 7.0% warning threshold
  hardStrainLimit: 0.08,             // 8.0% hard safety lock
  
  // 7 Central Wire Offsets (x, y in m relative to centerline)
  centralWireOffsets: [
    { x: 0.00000, y: 0.00000, label: 'W1' },
    { x: 0.00050, y: 0.00000, label: 'W2' },
    { x: 0.00025, y: 0.00043301, label: 'W3' },
    { x: -0.00025, y: 0.00043301, label: 'W4' },
    { x: -0.00050, y: 0.00000, label: 'W5' },
    { x: -0.00025, y: -0.00043301, label: 'W6' },
    { x: 0.00025, y: -0.00043301, label: 'W7' }
  ],

  // 4 Tendons (px, nx, py, ny)
  tendons: [
    { name: 'px', angle: 0.0, color: '#5289AD', label: '+X' },
    { name: 'nx', angle: Math.PI, color: '#355973', label: '-X' },
    { name: 'py', angle: Math.PI / 2, color: '#48c774', label: '+Y' },
    { name: 'ny', angle: -Math.PI / 2, color: '#243C4C', label: '-Y' }
  ]
};

// Derived motor linear displacement per command pulse (s = r_spool * theta_rad * 16)
const STEP_TRAVEL_M = DOCOC_PARAMS.spoolRadius * (DOCOC_PARAMS.stepAngleDeg * Math.PI / 180.0);
const COMMAND_DISPLACEMENT_M = DOCOC_PARAMS.microstepsPerPulse * STEP_TRAVEL_M; // ~0.251327 mm

class DocOcEngine {
  constructor() {
    this.xSteps = 0;
    this.ySteps = 0;
    this.totalSteps = 0;
    this.lastCommand = 'NONE';
    this.safetyLatched = false;
    this.activeKeys = new Set();
  }

  executeStep(key) {
    if (this.safetyLatched) return false;
    const k = key.toLowerCase();

    if (k === 'd') { // +X
      if (this.xSteps < DOCOC_PARAMS.maxCommandStepsX) {
        this.xSteps += 1;
        this.totalSteps += 1;
        this.lastCommand = 'D (+X)';
      }
    } else if (k === 'a') { // -X
      if (this.xSteps > -DOCOC_PARAMS.maxCommandStepsX) {
        this.xSteps -= 1;
        this.totalSteps += 1;
        this.lastCommand = 'A (-X)';
      }
    } else if (k === 'w') { // +Y
      if (this.ySteps < DOCOC_PARAMS.maxCommandStepsY) {
        this.ySteps += 1;
        this.totalSteps += 1;
        this.lastCommand = 'W (+Y)';
      }
    } else if (k === 's') { // -Y
      if (this.ySteps > -DOCOC_PARAMS.maxCommandStepsY) {
        this.ySteps -= 1;
        this.totalSteps += 1;
        this.lastCommand = 'S (-Y)';
      }
    } else {
      return false;
    }

    this.checkSafety();
    return true;
  }

  reset() {
    this.xSteps = 0;
    this.ySteps = 0;
    this.totalSteps = 0;
    this.lastCommand = 'RESET';
    this.safetyLatched = false;
    this.activeKeys.clear();
  }

  getTendonDeltas() {
    return {
      px: -this.xSteps * COMMAND_DISPLACEMENT_M,
      nx: +this.xSteps * COMMAND_DISPLACEMENT_M,
      py: -this.ySteps * COMMAND_DISPLACEMENT_M,
      ny: +this.ySteps * COMMAND_DISPLACEMENT_M
    };
  }

  solvePCC() {
    const { px, nx, py, ny } = this.getTendonDeltas();
    const L = DOCOC_PARAMS.L;
    const r = DOCOC_PARAMS.r;

    // Closed-form least-squares solution to J @ [kx, ky, eps] = delta_l
    const kx = (nx - px) / (2.0 * L * r);
    const ky = (ny - py) / (2.0 * L * r);
    const axialStrain = (px + nx + py + ny) / (4.0 * L);

    const curvature = Math.hypot(kx, ky);
    const bendingAngleRad = curvature * L;
    const bendingAngleDeg = bendingAngleRad * (180.0 / Math.PI);

    let directionRad = 0.0;
    if (curvature > 1e-12) {
      directionRad = Math.atan2(ky, kx);
    }
    const directionDeg = directionRad * (180.0 / Math.PI);

    return {
      kx,
      ky,
      curvature,
      bendingAngleRad,
      bendingAngleDeg,
      directionRad,
      directionDeg,
      axialStrain
    };
  }

  getTipPosition(pcc) {
    const { kx, ky, curvature, bendingAngleRad } = pcc;
    const L = DOCOC_PARAMS.L;

    if (curvature < 1e-12) {
      return { x: 0.0, y: 0.0, z: L, distXY: 0.0 };
    }

    const rho = 1.0 / curvature;
    const ux = kx / curvature;
    const uy = ky / curvature;

    const x = rho * (1.0 - Math.cos(bendingAngleRad)) * ux;
    const y = rho * (1.0 - Math.cos(bendingAngleRad)) * uy;
    const z = rho * Math.sin(bendingAngleRad);
    const distXY = Math.hypot(x, y);

    return { x, y, z, distXY };
  }

  getNitinolStrains(pcc) {
    const { kx, ky, axialStrain } = pcc;
    const L = DOCOC_PARAMS.L;
    const free = DOCOC_PARAMS.nitinolFreeLength;

    const strains = [];
    let maxTensile = 0.0;

    for (let i = 0; i < DOCOC_PARAMS.centralWireOffsets.length; i++) {
      const wire = DOCOC_PARAMS.centralWireOffsets[i];
      const wireLen = L * (1.0 + axialStrain - (kx * wire.x + ky * wire.y));
      const strain = (wireLen - free) / free;
      const tensile = Math.max(0.0, strain);
      strains.push({
        label: wire.label,
        strain,
        tensile
      });
      if (tensile > maxTensile) {
        maxTensile = tensile;
      }
    }

    return { strains, maxTensile };
  }

  checkSafety() {
    const pcc = this.solvePCC();
    const { maxTensile } = this.getNitinolStrains(pcc);
    if (maxTensile >= DOCOC_PARAMS.hardStrainLimit) {
      this.safetyLatched = true;
      this.lastCommand = 'SAFETY LOCK';
    }
  }
}

class DocOc3DViewer {
  constructor(canvasEl, engine, defaultView = 'iso') {
    this.canvas = canvasEl;
    this.ctx = canvasEl.getContext('2d');
    this.engine = engine;

    // View configurations
    if (defaultView === 'side') {
      this.azimuth = Math.PI / 2;
      this.elevation = 0;
    } else {
      this.azimuth = Math.PI / 4;   // 45 deg
      this.elevation = 22 * Math.PI / 180; // 22 deg
    }
    this.zoom = 1.0;

    this.isDragging = false;
    this.lastMouseX = 0;
    this.lastMouseY = 0;

    this.initEvents();
    this.resize();
    window.addEventListener('resize', () => {
      this.resize();
      this.render();
    });
  }

  setView(viewName) {
    if (viewName === 'iso') {
      this.azimuth = Math.PI / 4;
      this.elevation = 22 * Math.PI / 180;
    } else if (viewName === 'front') {
      this.azimuth = 0;
      this.elevation = 0;
    } else if (viewName === 'side') {
      this.azimuth = Math.PI / 2;
      this.elevation = 0;
    } else if (viewName === 'top') {
      this.azimuth = 0;
      this.elevation = 88 * Math.PI / 180;
    }
  }

  initEvents() {
    const c = this.canvas;
    c.addEventListener('mousedown', (e) => {
      this.isDragging = true;
      this.lastMouseX = e.clientX;
      this.lastMouseY = e.clientY;
    });

    window.addEventListener('mousemove', (e) => {
      if (!this.isDragging) return;
      const dx = e.clientX - this.lastMouseX;
      const dy = e.clientY - this.lastMouseY;
      this.lastMouseX = e.clientX;
      this.lastMouseY = e.clientY;

      this.azimuth += dx * 0.008;
      this.elevation += dy * 0.008;
      this.elevation = Math.max(-Math.PI / 2 + 0.05, Math.min(Math.PI / 2 - 0.05, this.elevation));
      this.render();
    });

    window.addEventListener('mouseup', () => {
      this.isDragging = false;
    });

    c.addEventListener('touchstart', (e) => {
      if (e.touches.length === 1) {
        this.isDragging = true;
        this.lastMouseX = e.touches[0].clientX;
        this.lastMouseY = e.touches[0].clientY;
      }
    }, { passive: true });

    window.addEventListener('touchmove', (e) => {
      if (!this.isDragging || e.touches.length !== 1) return;
      const dx = e.touches[0].clientX - this.lastMouseX;
      const dy = e.touches[0].clientY - this.lastMouseY;
      this.lastMouseX = e.touches[0].clientX;
      this.lastMouseY = e.touches[0].clientY;

      this.azimuth += dx * 0.008;
      this.elevation += dy * 0.008;
      this.elevation = Math.max(-Math.PI / 2 + 0.05, Math.min(Math.PI / 2 - 0.05, this.elevation));
      this.render();
    }, { passive: true });

    window.addEventListener('touchend', () => {
      this.isDragging = false;
    });
  }

  resize() {
    const rect = this.canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    const w = rect.width || this.canvas.width || 600;
    const h = rect.height || this.canvas.height || 380;
    this.canvas.width = w * dpr;
    this.canvas.height = h * dpr;
    this.width = w;
    this.height = h;
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  project(x, y, z) {
    const zCenter = DOCOC_PARAMS.L * 0.45;
    const xRel = x;
    const yRel = y;
    const zRel = z - zCenter;

    const cosA = Math.cos(this.azimuth);
    const sinA = Math.sin(this.azimuth);
    const x1 = xRel * cosA + yRel * sinA;
    const y1 = -xRel * sinA + yRel * cosA;
    const z1 = zRel;

    const cosE = Math.cos(this.elevation);
    const sinE = Math.sin(this.elevation);
    const x2 = x1;
    const y2 = y1 * cosE - z1 * sinE;
    const z2 = y1 * sinE + z1 * cosE;

    const dim = Math.max(280, Math.min(this.width, this.height));
    const scale = dim * 2.85 * this.zoom;
    const sx = this.width * 0.5 + x2 * scale;
    const sy = this.height * 0.52 - z2 * scale;

    return { x: sx, y: sy, depth: y2 };
  }

  render() {
    const ctx = this.ctx;
    ctx.clearRect(0, 0, this.width, this.height);

    const pcc = this.engine.solvePCC();
    const { kx, ky, curvature } = pcc;
    const L = DOCOC_PARAMS.L;
    const rTendon = DOCOC_PARAMS.r;
    const numDiscs = DOCOC_PARAMS.discCount;
    const discRadius = DOCOC_PARAMS.discDiameter / 2.0;

    const ux = curvature > 1e-12 ? kx / curvature : 0;
    const uy = curvature > 1e-12 ? ky / curvature : 0;
    const rho = curvature > 1e-12 ? 1.0 / curvature : 0;

    // Draw Ground Datum Grid
    this.drawGroundGrid();

    // 1. Compute 3D Discs and Frames
    const discs = [];
    for (let i = 0; i < numDiscs; i++) {
      const s = i * (L / (numDiscs - 1));
      let cx = 0, cy = 0, cz = s;
      let nx = ux, ny = uy, nz = 0;
      let bx = -uy, by = ux, bz = 0;

      if (curvature > 1e-12) {
        const theta_s = curvature * s;
        cx = rho * (1.0 - Math.cos(theta_s)) * ux;
        cy = rho * (1.0 - Math.cos(theta_s)) * uy;
        cz = rho * Math.sin(theta_s);

        nx = Math.cos(theta_s) * ux;
        ny = Math.cos(theta_s) * uy;
        nz = -Math.sin(theta_s);

        bx = -uy;
        by = ux;
        bz = 0;
      } else {
        nx = 1;
        ny = 0;
        nz = 0;
        bx = 0;
        by = 1;
        bz = 0;
      }

      // Compute disc perimeter in 3D
      const perimeter = [];
      const steps = 18;
      for (let j = 0; j < steps; j++) {
        const angle = (j / steps) * Math.PI * 2;
        const cosAng = Math.cos(angle);
        const sinAng = Math.sin(angle);
        
        const px = cx + discRadius * (cosAng * nx + sinAng * bx);
        const py = cy + discRadius * (cosAng * ny + sinAng * by);
        const pz = cz + discRadius * (cosAng * nz + sinAng * bz);
        perimeter.push({ x: px, y: py, z: pz });
      }

      // 4 Tendon Guide Holes in 3D
      const tendonHoles = {
        px: this.computeDiscPoint(cx, cy, cz, nx, ny, nz, bx, by, bz, rTendon, 0),
        nx: this.computeDiscPoint(cx, cy, cz, nx, ny, nz, bx, by, bz, rTendon, Math.PI),
        py: this.computeDiscPoint(cx, cy, cz, nx, ny, nz, bx, by, bz, rTendon, Math.PI / 2),
        ny: this.computeDiscPoint(cx, cy, cz, nx, ny, nz, bx, by, bz, rTendon, -Math.PI / 2)
      };

      // 7 Central Nitinol Core Holes in 3D
      const wireHoles = DOCOC_PARAMS.centralWireOffsets.map(wire => {
        const wx = cx + wire.x * nx + wire.y * bx;
        const wy = cy + wire.x * ny + wire.y * by;
        const wz = cz + wire.x * nz + wire.y * bz;
        return { x: wx, y: wy, z: wz };
      });

      const centerProj = this.project(cx, cy, cz);

      discs.push({
        index: i + 1,
        center: { x: cx, y: cy, z: cz },
        centerProj,
        depth: centerProj.depth,
        perimeter,
        tendonHoles,
        wireHoles
      });
    }

    // 2. Draw 7 Nitinol Core Wires
    for (let w = 0; w < DOCOC_PARAMS.centralWireOffsets.length; w++) {
      ctx.beginPath();
      ctx.strokeStyle = w === 0 ? 'rgba(172, 188, 191, 0.75)' : 'rgba(172, 188, 191, 0.45)';
      ctx.lineWidth = w === 0 ? 1.6 : 1.0;
      for (let i = 0; i < discs.length; i++) {
        const p = this.project(discs[i].wireHoles[w].x, discs[i].wireHoles[w].y, discs[i].wireHoles[w].z);
        if (i === 0) ctx.moveTo(p.x, p.y);
        else ctx.lineTo(p.x, p.y);
      }
      ctx.stroke();
    }

    // 3. Draw 4 Actuation Tendons
    const tendonConfigs = [
      { key: 'px', color: '#5289AD', width: 2.2 },
      { key: 'nx', color: '#355973', width: 1.8 },
      { key: 'py', color: '#48c774', width: 2.0 },
      { key: 'ny', color: '#243C4C', width: 1.8 }
    ];

    tendonConfigs.forEach(({ key, color, width }) => {
      ctx.beginPath();
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.shadowColor = color;
      ctx.shadowBlur = 4;
      for (let i = 0; i < discs.length; i++) {
        const pt = discs[i].tendonHoles[key];
        const p = this.project(pt.x, pt.y, pt.z);
        if (i === 0) ctx.moveTo(p.x, p.y);
        else ctx.lineTo(p.x, p.y);
      }
      ctx.stroke();
      ctx.shadowBlur = 0;
    });

    // 4. Draw 11 Precision Discs (Back-to-front depth sorting)
    const sortedDiscs = [...discs].sort((a, b) => a.depth - b.depth);
    sortedDiscs.forEach(disc => {
      this.drawDisc(disc);
    });

    // 5. Draw Tip Frame at Disc 11
    const tipDisc = discs[discs.length - 1];
    this.drawTipFrame(tipDisc, pcc);
  }

  computeDiscPoint(cx, cy, cz, nx, ny, nz, bx, by, bz, radius, angle) {
    const cosA = Math.cos(angle);
    const sinA = Math.sin(angle);
    return {
      x: cx + radius * (cosA * nx + sinA * bx),
      y: cy + radius * (cosA * ny + sinA * by),
      z: cz + radius * (cosA * nz + sinA * bz)
    };
  }

  drawGroundGrid() {
    const ctx = this.ctx;
    ctx.save();
    ctx.strokeStyle = 'rgba(172, 188, 191, 0.09)';
    ctx.lineWidth = 1;

    const r = 0.045; // 45 mm ground radius
    for (let rad = 0.015; rad <= r; rad += 0.015) {
      ctx.beginPath();
      const steps = 32;
      for (let i = 0; i <= steps; i++) {
        const a = (i / steps) * Math.PI * 2;
        const p = this.project(rad * Math.cos(a), rad * Math.sin(a), 0);
        if (i === 0) ctx.moveTo(p.x, p.y);
        else ctx.lineTo(p.x, p.y);
      }
      ctx.stroke();
    }

    const p1 = this.project(-r, 0, 0);
    const p2 = this.project(r, 0, 0);
    ctx.beginPath();
    ctx.moveTo(p1.x, p1.y);
    ctx.lineTo(p2.x, p2.y);
    ctx.stroke();

    const p3 = this.project(0, -r, 0);
    const p4 = this.project(0, r, 0);
    ctx.beginPath();
    ctx.moveTo(p3.x, p3.y);
    ctx.lineTo(p4.x, p4.y);
    ctx.stroke();

    // Base Mount Plate (Disc 1 anchor)
    const baseP = this.project(0, 0, 0);
    ctx.fillStyle = '#1c2e3b';
    ctx.strokeStyle = '#5289AD';
    ctx.lineWidth = 1.4;
    ctx.beginPath();
    ctx.arc(baseP.x, baseP.y, 16, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    ctx.restore();
  }

  drawDisc(disc) {
    const ctx = this.ctx;
    ctx.save();

    // 1. Draw Disc Face
    ctx.beginPath();
    for (let i = 0; i < disc.perimeter.length; i++) {
      const p = this.project(disc.perimeter[i].x, disc.perimeter[i].y, disc.perimeter[i].z);
      if (i === 0) ctx.moveTo(p.x, p.y);
      else ctx.lineTo(p.x, p.y);
    }
    ctx.closePath();

    ctx.fillStyle = disc.index === 11 ? '#F4FCFB' : 'rgba(235, 246, 248, 0.92)';
    ctx.fill();

    ctx.strokeStyle = disc.index === 11 ? '#5289AD' : 'rgba(105, 134, 150, 0.65)';
    ctx.lineWidth = disc.index === 11 ? 1.6 : 1.1;
    ctx.stroke();

    // 2. Draw Tendon Eyelets
    ['px', 'nx', 'py', 'ny'].forEach(tKey => {
      const hole = disc.tendonHoles[tKey];
      const p = this.project(hole.x, hole.y, hole.z);
      ctx.beginPath();
      ctx.fillStyle = '#1a2b37';
      ctx.arc(p.x, p.y, 1.8, 0, Math.PI * 2);
      ctx.fill();
    });

    // 3. Central Wire Bundle Bore
    const cp = disc.centerProj;
    ctx.beginPath();
    ctx.fillStyle = '#13212a';
    ctx.arc(cp.x, cp.y, 2.8, 0, Math.PI * 2);
    ctx.fill();

    // 4. Disc index numeral for first, middle, and tip discs
    if (disc.index === 1 || disc.index === 6 || disc.index === 11) {
      ctx.font = '600 9px monospace';
      ctx.fillStyle = 'rgba(36, 60, 76, 0.75)';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(`D${disc.index}`, cp.x + 10, cp.y - 6);
    }

    ctx.restore();
  }

  drawTipFrame(tipDisc, pcc) {
    const ctx = this.ctx;
    ctx.save();
    const cp = tipDisc.centerProj;

    ctx.beginPath();
    ctx.fillStyle = '#5289AD';
    ctx.shadowColor = '#5289AD';
    ctx.shadowBlur = 10;
    ctx.arc(cp.x, cp.y, 4.5, 0, Math.PI * 2);
    ctx.fill();
    ctx.shadowBlur = 0;

    const { kx, ky, curvature, bendingAngleRad } = pcc;
    let tx = 0, ty = 0, tz = 1;
    if (curvature > 1e-12) {
      const ux = kx / curvature;
      const uy = ky / curvature;
      tx = Math.sin(bendingAngleRad) * ux;
      ty = Math.sin(bendingAngleRad) * uy;
      tz = Math.cos(bendingAngleRad);
    }

    const tipVecLen = 0.018; // 18 mm vector
    const tipEnd = this.project(
      tipDisc.center.x + tx * tipVecLen,
      tipDisc.center.y + ty * tipVecLen,
      tipDisc.center.z + tz * tipVecLen
    );

    ctx.beginPath();
    ctx.strokeStyle = '#F4FCFB';
    ctx.lineWidth = 2.2;
    ctx.moveTo(cp.x, cp.y);
    ctx.lineTo(tipEnd.x, tipEnd.y);
    ctx.stroke();

    ctx.restore();
  }
}

// ===================================================================
// Instantiate Engine & Viewers for Hero Window & Kinematics Studio
// ===================================================================

const docOcEngine = new DocOcEngine();

// 1. Hero Canvas Viewer
const heroCanvas = document.getElementById('hero-arm-canvas');
let heroViewer = null;
if (heroCanvas) {
  heroViewer = new DocOc3DViewer(heroCanvas, docOcEngine, 'iso');
}

// 2. Studio Canvas Viewer
const studioCanvas = document.getElementById('studio-arm-canvas');
let studioViewer = null;
if (studioCanvas) {
  studioViewer = new DocOc3DViewer(studioCanvas, docOcEngine, 'iso');
}

// Telemetry DOM Bindings
// Hero Telemetry
const heroTelPos = document.getElementById('hero-tel-pos');
const heroTelCurvature = document.getElementById('hero-tel-curvature');
const heroTelStrain = document.getElementById('hero-tel-strain');

// Studio Telemetry
const safetyStatusBadge = document.getElementById('safety-status-badge');
const statLastCmd = document.getElementById('stat-last-cmd');
const statTotalSteps = document.getElementById('stat-total-steps');
const statXSteps = document.getElementById('stat-x-steps');
const statYSteps = document.getElementById('stat-y-steps');

const statTipX = document.getElementById('stat-tip-x');
const statTipY = document.getElementById('stat-tip-y');
const statTipZ = document.getElementById('stat-tip-z');
const statTipXY = document.getElementById('stat-tip-xy');

const statCurvTotal = document.getElementById('stat-curv-total');
const statBendAngle = document.getElementById('stat-bend-angle');
const statCurvX = document.getElementById('stat-curv-x');
const statCurvY = document.getElementById('stat-curv-y');
const statBendDir = document.getElementById('stat-bend-dir');
const statAxialStrain = document.getElementById('stat-axial-strain');

const wireEls = [1, 2, 3, 4, 5, 6, 7].map(i => document.getElementById(`wire-${i}`));
const wireMaxEl = document.getElementById('wire-max');
const strainProgressBar = document.getElementById('strain-progress-bar');

const tenPxVal = document.getElementById('ten-px-val');
const tenPxDelta = document.getElementById('ten-px-delta');
const tenNxVal = document.getElementById('ten-nx-val');
const tenNxDelta = document.getElementById('ten-nx-delta');
const tenPyVal = document.getElementById('ten-py-val');
const tenPyDelta = document.getElementById('ten-py-delta');
const tenNyVal = document.getElementById('ten-ny-val');
const tenNyDelta = document.getElementById('ten-ny-delta');

const activeKeysTag = document.getElementById('active-keys-tag');

function updateAllTelemetry() {
  const pcc = docOcEngine.solvePCC();
  const tip = docOcEngine.getTipPosition(pcc);
  const nitinol = docOcEngine.getNitinolStrains(pcc);
  const deltas = docOcEngine.getTendonDeltas();

  // Hero Window Telemetry
  if (heroTelPos) {
    heroTelPos.textContent = `${(tip.x * 1000).toFixed(1)}, ${(tip.y * 1000).toFixed(1)}, ${(tip.z * 1000).toFixed(1)} mm`;
  }
  if (heroTelCurvature) {
    heroTelCurvature.textContent = `${pcc.curvature.toFixed(4)} 1/m (θ: ${pcc.bendingAngleDeg.toFixed(1)}°)`;
  }
  if (heroTelStrain) {
    const maxTensilePct = (nitinol.maxTensile * 100).toFixed(2);
    const statusText = docOcEngine.safetyLatched ? '[LOCKED]' : 
                       nitinol.maxTensile >= DOCOC_PARAMS.warningStrain ? '[WARN]' : '[OK]';
    heroTelStrain.textContent = `${maxTensilePct}% ${statusText}`;
    heroTelStrain.style.color = docOcEngine.safetyLatched ? '#ef476f' :
                               nitinol.maxTensile >= DOCOC_PARAMS.warningStrain ? '#ffd166' : 'var(--primary-blue)';
  }

  // Studio Window Telemetry
  if (safetyStatusBadge) {
    if (docOcEngine.safetyLatched) {
      safetyStatusBadge.className = 'badge-status-locked';
      safetyStatusBadge.textContent = 'HARD LIMIT / LOCKED';
    } else if (nitinol.maxTensile >= DOCOC_PARAMS.warningStrain) {
      safetyStatusBadge.className = 'badge-status-warning';
      safetyStatusBadge.textContent = 'STRAIN WARNING';
    } else {
      safetyStatusBadge.className = 'badge-status-nominal';
      safetyStatusBadge.textContent = 'NOMINAL [OK]';
    }
  }

  if (statLastCmd) statLastCmd.textContent = docOcEngine.lastCommand;
  if (statTotalSteps) statTotalSteps.textContent = docOcEngine.totalSteps;
  if (statXSteps) statXSteps.innerHTML = `${docOcEngine.xSteps >= 0 ? '+' : ''}${docOcEngine.xSteps} <small>/ ±${DOCOC_PARAMS.maxCommandStepsX}</small>`;
  if (statYSteps) statYSteps.innerHTML = `${docOcEngine.ySteps >= 0 ? '+' : ''}${docOcEngine.ySteps} <small>/ ±${DOCOC_PARAMS.maxCommandStepsY}</small>`;

  if (statTipX) statTipX.textContent = `${(tip.x * 1000).toFixed(3)} mm`;
  if (statTipY) statTipY.textContent = `${(tip.y * 1000).toFixed(3)} mm`;
  if (statTipZ) statTipZ.textContent = `${(tip.z * 1000).toFixed(3)} mm`;
  if (statTipXY) statTipXY.textContent = `${(tip.distXY * 1000).toFixed(3)} mm`;

  if (statCurvTotal) statCurvTotal.textContent = `${pcc.curvature.toFixed(4)} 1/m`;
  if (statBendAngle) statBendAngle.textContent = `${pcc.bendingAngleDeg.toFixed(3)} deg`;
  if (statCurvX) statCurvX.textContent = `${pcc.kx.toFixed(4)} 1/m`;
  if (statCurvY) statCurvY.textContent = `${pcc.ky.toFixed(4)} 1/m`;
  if (statBendDir) statBendDir.textContent = `${pcc.directionDeg.toFixed(3)} deg`;
  if (statAxialStrain) statAxialStrain.textContent = `${(pcc.axialStrain * 100).toFixed(4)} %`;

  nitinol.strains.forEach((wire, idx) => {
    if (wireEls[idx]) {
      wireEls[idx].textContent = `${(wire.strain * 100).toFixed(3)}%`;
      wireEls[idx].style.color = wire.tensile >= DOCOC_PARAMS.hardStrainLimit ? '#ef476f' :
                                wire.tensile >= DOCOC_PARAMS.warningStrain ? '#ffd166' : 'var(--off-white)';
    }
  });

  const maxTensilePercent = nitinol.maxTensile * 100;
  if (wireMaxEl) {
    wireMaxEl.textContent = `${maxTensilePercent.toFixed(3)}%`;
    wireMaxEl.style.color = nitinol.maxTensile >= DOCOC_PARAMS.hardStrainLimit ? '#ef476f' :
                            nitinol.maxTensile >= DOCOC_PARAMS.warningStrain ? '#ffd166' : 'var(--primary-blue)';
  }

  if (strainProgressBar) {
    const pctOfHardLimit = Math.min(100, (nitinol.maxTensile / DOCOC_PARAMS.hardStrainLimit) * 100);
    strainProgressBar.style.width = `${pctOfHardLimit}%`;
    strainProgressBar.className = 'strain-progress-fill' + 
      (nitinol.maxTensile >= DOCOC_PARAMS.hardStrainLimit ? ' locked' :
       nitinol.maxTensile >= DOCOC_PARAMS.warningStrain ? ' warning' : '');
  }

  const L0_MM = DOCOC_PARAMS.L * 1000;
  if (tenPxVal) tenPxVal.textContent = `${(L0_MM + deltas.px * 1000).toFixed(3)} mm`;
  if (tenPxDelta) tenPxDelta.textContent = `Δ: ${(deltas.px * 1000).toFixed(3)} mm`;

  if (tenNxVal) tenNxVal.textContent = `${(L0_MM + deltas.nx * 1000).toFixed(3)} mm`;
  if (tenNxDelta) tenNxDelta.textContent = `Δ: ${(deltas.nx * 1000).toFixed(3)} mm`;

  if (tenPyVal) tenPyVal.textContent = `${(L0_MM + deltas.py * 1000).toFixed(3)} mm`;
  if (tenPyDelta) tenPyDelta.textContent = `Δ: ${(deltas.py * 1000).toFixed(3)} mm`;

  if (tenNyVal) tenNyVal.textContent = `${(L0_MM + deltas.ny * 1000).toFixed(3)} mm`;
  if (tenNyDelta) tenNyDelta.textContent = `Δ: ${(deltas.ny * 1000).toFixed(3)} mm`;

  if (activeKeysTag) {
    const keys = Array.from(docOcEngine.activeKeys).sort().join('+').toUpperCase();
    activeKeysTag.textContent = keys || 'NONE';
  }
}

// Initial render & sync
if (heroViewer) heroViewer.render();
if (studioViewer) studioViewer.render();
updateAllTelemetry();

// Unified Actuation Handlers (Hero and Studio buttons + Keyboard)
let docOcRepeatTimer = null;
let docOcInitialTimer = null;

const allKeyButtons = {
  w: [document.getElementById('hero-key-w'), document.getElementById('key-w')],
  a: [document.getElementById('hero-key-a'), document.getElementById('key-a')],
  s: [document.getElementById('hero-key-s'), document.getElementById('key-s')],
  d: [document.getElementById('hero-key-d'), document.getElementById('key-d')]
};

function renderAllViewers() {
  if (heroViewer) heroViewer.render();
  if (studioViewer) studioViewer.render();
}

function startActuation(key) {
  if (docOcEngine.safetyLatched) return;
  const k = key.toLowerCase();
  docOcEngine.activeKeys.add(k);

  docOcEngine.executeStep(k);
  updateAllTelemetry();
  renderAllViewers();

  if (allKeyButtons[k]) {
    allKeyButtons[k].forEach(b => b?.classList.add('active'));
  }

  stopActuationTimers();

  docOcInitialTimer = setTimeout(() => {
    docOcRepeatTimer = setInterval(() => {
      if (docOcEngine.safetyLatched) {
        stopActuationTimers();
        return;
      }
      docOcEngine.executeStep(k);
      updateAllTelemetry();
      renderAllViewers();
    }, DOCOC_PARAMS.repeatPeriodMs);
  }, 180);
}

function stopActuationTimers() {
  if (docOcInitialTimer) {
    clearTimeout(docOcInitialTimer);
    docOcInitialTimer = null;
  }
  if (docOcRepeatTimer) {
    clearInterval(docOcRepeatTimer);
    docOcRepeatTimer = null;
  }
}

function stopActuation(key) {
  if (key) {
    const k = key.toLowerCase();
    docOcEngine.activeKeys.delete(k);
    if (allKeyButtons[k]) {
      allKeyButtons[k].forEach(b => b?.classList.remove('active'));
    }
  }
  stopActuationTimers();
  updateAllTelemetry();
}

// Bind Hero and Studio Buttons
['w', 'a', 's', 'd'].forEach(k => {
  const heroBtn = document.getElementById(`hero-key-${k}`);
  if (heroBtn) {
    heroBtn.addEventListener('mousedown', (e) => { e.preventDefault(); startActuation(k); });
    heroBtn.addEventListener('mouseup', () => stopActuation(k));
    heroBtn.addEventListener('mouseleave', () => stopActuation(k));
    heroBtn.addEventListener('touchstart', (e) => { e.preventDefault(); startActuation(k); }, { passive: false });
    heroBtn.addEventListener('touchend', () => stopActuation(k));
    heroBtn.addEventListener('touchcancel', () => stopActuation(k));
  }

  const studioBtn = document.getElementById(`key-${k}`);
  if (studioBtn) {
    studioBtn.addEventListener('mousedown', (e) => { e.preventDefault(); startActuation(k); });
    studioBtn.addEventListener('mouseup', () => stopActuation(k));
    studioBtn.addEventListener('mouseleave', () => stopActuation(k));
    studioBtn.addEventListener('touchstart', (e) => { e.preventDefault(); startActuation(k); }, { passive: false });
    studioBtn.addEventListener('touchend', () => stopActuation(k));
    studioBtn.addEventListener('touchcancel', () => stopActuation(k));
  }
});

// Reset Buttons
const heroResetBtn = document.getElementById('hero-key-reset');
if (heroResetBtn) {
  heroResetBtn.addEventListener('click', () => {
    docOcEngine.reset();
    stopActuationTimers();
    updateAllTelemetry();
    renderAllViewers();
  });
}

const studioResetBtn = document.getElementById('key-reset');
if (studioResetBtn) {
  studioResetBtn.addEventListener('click', () => {
    docOcEngine.reset();
    stopActuationTimers();
    updateAllTelemetry();
    renderAllViewers();
  });
}

// Studio View Angle Presets
const viewBtns = {
  iso: document.getElementById('view-iso'),
  front: document.getElementById('view-front'),
  side: document.getElementById('view-side'),
  top: document.getElementById('view-top')
};

Object.entries(viewBtns).forEach(([viewKey, btn]) => {
  if (!btn) return;
  btn.addEventListener('click', () => {
    Object.values(viewBtns).forEach(b => b?.classList.remove('active'));
    btn.classList.add('active');
    if (studioViewer) {
      studioViewer.setView(viewKey);
      studioViewer.render();
    }
  });
});

// Physical Keyboard Listener (WASD, Arrows, Reset)
window.addEventListener('keydown', (e) => {
  if (['INPUT', 'TEXTAREA', 'SELECT'].includes(document.activeElement?.tagName)) return;

  let mappedKey = null;
  if (e.key === 'w' || e.key === 'W' || e.key === 'ArrowUp') mappedKey = 'w';
  else if (e.key === 'a' || e.key === 'A' || e.key === 'ArrowLeft') mappedKey = 'a';
  else if (e.key === 's' || e.key === 'S' || e.key === 'ArrowDown') mappedKey = 's';
  else if (e.key === 'd' || e.key === 'D' || e.key === 'ArrowRight') mappedKey = 'd';
  else if (e.key === 'r' || e.key === 'R' || e.key === 'Escape') {
    docOcEngine.reset();
    stopActuationTimers();
    updateAllTelemetry();
    renderAllViewers();
    return;
  }

  if (mappedKey && !docOcEngine.activeKeys.has(mappedKey)) {
    e.preventDefault();
    startActuation(mappedKey);
  }
});

window.addEventListener('keyup', (e) => {
  let mappedKey = null;
  if (e.key === 'w' || e.key === 'W' || e.key === 'ArrowUp') mappedKey = 'w';
  else if (e.key === 'a' || e.key === 'A' || e.key === 'ArrowLeft') mappedKey = 'a';
  else if (e.key === 's' || e.key === 'S' || e.key === 'ArrowDown') mappedKey = 's';
  else if (e.key === 'd' || e.key === 'D' || e.key === 'ArrowRight') mappedKey = 'd';

  if (mappedKey) {
    stopActuation(mappedKey);
  }
});


// -------------------------------------------------------------------
// 4. Project Updates Filter Logic
// -------------------------------------------------------------------
const filterBtns = document.querySelectorAll('.filter-btn, .filter-chip');
const timelineEntries = document.querySelectorAll('.timeline-entry, .timeline-item');

filterBtns.forEach(btn => {
  btn.addEventListener('click', () => {
    filterBtns.forEach(b => b.classList.remove('active'));
    btn.classList.add('active');

    const filter = btn.getAttribute('data-filter');

    timelineEntries.forEach(entry => {
      const categories = entry.getAttribute('data-category') || '';
      if (filter === 'all' || categories.includes(filter)) {
        entry.classList.remove('hidden');
      } else {
        entry.classList.add('hidden');
      }
    });
  });
});



// -------------------------------------------------------------------
// 6. Photo Enlargement Lightbox Modal
// -------------------------------------------------------------------
const photoModal = document.getElementById('photo-modal');
const modalImg = document.getElementById('modal-img');
const modalName = document.getElementById('modal-name');
const modalRole = document.getElementById('modal-role');
const modalCloseBtn = document.getElementById('modal-close');
const modalBackdrop = document.getElementById('modal-backdrop');

function openPhotoModal(src, name, role) {
  if (!photoModal || !modalImg) return;
  modalImg.src = src;
  modalImg.alt = name;
  if (modalName) modalName.textContent = name;
  if (modalRole) modalRole.textContent = role;
  photoModal.classList.add('active');
  photoModal.setAttribute('aria-hidden', 'false');
  document.body.style.overflow = 'hidden';
}

function closePhotoModal() {
  if (!photoModal) return;
  photoModal.classList.remove('active');
  photoModal.setAttribute('aria-hidden', 'true');
  document.body.style.overflow = '';
}

if (modalCloseBtn) modalCloseBtn.addEventListener('click', closePhotoModal);
if (modalBackdrop) modalBackdrop.addEventListener('click', closePhotoModal);

window.addEventListener('keydown', (e) => {
  if (e.key === 'Escape' && photoModal && photoModal.classList.contains('active')) {
    closePhotoModal();
  }
});

// Collaborator photo clicks (Enlarge on click)
const colabPhotos = document.querySelectorAll('.colab-avatar-container');
colabPhotos.forEach((container) => {
  container.addEventListener('click', () => {
    const img = container.querySelector('.member-avatar');
    if (img) {
      const name = img.getAttribute('data-name') || 'Researcher';
      const role = img.getAttribute('data-role') || 'Project Collaborator';
      openPhotoModal(img.currentSrc || img.src, name, role);
    }
  });
});

// CAD Figure Cards & Technical Diagrams (Enlarge on click)
const cadCards = document.querySelectorAll('.cad-figure-card');
cadCards.forEach((card) => {
  card.addEventListener('click', () => {
    const img = card.querySelector('img');
    const title = card.querySelector('.cad-fig-title')?.textContent || 'Technical Figure';
    const num = card.querySelector('.cad-fig-num')?.textContent || 'SCPA Hardware';
    if (img) {
      openPhotoModal(img.currentSrc || img.src, title, num);
    }
  });
});

// -------------------------------------------------------------------
// 7. Lead Photo Sliding Window Controller & Enlarge Trigger
// -------------------------------------------------------------------
const leadSliders = document.querySelectorAll('.lead-slider-wrapper');

leadSliders.forEach((sliderWrap) => {
  const track = sliderWrap.querySelector('.lead-slider-track');
  const dots = sliderWrap.querySelectorAll('.dot-btn');
  const prevBtn = sliderWrap.querySelector('.prev-btn');
  const nextBtn = sliderWrap.querySelector('.next-btn');
  const windowEl = sliderWrap.querySelector('.lead-slider-window');
  const slides = sliderWrap.querySelectorAll('.lead-slide');
  let currentSlide = 0;

  function goToSlide(index) {
    currentSlide = index;
    if (track) track.setAttribute('data-active', currentSlide);
    dots.forEach((dot, idx) => {
      dot.classList.toggle('active', idx === currentSlide);
    });
  }

  // Click on indicator dots
  dots.forEach((dot) => {
    dot.addEventListener('click', (e) => {
      e.stopPropagation();
      const targetIndex = parseInt(dot.getAttribute('data-slide'), 10) || 0;
      goToSlide(targetIndex);
    });
  });

  // Next / Prev button arrows to switch slide
  if (nextBtn) {
    nextBtn.addEventListener('click', (e) => {
      e.stopPropagation();
      goToSlide(currentSlide === 0 ? 1 : 0);
    });
  }

  if (prevBtn) {
    prevBtn.addEventListener('click', (e) => {
      e.stopPropagation();
      goToSlide(currentSlide === 0 ? 1 : 0);
    });
  }

  // Clicking on the photo window enlarges the currently active photo
  if (windowEl) {
    windowEl.addEventListener('click', (e) => {
      // Don't trigger enlargement if user clicked directly on navigation arrows or dots
      if (e.target.closest('.slider-arrow') || e.target.closest('.slider-dots')) return;

      const activeSlide = slides[currentSlide];
      const img = activeSlide ? activeSlide.querySelector('.member-avatar') : null;
      if (img) {
        const name = img.getAttribute('data-name') || 'Project Lead';
        const role = img.getAttribute('data-role') || 'Project Lead';
        openPhotoModal(img.currentSrc || img.src, name, role);
      }
    });
  }
});


