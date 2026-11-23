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

// -------------------------------------------------------------------
// 2. Interactive Soft Continuum Arm Simulation (Canvas)
// -------------------------------------------------------------------
const canvas = document.getElementById('continuum-canvas');
const ctx = canvas ? canvas.getContext('2d') : null;

// HUD Telemetry Elements
const telPos = document.getElementById('tel-pos');
const telCurvature = document.getElementById('tel-curvature');
const telStrain = document.getElementById('tel-strain');
const toggleModeBtn = document.getElementById('toggle-sim-mode');
const simModeText = document.getElementById('sim-mode-text');

// Studio Sliders & Elements
const seg1Slider = document.getElementById('seg1-slider');
const seg2Slider = document.getElementById('seg2-slider');
const seg3Slider = document.getElementById('seg3-slider');
const seg1Val = document.getElementById('seg1-val');
const seg2Val = document.getElementById('seg2-val');
const seg3Val = document.getElementById('seg3-val');
const strainDorsal = document.getElementById('strain-dorsal');
const strainVentral = document.getElementById('strain-ventral');
const tipAngleEl = document.getElementById('tip-angle');
const btnReset = document.getElementById('btn-reset-sliders');
const btnSCurve = document.getElementById('btn-preset-s-curve');
const btnSpiral = document.getElementById('btn-preset-spiral');

let simMode = 'interactive'; // 'interactive' or 'auto'
let mouseTarget = { x: 0, y: 0, active: false };

// Kinematic State (Curvatures for 3 segments)
let k1 = 0, k2 = 0, k3 = 0;
let targetK1 = 0, targetK2 = 0, targetK3 = 0;
const SEGMENT_LENGTH = 75; // px per segment
const NUM_VERTEBRAE = 24;  // discretization steps for smooth rendering
const ARM_RADIUS = 16;      // radius/thickness of soft silicone body

// End effector trail
const trail = [];
const MAX_TRAIL = 35;

function resizeCanvas() {
  if (!canvas || !ctx) return;
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = rect.width * dpr;
  canvas.height = rect.height * dpr;
  ctx.scale(dpr, dpr);
}

if (canvas) {
  window.addEventListener('resize', resizeCanvas);
  resizeCanvas();

  // Mouse tracking on Canvas
  canvas.addEventListener('mousemove', (e) => {
    const rect = canvas.getBoundingClientRect();
    mouseTarget.x = e.clientX - rect.left;
    mouseTarget.y = e.clientY - rect.top;
    mouseTarget.active = true;
  });

  canvas.addEventListener('mouseleave', () => {
    mouseTarget.active = false;
  });
}

if (toggleModeBtn && simModeText) {
  toggleModeBtn.addEventListener('click', () => {
    simMode = simMode === 'interactive' ? 'auto' : 'interactive';
    simModeText.textContent = simMode === 'interactive' ? 'Mode: Interactive' : 'Mode: Auto-Sweep';
    toggleModeBtn.style.color = simMode === 'interactive' ? '#5289AD' : '#ACBCBF';
  });
}

// Compute Soft Arm Backbone points using Piecewise Constant Curvature (PCC)
function computeBackbone(curvatures, basePos) {
  const points = [];
  const orientations = [];
  
  let currentPos = { x: basePos.x, y: basePos.y };
  let currentAngle = -Math.PI / 2; // Pointing upwards initially

  points.push({ ...currentPos });
  orientations.push(currentAngle);

  const totalSegments = curvatures.length;
  const subStepsPerSegment = NUM_VERTEBRAE / totalSegments;

  for (let s = 0; s < totalSegments; s++) {
    const kappa = curvatures[s];
    const ds = SEGMENT_LENGTH / subStepsPerSegment;

    for (let i = 0; i < subStepsPerSegment; i++) {
      if (Math.abs(kappa) < 0.0001) {
        // Straight segment
        currentPos.x += ds * Math.cos(currentAngle);
        currentPos.y += ds * Math.sin(currentAngle);
      } else {
        // Arc segment
        const dTheta = kappa * ds;
        currentPos.x += (Math.sin(currentAngle + dTheta) - Math.sin(currentAngle)) / kappa;
        currentPos.y += (-Math.cos(currentAngle + dTheta) + Math.cos(currentAngle)) / kappa;
        currentAngle += dTheta;
      }
      points.push({ ...currentPos });
      orientations.push(currentAngle);
    }
  }

  return { points, orientations };
}

// Main Animation Loop
let animTime = 0;

function renderSimulation() {
  if (!canvas || !ctx) return;
  animTime += 0.025;
  const rect = canvas.getBoundingClientRect();
  const width = rect.width;
  const height = rect.height;
  const basePos = { x: width * 0.5, y: height * 0.88 };

  // Control Logic based on mode
  if (simMode === 'auto') {
    targetK1 = Math.sin(animTime * 0.8) * 0.012;
    targetK2 = Math.cos(animTime * 1.1) * 0.015;
    targetK3 = Math.sin(animTime * 1.4 + 1.0) * 0.018;
  } else if (mouseTarget.active) {
    // Map mouse position relative to base to segment curvatures
    const dx = (mouseTarget.x - basePos.x) / (width * 0.5);
    const dy = (basePos.y - mouseTarget.y) / (height * 0.8);
    
    // Smooth inverse target mapping
    targetK1 = dx * 0.014;
    targetK2 = (dx * 1.1 + Math.sin(dy * 2.5) * 0.2) * 0.015;
    targetK3 = (dx * 1.4 - 0.2) * 0.018;
  }

  // Smooth low-pass filter (spring damper) to simulate soft viscoelastic response
  k1 += (targetK1 - k1) * 0.08;
  k2 += (targetK2 - k2) * 0.08;
  k3 += (targetK3 - k3) * 0.08;

  // Sync with interactive studio sliders if user isn't actively dragging them
  if (seg1Slider && seg2Slider && seg3Slider) {
    if (!document.activeElement || !document.activeElement.id.includes('slider')) {
      seg1Slider.value = (k1 * 100).toFixed(2);
      seg2Slider.value = (k2 * 100).toFixed(2);
      seg3Slider.value = (k3 * 100).toFixed(2);
      updateStudioReadouts();
    }
  }

  // Clear canvas
  ctx.clearRect(0, 0, width, height);

  // Draw Subtle Tech Grid & Base Platform
  drawGridAndBase(width, height, basePos);

  // Compute backbone curve
  const { points, orientations } = computeBackbone([k1, k2, k3], basePos);

  // Record tip trail
  const tip = points[points.length - 1];
  trail.push({ x: tip.x, y: tip.y });
  if (trail.length > MAX_TRAIL) trail.shift();

  // Draw Tip Trail
  drawTrail();

  // Draw Soft Body & Proprioceptive Sensor Fibers
  drawSoftArmBody(points, orientations);

  // Draw Tip End-Effector / Coordinate Frame
  drawEndEffector(tip, orientations[orientations.length - 1]);

  // Update HUD Telemetry text
  updateHUDTelemetry(tip, basePos);

  requestAnimationFrame(renderSimulation);
}

function drawGridAndBase(width, height, basePos) {
  ctx.save();
  // Engineering grid lines - soft gray hairline
  ctx.strokeStyle = 'rgba(172, 188, 191, 0.08)';
  ctx.lineWidth = 1;
  const step = 32;
  for (let x = 0; x < width; x += step) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
    ctx.stroke();
  }
  for (let y = 0; y < height; y += step) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }

  // Base Mount Platform - Deep Navy chassis with subtle Primary Blue hairline
  ctx.fillStyle = '#1b2e3b';
  ctx.strokeStyle = '#5289AD';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  if (ctx.roundRect) {
    ctx.roundRect(basePos.x - 38, basePos.y, 76, 14, 4);
  } else {
    ctx.rect(basePos.x - 38, basePos.y, 76, 14);
  }
  ctx.fill();
  ctx.stroke();

  // Base mounting screws / alignment datum
  ctx.fillStyle = '#ACBCBF';
  ctx.beginPath();
  ctx.arc(basePos.x - 24, basePos.y + 7, 2, 0, Math.PI * 2);
  ctx.arc(basePos.x + 24, basePos.y + 7, 2, 0, Math.PI * 2);
  ctx.fill();

  ctx.restore();
}

function drawTrail() {
  if (trail.length < 2) return;
  ctx.save();
  for (let i = 1; i < trail.length; i++) {
    const alpha = (i / trail.length) * 0.35;
    ctx.strokeStyle = `rgba(82, 137, 173, ${alpha})`;
    ctx.lineWidth = 1.8;
    ctx.beginPath();
    ctx.moveTo(trail[i - 1].x, trail[i - 1].y);
    ctx.lineTo(trail[i].x, trail[i].y);
    ctx.stroke();
  }
  ctx.restore();
}

function drawSoftArmBody(points, orientations) {
  const leftEdge = [];
  const rightEdge = [];
  const dorsalSensor = [];
  const ventralSensor = [];

  for (let i = 0; i < points.length; i++) {
    const p = points[i];
    const angle = orientations[i];
    const normalAngle = angle + Math.PI / 2;

    // Outer silicone contours
    const nx = Math.cos(normalAngle);
    const ny = Math.sin(normalAngle);
    
    // Taper slightly towards the tip
    const taper = 1.0 - (i / points.length) * 0.28;
    const r = ARM_RADIUS * taper;

    leftEdge.push({ x: p.x - nx * r, y: p.y - ny * r });
    rightEdge.push({ x: p.x + nx * r, y: p.y + ny * r });

    // Embedded proprioceptive sensor fiber channels
    const sensorOffset = r * 0.62;
    dorsalSensor.push({ x: p.x - nx * sensorOffset, y: p.y - ny * sensorOffset });
    ventralSensor.push({ x: p.x + nx * sensorOffset, y: p.y + ny * sensorOffset });
  }

  // 1. Draw Translucent Soft Silicone Body (Skin)
  ctx.save();
  ctx.beginPath();
  ctx.moveTo(leftEdge[0].x, leftEdge[0].y);
  for (let i = 1; i < leftEdge.length; i++) {
    ctx.lineTo(leftEdge[i].x, leftEdge[i].y);
  }
  for (let i = rightEdge.length - 1; i >= 0; i--) {
    ctx.lineTo(rightEdge[i].x, rightEdge[i].y);
  }
  ctx.closePath();

  // Subtle material gradient using Primary Blue (#5289AD) and Muted Blue-Gray (#698696)
  const bodyGrad = ctx.createLinearGradient(leftEdge[0].x, leftEdge[0].y, rightEdge[0].x, rightEdge[0].y);
  bodyGrad.addColorStop(0, 'rgba(82, 137, 173, 0.22)');
  bodyGrad.addColorStop(0.5, 'rgba(105, 134, 150, 0.14)');
  bodyGrad.addColorStop(1, 'rgba(82, 137, 173, 0.22)');
  ctx.fillStyle = bodyGrad;
  ctx.fill();

  ctx.strokeStyle = 'rgba(82, 137, 173, 0.55)';
  ctx.lineWidth = 1.4;
  ctx.stroke();

  // 2. Draw Internal Vertebrae Discs / Ribs (Soft Gray)
  for (let i = 2; i < points.length; i += 3) {
    ctx.beginPath();
    ctx.moveTo(leftEdge[i].x, leftEdge[i].y);
    ctx.lineTo(rightEdge[i].x, rightEdge[i].y);
    ctx.strokeStyle = 'rgba(172, 188, 191, 0.25)';
    ctx.lineWidth = 1;
    ctx.stroke();

    // Central core spine marker
    ctx.fillStyle = 'rgba(82, 137, 173, 0.65)';
    ctx.beginPath();
    ctx.arc(points[i].x, points[i].y, 1.8, 0, Math.PI * 2);
    ctx.fill();
  }

  // 3. Draw Embedded Proprioceptive Sensor Fibers (Tension vs Compression)
  const dorsalStrainVal = (k1 + k2 + k3) * 50;

  // Left fiber (Dorsal)
  ctx.beginPath();
  ctx.moveTo(dorsalSensor[0].x, dorsalSensor[0].y);
  for (let i = 1; i < dorsalSensor.length; i++) {
    ctx.lineTo(dorsalSensor[i].x, dorsalSensor[i].y);
  }
  const dorsalColor = dorsalStrainVal > 0 ? '#5289AD' : '#698696';
  ctx.strokeStyle = dorsalColor;
  ctx.lineWidth = 2.0;
  ctx.shadowColor = dorsalColor;
  ctx.shadowBlur = 5;
  ctx.stroke();

  // Right fiber (Ventral)
  ctx.beginPath();
  ctx.moveTo(ventralSensor[0].x, ventralSensor[0].y);
  for (let i = 1; i < ventralSensor.length; i++) {
    ctx.lineTo(ventralSensor[i].x, ventralSensor[i].y);
  }
  const ventralColor = dorsalStrainVal <= 0 ? '#5289AD' : '#698696';
  ctx.strokeStyle = ventralColor;
  ctx.shadowColor = ventralColor;
  ctx.stroke();

  ctx.restore();
}

function drawEndEffector(tip, angle) {
  ctx.save();
  ctx.translate(tip.x, tip.y);
  ctx.rotate(angle);

  // Tip mount disc - Off-White with Primary Blue subtle glow
  ctx.fillStyle = '#F4FCFB';
  ctx.shadowColor = '#5289AD';
  ctx.shadowBlur = 8;
  ctx.beginPath();
  ctx.arc(0, 0, 4.5, 0, Math.PI * 2);
  ctx.fill();

  // Local Coordinate axes at tip
  ctx.shadowBlur = 0;
  ctx.lineWidth = 1.8;

  // Tangent axis (Primary Blue)
  ctx.strokeStyle = '#5289AD';
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(15, 0);
  ctx.stroke();

  // Normal axis (Soft Gray)
  ctx.strokeStyle = '#ACBCBF';
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(0, -11);
  ctx.stroke();

  ctx.restore();
}

function updateHUDTelemetry(tip, basePos) {
  if (!telPos || !telCurvature || !telStrain) return;
  // Compute relative tip coords in simulated mm
  const dxMM = ((tip.x - basePos.x) * 1.5).toFixed(1);
  const dyMM = ((basePos.y - tip.y) * 1.5).toFixed(1);
  telPos.textContent = `${dxMM}, ${dyMM} mm`;

  const meanCurvature = ((k1 + k2 + k3) / 3).toFixed(4);
  telCurvature.textContent = `${meanCurvature} rad/mm`;

  const strainPercent = ((k1 + k2 + k3) * 65).toFixed(2);
  const sign = strainPercent >= 0 ? '+' : '';
  telStrain.textContent = `Δε: ${sign}${strainPercent}%`;
}


// -------------------------------------------------------------------
// 3. Studio Sliders & Manual Kinematic Override
// -------------------------------------------------------------------
function updateStudioReadouts() {
  if (!seg1Val || !seg2Val || !seg3Val || !strainDorsal || !strainVentral || !tipAngleEl) return;
  seg1Val.textContent = `${parseFloat(seg1Slider.value).toFixed(2)} rad`;
  seg2Val.textContent = `${parseFloat(seg2Slider.value).toFixed(2)} rad`;
  seg3Val.textContent = `${parseFloat(seg3Slider.value).toFixed(2)} rad`;

  const v1 = parseFloat(seg1Slider.value);
  const v2 = parseFloat(seg2Slider.value);
  const v3 = parseFloat(seg3Slider.value);
  const netStrain = ((v1 + v2 + v3) * 1.8).toFixed(2);

  strainDorsal.textContent = `${(Math.max(0, netStrain)).toFixed(2)} % (Tension)`;
  strainVentral.textContent = `${(Math.max(0, -netStrain)).toFixed(2)} % (Compression)`;
  tipAngleEl.textContent = `${((v1 + v2 + v3) * 32).toFixed(1)}°`;
}

function onSliderInput() {
  simMode = 'interactive';
  if (simModeText) simModeText.textContent = 'Mode: Interactive';
  if (toggleModeBtn) toggleModeBtn.style.color = '#5289AD';
  k1 = targetK1 = parseFloat(seg1Slider.value) / 100;
  k2 = targetK2 = parseFloat(seg2Slider.value) / 100;
  k3 = targetK3 = parseFloat(seg3Slider.value) / 100;
  updateStudioReadouts();
}

if (seg1Slider && seg2Slider && seg3Slider) {
  seg1Slider.addEventListener('input', onSliderInput);
  seg2Slider.addEventListener('input', onSliderInput);
  seg3Slider.addEventListener('input', onSliderInput);
}

if (btnReset && seg1Slider && seg2Slider && seg3Slider) {
  btnReset.addEventListener('click', () => {
    targetK1 = targetK2 = targetK3 = 0;
    seg1Slider.value = 0;
    seg2Slider.value = 0;
    seg3Slider.value = 0;
    updateStudioReadouts();
  });
}

if (btnSCurve && seg1Slider && seg2Slider && seg3Slider) {
  btnSCurve.addEventListener('click', () => {
    seg1Slider.value = -0.8;
    seg2Slider.value = 0.9;
    seg3Slider.value = -0.7;
    onSliderInput();
  });
}

if (btnSpiral && seg1Slider && seg2Slider && seg3Slider) {
  btnSpiral.addEventListener('click', () => {
    seg1Slider.value = 0.9;
    seg2Slider.value = 1.1;
    seg3Slider.value = 1.3;
    onSliderInput();
  });
}

// Start simulation if canvas is present
if (canvas) {
  renderSimulation();
}


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
// 5. 1-Click BibTeX Citation Copy
// -------------------------------------------------------------------
const copyBibtexBtn = document.getElementById('copy-bibtex-btn');
const copyBtnText = document.getElementById('copy-btn-text');
const bibtexCode = document.getElementById('bibtex-code');

if (copyBibtexBtn && bibtexCode) {
  copyBibtexBtn.addEventListener('click', async () => {
    try {
      await navigator.clipboard.writeText(bibtexCode.innerText);
      copyBibtexBtn.classList.add('copied');
      if (copyBtnText) copyBtnText.textContent = 'Copied to Clipboard!';
      setTimeout(() => {
        copyBibtexBtn.classList.remove('copied');
        if (copyBtnText) copyBtnText.textContent = 'Copy BibTeX';
      }, 2500);
    } catch (err) {
      console.error('Clipboard copy failed:', err);
    }
  });
}

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


