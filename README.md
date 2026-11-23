# Soft Continuum Proprioceptive Arm (SCPA) — Official Website

Official research landing page, dynamic digital twin visualizer, and progress log for the **Soft Continuum Proprioceptive Arm (SCPA)**.

---

## ⚡ Editorial Engineering Design & Features

- **Art-Directed Color Palette**: Built on a calibrated 5-color palette: Primary Blue (`#5289AD`), Deep Navy (`#243C4C`), Muted Blue-Gray (`#698696`), Soft Gray (`#ACBCBF`), and Off-White (`#F4FCFB`).
- **Senior UI/UX Aesthetic**: Replaces generic AI card-boxes with whitespace hierarchy, asymmetrical layouts, editorial typography, and understated 1px hairlines.
- **Live Continuum Arm Digital Twin**: Interactive 2D canvas simulation using Piecewise Constant Curvature (PCC) kinematics. It reacts in real-time to cursor movements or runs in autonomous oscillatory sweep mode.
- **Embedded Proprioception Visualization**: Displays dynamic strain fiber lines along the flexible silicone body that change color and glow depending on tension/compression.
- **Real-Time Telemetry HUD**: Live computation of tip coordinate $(X, Y)$, mean curvature $\kappa$, and differential strain $\Delta\epsilon$.
- **Interactive Parameter Studio**: Sliders allowing visitors or reviewers to directly test segment curvatures $\kappa_1, \kappa_2, \kappa_3$ and observe the proprioceptive state reconstruction.
- **Filterable Research Journal**: Easily categorize project updates (`Hardware & CAD`, `Sensors & Proprioception`, `Kinematics & Software`, `Milestones`).
- **Academic BibTeX Card**: Includes 1-click citation copy button.
- **Team Lightbox & Sliding Window**:
  - Leads feature an interactive 2-photo sliding window (`firstname.jpeg` & `firstname2.jpeg`) with navigation dots and arrows.
  - Plain circular boundaries with subtle hover elevation.
  - 1-click enlargement into a high-definition lightbox modal.
  - Masked LinkedIn and Gmail hyperlinks.
- **Optimized for Vercel**: Includes `vercel.json` with clean URLs and security headers for instant deployment.

---

## 🚀 Instant Deployment to Vercel

### Method 1: Via Vercel Dashboard (Zero Setup)
1. Push this folder to your GitHub repository:
   ```bash
   git init
   git add .
   git commit -m "Initial commit of SCPA website"
   git branch -M main
   git remote add origin https://github.com/Team-SoftRobotics/Website.git
   git push -u origin main
   ```
2. Log in to [vercel.com](https://vercel.com).
3. Click **"Add New..."** → **"Project"**.
4. Import your GitHub repository.
5. Leave the default settings (Framework Preset: **Other**) and click **Deploy**.
6. Your website is instantly live with free SSL!

### Method 2: Via Vercel CLI
If you have Node.js installed:
```bash
npx vercel
```
Follow the prompts in the terminal to deploy in seconds.

---

## 🛠️ Customization Guide

### 1. Team Roster, Links & Emails

**Project Leads (2 Photos per Lead, Plain Circle, Click to Enlarge):**
1. **Satvik Sharma**
   - LinkedIn: [satvik-sharma-93498537b](https://www.linkedin.com/in/satvik-sharma-93498537b/)
   - Gmail: `satvik.sharma1307@gmail.com`
   - Photos: `images/team/satvik.jpeg` (Photo 1) & `images/team/satvik2.jpeg` (Photo 2)
2. **Pranjal Kumar**
   - LinkedIn: [pranjal-kumar-74859a40b](https://www.linkedin.com/in/pranjal-kumar-74859a40b/)
   - Gmail: `mail.pranjal.kumar.1@gmail.com`
   - Photos: `images/team/pranjal.jpeg` (Photo 1) & `images/team/pranjal2.jpeg` (Photo 2)
3. **Anshuman Srivastava**
   - LinkedIn: [anshuman-srivastava-264b64328](https://www.linkedin.com/in/anshuman-srivastava-264b64328/)
   - Gmail: `anshumansrivastava2006@gmail.com`
   - Photos: `images/team/anshuman.jpeg` (Photo 1) & `images/team/anshuman2.jpeg` (Photo 2)

**Project Collaborators (1 Photo per Person, Plain Circle, Click to Enlarge):**
1. **S Keshav Kumar**: [s-keshav-kumar-2b25b8212](https://www.linkedin.com/in/s-keshav-kumar-2b25b8212/) (`images/team/keshav.jpeg`)
2. **Aditya Samdadiya**: [aditya-samdadiya](https://www.linkedin.com/in/aditya-samdadiya) (`images/team/aditya.jpeg`)
3. **Ayushman Srivastava**: [ayushman-srivastava-47466b31a](https://www.linkedin.com/in/ayushman-srivastava-47466b31a/) (`images/team/ayushman.jpeg`)
4. **Jay Potnis**: [jay-potnis](https://www.linkedin.com/in/jay-potnis/) (`images/team/jay.jpeg`)
5. **Atharva Mehta**: [atharv-mehta-1529a7284](https://www.linkedin.com/in/atharv-mehta-1529a7284/) (`images/team/atharva.jpeg`)
6. **Gauransh Singh**: [gauransh-singh](https://www.linkedin.com/in/gauransh-singh) (`images/team/gauransh.jpeg`)
7. **Prakhar Agrawal**: [prakhar-agrawal-2a5227286](https://www.linkedin.com/in/prakhar-agrawal-2a5227286/) (`images/team/prakhar.jpeg`)

### 2. Photo Styling & Features
- **Plain Circle Boundary**: Minimalist, professional border with a subtle hover zoom and elevation.
- **Click to Enlarge (Lightbox Modal)**: Clicking any photo opens a high-definition enlarged view with researcher name, role badge, and backdrop blur.
- **Sliding Window for Leads**: Arrows and indicator dots switch between photo 1 and photo 2 without triggering the zoom. Clicking directly on the photo itself opens the enlarged view.

### 3. Adding a New Project Update
In [index.html](index.html), find the `<div class="timeline-stream">` section and copy-paste this block:

```html
<div class="timeline-entry" data-category="hardware sensors">
  <div class="timeline-date-col">
    <span class="timeline-date">December 2026</span>
    <span class="timeline-version">v1.2 · Prototype Rev 3</span>
  </div>
  <div class="timeline-content-col">
    <div class="timeline-badge-row">
      <span class="category-tag">Sensors & Proprioception</span>
      <span class="category-tag">Hardware & CAD</span>
    </div>
    <h3 class="timeline-entry-title">Your Update Title Here</h3>
    <p class="timeline-entry-summary">
      A 1-2 sentence high-level summary of what was accomplished.
    </p>
    <ul class="timeline-entry-bullets">
      <li>Specific milestone or benchmark metric reached.</li>
      <li>Hardware adjustment or algorithmic optimization details.</li>
    </ul>
  </div>
</div>
```
```

Supported `data-category` tags for filtering:
- `hardware`
- `sensors`
- `software`
- `milestone`
