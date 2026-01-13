# waveforge-3D-simulation

# 🌊 WaveForge 3D — FLIP Water Simulator

WaveForge 3D is an interactive, browser-based 3D fluid simulation that demonstrates a **FLIP/PIC particle–grid hybrid solver**. Users can sculpt environments, inject water, adjust physics parameters in real time, and explore how fluids behave in a dynamic 3D space.

This project is both a **visual simulation** and an **educational tool** for understanding computational fluid dynamics (CFD) concepts.

---

##  How to Run the Simulation

###  Run Locally

1. Clone the repository:

   ```bash
   git clone https://github.com/YOUR_USERNAME/waveforge-3d.git
   ```
2. Open the project folder.
3. Double-click `index.html` **or** serve it locally:

   ```bash
   python -m http.server
   ```
4. Open your browser and go to:

   ```
   http://localhost:8000
   ```

>  A local server is recommended to avoid browser security restrictions.

---

##  Core Controls & Buttons

###  Play / ⟳ Reset

* **Play**: Starts or resumes the simulation loop.
* **Reset**: Clears particles, velocity fields, dye, and resets the scene.

###  Clear Obstacles

* Removes all painted solid geometry from the simulation grid.

---

##  Editing Tools

###  Paint Obstacle (Key: `O`)

* Click and drag on the simulation to paint solid walls.
* Obstacles block fluid flow and affect pressure and velocity.

###  Erase (Key: `E`)

* Removes obstacles or pipes where dragged.

###  Add Pipe (Key: `P`)

* Places a pipe that continuously injects water particles.
* Right-click a pipe to delete it.

###  Dye Splash (Key: `D`)

* Injects colored dye into the fluid.
* Used to visualize velocity, vortices, and mixing.

###  Add / Remove Boat (Key: `B`)

* Adds a floating boat that responds to:

  * Buoyancy
  * Gravity
  * Drag forces
* The boat tilts and moves naturally with waves.

---

## Environment Controls

###  Rain Toggle

* **Rain ON/OFF**: Adds particles from above the simulation volume.
* Useful for filling large scenes naturally.

###  Pulse

* Injects a short burst of energy into the fluid.
* Creates waves and turbulence.

---

##  Physics Parameters

All sliders update the simulation **in real time**.

### Gravity

* Controls downward acceleration applied to particles.

### Damping

* Reduces velocity over time.
* Higher values result in calmer, thicker fluid motion.

### Over-Relaxation

* Affects pressure solver convergence.
* Higher values produce sharper incompressibility but can destabilize the system.

### PIC / FLIP Mix

* Blends between:

  * **PIC** (stable, damped)
  * **FLIP** (energetic, detailed)
* Recommended range: `0.05 – 0.15`

---

##  Grid & Volume Controls

### Grid Size

* Changes the simulation resolution (e.g. `32 × 24 × 24`).
* Higher values = more detail, slower performance.

### Z-Slice

* Selects which depth slice you are editing.
* Enables true **3D painting and pipe placement**.

---

##  Musical Coral (Advanced Feature)

* Coral nodes respond to nearby fluid velocity.
* Stronger flow produces higher audio intensity.
* Demonstrates how fluid energy can be mapped to sound.

---

##  Camera Controls

* **Click + Drag**: Rotate the 3D view
* **Scroll Wheel**: Zoom in/out
* The simulation volume is fully navigable in 3D.

---

##  Educational Concepts Demonstrated

* FLIP / PIC hybrid fluid solvers
* Particle-to-grid and grid-to-particle transfers
* Pressure projection for incompressibility
* Boundary conditions and solid obstacles
* Buoyancy and rigid-body interaction
* Real-time parameter tuning

---

##  Project Structure


---

##  Tips for Best Results

* Start with **Rain ON** to fill the volume.
* Add obstacles to create channels or pools.
* Use dye to visualize turbulence.
* Experiment with PIC/FLIP blending.
* Keep grid size modest for smooth performance.

---

##  License

This project is provided for educational and demonstration purposes.

---

**WaveForge 3D** — Real-time fluid simulation in the browser 🌊
