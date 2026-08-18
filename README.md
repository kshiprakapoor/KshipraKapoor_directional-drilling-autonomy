# Directional Drilling Autonomous Obstacle Avoidance

A reduced-order engineering demonstration of **directional-drilling trajectory control, supervisory hazard avoidance, and structural-health monitoring** built with Streamlit.

The project separates the autonomy stack into three layers:

1. **Reduced-order wellbore dynamics** — advances the bit trajectory and estimates drilling response, vibration, ROP, and fatigue.
2. **Constrained trajectory MPC** — tracks trajectory setpoints while enforcing a maximum dogleg-severity limit.
3. **Autonomous supervisory layer** — monitors telemetry, detects an approaching exclusion zone, generates a DLS-bounded detour, and replaces the local controller's trajectory setpoints.

The Streamlit interface lets a user compare **Legacy Automation Mode** with **Agentic Override Mode** in the same drilling scenario.

> **Engineering disclaimer**  
> This is an independent, reduced-order educational/R&D demonstration. It is not a Halliburton product, does not use Halliburton proprietary software or data, and is not a field-calibrated RSS, BHA, drilling-dynamics, or well-control simulator. Numerical outputs are illustrative simulation results only.

---

## Why this project exists

Directional drilling is a useful example of a modern cyber-physical system because it combines:

- a physical plant with nonlinear and uncertain behavior,
- incomplete/noisy telemetry,
- trajectory and equipment constraints,
- real-time optimization,
- closed-loop steering,
- structural-health monitoring,
- and supervisory decision logic.

This project explores how those pieces can be organized into a modular autonomy stack without allowing the supervisory AI layer to bypass the deterministic low-level controller or its geometric safety constraint.

---

## Architecture

```text
                   Desired well / reservoir target
                              |
                              v
                  Supervisory trajectory layer
                     /                    \
          Legacy setpoints          Hazard-aware detour
                                         |
                                         v
                                Constrained Trajectory MPC
                                         |
                                         v
                               RSS steering pad command
                                         |
                                         v
                          Reduced-order BHA / wellbore plant
                                         |
                                         v
                    Position + vibration + ROP + health telemetry
                                         |
                                         +---------------------> feedback
```

### Module boundaries

```text
app.py
├── operator controls + visualization
├── simulation lifecycle
└── project archive download

physics_engine.py
└── WellboreDynamics
    ├── 3D displacement propagation
    ├── lithology-dependent ROP surrogate
    ├── obstacle collision detection
    ├── severe stick-slip response
    └── exponential fatigue accumulation

control_theory.py
└── TrajectoryMPC
    ├── finite-horizon trajectory prediction
    ├── tracking objective
    ├── steering-effort penalty
    ├── command-smoothing penalty
    └── hard DLS steering bound

agent_orchestrator.py
└── AutonomousSuperintendent
    ├── telemetry watch
    ├── 400-ft hazard trigger
    ├── deterministic tool invocation
    ├── DLS-bounded detour generation
    └── supervisory target replacement
```

---

## Demo scenario

The default target trajectory intersects a simulated exclusion zone:

- **Horizontal displacement:** 600–1,200 ft
- **True vertical depth:** 5,800–6,400 ft

### Legacy Automation Mode

The local MPC continues to track the original trajectory because no supervisory replanning layer is active. If the bit enters the obstacle volume, the plant model triggers a high-severity torsional stick-slip response and nonlinear fatigue accumulation.

### Agentic Override Mode

The supervisory layer monitors bit telemetry and obstacle stand-off distance. Inside a 400-ft look-ahead threshold it:

1. records a public engineering assessment event,
2. invokes `calculate_dynamic_detour()`,
3. creates a clearance trajectory under a 3.5°/100-ft planning ceiling,
4. replaces the MPC target path,
5. keeps local steering under deterministic constrained control,
6. and guides the trajectory back toward the final target after the hazard is cleared.

The on-screen audit trail deliberately reports **assessment, tool invocation, result, and control action** rather than hidden chain-of-thought.

---

## Control formulation

The local controller uses a finite prediction horizon and minimizes a simplified objective containing:

- cross-track trajectory error,
- steering effort,
- command-rate/smoothness penalty,
- terminal tracking error.

The controller also converts the specified dogleg-severity ceiling into an admissible steering-force bound. The simulation uses:

- **MPC DLS ceiling:** 4.0°/100 ft
- **Supervisory detour planning ceiling:** 3.5°/100 ft
- **Simulation MD increment:** 50 ft

This is a conceptual reduced-order implementation intended to demonstrate control architecture and constraint handling rather than reproduce a field controller.

---

## Structural-health model

When the bit intersects the obstacle bounding box, `WellboreDynamics` triggers:

- vibration at or above 8 g,
- penetration-dependent shock severity,
- exponentially increasing fatigue damage,
- and rapid tool-health degradation.

Outside the exclusion zone, vibration is calculated deterministically from WOB, curvature, lithology surrogate, and measured depth rather than generated from a random-number placeholder.

---

## Repository structure

```text
directional-drilling-autonomy/
├── app.py
├── physics_engine.py
├── control_theory.py
├── agent_orchestrator.py
├── requirements.txt
├── README.md
├── DEPLOYMENT.md
├── .gitignore
└── .streamlit/
    └── config.toml
```

---

## Run locally

Python 3.12 or newer is recommended for the pinned scientific stack.

```bash
python -m venv .venv
source .venv/bin/activate          # macOS / Linux
# .venv\\Scripts\\activate         # Windows PowerShell

python -m pip install --upgrade pip
pip install -r requirements.txt
streamlit run app.py
```

---

## Deploy on Streamlit Community Cloud

1. Create a GitHub repository.
2. Upload the **extracted repository files**, not just the ZIP archive.
3. Keep `app.py` and `requirements.txt` in the repository root.
4. In Streamlit Community Cloud, choose the GitHub repository and `main` branch.
5. Set the entrypoint to `app.py`.
6. Use Python 3.12 or newer in Advanced Settings.
7. Deploy.

See [`DEPLOYMENT.md`](DEPLOYMENT.md) for a click-by-click checklist.

---

## Technology stack

- Python
- Streamlit
- NumPy
- SciPy / SLSQP constrained optimization
- pandas
- Plotly

Dependency versions are explicitly pinned in `requirements.txt` for reproducible deployment.

---

## Suggested interview walkthrough

A concise demonstration can be run in under two minutes:

1. Briefly explain the architecture: **supervisory layer → constrained MPC → plant → telemetry feedback**.
2. Run **Legacy Automation Mode** and show that the controller faithfully follows an unsafe nominal target because it has no higher-level hazard context.
3. Reset and run **Agentic Override Mode**.
4. Point out the 400-ft trigger, detour-tool invocation, updated trajectory setpoints, and preserved low-level DLS constraint.
5. Emphasize that the supervisory layer does **not** directly control the actuator; deterministic control retains low-level steering authority.
6. Close by noting that the model is intentionally reduced order and the value of the project is the modular engineering architecture, not a claim of field-calibrated drilling performance.

---

## Design principles

- Deterministic low-level safety constraints remain outside the supervisory AI layer.
- Public audit logs are used instead of hidden model reasoning.
- No proprietary oilfield data, code, or tool models are included.
- Physics surrogates are deterministic and interpretable.
- Modules have narrow responsibilities so individual components can later be replaced with higher-fidelity models.

---

## Author

**Kshipra S. Kapoor, PhD**  
Electrical & Computer Engineering | Automation | AI-enabled technical systems

