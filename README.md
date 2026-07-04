# PINN-Augmented Manipulator: Recycling Pick-and-Place

A high-speed industrial recycling manipulator simulator that demonstrates how **Physics-Informed Neural Networks (PINNs)** can compensate for unknown payload masses in aggressive minimum-jerk trajectories. The system pits classical **Computed Torque Control (CTC)** — which relies on perfect mass models — against an asynchronous PINN that learns the inverse dynamics online.

The narrative: *When forced to execute aggressive trajectories with unknown payloads, CTC exhibits massive tracking errors and wastes energy. By augmenting the architecture with an asynchronous PINN, the system learns compensation torques that reduce tracking error and minimize total control effort.*

---

## Thesis Narrative

**The Problem:** Classical CTC computes `τ = M(q)·u + C(q, q̇) + G(q)` where `u = q̈_des − K_d·ė − K_p·e`. When the controller's internal mass model (`m_est`) mismatches the plant's true mass (`m_true`), the computed `M`, `C`, `G` are wrong — the torque is misplaced, tracking suffers, and energy is wasted.

**The Solution:** A PINN predicts the *residual compensation torque* `τ_PINN` such that `τ_total = τ_CTC + τ_PINN` drives the plant correctly. The PINN learns the mapping from state (q, q̇, commanded u, estimated mass) to the torque error left by the incorrect mass model.

**Training:** Domain randomization. Thousands of simulation runs with randomized start positions, payload masses (0.01–2.0 kg), and trajectory deadlines expose the network to extreme inertial regimes. The physics loss `||M(q_true)·u + C(q_true, q̇_true) + G(q_true) − τ_PINN||²` enforces Lagrangian consistency.

---

## Systems Architecture

The project abandons standard single-threaded simulation in favor of a **Distributed Multi-Process Architecture** using POSIX Shared Memory (`/dev/shm`). Three completely decoupled processes:

### 1. The Real-Time Control Loop (C++) — 1000Hz
- **Deterministic pacing:** `clock_nanosleep` with `CLOCK_MONOTONIC`, `SCHED_FIFO` priority 99.
- **Plant (ground truth):** Integrates forward dynamics `q̈ = M⁻¹·(τ − C − G)` via semi-implicit Euler. Mass dynamically switches during grab/release phases.
- **Model (controller's belief):** Lagrangian dynamics using `AVERAGE_OBJECT_MASS` (not the true mass). This mismatch is the source of the error the PINN learns to correct.
- **CTC controller:** Computes `u = q̈_des − K_d·ė − K_p·e`, then `τ = M_model·u + C_model + G_model`. The commanded acceleration `u` is exposed via `getCommandedAcc()`.
- **CLIK (Closed-Loop Inverse Kinematics):** Pseudo-inverse Jacobian with proportional position error correction (`K_e = 5`) to prevent Cartesian drift in the redundant 3-DOF → 2-DOF task mapping.
- **Trajectories:** Quintic Hermite splines (5th-order Minimum-Jerk) through randomized waypoints, guaranteeing C² continuity (no torque spikes from acceleration discontinuities).
- **Cycle switching:** After each `CYCLE_TIME + WAIT_TIME`, the robot swaps between pick and drop modes. The plant grabs/releases a random-mass object; the model only knows `AVERAGE_OBJECT_MASS`.

### 2. The Graphics Engine (C++ / OpenGL) — 60Hz
- Standalone executable with **total fault isolation** — a graphics crash cannot interrupt the real-time loop.
- Instanced rendering of 5-link DH-parameterized manipulator (base, link-1 halves, wrist joint, link-2, end-effector).
- Dual snail-trail overlays: desired path (white) vs. actual traced path (red).
- Orbital camera (mouse drag + scroll zoom), wireframe overlay mode.

### 3. The PINN Inference Engine (Python / JAX) — 50–100Hz
- Asynchronously polls the robot state from shared memory via seqlock.
- Executes a 4-layer Flax network (128 → 64 → 32 → 4) compiled through XLA.
- Writes predicted compensation torques back to shared memory for the C++ loop to consume (injection plumbing pending).

---

## The PINN Training Stack

### Network Architecture (`src_python/models/pinn_network.py`)
```
Input (10):  [q1, q2, q3, qdot1, qdot2, qdot3, u1, u2, u3, m_est]
  → Dense(128) → tanh → Dense(64) → tanh → Dense(32) → tanh → Dense(4)
Output (4): [τ_PINN_1, τ_PINN_2, τ_PINN_3, m_pred]
```
Built with Flax (`flax.linen`). Forces CPU mode via `JAX_PLATFORMS=cpu` for development.

### Physics Loss (`src_python/models/physics_loss.py`)
The loss function combines supervised data loss with a **physics-informed residual**:

```
loss = MSE(τ_pred, τ_target) + 0.1 · MSE(residual)

where:
  residual = M(q_true) · u + C(q_true, q̇_true) + G(q_true) − τ_pred
```

The kinematics functions `get_M`, `get_C`, `get_G` in `src_python/models/kinematics.py` are pure-JAX re-implementations of the C++ `RobotModel.cpp` dynamics, enabling JAX's `jax.vmap` for efficient batch computation and full autodifferentiation through the physics equations.

### Training Data Pipeline
The 1000Hz C++ loop logs every cycle to a CSV via a lock-free SPSC queue (boost::lockfree::spsc_queue):
```
sysTime, q1-q3, qdot1-qdot3, qddot1-qddot3, u1-u3,
dq1-dq3, dqdot1-dqdot3, dqddot1-dqddot3, tau1-tau3, m_est, m_true
```
The CSV includes both:
- `qddot`: the plant's *actual* joint acceleration from `M⁻¹·(τ − C − G)`
- `u`: the controller's *commanded* acceleration (`q̈_des − K_d·ė − K_p·e`)

These are theoretically identical when the model and plant match, but will diverge when model uncertainty is introduced — enabling future studies.

### Notes on the Mass Feedback Loop
Although the PINN inherently learns to infer the true object mass from the residual torques (and the network's 4th output head predicts `m_pred`), **the IPC structure deliberately does not feed the iteratively converging mass estimate back to the controller**. This is by design: the project aims to measure how well pure compensation torques can correct for a persistently incorrect mass model, isolating the PINN's torque-correction efficacy from any mass-adaptation effects. All scaffolding for closing this loop is in place — the shared memory `estimatedMass` field, the network's mass prediction head, and the `m_est` input feature — but the feedback path remains intentionally disconnected for experimental validity.

---

## IPC Layer: Three Shared Memory Blocks

| Block | Path | Sync | Contents | Frequency |
|---|---|---|---|---|
| Telemetry | `/dev/shm/pinn_manip_telemetry` | Seqlock (atomic `sequenceCounter`) | `q[3]`, `qdot[3]`, `u[3]`, `estimatedMass`, `tauPINN[3]`, `pathVersion` | 1000Hz writes |
| Path | `/dev/shm/pinn_manip_path` | Process-shared `pthread_mutex_t` | `pathX/Y/Z[5000]`, `num_points` | On path generation |
| Command | `/dev/shm/pinn_manip_command` | (Future) | Control scheme toggle, mass override | N/A |

### Seqlock Protocol (Telemetry)
- **Writer (C++):** increment counter to odd → write data → increment to even.
- **Reader (Python/Graphics):** snapshot counter → copy data → verify counter unchanged and even. If invalid, `sleep(0)` and retry (yields to SCHED_FIFO).

---

## Build and Run

### Dependencies
- **C++:** CMake (≥3.10), GCC (C++17), Eigen3, Boost, GLFW, OpenGL (GLAD/GLM vendored).
- **Python:** Poetry, JAX, Flax, Optax, NumPy, Pandas, Matplotlib.
- **OS:** Linux (POSIX shared memory, `clock_nanosleep`, `SCHED_FIFO`).

### Compilation
```bash
git clone --recursive <repo>
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Execution
Start order matters — the control loop creates the IPC blocks, visualizer and inference attach to existing memory:
```bash
# Terminal 1: Real-Time Control Loop (must be root for SCHED_FIFO)
sudo ./build/RoboticSim

# Terminal 2: OpenGL Visualizer
./build/GraphicsApp

# Terminal 3: Python Inference (optional — currently reads shared memory)
cd src_python && poetry run python inference/shared_memory_node.py
```

### Post-Simulation Analysis
```bash
cd src_python && poetry run python analysis/plot_telemetry.py
```
Generates tracking-error histograms, control-effort bar charts, and real-time timing diagnostics in `logs/`.

### PINN Training
```bash
cd src_python && poetry run python models/physics_loss.py
```
Validates the loss computation and gradient flow. Full training pipeline uses `src_python/training/train.py` (WIP).

---

## Project Map

```
pinn_project/
├── CMakeLists.txt                          # Root CMake (C++17, Eigen, Boost)
├── src_cpp/
│   ├── main.cpp                            # 1000Hz deterministic control loop
│   ├── plant/
│   │   ├── ManipulatorPlant.{h,cpp}        # Ground-truth physics integration
│   │   └── config.h                        # Link masses, lengths, initial conditions
│   ├── models/
│   │   └── RobotModel.{h,cpp}              # Controller's belief model + CLIK
│   ├── controllers/
│   │   ├── ComputedTorqueControl.{h,tpp}   # CTC law (τ = M·u + C + G)
│   │   └── config.h                        # Kp=1000, Kd=100
│   ├── trajectory/
│   │   └── TrajectoryGenerator.{h,cpp}     # Quintic Hermite splines + Catmull-Rom velocities
│   ├── ipc/
│   │   └── SharedMemoryLink.{h,cpp}        # POSIX shared memory + seqlock
│   ├── sensors/
│   │   └── RobotSensors.{h,cpp}            # Sensor readout (pass-through currently)
│   ├── graphics/
│   │   ├── GraphicsEngine.{h,cpp}          # OpenGL instanced renderer
│   │   ├── GraphicsApp.cpp                 # Standalone visualizer entrypoint
│   │   ├── kinematics.h                    # DH-parameterized visual kinematics
│   │   ├── robot.h                         # 5-link visual model
│   │   └── config.h                        # Visual dimensions
│   └── include/
│       ├── ControllerInterface.h           # Abstract controller interface
│       ├── DataLogging.h                   # SPSC queue → CSV logging
│       ├── EndPointGenerator.h             # Random endpoint generator
│       └── helpers.h                       # Real-time priority + path sampling
├── src_python/
│   ├── pyproject.toml                      # Poetry: JAX, Flax, Optax, etc.
│   ├── models/
│   │   ├── pinn_network.py                 # 4-layer Flax PINN (10→128→64→32→4)
│   │   ├── kinematics.py                   # JAX dynamics: get_M, get_C, get_G
│   │   └── physics_loss.py                 # Data + physics-informed residual loss
│   ├── inference/
│   │   └── shared_memory_node.py           # Seqlock reader/writer via ctypes
│   ├── training/
│   │   └── train.py                        # Training loop (stub)
│   ├── data/
│   │   └── dataset_loader.py               # Dataset loading (stub)
│   └── analysis/
│       └── plot_telemetry.py               # Post-sim analysis: RMSE, control effort, timing
├── vendor/                                  # Git submodules: Eigen, GLAD, GLM
├── training_data/                           # CSV telemetry logs
└── logs/                                    # Analysis plots and reports
```
