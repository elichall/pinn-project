# Containerization Plan: PINN Manipulator

## 1. Philosophy: Dev Container Pattern

**Edit on host, run in container.**

```
Host (Arch)                Container
──────────                 ──────────
nvim     ──→ mounts: ──→  /workspace/pinn_project/
tmux     ──→ source code   (read/write, live sync)
opencode │                 │
         │                 ├── build/            (build artifacts)
Terminal │                 ├── training_data/    (logs)
  │      │                 └── /dev/shm/         (IPC)
  └──────┼── exec ──────→  bash inside container
         │                (build, run, test)
```

Keep native tools (nvim, tmux, opencode) with their configs and keybinds on the host. The container is purely for **build dependencies + execution environment** — ensuring reproducibility across machines without polluting the host with 20 library packages.

---

## 2. Industry Patterns Used

| Pattern | Why |
|---------|-----|
| **Multi-stage Dockerfile** | Smaller final image. Build stage has CMake/GCC/all `-dev` headers; runtime stage has only the libraries needed to run |
| **Compose orchestration** | Manage multiple entrypoints (sim, inference, graphics) from a single `docker-compose.yml` |
| **Volume mounts** | Source code lives on host; container sees it live. No rebuild needed for code changes |
| **.dockerignore** | Prevents sending `build/`, `vendor/`, `training_data/` to the Docker daemon build context |

---

## 3. File Structure to Create

```
pinn_project/
├── Dockerfile                  # Multi-stage build
├── docker-compose.yml          # Service definitions
├── .dockerignore               # Context exclusions
└── docker/
    ├── build.sh                # Convenience: build + enter
    ├── run_sim.sh              # Convenience: run the sim
    └── run_inference.sh        # Convenience: run Python inference
```

---

## 4. Dockerfile — Multi-Stage Breakdown

### Stage 1: `builder`

Installs all build-time deps, compiles C++.

```dockerfile
FROM archlinux:latest AS builder

# Install build dependencies
RUN pacman -Syu --noconfirm \
    base-devel cmake gcc \
    boost boost-libs \
    glfw-x11 glm \
    python python-pip python-poetry \
    && pacman -Scc --noconfirm

# Vendored libraries — copy them in
COPY vendor/eigen /workspace/vendor/eigen
COPY vendor/glad /workspace/vendor/glad
COPY vendor/glm /workspace/vendor/glm

# Copy source tree
COPY src_cpp /workspace/src_cpp
COPY CMakeLists.txt /workspace/

# Build
WORKDIR /workspace/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)
```

### Stage 2: `runtime`

Minimal image with only what's needed to run.

```dockerfile
FROM archlinux:latest AS runtime

# Only runtime libs (no -dev, no cmake, no gcc)
RUN pacman -Syu --noconfirm \
    boost-libs glfw-x11 \
    python python-pip python-poetry \
    && pacman -Scc --noconfirm

# Copy built binary from builder
COPY --from=builder /workspace/build/src_cpp/RoboticSim /usr/local/bin/
COPY --from=builder /workspace/build/src_cpp/GraphicsApp /usr/local/bin/

# Copy Python source
COPY src_python /workspace/src_python
COPY pyproject.toml /workspace/

WORKDIR /workspace
RUN poetry install --only main

ENTRYPOINT ["RoboticSim"]
```

---

## 5. Docker Compose — Service Architecture

```yaml
services:
  sim:
    build: .
    image: pinn-sim
    container_name: pinn-sim
    entrypoint: RoboticSim
    cap_add:
      - SYS_NICE          # Allow SCHED_FIFO priority
    ulimits:
      rtprio: 99          # Real-time priority limit
    volumes:
      - ./src_cpp:/workspace/src_cpp          # Live source for rebuilds
      - ./src_python:/workspace/src_python
      - ./CMakeLists.txt:/workspace/CMakeLists.txt
      - ./build:/workspace/build              # Persist build cache
      - ./training_data:/workspace/training_data
      - /tmp/.X11-unix:/tmp/.X11-unix        # X11 for optional graphics
      - shm_volume:/dev/shm                   # Shared memory IPC
    environment:
      - DISPLAY=${DISPLAY}
    network_mode: host                        # X11 needs host networking
    stdin_open: true
    tty: true

  inference:
    build: .
    image: pinn-sim
    container_name: pinn-inference
    entrypoint: poetry run python src_python/inference/shared_memory_node.py
    volumes:
      - ./src_python:/workspace/src_python
      - shm_volume:/dev/shm                   # Same shared memory as sim
    depends_on:
      - sim
    network_mode: host
    stdin_open: true
    tty: true

  visualizer:
    build: .
    image: pinn-sim
    container_name: pinn-viz
    entrypoint: GraphicsApp
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - shm_volume:/dev/shm
    environment:
      - DISPLAY=${DISPLAY}
    network_mode: host
    depends_on:
      - sim

volumes:
  shm_volume:
    driver: local
    driver_opts:
      type: tmpfs
      device: tmpfs
      o: size=64m
```

---

## 6. Critical Constraint Solutions

| Constraint | Solution |
|------------|----------|
| **SCHED_FIFO priority 99** | `cap_add: SYS_NICE` + `ulimit rtprio=99`. Podman may also need `--privileged` in rootless mode. Docker needs dockerd running. |
| **POSIX shared memory** | Dedicated `tmpfs` volume at `/dev/shm`, mounted identically into `sim` and `inference` containers. Both see the same backing store. |
| **OpenGL / X11** | Mount `/tmp/.X11-unix` + pass `DISPLAY`. Needs `network_mode: host` on most setups. No GPU passthrough needed (software rendering at 60Hz is fine). |
| **1000Hz timing** | Container adds negligible jitter (<1µs on bare metal with `--pid=host`). Validate with `wakeJitter` logging. |
| **JAX CPU-only** | JAX on CPU has no GPU dependency. Works out of the box. |

---

## 7. Development Workflow

Day-to-day usage:

```bash
# ── Terminal 1 (host): edit code ──
nvim src_cpp/main.cpp                # opencode, tmux, nvim all native

# ── Terminal 2 (inside container): build and test ──
cd pinn_project
docker compose run --rm sim bash       # or podman-compose

# Inside container:
cd build && cmake .. && make -j$(nproc)
./src_cpp/RoboticSim

# Or run everything with one command:
docker compose up sim inference        # both start, share /dev/shm
```

**Typical tmux layout:**
- **Left pane**: nvim/opencode on the host, editing source
- **Right pane**: `docker compose exec sim bash` — rebuild + run

No rebuild needed to edit code — the volume mount means source changes are live. Just re-run `make` inside the container.

---

## 8. Installation Steps (Arch)

Since neither Docker nor Podman is installed:

```bash
# Option A: Docker
sudo pacman -S docker docker-compose
sudo systemctl enable --now docker
sudo usermod -aG docker $USER          # log out & back in

# Option B: Podman (recommended for Arch, daemonless)
sudo pacman -S podman podman-compose
```

Podman is preferred on Arch:
- No daemon (no `systemctl start docker`)
- Rootless by default
- Native `podman-compose` support
- Drop-in: just `alias docker=podman`

---

## 9. Implementation Order

1. **Create `Dockerfile`** — multi-stage, Arch-based
2. **Create `.dockerignore`** — exclude `build/`, `vendor/`, `training_data/`, `.git/`, `.agents/`
3. **Create `docker-compose.yml`** — `sim`, `inference`, `visualizer` services
4. **Build and validate** — `docker compose build sim`, check compilation
5. **Test runtime** — `docker compose run sim` (without SCHED_FIFO first, then with it)
6. **Test IPC** — run `sim` + `inference` together, verify seqlock works
7. **Update `.gitignore`** — add `.docker/` if needed (docker-compose files are track-worthy)

---

## 10. Risks & Trade-offs

| Risk | Mitigation |
|------|------------|
| **SCHED_FIFO blocked in Docker** | Docker's default cgroup v2 allows `SYS_NICE`. If not, fall back to `SCHED_OTHER` with `--privileged` or use Podman which handles this better |
| **X11 not working** | Test visualizer last — if graphics are flaky, skip it. The sim runs headless fine |
| **Container clock skew** | Containers inherit host clock. Deterministic timing relies on `CLOCK_MONOTONIC` which is per-container namespace. Verify `wakeJitter` stays < 100µs |
| **Shared memory permissions** | The C++ binary and Python process must run as the same UID inside the container to share `/dev/shm`. Use `user: "${UID}:${GID}"` in compose |
| **opencode inside container** | You asked to keep it outside. This works fine. If you ever want opencode inside the container, you'd need to mount `~/.config/opencode/` and `~/.opencode/` and install the binary |

---

## 11. Verification Checklist

- [ ] `docker compose build sim` exits 0
- [ ] `docker compose run sim RoboticSim` runs a full cycle and logs CSV
- [ ] IPC shared memory is shared between `sim` and `inference` containers
- [ ] `wakeJitter` values with container are comparable to native
- [ ] Ctrl-C cleanly shuts down and writes manifest
- [ ] X11 forwarding works for `GraphicsApp` (stretch goal)
