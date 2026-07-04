#pragma once

#include <atomic>
#include <cstdint>
#include <pthread.h>

#define MAX_PATH_POINTS 5000 // fixed memory size for path

namespace IPC {

// ipc 1 for python and visualizer
// not locked, 1000Hz
// /dev/shm/pinn_manip_telemetry
struct TelemetryIPC {
private:
  // constructor
  TelemetryIPC() {
    sequenceCounter = 0;
    pathVersion = 0;
  }

public:
  std::atomic<uint32_t> sequenceCounter; // Seqlock for lock-free reading

  double q[3];
  double qdot[3];
  double u[3]; // commanded acceleration from CTC
  double estimatedMass;
  double tauPINN[3];

  // The control loop increments this when a new path is generated to tell the
  // visualizer a new path needs drawn
  std::atomic<uint32_t> pathVersion;
};

// ipc 2 for visualizer
// Mutex locked
// /dev/shm/pinn_manip_path
struct PathIPC {
  // A hardware-level mutex residing directly in shared memory
  pthread_mutex_t mutex;

  int num_points;

  double pathX[MAX_PATH_POINTS];
  double pathY[MAX_PATH_POINTS];
  double pathZ[MAX_PATH_POINTS];
};

// ipc 3 for UI to control loop
// /dev/shm/pinn_manip_command
struct CommandIPC {
  // contains data/vars that the user wants to modulate during sim
  // ex. end effector mass, control scheme, pinn support or not
};

// --- Initialization Methods ---
TelemetryIPC *initTelemetryIPC();
PathIPC *initPathIPC();

// --- Memory Write Methods ---
void writeTelemetry(TelemetryIPC *ipc, const double *q, const double *qdot,
                    const double *u, const double estimatedMass,
                    const double *tau);
void writePath(const double *pathX, const double *pathY, const double *pathZ,
               int numPoints, PathIPC *pathIPC, TelemetryIPC *telemetryIPC);

} // namespace IPC
