#include "SharedMemoryLink.h"

#include <cstdlib>
#include <fcntl.h> // For O_CREAT, O_RDWR
#include <iostream>
#include <sys/mman.h> // For shm_open, mmap, PROT_READ, PROT_WRITE
#include <sys/stat.h> // For fchmod
#include <unistd.h>   // For ftruncate

namespace IPC {

TelemetryIPC *initTelemetryIPC() {
  // Create objects which will be the SharedMemoryLink
  shm_unlink("/pinn_manip_telemetry"); // Clear existing memory to prevent
                                       // permission conflicts
  int fd_telemetry = shm_open("/pinn_manip_telemetry", O_CREAT | O_RDWR, 0666);
  if (fd_telemetry == -1) {
    std::cerr << "Failed to open shared memory for telemetry" << std::endl;
    exit(1);
  }
  // Force 0666 permissions bypassing root's umask so normal users can
  // read/write
  fchmod(fd_telemetry, 0666);

  // allocate the space in ram
  if (ftruncate(fd_telemetry, sizeof(TelemetryIPC)) == -1) {
    std::cerr << "Failed to allocate shared memory for telemetry" << std::endl;
    exit(1);
  }

  // maps the memory to cpp pointers
  TelemetryIPC *telemetryIPC =
      (TelemetryIPC *)mmap(NULL, sizeof(TelemetryIPC), PROT_READ | PROT_WRITE,
                           MAP_SHARED, fd_telemetry, 0);
  if (telemetryIPC == MAP_FAILED) {
    std::cerr << "Failed to map shared memory for telemetry" << std::endl;
    exit(1);
  }

  // initalize the counters to zero
  telemetryIPC->sequenceCounter.store(0);
  telemetryIPC->pathVersion.store(0);

  return telemetryIPC;
}

PathIPC *initPathIPC() {
  shm_unlink("/pinn_manip_path"); // Clear existing memory to prevent permission
                                  // conflicts
  int fd_path = shm_open("/pinn_manip_path", O_CREAT | O_RDWR, 0666);
  if (fd_path == -1) {
    std::cerr << "Failed to open shared memory for path" << std::endl;
    exit(1);
  }
  // Force 0666 permissions bypassing root's umask so normal users can lock the
  // mutex
  fchmod(fd_path, 0666);

  // allocate the space in ram
  if (ftruncate(fd_path, sizeof(PathIPC)) == -1) {
    std::cerr << "Failed to allocate shared memory for path" << std::endl;
    exit(1);
  }

  PathIPC *pathIPC = (PathIPC *)mmap(
      NULL, sizeof(PathIPC), PROT_READ | PROT_WRITE, MAP_SHARED, fd_path, 0);
  if (pathIPC == MAP_FAILED) {
    std::cerr << "Failed to map shared memory for path" << std::endl;
    exit(1);
  }

  // Set access privilages
  // Configure Mutex Attributes
  pthread_mutexattr_t attr;
  pthread_mutexattr_init(&attr);
  // CRITICAL: Tells the kernel this mutex must be respected across process
  // boundaries
  pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);

  // Initialize the mutex located at path_ipc->mutex using these attributes
  pthread_mutex_init(&pathIPC->mutex, &attr);
  // clean up memory
  pthread_mutexattr_destroy(&attr);

  return pathIPC;
}

void writeTelemetry(TelemetryIPC *ipc, const double *q, const double *qdot,
                    const double *u, const double estimatedMass,
                    const double *tau) {
  // on odd python and graphics won't grab data
  // if squenceCounter is different after fetching, they discard
  ipc->sequenceCounter.fetch_add(1, std::memory_order_release);
  ipc->estimatedMass = estimatedMass;
  for (int i = 0; i < 3; ++i) {
    ipc->q[i] = q[i];
    ipc->qdot[i] = qdot[i];
    ipc->u[i] = u[i];
    ipc->tauPINN[i] = tau[i];
  } // increment to even so other programs know they can read
  ipc->sequenceCounter.fetch_add(1, std::memory_order_release);
}

void writePath(const double *pathX, const double *pathY, const double *pathZ,
               int numPoints, PathIPC *pathIPC, TelemetryIPC *telemetryIPC) {
  // unlock mutex
  pthread_mutex_lock(&pathIPC->mutex);

  pathIPC->num_points = numPoints;

  for (int i = 0; i < numPoints && i < MAX_PATH_POINTS; i++) {
    pathIPC->pathX[i] = pathX[i];
    pathIPC->pathY[i] = pathY[i];
    pathIPC->pathZ[i] = pathZ[i];
  }

  // lock again and increment path flag to let graphics know there is a
  // change
  pthread_mutex_unlock(&pathIPC->mutex);
  telemetryIPC->pathVersion.fetch_add(1, std::memory_order_relaxed);
}

} // namespace IPC
