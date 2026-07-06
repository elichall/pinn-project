#pragma once

#include "ControllerInterface.h"
#include "Eigen/src/Core/Matrix.h"
#include <atomic>
#include <boost/lockfree/spsc_queue.hpp>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>

template <int DOF> struct LoggingData {
  // State Values
  double sysTime;                      // time since sim start
  Controller::RobotState<DOF> q;       // actual joint space state
  Eigen::Matrix<double, DOF, 1> qddot; // the joint acceleration for the PINN
  Eigen::Matrix<double, DOF, 1> u;     // commanded acceleration from CTC
  Controller::DesiredState<DOF> qDes;  // desired joint space state
  Controller::Torque<DOF> tau;         // control effort
  double objectMassEst;                // estimated mass
  double objectMass;                   // true mass

  // Time Keeping
  long wakeJitter;    // ns off deterministic 1000Hz that sample time
  long executionTime; // ns taken to do the math

  // Validation Parameters
  bool isHoldingObject;           // is the manipulator holding an object
  bool prismaticLimitActive;      // did the prismatic clamp activate this tic
  double manipulability;          // sqrt(det(J * J^T)) at this tick
  double cartesianTrackingErrorX; // actual end-effector X minus desired X
  double cartesianTrackingErrorY; // actual end-effector Y minus desired Y
};

// data needed to id and recreate the run
struct RunManifest {
  std::string runId;
  uint64_t rngSeed;
  double q0[3];
  double fkX0;
  double fkY0;
};

void telemetryLoggerThread(
    std::atomic<bool> &simulationRunning,
    boost::lockfree::spsc_queue<
        LoggingData<3>, boost::lockfree::capacity<16384>> &telemetryQueue,
    const RunManifest &manifest) {

  // Generate second-scale timestamp for the run directory
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream dirPath;
  dirPath << "../training_data/" << std::put_time(&tm, "%Y%m%d_%H%M%S");
  std::string runDir = dirPath.str();
  std::filesystem::create_directories(runDir);

  // Open the telemetry CSV in the run directory
  std::ofstream csvFile(runDir + "/" + manifest.runId + "_telemetry_log.csv");
  csvFile << std::fixed << std::setprecision(10);
  csvFile
      << "sysTime,wakeJitter,executionTime,"
      << "q1,q2,q3,qdot1,qdot2,qdot3,qddot1,qddot2,qddot3,"
      << "u1,u2,u3,"
      << "dq1,dq2,dq3,dqdot1,dqdot2,dqdot3,dqddot1,dqddot2,dqddot3,"
      << "tau1,tau2,tau3,"
      << "m_est,m_true,"
      << "isHolding,prismaticClamped,manipulability,cartErrorX,cartErrorY\n";

  LoggingData<3> dataBuffer;
  double finalSysTime = 0.0;

  while (simulationRunning.load()) {
    // Sleep for 100ms (10Hz). The 1000Hz loop will have pushed ~100 items
    // into the queue while sleeping.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Pop items until the queue is completely empty
    while (telemetryQueue.pop(dataBuffer)) {
      finalSysTime = dataBuffer.sysTime;
      csvFile << dataBuffer.sysTime << "," << dataBuffer.wakeJitter << ","
              << dataBuffer.executionTime << "," << dataBuffer.q.q(0) << ","
              << dataBuffer.q.q(1) << "," << dataBuffer.q.q(2) << ","
              << dataBuffer.q.qdot(0) << "," << dataBuffer.q.qdot(1) << ","
              << dataBuffer.q.qdot(2) << "," << dataBuffer.qddot(0) << ","
              << dataBuffer.qddot(1) << "," << dataBuffer.qddot(2) << ","
              << dataBuffer.u(0) << "," << dataBuffer.u(1) << ","
              << dataBuffer.u(2) << "," << dataBuffer.qDes.dq(0) << ","
              << dataBuffer.qDes.dq(1) << "," << dataBuffer.qDes.dq(2) << ","
              << dataBuffer.qDes.dqdot(0) << "," << dataBuffer.qDes.dqdot(1)
              << "," << dataBuffer.qDes.dqdot(2) << ","
              << dataBuffer.qDes.dqddot(0) << "," << dataBuffer.qDes.dqddot(1)
              << "," << dataBuffer.qDes.dqddot(2) << ","
              << dataBuffer.tau.tau(0) << "," << dataBuffer.tau.tau(1) << ","
              << dataBuffer.tau.tau(2) << "," << dataBuffer.objectMassEst << ","
              << dataBuffer.objectMass << "," << dataBuffer.isHoldingObject
              << "," << dataBuffer.prismaticLimitActive << ","
              << dataBuffer.manipulability << ","
              << dataBuffer.cartesianTrackingErrorX << ","
              << dataBuffer.cartesianTrackingErrorY << "\n";
    }

    // Flush the buffer to disk at 10Hz so external scripts can read cleanly
    csvFile.flush();
  }

  // Final drain right before shutdown
  while (telemetryQueue.pop(dataBuffer)) {
    finalSysTime = dataBuffer.sysTime;
    csvFile << dataBuffer.sysTime << "," << dataBuffer.wakeJitter << ","
            << dataBuffer.executionTime << "," << dataBuffer.q.q(0) << ","
            << dataBuffer.q.q(1) << "," << dataBuffer.q.q(2) << ","
            << dataBuffer.q.qdot(0) << "," << dataBuffer.q.qdot(1) << ","
            << dataBuffer.q.qdot(2) << "," << dataBuffer.qddot(0) << ","
            << dataBuffer.qddot(1) << "," << dataBuffer.qddot(2) << ","
            << dataBuffer.u(0) << "," << dataBuffer.u(1) << ","
            << dataBuffer.u(2) << "," << dataBuffer.qDes.dq(0) << ","
            << dataBuffer.qDes.dq(1) << "," << dataBuffer.qDes.dq(2) << ","
            << dataBuffer.qDes.dqdot(0) << "," << dataBuffer.qDes.dqdot(1)
            << "," << dataBuffer.qDes.dqdot(2) << ","
            << dataBuffer.qDes.dqddot(0) << "," << dataBuffer.qDes.dqddot(1)
            << "," << dataBuffer.qDes.dqddot(2) << "," << dataBuffer.tau.tau(0)
            << "," << dataBuffer.tau.tau(1) << "," << dataBuffer.tau.tau(2)
            << "," << dataBuffer.objectMassEst << "," << dataBuffer.objectMass
            << "," << dataBuffer.isHoldingObject << ","
            << dataBuffer.prismaticLimitActive << ","
            << dataBuffer.manipulability << ","
            << dataBuffer.cartesianTrackingErrorX << ","
            << dataBuffer.cartesianTrackingErrorY << "\n";
  }
  csvFile.close();

  // Write the manifest now that runDuration is known
  std::ofstream manFile(runDir + "/" + manifest.runId + "_manifest.csv");
  manFile << "runId,rngSeed,q0_1,q0_2,q0_3,fkX0,fkY0,runDuration\n"
          << manifest.runId << "," << manifest.rngSeed << "," << manifest.q0[0]
          << "," << manifest.q0[1] << "," << manifest.q0[2] << ","
          << manifest.fkX0 << "," << manifest.fkY0 << "," << finalSysTime
          << "\n";
  manFile.close();
}
