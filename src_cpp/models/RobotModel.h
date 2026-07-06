#pragma once
#include "ControllerInterface.h"
#include "Eigen/src/Core/Matrix.h"
#include "TrajectoryGenerator.h"
#include <Eigen/Core>
#include <sched.h>

namespace Model {

class InverseKinematics {
public:
};

class Robot {
public:
  // constuctor
  Robot(Controller::RobotState<3> &stateEst);
  // destructor
  ~Robot() = default;

  Eigen::Matrix4d T;

  Eigen::Matrix3d M;
  Eigen::Vector3d C;
  Eigen::Vector3d G;
  Eigen::Matrix<double, 2, 3> J;
  Eigen::Matrix<double, 2, 3> Jdot;

  // side-effects from invKinematics (set every tick)
  bool prismaticLimitActive;
  double manipulability;

  // public methods
  bool isHoldingObject() const { return opMode == 1; }
  void update(); // update the models values

  void setMode(int mode);

  Controller::DesiredState<3> invKinematics(Path::DesiredPosition dpos,
                                            double dt);

  Eigen::Vector3d forwardKinematics(Eigen::Vector3d q);

private:
  // flags
  int opMode; // 0: all three, 2: static prismatic joint, 3: th2 = 0

  // physical constants
  double L, I2, m1, m2, mE;

  // stored state values
  double c1, c2, c12, s1, s2, s12;

  double avgObjectMass;

  // state values
  Controller::RobotState<3> &qEst;
  Eigen::Vector3d q; // th1, d, th2
  Eigen::Vector3d qdot;

  Controller::DesiredState<3> dqOld;

  // CLIK state error correction
  Eigen::Vector3d linearizationError;
  double Ke;

  // private methods
  void spatialMat();
  void massMat();     // inertial matrix
  void cncMat();      // coriolis and centrifugal
  void jacobianMat(); // jacobian
  void calcJacobian(const Eigen::Vector3d &q_in, const Eigen::Vector3d &qdot_in,
                    Eigen::Matrix<double, 2, 3> &J_out,
                    Eigen::Matrix<double, 2, 3> &Jdot_out);

  void trig();
};

} // namespace Model
