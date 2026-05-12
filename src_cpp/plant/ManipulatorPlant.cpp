#include "ManipulatorPlant.h"
#include "config.h"
#include <Eigen/Core>
#include <Eigen/Dense>

namespace Plant {

Robot::Robot() {
  opMode = 1;

  m1 = LINK_1_MASS;

  m2 = LINK_2_MASS;
  I2 = LINK_2_INERTIAL_MASS;
  L = LINK_2_LENGTH;

  mE = END_MASS;

  q = INITAL_STATE;
  qdot = INITAL_STATE_DOT;

  trig();

  spatialMat();
  massMat();
  cncMat();
};

void Robot::update(const Eigen::Vector3d tau, double dt) {
  // update state vector with previous state values
  qddot = M.inverse() * (tau - C);
  qdot = qdot + qddot * dt;
  q = q + qdot * dt;

  trig();

  // update state values to current state vector
  spatialMat();
  massMat();
  cncMat();
};

void Robot::setMode(int mode) { opMode = mode; };

void Robot::spatialMat() {
  // Forward Kinematics Transformation
  T(0, 0) = c12;
  T(0, 1) = -s12;
  T(0, 2) = 0.0;
  T(0, 3) = L * c12 + q[1] * c1;
  T(1, 0) = s12;
  T(1, 1) = c12;
  T(1, 2) = 0.0;
  T(1, 3) = L * s12 + q[1] * s1;
  T(2, 0) = 0.0;
  T(2, 1) = 0.0;
  T(2, 2) = 1.0;
  T(2, 3) = 0.0;
  T(3, 0) = 0.0;
  T(3, 1) = 0.0;
  T(3, 2) = 0.0;
  T(3, 3) = 1.0;
};

void Robot::massMat() {
  // Intermediary mass terms
  double ma = mE + m1 / 4.0 + m2;
  double mb = mE + m2 / 4.0;
  double mc = 2.0 * mE + m2;
  double d = q[1];

  // Inertia of Link 1 varies quadratically with prismatic extension
  double I1 = 0.2 * m1 * d * d;

  // Mass Matrix Elements
  M(0, 0) = I1 + I2 + ma * d * d + mb * L * L + mc * L * d * c2;
  M(0, 1) = -0.5 * mc * L * s2;
  M(0, 2) = I2 + mb * L * L + 0.5 * mc * L * d * c2;

  M(1, 0) = M(0, 1);
  M(1, 1) = ma;
  M(1, 2) = -0.5 * mc * L * s2;

  M(2, 0) = M(0, 2);
  M(2, 1) = M(1, 2);
  M(2, 2) = I2 + mb * L * L;
};

void Robot::cncMat() {
  // Intermediary terms
  double ma = mE + m1 / 4.0 + m2;
  double mc = 2.0 * mE + m2;
  double d = q[1];

  double qd0 = qdot[0];
  double qd1 = qdot[1];
  double qd2 = qdot[2];

  // Coriolis and Centrifugal Array
  C[0] = (2.0 * ma * d + mc * L * c2) * qd0 * qd1 -
         mc * L * d * s2 * qd0 * qd2 - 0.5 * mc * L * d * s2 * qd2 * qd2;

  C[1] = -(ma * d + 0.5 * mc * L * c2) * qd0 * qd0 -
         0.5 * mc * L * c2 * qd2 * qd2 - mc * L * c2 * qd0 * qd2;

  C[2] = 0.5 * mc * L * d * s2 * qd0 * qd0 + mc * L * c2 * qd0 * qd1;
};

void Robot::trig() {
  c1 = std::cos(q[0]);
  s1 = std::sin(q[0]);
  c2 = std::cos(q[2]);
  s2 = std::sin(q[2]);
  c12 = std::cos(q[0] + q[2]);
  s12 = std::sin(q[0] + q[2]);
};

} // namespace Plant
