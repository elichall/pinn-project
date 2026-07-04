#include "RobotModel.h"
#include "ControllerInterface.h"
#include "Eigen/src/Core/Matrix.h"
#include "TrajectoryGenerator.h"
#include "config.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

namespace Model {

Robot::Robot(Controller::RobotState<3> &stateEst) : qEst(stateEst) {
    opMode = 1;

    m1 = LINK_1_MASS;

    m2 = LINK_2_MASS;
    I2 = LINK_2_INERTIAL_MASS;
    L = LINK_2_LENGTH;

    mE = END_MASS;
    opMode = 0;
    avgObjectMass = AVERAGE_OBJECT_MASS;

    update();

    dqOld.dq = stateEst.q;
    dqOld.dqdot = stateEst.qdot;
    dqOld.dqddot = {0.0, 0.0, 0.0};

    linearizationError = {0.0, 0.0, 0.0};
    Ke = 5;

    trig();

    spatialMat();
    massMat();
    cncMat();
};

void Robot::update() {
    q = qEst.q;
    qdot = qEst.qdot;

    // update state vector with previous state values
    trig();

    // update state values to current state vector
    spatialMat();

    massMat();
    cncMat();
};

void Robot::setMode(int mode) {
    opMode = mode;
    switch (mode) {
    case 0:
        mE -= avgObjectMass;
        break;
    case 1:
        mE += avgObjectMass;
        break;
    }
};

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

void Robot::calcJacobian(const Eigen::Vector3d &q_in,
                         const Eigen::Vector3d &qdot_in,
                         Eigen::Matrix<double, 2, 3> &J_out,
                         Eigen::Matrix<double, 2, 3> &Jdot_out) {
    double d = q_in[1];
    double c1_val = std::cos(q_in[0]);
    double s1_val = std::sin(q_in[0]);
    double c12_val = std::cos(q_in[0] + q_in[2]);
    double s12_val = std::sin(q_in[0] + q_in[2]);

    J_out(0, 0) = -s1_val * d - L * s12_val;
    J_out(0, 1) = c1_val;
    J_out(0, 2) = -L * s12_val;
    J_out(1, 0) = c1_val * d + L * c12_val;
    J_out(1, 1) = s1_val;
    J_out(1, 2) = L * c12_val;

    double q1dot = qdot_in[0];
    double ddot = qdot_in[1];
    double q2dot = qdot_in[2];

    Jdot_out(0, 0) =
        -c1_val * d * q1dot - s1_val * ddot - L * c12_val * (q1dot + q2dot);
    Jdot_out(0, 1) = -s1_val * q1dot;
    Jdot_out(0, 2) = -L * c12_val * (q1dot + q2dot);
    Jdot_out(1, 0) =
        -s1_val * d * q1dot + c1_val * ddot - L * s12_val * (q1dot + q2dot);
    Jdot_out(1, 1) = c1_val * q1dot;
    Jdot_out(1, 2) = -L * s12_val * (q1dot + q2dot);
}

void Robot::jacobianMat() { calcJacobian(q, qdot, J, Jdot); };

void Robot::trig() {
    c1 = std::cos(q[0]);
    s1 = std::sin(q[0]);
    c2 = std::cos(q[2]);
    s2 = std::sin(q[2]);
    c12 = std::cos(q[0] + q[2]);
    s12 = std::sin(q[0] + q[2]);
};

Eigen::Vector3d Robot::forwardKinematics(Eigen::Vector3d q) {
    double c1_local = std::cos(q[0]);
    double s1_local = std::sin(q[0]);
    double c12_local = std::cos(q[0] + q[2]);
    double s12_local = std::sin(q[0] + q[2]);
    return {L * c12_local + q[1] * c1_local, L * s12_local + q[1] * s1_local,
            0.0};
}

Controller::DesiredState<3> Robot::invKinematics(Path::DesiredPosition dpos,
                                                 double dt) {
    Controller::DesiredState<3> desState;

    Eigen::Matrix<double, 2, 3> J_des;
    Eigen::Matrix<double, 2, 3> Jdot_des;
    calcJacobian(dqOld.dq, dqOld.dqdot, J_des, Jdot_des);

    // potentially implement damped pesudo inverse
    Eigen::Matrix<double, 3, 2> pseudoJ =
        J_des.transpose() * (J_des * J_des.transpose()).inverse();

    desState.dqdot =
        pseudoJ *
            (dpos.velocity.head<2>() + Ke * linearizationError.head<2>()) +
        (Eigen::Matrix3d::Identity() - pseudoJ * J_des) * dqOld.dqdot;

    desState.dqddot =
        pseudoJ * (dpos.acceleration.head<2>() - Jdot_des * dqOld.dqdot) +
        (Eigen::Matrix3d::Identity() - pseudoJ * J_des) * dqOld.dqddot;

    desState.dq =
        0.5 * desState.dqddot * dt * dt + desState.dqdot * dt + dqOld.dq;

    Eigen::Vector3d fkPos = forwardKinematics(desState.dq);

    linearizationError = dpos.position - fkPos;

    dqOld = desState;

    return desState;
};

} // namespace Model
