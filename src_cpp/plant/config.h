#include <Eigen/Core>

// Physical Parameters
const float LINK_1_MASS = 10.0f; // kg

const float LINK_2_MASS = 4.0f;          // kg
const float LINK_2_LENGTH = 0.8f;        // m
const float LINK_2_INERTIAL_MASS = 0.8f; // kg-m^2

const float END_MASS = 0.5f; // kg

const Eigen::Vector3f INITAL_STATE = {1.0f, 1.0f, 1.0f};
const Eigen::Vector3f INITAL_STATE_DOT = {1.0f, 1.0f, 1.0f};
