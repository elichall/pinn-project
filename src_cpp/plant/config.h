#pragma once

#include <Eigen/Core>

// Physical Parameters
const double LINK_1_MASS = 10.0; // kg

const double PRIS_LINK_MIN = 0.2; // m
const double PRIS_LINK_MAX = 1.0;

const double LINK_2_MASS = 4.0;          // kg
const double LINK_2_LENGTH = 0.8;        // m
const double LINK_2_INERTIAL_MASS = 0.8; // kg-m^2

const double END_MASS = 0.5; // kg
const double AVERAGE_OBJECT_MASS = 1;

const double CYCLE_TIME = 1.2; // sec
const double WAIT_TIME = 0.1;  // s

// const Eigen::Vector3d INITAL_STATE_EPS = {1.0, 0.5, 0.0}; // m
// const Eigen::Vector3d INITAL_STATE_EPS = {0.0, -1.2, 0.0};       // m
const Eigen::Vector3d INITAL_STATE_EPS = {0.8, -0.6, 0.0};       // m
const Eigen::Vector3d INITAL_STATE_DOT_EPS = {0.0f, 0.0f, 0.0f}; // m/s

// const Eigen::Vector3d INITAL_STATE = {-2.1587989, 1.4422205, 2.1587989};
const Eigen::Vector3d INITAL_STATE = {-1.5707963, 0.6, 1.5707963};
const Eigen::Vector3d INITAL_STATE_DOT = {0.0, 0.0, 0.0};

// const Eigen::Vector3d FINAL_STATE_EPS = {1.0, -0.7, 0.0};
// const double pickupArea[4] = {0.8, 1.3, -1.5, 0.5};
const double pickupArea[4] = {0.8, 1.2, 0.0, 0.8};
