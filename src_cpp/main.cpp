#include "ComputedTorqueControl.h"
#include "ControllerInterface.h"
#include "ManipulatorPlant.h"
#include "RobotModel.h"
#include "RobotSensors.h"
#include "TrajectoryGenerator.h"
#include "plant/config.h"
#include <Eigen/Core>

// constants
const double samplingTime = 0.001;
const int DOF = 3;

// data
Controller::RobotState<DOF> state;
Path::DesiredPosition desiredPosition;
Controller::DesiredState<DOF> desiredState;

// initalizations
Plant::Robot plant;
Sensors::JointSensors sensors(plant);
Model::Robot model(state);

Controller::CTC<DOF> ctc(model);

Path::TrajectoryGenerator path = {INITAL_STATE_EPS, FINAL_STATE_EPS, CYCLE_TIME,
                                  5};

Controller::ControllerInterface<DOF> *activeController = &ctc;

// time keeping
double systemTime = 0.0;
double cycleTime = 0.0;

int main() {
  systemTime += samplingTime;
  cycleTime += samplingTime;

  // get this time's desiredState
  desiredPosition = path.getDesiredPosition(cycleTime);
  desiredState = model.invKinematics(desiredPosition, samplingTime);

  // compute control effort
  Eigen::VectorXd tau =
      activeController->computeControl(state, desiredState, samplingTime);

  // system response (theoretical step)
  plant.applyControl(tau, samplingTime);

  // read response from sensors
  sensors.readSensors();
  state = sensors.qEst;

  // update the model's values
  model.update();
}
