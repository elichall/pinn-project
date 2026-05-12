#include <Eigen/Core>

namespace Plant {

class Robot {
public:
  // constuctor
  Robot();
  // destructor
  ~Robot();

  // state values
  Eigen::Vector3d q; // th1, d, th2
  Eigen::Vector3d qdot;
  Eigen::Vector3d qddot;

  Eigen::Matrix4d T;

  Eigen::Matrix3d M;
  Eigen::Vector3d C;

  // public methods
  void update(const Eigen::Vector3d tau,
              double dt); // given a torque input respond

  void setMode(int mode);

private:
  // flags
  int opMode; // 0: all three, 2: static prismatic joint, 3: th2 = 0

  // physical constants
  double L, I2, m1, m2, mE;

  // stored state values
  double c1, c2, c12, s1, s2, s12;

  // private methods
  void spatialMat();
  void massMat(); // inertial matrix
  void cncMat();  // coriolis and centrifugal

  void trig();
};

} // namespace Plant
