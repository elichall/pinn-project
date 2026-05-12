#include "ManipulatorPlant.h"
#include <Eigen/Core>

namespace Controller {

class CTC {
public:
  // constructor and destructor
  CTC(Plant::Robot robot, double samplingTime);
  ~CTC();

  Eigen::Vector3d step();

private:
};

} // namespace Controller
