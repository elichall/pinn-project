#include <Eigen/Core>
#include <random>

using RngSeed = std::mt19937::result_type;

class EndPoint {
public:
  EndPoint(double x1, double x2, double y1, double y2, double z1 = 0.0, double z2 = 0.0)
      : x_min(x1), x_max(x2), y_min(y1), y_max(y2), z_min(z1), z_max(z2),
        gen(std::random_device{}()) {}

  EndPoint(double x1, double x2, double y1, double y2, RngSeed seed,
           double z1 = 0.0, double z2 = 0.0)
      : x_min(x1), x_max(x2), y_min(y1), y_max(y2), z_min(z1), z_max(z2),
        gen(seed) {}

  Eigen::Vector3d generateEndPoint() {
    std::uniform_real_distribution<double> distribX(x_min, x_max);
    std::uniform_real_distribution<double> distribY(y_min, y_max);
    std::uniform_real_distribution<double> distribZ(z_min, z_max);

    return Eigen::Vector3d(distribX(gen), distribY(gen), distribZ(gen));
  }

private:
  double x_min, x_max, y_min, y_max, z_min, z_max;
  std::mt19937 gen;
};
