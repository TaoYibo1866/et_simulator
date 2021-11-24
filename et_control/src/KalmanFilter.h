#include <Eigen/Eigen>
#include <array>

class KalmanFilter
{
private:
  bool is_initialized_;
  Eigen::Matrix<double, 6, 1> X_;
  Eigen::Matrix<double, 6, 6> P_;
  void predictFcn(double dt);
  void correctFcn1(Eigen::Matrix<double, 3, 1> Z, Eigen::Matrix<double, 3, 3> R, Eigen::Matrix3d cRw);
  void correctFcn2(Eigen::Matrix<double, 2, 1> Z, Eigen::Matrix<double, 2, 2> R, Eigen::Matrix3d cRw);
  void correctFcn3(Eigen::Matrix<double, 1, 1> Z, Eigen::Matrix<double, 1, 1> R, Eigen::Matrix3d cRw);
public:
  KalmanFilter();
  ~KalmanFilter();
  std::array<double, 3 > getZhat(double joint1, double joint2);
  std::array<double, 6 > getXhat();
  std::array<double, 36> getP();
  bool isInitialized();
  void initialize(double horz, double vert, double dist, double joint1, double joint2);
  void correct1(double horz, double vert, double dist, double joint1, double joint2, double sigma1, double sigma2);
  void correct2(double horz, double vert, double joint1, double joint2, double sigma);
  void correct3(double dist, double joint1, double joint2, double sigma);
  void predict(double dt);
};
