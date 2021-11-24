#include "KalmanFilter.h"
#include <cmath>

using namespace Eigen;
using namespace std;

KalmanFilter::KalmanFilter()
{
  is_initialized_ = false;
  X_ = Eigen::Matrix<double, 6, 1>::Zero();
  P_ = Eigen::Matrix<double, 6, 6>::Identity();
}

KalmanFilter::~KalmanFilter()
{
}

bool KalmanFilter::isInitialized()
{
  return is_initialized_;
}

array<double, 3> KalmanFilter::getZhat(double joint1, double joint2)
{
  array<double, 3> Zhat;
  Vector3d wP, cP;
  wP << X_(0, 0), X_(2, 0), X_(4, 0);
  Quaterniond wQc = AngleAxisd(-joint1, Vector3d::UnitZ()) * AngleAxisd(-joint2, Vector3d::UnitY());
  cP = wQc.matrix().transpose() * wP;
  double x = cP.x();
  double y = cP.y();
  double z = cP.z();
  Zhat[0] = atan2(-y, x);
  Zhat[1] = atan2( z, x);
  Zhat[2] = sqrt(x*x + y*y + z*z);
  return Zhat;
}

array<double, 6> KalmanFilter::getXhat()
{
  array<double, 6> Xhat;
  for (int i = 0; i < 6; ++i)
  {
    Xhat[i] = X_(i, 0);
  }
  return Xhat;
}

array<double, 36> KalmanFilter::getP()
{
  array<double, 36> P;
  for (int i = 0; i < 6; ++i)
  {
    for (int j = 0; j < 6; ++j)
    {
      P[6 * i + j] = P_(i, j);
    }
  }
  return P;
}

void KalmanFilter::initialize(double horz, double vert, double dist, double joint1, double joint2)
{
  double tanH = tan(horz);
  double tanV = tan(vert);
  double x = dist / sqrt(1 + tanH * tanH + tanV * tanV);
  double y = -tanH * x;
  double z =  tanV * x;
  Vector3d cP(x, y, z);
  Quaterniond wQc = AngleAxisd(-joint1, Vector3d::UnitZ()) * AngleAxisd(-joint2, Vector3d::UnitY());
  Vector3d wP = wQc.matrix() * cP;
  X_ << wP.x(), 0, wP.y(), 0, wP.z(), 0;
  P_ << 1e1 , 1e1 , 0   , 0   , 0   , 0   ,
        1e1 , 1e2 , 0   , 0   , 0   , 0   ,
        0   , 0   , 1e1 , 1e1 , 0   , 0   ,
        0   , 0   , 1e1 , 1e2 , 0   , 0   ,
        0   , 0   , 0   , 0   , 1e1 , 1e1 ,
        0   , 0   , 0   , 0   , 1e1 , 1e2 ;
  is_initialized_ = true;
}

void KalmanFilter::predictFcn(double dt)
{
  Matrix<double, 6, 6> F, Q;
  F << 1   , dt  , 0   , 0   , 0   , 0   ,
       0   , 1   , 0   , 0   , 0   , 0   ,
       0   , 0   , 1   , dt  , 0   , 0   ,
       0   , 0   , 0   , 1   , 0   , 0   ,
       0   , 0   , 0   , 0   , 1   , dt  ,
       0   , 0   , 0   , 0   , 0   , 1   ;
  Q << dt*dt, dt   , 0    , 0    , 0    , 0 ,
       dt   , 1    , 0    , 0    , 0    , 0 ,
       0    , 0    , dt*dt, dt   , 0    , 0 ,
       0    , 0    , dt   , 1    , 0    , 0 ,
       0    , 0    , 0    , 0    , dt*dt, dt,
       0    , 0    , 0    , 0    , dt   , 1 ;
  Q = Q * 1e-6;
  X_ = F * X_;
  P_ = F * P_ * F.transpose() + Q;
}

void KalmanFilter::correctFcn1(Eigen::Matrix<double, 3, 1> Z, Eigen::Matrix<double, 3, 3> R, Eigen::Matrix3d cRw)
{
  Vector3d wP, cP;
  wP << X_(0, 0), X_(2, 0), X_(4, 0);
  cP = cRw * wP;
  double x = cP.x();
  double y = cP.y();
  double z = cP.z();

  Matrix<double, 3, 3> M;
  M = Matrix<double, 3, 3>::Zero();
  M(0, 0) =  y / (x*x + y*y);
  M(0, 1) = -x / (x*x + y*y);
  M(1, 0) = -z / (x*x + z*z);
  M(1, 2) =  x / (x*x + z*z);
  M(2, 0) =  x / sqrt(x*x + y*y + z*z);
  M(2, 1) =  y / sqrt(x*x + y*y + z*z);
  M(2, 2) =  z / sqrt(x*x + y*y + z*z);
  M = M * cRw;

  Matrix<double, 3, 6> H;
  H = Matrix<double, 3, 6>::Zero();
  H(0, 0) = M(0, 0);
  H(0, 2) = M(0, 1);
  H(1, 0) = M(1, 0);
  H(1, 4) = M(1, 2);
  H(2, 0) = M(2, 0);
  H(2, 2) = M(2, 1);
  H(2, 4) = M(2, 2);

  Matrix<double, 3, 1> Zhat;
  Zhat << atan2(-y, x), atan2(z, x), sqrt(x*x + y*y + z*z);

  Matrix<double, 6, 3> K; 
  K = P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse();
  X_ = X_ + K * (Z - Zhat);
  P_ = P_ - K * H * P_;
  P_ = (P_ + P_.transpose()) / 2;
}

void KalmanFilter::correctFcn2(Eigen::Matrix<double, 2, 1> Z, Eigen::Matrix<double, 2, 2> R, Eigen::Matrix3d cRw)
{
  Vector3d wP, cP;
  wP << X_(0, 0), X_(2, 0), X_(4, 0);
  cP = cRw * wP;
  double x = cP.x();
  double y = cP.y();
  double z = cP.z();

  Matrix<double, 2, 3> M;
  M = Matrix<double, 2, 3>::Zero();
  M(0, 0) =  y / (x*x + y*y);
  M(0, 1) = -x / (x*x + y*y);
  M(1, 0) = -z / (x*x + z*z);
  M(1, 2) =  x / (x*x + z*z);
  M = M * cRw;

  Matrix<double, 2, 6> H;
  H = Matrix<double, 2, 6>::Zero();
  H(0, 0) = M(0, 0);
  H(0, 2) = M(0, 1);
  H(1, 0) = M(1, 0);
  H(1, 4) = M(1, 2);

  Matrix<double, 2, 1> Zhat;
  Zhat << atan2(-y, x), atan2(z, x);

  Matrix<double, 6, 2> K; 
  K = P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse();
  X_ = X_ + K * (Z - Zhat);
  P_ = P_ - K * H * P_;
  P_ = (P_ + P_.transpose()) / 2;
}

void KalmanFilter::correctFcn3(Eigen::Matrix<double, 1, 1> Z, Eigen::Matrix<double, 1, 1> R, Eigen::Matrix3d cRw)
{
  Vector3d wP, cP;
  wP << X_(0, 0), X_(2, 0), X_(4, 0);
  cP = cRw * wP;
  double x = cP.x();
  double y = cP.y();
  double z = cP.z();

  Matrix<double, 1, 3> M;
  M(0, 0) =  x / sqrt(x*x + y*y + z*z);
  M(0, 1) =  y / sqrt(x*x + y*y + z*z);
  M(0, 2) =  z / sqrt(x*x + y*y + z*z);
  M = M * cRw;

  Matrix<double, 1, 6> H;
  H(0, 0) = M(0, 0);
  H(0, 2) = M(0, 1);
  H(0, 4) = M(0, 2);

  Matrix<double, 1, 1> Zhat;
  Zhat << sqrt(x*x + y*y + z*z);
  
  Matrix<double, 6, 1> K; 
  K = P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse();
  X_ = X_ + K * (Z - Zhat);
  P_ = P_ - K * H * P_;
  P_ = (P_ + P_.transpose()) / 2;
}

void KalmanFilter::predict(double dt)
{
  if (is_initialized_)
  {
    predictFcn(dt);
  }
}

void KalmanFilter::correct1(double horz, double vert, double dist, double joint1, double joint2, double sigma1, double sigma2)
{
  if (is_initialized_)
  {
    Matrix3d cRw = (AngleAxisd(-joint1, Vector3d::UnitZ()) * AngleAxisd(-joint2, Vector3d::UnitY())).matrix().transpose();
    Matrix3d R = Matrix3d::Identity();
    R(0, 0) = sigma1 * sigma1;
    R(1, 1) = sigma1 * sigma1;
    R(2, 2) = sigma2 * sigma2;
    Vector3d Z(horz, vert, dist);
    correctFcn1(Z, R, cRw);
  }
}

void KalmanFilter::correct2(double horz, double vert, double joint1, double joint2, double sigma)
{
  if (is_initialized_)
  {
    Matrix3d cRw = (AngleAxisd(-joint1, Vector3d::UnitZ()) * AngleAxisd(-joint2, Vector3d::UnitY())).matrix().transpose();
    Matrix2d R = Matrix2d::Identity();
    R(0, 0) = sigma * sigma;
    R(1, 1) = sigma * sigma;
    Vector2d Z(horz, vert);
    correctFcn2(Z, R, cRw);
  }
}

void KalmanFilter::correct3(double dist, double joint1, double joint2, double sigma)
{
  if (is_initialized_)
  {
    Matrix3d cRw = (AngleAxisd(-joint1, Vector3d::UnitZ()) * AngleAxisd(-joint2, Vector3d::UnitY())).matrix().transpose();
    Matrix<double, 1, 1> R;
    R(0, 0) = sigma * sigma;
    Matrix<double, 1, 1> Z;
    Z(0, 0) = dist;
    correctFcn3(Z, R, cRw);
  }
}
