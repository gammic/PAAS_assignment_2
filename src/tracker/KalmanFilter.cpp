#include "tracker/KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::init(double dt)
{
  dt_ = dt;

  // create a 4D state vector
  x_ = Eigen::VectorXd(4); //5 dimensioni nel caso di yaw

  // Initialize the state covariance matrix P
  P_ = Eigen::MatrixXd(4, 4); //(5,5)
  P_ << 1., 0., 0., 0.,
      0., 1., 0., 0.,
      0., 0., 1000., 0.,
      0., 0., 0., 1000.;

  // measurement covariance
  R_ = Eigen::MatrixXd(2, 2); //(3,3)
  R_ << 0.0225, 0.,
      0., 0.0225;

  // measurement matrix
  H_ = Eigen::MatrixXd(2, 4); //(3,5)
  H_ << 1., 0., 0., 0.,
      0., 1., 0., 0.;

  // the transition matrix F
  F_ = Eigen::MatrixXd(4, 4); //(5,5)
  F_ << 1., 0., dt_, 0.,
      0., 1., 0., dt_,
      0., 0., 1., 0.,
      0.,0.,0.,1.;

  // set the acceleration noise components
  double noise_ax_ = 2.;
  double noise_ay_ = 2.;

  double dt_2 = dt_ * dt_;
  double dt_3 = dt_2 * dt_;
  double dt_4 = dt_3 * dt_;

  // set the process covariance matrix Q
  Q_ = Eigen::MatrixXd(4, 4); //(5,5)
  Q_ << dt_4 / 4. * noise_ax_, 0., dt_3 / 2. * noise_ax_, 0.,
      0., dt_4 / 4. * noise_ay_, 0., dt_3 / 2. * noise_ay_,
      dt_3 / 2. * noise_ax_, 0., dt_2 * noise_ax_, 0.,
      0., dt_3 / 2. * noise_ay_, 0., dt_2 * noise_ay_;


}

Eigen::Vector2d KalmanFilter::getPosition() const
{
    return Eigen::Vector2d(x_(0), x_(1));
}
void KalmanFilter::predict()
{
  x_=F_*x_;
  P_=F_*P_*F_.transpose()+Q_;
}

void KalmanFilter::update(const Eigen::VectorXd &z)
{
  Eigen::VectorXd y = z - H_ * x_;
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

  x_ = x_ + K * y;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::setState(double x, double y)
{
  x_ << x, y, 0., 0.; //Qui veniva inserito anche yaw
}

Eigen::MatrixXd KalmanFilter::getSMatrix(){
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
  return S; 
}

Eigen::VectorXd KalmanFilter::getMeasureDifferenceY(const Eigen::VectorXd &z){
  Eigen::VectorXd y = z - H_ * x_;
  return y; 
}