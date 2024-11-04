#ifndef TRACKLET_H_
#define TRACKLET_H_

#include <vector>
#include <cmath>

#include "KalmanFilter.h"

class Tracklet
{
public:
  Tracklet(int idTrack, double x, double y);
  ~Tracklet();

  void predict();
  void update(double x, double y, bool lidarStatus);

  // getters
  double getX() const { return kf_.getX(); }
  double getY() const { return kf_.getY(); }
  //double getYaw() const {return kf_.getYaw();}
  double getXCovariance() { return kf_.getXCovariance(); }
  double getYCovariance() { return kf_.getYCovariance(); }
  int getLossCount() { return loss_count_; }
  int getId() const { return id_; }
  double getTotalDistanceTraveled() const;
  Eigen::MatrixXd getSMatrix() { return kf_.getSMatrix(); }
  Eigen::VectorXd getMeasureDifferenceY(const Eigen::VectorXd &z){ return kf_.getMeasureDifferenceY(z); }

private:
  // filter
  KalmanFilter kf_;
  double total_dist = 0.0;
  Eigen::Vector2d last_position_;
  // tracklet id
  int id_;

  // number of loss since last update
  int loss_count_;
};

#endif // TRACKLET_H_
