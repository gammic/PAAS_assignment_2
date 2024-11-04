#include "tracker/Tracklet.h"

Tracklet::Tracklet(int idTrack, double x, double y) //Qui veniva passato anche yaw
{
  // set id
  id_ = idTrack;

  //distanza tot
  
  // initialize filter
  kf_.init(0.1);
  kf_.setState(x, y);

  // set loss count to 0
  loss_count_ = 0;
  total_dist=0.0;
  last_position_ = Eigen::Vector2d(x, y);
}

Tracklet::~Tracklet()
{
}

// Predict a single measurement
void Tracklet::predict()
{
  kf_.predict();
  loss_count_++;
}

// Update with a real measurement
void Tracklet::update(double x, double y, bool lidarStatus) //Qui veniva passato yaw
{
  Eigen::VectorXd raw_measurements_ = Eigen::VectorXd(2);

  Eigen::Vector2d current_position = kf_.getPosition();

  // measurement update
  if (lidarStatus)
  {
    double distance = (current_position - last_position_).norm();
    total_dist += distance;
    last_position_ = current_position;
    raw_measurements_ << x, y;
    kf_.update(raw_measurements_);
    loss_count_ = 0;
  }
}

double Tracklet::getTotalDistanceTraveled() const
{
    return total_dist;
}

