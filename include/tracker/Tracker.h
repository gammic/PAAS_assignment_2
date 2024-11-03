#ifndef TRACKER_H_
#define TRACKER_H_

#include "tracker/Tracklet.h"
#include <limits>
#include <unordered_set>
#include <unordered_map>

class Tracker
{
public:
  Tracker();
  ~Tracker();
  std::pair<int, double> getLongestDistanceTracklet() const;
  std::pair<int, double> getLongestTrackInfo() const;

  // handle tracklets
  void removeTracks();
  void addTracks(const std::vector<bool> &associated_detections,
                 const std::vector<double> &centroids_x,
                 const std::vector<double> &centroids_y);

  // associate tracklets and detections
  void dataAssociation(std::vector<bool> &associated_detections,
                       const std::vector<double> &centroids_x,
                       const std::vector<double> &centroids_y);

  // track objects
  void track(const std::vector<double> &centroids_x,
             const std::vector<double> &centroids_y,
             bool lidarStatus);

  void updateAreaCount();
  std::pair<int, std::vector<int>> getAreaCountandIds() const;
  std::pair<int, unsigned int> getIdMostTimeinArea();
  // getters
  const std::vector<Tracklet> &getTracks() { return tracks_; }

private:
  // tracklets
  std::vector<Tracklet> tracks_;
  int cur_id_;

  // association
  std::vector<std::pair<int, int>> associated_track_det_ids_;

  // thresholds
  double distance_threshold_;
  double covariance_threshold;
  int loss_threshold;

  std::unordered_map<int,unsigned int> tracklets_in_area;
  bool isTrackletInArea(const Tracklet &tracklet);
};

#endif // TRACKER_H_
