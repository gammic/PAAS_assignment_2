#include "tracker/Tracker.h"

double area_min_x = -2.0;
double area_max_x = 2.0;
double area_min_y = -2.0;
double area_max_y = 2.0;
double area_min_z = -1.0;
double area_max_z = 1.0;

Tracker::Tracker()
{
    cur_id_ = 0;
    distance_threshold_ = 2; // meters
    covariance_threshold = 1; 
    loss_threshold = 30; //number of frames the track has not been seen
}
Tracker::~Tracker()
{
}



/*
    This function removes tracks based on any strategy
*/
void Tracker::removeTracks()
{
    std::vector<Tracklet> tracks_to_keep;

    for (size_t i = 0; i < tracks_.size(); ++i)
    {
        bool logic_to_keep = tracks_[i].getLossCount() < loss_threshold && tracks_[i].getXCovariance() < covariance_threshold && tracks_[i].getYCovariance() < covariance_threshold;
        if (logic_to_keep)
            tracks_to_keep.push_back(tracks_[i]);
    }

    tracks_.swap(tracks_to_keep);
}

/*
    This function add new tracks to the set of tracks ("tracks_" is the object that contains this)
*/
//Qui veniva passato il vettore dei yaw
void Tracker::addTracks(const std::vector<bool> &associated_detections, const std::vector<double> &centroids_x, const std::vector<double> &centroids_y)
{
    // Adding not associated detections
    for (size_t i = 0; i < associated_detections.size(); ++i)
        if (!associated_detections[i])
            tracks_.push_back(Tracklet(cur_id_++, centroids_x[i], centroids_y[i]));
}

/*
    This function associates detections (centroids_x,centroids_y) with the tracks (tracks_)
    Input:
        associated_detection an empty vector to host the associated detection
        centroids_x & centroids_y measurements representing the detected objects
*/
void Tracker::dataAssociation(std::vector<bool> &associated_detections, const std::vector<double> &centroids_x, const std::vector<double> &centroids_y)
{

    //Remind this vector contains a pair of tracks and its corresponding
    associated_track_det_ids_.clear();

    for (size_t i = 0; i < tracks_.size(); ++i)
    {

        int closest_point_id = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t j = 0; j < associated_detections.size(); ++j)
        {
            // TODO
            // Implement logic to find the closest detection (centroids_x,centroids_y) 
            // to the current track (tracks_)
            double dx = centroids_x[j] - tracks_[i].getX();
            double dy = centroids_y[j] - tracks_[i].getY();
            Eigen::MatrixXd S = tracks_[i].getSMatrix();
            double dist;
            Eigen::VectorXd z = Eigen::VectorXd(2);
            z << centroids_x[j], centroids_y[j];

            Eigen::MatrixXd y = tracks_[i].getMeasureDifferenceY(z);
            
            auto dist2 = (y.transpose() * S.inverse() * y)(0);
            dist = std::sqrt(dist2);

            if (dist < min_dist)
            {
                min_dist = dist;
                closest_point_id = j;
            }
            
        }

        // Associate the closest detection to a tracklet
        if (min_dist < distance_threshold_ && !associated_detections[closest_point_id])
        {
            associated_track_det_ids_.push_back(std::make_pair(closest_point_id, i));
            associated_detections[closest_point_id] = true;
        }
    }
}

void Tracker::track(const std::vector<double> &centroids_x,
                    const std::vector<double> &centroids_y,
                    bool lidarStatus)
{

    std::vector<bool> associated_detections(centroids_x.size(), false);

    // TODO: Predict the position
    //For each track --> Predict the position of the tracklets
    for (auto &track : tracks_)
        track.predict();
    
    // TODO: Associate the predictions with the detections

    dataAssociation(associated_detections,centroids_x,centroids_y);

    // Update tracklets with the new detections
    for (int i = 0; i < associated_track_det_ids_.size(); ++i)
    {
        auto det_id = associated_track_det_ids_[i].first;
        auto track_id = associated_track_det_ids_[i].second;
        tracks_[track_id].update(centroids_x[det_id], centroids_y[det_id],lidarStatus);
    }

    // TODO: Remove dead tracklets

    removeTracks();

    // TODO: Add new tracklets

    addTracks(associated_detections,centroids_x,centroids_y);
}

std::pair<int, double> Tracker::getLongestDistanceTracklet() const
{
    int longest_tracklet_id = -1;
    double max_distance = 0.0;

    for (const auto &track : tracks_)
    {
        if (track.getTotalDistanceTraveled() > max_distance)
        {
            max_distance = track.getTotalDistanceTraveled();
            longest_tracklet_id = track.getId(); 
        }
    }

    return {longest_tracklet_id, max_distance};
}

std::pair<int, double> Tracker::getLongestTrackInfo() const {
    int longest_tracklet_id = -1;
    double max_distance = 0.0;

    for (const auto& track : tracks_) {
        double distance = track.getTotalDistanceTraveled();
        if (distance > max_distance) {
            max_distance = distance;
            longest_tracklet_id = track.getId();
        }
    }

    return {longest_tracklet_id, max_distance};
}

bool Tracker::isTrackletInArea(const Tracklet &tracklet) {
    double x = tracklet.getX();
    double y = tracklet.getY();

    return (x >= area_min_x && x <= area_max_x && y >= area_min_y && y <= area_max_y);
}

void Tracker::updateAreaCount() {
    for (const auto &tracklet : tracks_) {
        int id=tracklet.getId();
        if (isTrackletInArea(tracklet)) {
            tracklets_in_area[id]++;
        }
    }
}

std::pair<int, std::vector<int>> Tracker::getAreaCountandIds() const {
    std::vector<int> ids;
    for (const auto& entry : tracklets_in_area) {
        ids.push_back(entry.first);
    }
    return {ids.size(), ids};
}


std::pair<int, unsigned int> Tracker::getIdMostTimeinArea(){
    int id=-1;
    unsigned int max_time=0;
    for(auto i = tracklets_in_area.begin(); i!=tracklets_in_area.end(); i++){
        if(i->second > max_time){
            id=i->first;
            max_time = i->second;
        }
    }
    return {id, max_time};
}