#include <fstream>

#include "viewer/Renderer.h"
#include "tracker/Tracker.h"
#include "CloudManager.h"
#include <GLFW/glfw3.h>

int main(int argc, char *argv[])
{
    int64_t freq = 100;            // Frequency of the thread dedicated to process the point cloud
    std::string log_path = "log";  // TODO: define the path to the log folder
    double Max_dist=0;
    int id=-1;

    double area_min_x = -1.0;
    double area_max_x = 0.0;
    double area_min_y = -1.0;
    double area_max_y = 0.0;
    double area_min_z = -2.0;
    double area_max_z = 1.0;

    bool is_running=true;

    std::ifstream dataFile(log_path, std::ios::in | std::ios::binary);
    if (!dataFile)
    {
        std::cerr << "ERROR: The file '" << log_path << "' does not exist. Exiting.\n";
        return 1;
    }

    // Init renderer
    viewer::Renderer renderer;
    renderer.initCamera(viewer::CameraAngle::XY);
    renderer.clearViewer();

    // Instantiate the tracker
    Tracker tracker;

    // Spawn the thread that process the point cloud and performs the clustering
    CloudManager lidar_cloud(log_path, freq, renderer);
    std::thread t(&CloudManager::startCloudManager, &lidar_cloud);
    viewer::Box area;
    area.x_min = area_min_x;
    area.y_min = area_min_y;
    area.z_min = area_min_z;
    area.x_max = area_max_x;
    area.y_max = area_max_y;
    area.z_max = area_max_z;
    viewer::Color areaColor(0,1,0);
    while (true)
    {
        char key = getKeyPress();
        handleKeyboardInput(key);
        if (is_running){
        // Clear the render
            renderer.clearViewer();

            while (!lidar_cloud.new_measurement)
                ; // wait for new data (we will execute the following code each 100ms)

            // fetch data
            lidar_cloud.mtxData.lock();
            auto cloud = lidar_cloud.getCloud();
            auto color = lidar_cloud.getColor();
            auto boxes = lidar_cloud.getBoxes();
            auto centroids_x = lidar_cloud.getCentroidsX();
            auto centroids_y = lidar_cloud.getCentroidsY();
            lidar_cloud.new_measurement = false;
            lidar_cloud.mtxData.unlock();

            // render pointcloud and boxes
            renderer.renderPointCloud(cloud, "pointCloud", color);
            for (size_t i = 0; i < boxes.size(); ++i)
                renderer.renderBox(boxes[i], i);

            // Call the tracker on the detected clusters
            tracker.track(centroids_x, centroids_y, renderer.getLidarStatus());

            
            // retrieve tracklets and render the trackers
            auto tracks = tracker.getTracks();
            for (size_t i = 0; i < tracks.size(); ++i)
            {
                renderer.addCircle(tracks[i].getX(), tracks[i].getY(), tracks[i].getId());
                renderer.addText(tracks[i].getX() + 0.01, tracks[i].getY() + 0.01, tracks[i].getId());
            }

            // Retrieve and display the ID and distance of the longest-traveled tracklet
            auto longest_track_info = tracker.getLongestTrackInfo();
            int longest_tracklet_id = longest_track_info.first;
            double longest_tracklet_distance = longest_track_info.second;
            if (id==-1)
            {
                id=longest_tracklet_id;
                Max_dist=longest_tracklet_distance;
            }
            else if (Max_dist<longest_tracklet_distance)
            {
                Max_dist=longest_tracklet_distance;
                id=longest_tracklet_id;
            }
            
            std::cout << "La traccia con ID " << id
                    << " ha percorso la distanza maggiore fino a questo momento, pari a " << Max_dist << " metri\n";

            tracker.updateAreaCount();
            int count = tracker.getAreaCount();
            std::cout << "Nell'area verde sono passate " << count
                    << " persone\n";
            
            renderer.renderBox(area, -1, areaColor,0.25);
            renderer.spinViewerOnce();
        }
        else{
            std::cout<< "Visualizzazione attualmente in pausa, premere P nuovamente per riattivarla\n";
        }
    }

    t.join();
    return 0;
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_P && action == GLFW_PRESS) {
        isRunning = !isRunning;
    }
}

