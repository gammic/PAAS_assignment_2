#include <fstream>
#include <iostream>
#include <thread>
#include <atomic>
#include "viewer/Renderer.h"
#include "tracker/Tracker.h"
#include "CloudManager.h"

std::atomic<bool> running(true);

void inputThread() {
    char c;
    while (running) {
        std::cin >> c;
        if (c == 'P' || c == 'p') {
            std::cout << "Visualizzazione attualmente in pausa. Premere P per riattivarla.\n";
            running = false; 
            
            std::cin >> c; 
            if (c == 'P' || c == 'p') {
                running = true; 
            }
        }
    }
}


int main(int argc, char *argv[])
{
    std::thread input(inputThread);

    int64_t freq = 100;            // Frequency of the thread dedicated to process the point cloud
    std::string log_path = "log";  // TODO: define the path to the log folder
    double Max_dist=0;
    int id=-1;

    
    double area_min_x = -2.0;
    double area_max_x = 2.0;
    double area_min_y = -2.0;
    double area_max_y = 2.0;
    double area_min_z = -1.0;
    double area_max_z = 1.0;

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
        
        if (running){
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
            //auto yaws=lidar_cloud.getYaws(); //Utilizza il metodo creato per ottenere i yaw tramite PCA
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
                double X =tracks[i].getX();
                double Y =tracks[i].getY();
                /*double yaw= tracks[i].getYaw();
                std::cout<< yaw;*/
                renderer.addCircle(X, Y, tracks[i].getId());
                renderer.addText(X + 0.01, Y + 0.01, tracks[i].getId());

                //Visualizzazione yaw tramite box rappresentante la direzione
                /*double dir_length=0.1;
                double dir_end_x = X + dir_length * cos(yaw);
                double dir_end_y = Y + dir_length * sin(yaw);
                viewer::Box dir;
                dir.x_min = X;
                dir.y_min = Y;
                dir.z_min = -0.9;
                dir.x_max = dir_end_x;
                dir.y_max = dir_end_y;
                dir.z_max = -1.0;
                viewer::Color dirColor(1,1,1);*/

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
            auto [count,ids] = tracker.getAreaCountandIds();
            std::cout << "Nell'area verde sono passate " << count
                    << " persone, quelle con ID: ";
            for(int id:ids){
                std::cout << id << " ";
            }
            std::cout << std::endl;
            auto [idMT, max_time]= tracker.getIdMostTimeinArea();
            std::cout << "La persona che ha passato più tempo nell'area verde è quella con ID " << idMT << " con una durata pari a " << max_time << " frame\n";
            renderer.renderBox(area, -1, areaColor,0.25);
            renderer.spinViewerOnce();
        }
    }

    t.join();
    return 0;
}



