#include "pcl/pcl_tools.hpp"

const float DRONE_RADIUS = 1.5;

struct InputValues {
    std::string ply_file_path;
    std::string output_csv_path;
    float landing_x;
    float landing_y;
    float landing_z;
    bool should_view;
};

void saveToCSV(
    const std::string& _filename,
    const pcl_tools::Features& _features)
{
    // Open the file for writing
    std::ofstream file;
    file.open(_filename);

    if (!file.is_open()) {
        std::cout << "Error: Could not open file " << _filename << " for writing." << std::endl;
        return;
    }

    // Write headers
    file << "Max_Curvature,Min_Curvature,Mean_Curvature,Gaussian_Curvature,"
         << "Tree_Major_Diameter,Tree_Minor_Diameter,"
         << "Density,Slope,Standard_Deviation,Distance_Top,"
         << "Distance_Tree_Center_2D,Distance_Tree_Center_3D,Ratio_Tree_Center_2D,Ratio_Tree_Center_3D,"
         << "Distance_Tree_Highest_Point_2D,Distance_Tree_Highest_Point_3D,Ratio_Tree_Highest_Point_2D,Ratio_Tree_Highest_Point_3D\n";

    // Write data
    file << _features.curvatures.pc1 << "," << _features.curvatures.pc2 << "," << (_features.curvatures.pc1 + _features.curvatures.pc2) / 2.0f << "," << _features.curvatures.pc1 * _features.curvatures.pc2 << ","
         << std::max(_features.treeBB.width, _features.treeBB.height) << "," << std::min(_features.treeBB.width, _features.treeBB.height) << ","
         << _features.density << ","
         << _features.slope << ","
         << _features.stdDev << ","
         << _features.distTop << ",";


    for(size_t i = 0; i < _features.distsOfInterest.size(); ++i)
    {
        for(size_t j = 0; j < _features.distsOfInterest[i].size(); ++j)
        {
            file << _features.distsOfInterest[i][j];

            if (j < _features.distsOfInterest[i].size() - 1) {
                file << ",";
            }
        }
        if (i < _features.distsOfInterest.size() - 1) {
            file << ",";
        }
    }

    file << "\n";

    // Close the file
    file.close();

    std::cout << "Data saved to " << _filename << std::endl;
}

void saveToCSV(const std::string& _filename)
{
    saveToCSV(
        _filename,
        pcl_tools::Features()
    );
}

void saveFeatures(
    const InputValues& _inputValues,
    const pcl::PointXYZRGB& _landingPoint,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr _treeCloud,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr _landingSurfaceCloud)
{
    pcl_tools::Features features = pcl_tools::computeFeatures(
        _landingPoint,
        _treeCloud,
        _landingSurfaceCloud,
        DRONE_RADIUS
    );

    saveToCSV(_inputValues.output_csv_path, features);
}

int main(int argc, char* argv[])
{
    // std::cout << "Number of args received: " << argc << "\n";
    InputValues inputValues;

    // inputValues.ply_file_path = "/home/docker/tree_landing_eda/data/inputs/0/rtabmap_cloud.ply";
    // inputValues.output_csv_path = "/home/docker/tree_landing_eda/data/outputs/0/output_pcl.csv";
    // inputValues.landing_x = -96.0; // /0
    // inputValues.landing_y = -14.0; // /0
    // inputValues.landing_z = 20.0; // /0
    // inputValues.ply_file_path = "/home/docker/tree_landing_eda/data/inputs/no_landings/0/rtabmap_cloud.ply";
    // inputValues.output_csv_path = "/home/docker/tree_landing_eda/data/outputs/no_landings/0/output_pcl.csv";
    // inputValues.landing_x = -80.0; // /no_landings 0
    // inputValues.landing_y = -19.0; // /no_landings 0
    // inputValues.landing_z = 18.5; // /no_landings 0
    // inputValues.ply_file_path = "/home/docker/tree_landing_eda/data/inputs/1/rtabmap_cloud.ply";
    // inputValues.output_csv_path = "/home/docker/tree_landing_eda/data/outputs/1/output_pcl.csv";
    // inputValues.landing_x = -40.0; // /1
    // inputValues.landing_y = -39.0; // /1
    // inputValues.landing_z = 18.0; // /1
    // inputValues.ply_file_path = "/home/docker/tree_landing_eda/data/inputs/2/rtabmap_cloud.ply";
    // inputValues.output_csv_path = "/home/docker/tree_landing_eda/data/outputs/2/output_pcl.csv";
    // inputValues.landing_x = 0.0; // /2
    // inputValues.landing_y = -75.0; // /2
    // inputValues.landing_z = 21.0; // /2
    // inputValues.ply_file_path = "/home/docker/tree_landing_eda/data/inputs/archive/25/rtabmap_cloud.ply";
    // inputValues.output_csv_path = "/home/docker/tree_landing_eda/data/outputs/archive/25/output_pcl.csv";
    // inputValues.landing_x = 0.0; // /25
    // inputValues.landing_y = 0.0; // /25
    // inputValues.landing_z = 1.1; // /25
    inputValues.ply_file_path = "/home/docker/tree_landing_eda/data/inputs/17/rtabmap_cloud.ply";
    inputValues.output_csv_path = "/home/docker/tree_landing_eda/data/outputs/17/output_pcl.csv";
    inputValues.landing_x = 0.0; // /17
    inputValues.landing_y = -45.0; // /17
    inputValues.landing_z = 15.0; // /17

    inputValues.should_view = true;

    // Check if the correct number of arguments is provided
    if (argc == 7) {
        // Parse command-line arguments
        inputValues.ply_file_path = argv[1];
        inputValues.output_csv_path = argv[2];
        inputValues.landing_x = std::stof(argv[3]);
        inputValues.landing_y = std::stof(argv[4]);
        inputValues.landing_z = std::stof(argv[5]);
        std::istringstream(argv[6]) >> std::boolalpha >> inputValues.should_view;
    }
    else if (argc != 1) {
        std::cout << "Usage: " << argv[0] << " <ply_file_path> <output_csv_path> <landing_x> <landing_y> <landing_z> <should_view>\n";
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ogCloud = pcl_tools::loadPly(inputValues.ply_file_path);
    bool isLandingInbound = pcl_tools::checkInboundPoints(ogCloud, std::vector<float>{inputValues.landing_x, inputValues.landing_y, inputValues.landing_z});

    if(!isLandingInbound){
        std::cout << "\n\nWARNING: Saving nan csv line because a point is not inbound\n\n";
        saveToCSV(inputValues.output_csv_path);
        return 0;
    }

    const float DOWNSAMPLE = DRONE_RADIUS/10.0;
    const float MAX_GAP = DRONE_RADIUS/3.0;
    const float SURFACE_DOWNSAMPLE = 2.0*DOWNSAMPLE;
    const float WSHED_DOWNSAMPLE = 1.2*SURFACE_DOWNSAMPLE;
    const float WSHED_THRESH = 0.95;
    const float SMOOTH_FACTOR = 2.5;
    const int MEDIAN_KERNEL = 5;
    const int TOP_HAT_KERNEL = 9;
    const float TOP_HAT_AMP = 10.0;
    const float PACMAN_SOLIDITY = 0.6;
    const float CLUSTERS_SIZE_RATIO = 0.5;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZRGB>(*ogCloud));
    pcl_tools::downSamplePC(clusterCloud, DOWNSAMPLE);
    pcl_tools::extractBiggestCluster(clusterCloud, MAX_GAP, CLUSTERS_SIZE_RATIO);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr surfaceCloud(new pcl::PointCloud<pcl::PointXYZRGB>(*clusterCloud));
    pcl_tools::extractSurface(surfaceCloud, SURFACE_DOWNSAMPLE);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segCloud(new pcl::PointCloud<pcl::PointXYZRGB>(*surfaceCloud));
    pcl::PointXYZRGB landingPoint(inputValues.landing_x, inputValues.landing_y, inputValues.landing_z, 255, 255, 255);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr treeCloud = pcl_tools::computeWatershed(
        segCloud,
        WSHED_DOWNSAMPLE,
        DRONE_RADIUS,
        SMOOTH_FACTOR,
        MEDIAN_KERNEL,
        TOP_HAT_KERNEL,
        TOP_HAT_AMP,
        PACMAN_SOLIDITY,
        inputValues.should_view,
        landingPoint
    );

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr landingCloud(new pcl::PointCloud<pcl::PointXYZRGB>(*clusterCloud));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr landingSurfaceCloud(new pcl::PointCloud<pcl::PointXYZRGB>(*treeCloud));

    // pcl_tools::extractNeighborPC(landingCloud, landingPoint, 2.5*DRONE_RADIUS);
    pcl_tools::extractNeighborCirclePC(landingSurfaceCloud, landingPoint, 2.5*DRONE_RADIUS);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr extremeCloud = pcl_tools::extractLocalExtremums(surfaceCloud, DRONE_RADIUS);

    saveFeatures(inputValues, landingPoint, treeCloud, landingSurfaceCloud);

    if(inputValues.should_view){
        std::cout << "Viewing" << std::endl;
        // pcl_tools::colorSegmentedPoints(ogCloud, pcl::RGB(255,255,255));
        pcl_tools::colorSegmentedPoints(clusterCloud, pcl::RGB(255,255,0));
        pcl_tools::colorSegmentedPoints(treeCloud, pcl::RGB(255,0,0));
        pcl_tools::colorSegmentedPoints(surfaceCloud, pcl::RGB(255,255,255));
        // pcl_tools::colorSegmentedPoints(landingCloud, pcl::RGB(0,255,0));
        pcl_tools::colorSegmentedPoints(landingSurfaceCloud, pcl::RGB(0,0,255));
        // pcl_tools::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{ogCloud, clusterCloud});
        // pcl_tools::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{ogCloud, landingCloud, treeCloud, landingSurfaceCloud});
        // pcl_tools::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{surfaceCloud, landingSurfaceCloud});
        // pcl_tools::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{surfaceCloud, treeCloud});
        pcl_tools::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{segCloud});
    }

    return 0;
}
