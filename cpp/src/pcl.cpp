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
    const pcl::PrincipalCurvatures& _curvatures,
    const pcl_tools::BoundingBox _treeBB,
    const float _density,
    const float _slope,
    const float _stdDev,
    const float _distTop,
    const std::vector<std::pair<float, float>>& _distsOfInterest)
{
    // Open the file for writing
    std::ofstream file;
    file.open(_filename);

    if (!file.is_open()) {
        std::cout << "Error: Could not open file " << _filename << " for writing." << std::endl;
        return;
    }

    // Write headers
    file << "Curvature_PC1,Curvature_PC2,Mean_Curvature,Gaussian_Curvature,"
         << "Tree_Major_Diameter,Tree_Minor_Diameter,"
         << "Density,Slope,Standard_Deviation,Distance_Top,"
         << "Distance_Tree_Center_2D,Distance_Tree_Highest_Point_2D,Distance_Tree_Center_3D,Distance_Tree_Highest_Point_3D\n";

    // Write data
    file << _curvatures.pc1 << "," << _curvatures.pc2 << "," << (_curvatures.pc1 + _curvatures.pc2) / 2.0f << "," << _curvatures.pc1 * _curvatures.pc2 << ","
         << std::max(_treeBB.width, _treeBB.height) << "," << std::min(_treeBB.width, _treeBB.height) << ","
         << _density << ","
         << _slope << ","
         << _stdDev << ","
         << _distTop << ",";


    for (size_t i = 0; i < _distsOfInterest.size(); ++i)
    {
        file << _distsOfInterest[i].first << "," << _distsOfInterest[i].second;
        if (i < _distsOfInterest.size() - 1) {
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
        pcl::PrincipalCurvatures(
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN()
        ),
        pcl_tools::BoundingBox(),
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        std::vector<std::pair<float, float>>({
            {std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN()},
            {std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN()}
        })
    );
}

void computeFeatures(
    const InputValues& _inputValues,
    const pcl::PointXYZRGB& _landingPoint,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr _treeCloud,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr _landingCloud,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr _landingSurfaceCloud)
{
    pcl_tools::BoundingBox treeBB = pcl_tools::getBB(_treeCloud);

    pcl::PointXYZRGB treeCenterPoint(treeBB.centroid[0], treeBB.centroid[1], 0.0, 255, 255, 255);
    pcl_tools::projectPoint(_treeCloud, treeCenterPoint);

    pcl::PointXYZRGB highestPoint = pcl_tools::getHighestPoint(_treeCloud);

    pcl::PrincipalCurvatures curvatures = pcl_tools::computeCurvature(_landingSurfaceCloud, _landingPoint, 2*DRONE_RADIUS);
    float density = pcl_tools::computeDensity(_landingCloud, DRONE_RADIUS);
    Eigen::Vector4f coef = pcl_tools::computePlane(_landingSurfaceCloud);
    float slope = pcl_tools::computePlaneAngle(coef);
    float stdDev = pcl_tools::computeStandardDeviation(_landingSurfaceCloud, coef);
    float distTop = highestPoint.z - _landingPoint.z;

    std::vector<std::pair<float, float>> distsOfInterest = pcl_tools::computeDistToPointsOfInterest(
        _landingPoint, 
        std::vector<pcl::PointXYZRGB>({
            treeCenterPoint,
            highestPoint
        })
    );

    saveToCSV(_inputValues.output_csv_path, curvatures, treeBB, density, slope, stdDev, distTop, distsOfInterest);
}

int main(int argc, char* argv[])
{
    // std::cout << "Number of args received: " << argc << "\n";
    InputValues inputValues;

    // inputValues.ply_file_path = "/home/docker/tree_landing_eda/data/inputs/16/rtabmap_cloud.ply";
    // inputValues.output_csv_path = "/home/docker/tree_landing_eda/data/outputs/16/output_pcl.csv";
    // inputValues.landing_x = -96.0; // /16
    // inputValues.landing_y = -14.0; // /16
    // inputValues.landing_z = 20.0; // /16
    // inputValues.ply_file_path = "/home/docker/tree_landing_eda/data/inputs/17/rtabmap_cloud.ply";
    // inputValues.output_csv_path = "/home/docker/tree_landing_eda/data/outputs/17/output_pcl.csv";
    // inputValues.landing_x = -80.0; // /17
    // inputValues.landing_y = -19.0; // /17
    // inputValues.landing_z = 18.5; // /17
    // inputValues.ply_file_path = "/home/docker/tree_landing_eda/data/inputs/18/rtabmap_cloud.ply";
    // inputValues.output_csv_path = "/home/docker/tree_landing_eda/data/outputs/18/output_pcl.csv";
    // inputValues.landing_x = -40.0; // /18
    // inputValues.landing_y = -39.0; // /18
    // inputValues.landing_z = 18.0; // /18
    inputValues.ply_file_path = "/home/docker/tree_landing_eda/data/inputs/19/rtabmap_cloud.ply";
    inputValues.output_csv_path = "/home/docker/tree_landing_eda/data/outputs/19/output_pcl.csv";
    inputValues.landing_x = 0.0; // /19
    inputValues.landing_y = -75.0; // /19
    inputValues.landing_z = 21.0; // /19

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
        landingPoint,
        WSHED_DOWNSAMPLE,
        DRONE_RADIUS,
        MEDIAN_KERNEL,
        TOP_HAT_KERNEL,
        TOP_HAT_AMP,
        PACMAN_SOLIDITY,
        inputValues.should_view
    );

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr landingCloud(new pcl::PointCloud<pcl::PointXYZRGB>(*clusterCloud));
    pcl_tools::extractNeighborPC(landingCloud, landingPoint, 2.5*DRONE_RADIUS);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr landingSurfaceCloud(new pcl::PointCloud<pcl::PointXYZRGB>(*treeCloud));
    pcl_tools::extractNeighborPC(landingSurfaceCloud, landingPoint, 2.5*DRONE_RADIUS);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr extremeCloud = pcl_tools::extractLocalExtremums(surfaceCloud, DRONE_RADIUS);

    computeFeatures(inputValues, landingPoint, treeCloud, landingCloud, landingSurfaceCloud);

    if(inputValues.should_view){
        std::cout << "Viewing" << std::endl;
        // pcl_tools::colorSegmentedPoints(ogCloud, pcl::RGB(255,255,255));
        pcl_tools::colorSegmentedPoints(clusterCloud, pcl::RGB(255,255,0));
        pcl_tools::colorSegmentedPoints(treeCloud, pcl::RGB(255,0,0));
        pcl_tools::colorSegmentedPoints(surfaceCloud, pcl::RGB(255,255,255));
        pcl_tools::colorSegmentedPoints(landingCloud, pcl::RGB(0,255,0));
        pcl_tools::colorSegmentedPoints(landingSurfaceCloud, pcl::RGB(0,0,255));
        // pcl_tools::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{ogCloud, clusterCloud});
        pcl_tools::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{ogCloud, landingCloud, treeCloud, landingSurfaceCloud});
        // pcl_tools::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{surfaceCloud, landingSurfaceCloud});
        // pcl_tools::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{surfaceCloud, treeCloud});
        // pcl_tools::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{segCloud});
    }

    return 0;
}
