#include "pcl/pcl_tools.hpp"

const int N_NEIGHBORS_SEARCH = 20;
const float DRONE_RADIUS = 1.5;

void saveToCSV(
    const std::string& _filename,
    const pcl::PrincipalCurvatures& _curvatures,
    const pcl_tools::BoundingBox _clusterBB,
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
         << "BB_Width,BB_Height,BB_Depth,BB_Volume"
         << "Density,Slope,Standard_Deviation,Distance_Top"
         << "Distance_RGB_Center_2D,Distance_PC_Center_2D,Distance_RGB_Center_3D,Distance_PC_Center_3D\n";

    // Write data
    file << _curvatures.pc1 << "," << _curvatures.pc2 << "," << (_curvatures.pc1 + _curvatures.pc2) / 2.0f << "," << _curvatures.pc1 * _curvatures.pc2 << ","
         << _clusterBB.width << "," << _clusterBB.height << "," << _clusterBB.depth << "," << _clusterBB.volume << ","
         << _density << ","
         << _slope << ","
         << _stdDev << ","
         << _distTop << ",";


    for(auto& distOfInterest : _distsOfInterest)
    {
        file << distOfInterest.first << "," << distOfInterest.second;
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

int main(int argc, char* argv[])
{
    // std::cout << "Number of args received: " << argc << "\n";

    std::string ply_file_path = "/home/docker/tree_landing_eda/data/inputs/9/rtabmap_cloud.ply";
    std::string output_csv_path = "/home/docker/tree_landing_eda/data/outputs/9/output_pcl.csv";
    float landing_x = -50.50081099; // /9
    float landing_y = -70.76794873; // /9
    // float landing_x = 1.50081099; // /10
    // float landing_y = -105.76794873; // /10
    bool shouldView = true;

    // Check if the correct number of arguments is provided
    if (argc == 5) {
        // Parse command-line arguments
        ply_file_path = argv[1];
        output_csv_path = argv[2];
        landing_x = std::stof(argv[3]);
        landing_y = std::stof(argv[4]);
        shouldView = false;
    }
    else if (argc != 1) {
        std::cout << "Usage: " << argv[0] << " <ply_file_path> <landing_x> <landing_y> <center_x> <center_y> <output_csv_path>\n";
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ogCloud = pcl_tools::loadPly(ply_file_path);

    pcl::PointXYZRGB min_pt, max_pt;
    pcl::getMinMax3D(*ogCloud, min_pt, max_pt);
    bool isLandingInbound = pcl_tools::checkInboundPoints(min_pt, max_pt, landing_x, landing_y);

    if(!isLandingInbound){
        std::cout << "\n\nWARNING: Saving nan csv line because a point is not inbound\n\n";
        saveToCSV(output_csv_path);
        return 0;
    }

    pcl::PointXYZRGB rgbCenterPoint(-25.0, -25.0, 0.0, 255, 255, 255); //TODO
    pcl::PointXYZRGB landingPoint(landing_x, landing_y, 0.0, 255, 255, 255);

    const float downsample = DRONE_RADIUS/10.0;
    const float maxGap = DRONE_RADIUS/3.0;
    const float surfaceDownsample = DRONE_RADIUS/10.0;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZRGB>(*ogCloud));
    pcl_tools::downSamplePC(clusterCloud, downsample);
    pcl_tools::extractBiggestCluster(clusterCloud, maxGap);
    pcl_tools::BoundingBox clusterBB = pcl_tools::getBB(clusterCloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr surfaceCloud(new pcl::PointCloud<pcl::PointXYZRGB>(*clusterCloud));
    pcl_tools::extractSurface(surfaceCloud, surfaceDownsample);

    cv::Mat depthMap = pcl_tools::computeDepthMap(surfaceCloud, surfaceDownsample);
    pcl_tools::segmentWatershed(depthMap, 0.2);
    // pcl_tools::saveDepthMapAsTiff(depthMap, "/home/docker/tree_landing_eda/data/outputs/9/test_filtered_3.tiff");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segCloud = pcl_tools::computeSegmentation(
        surfaceCloud,
        4*DRONE_RADIUS/surfaceDownsample,
        25.0,
        0.03
    );

    pcl_tools::smoothPC(surfaceCloud, DRONE_RADIUS/2.0);

    pcl_tools::projectPoint(surfaceCloud, rgbCenterPoint);
    pcl_tools::projectPoint(surfaceCloud, landingPoint);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr landingCloud = pcl_tools::extractNeighborPC(ogCloud, landingPoint, 2*DRONE_RADIUS);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr extremeCloud = pcl_tools::extractLocalExtremums(surfaceCloud, 2*DRONE_RADIUS);

    pcl::PointXYZRGB highestPoint = pcl_tools::getHighestPoint(surfaceCloud);

    pcl::PrincipalCurvatures curvatures = pcl_tools::computeCurvature(surfaceCloud, landingPoint, 2*DRONE_RADIUS);
    float density = pcl_tools::computeDensity(landingCloud, DRONE_RADIUS);
    Eigen::Vector4f coef = pcl_tools::computePlane(landingCloud);
    float slope = pcl_tools::computePlaneAngle(coef);
    float stdDev = pcl_tools::computeStandardDeviation(landingCloud, coef);
    float distTop = highestPoint.z - landingPoint.z;

    std::vector<std::pair<float, float>> distsOfInterest = pcl_tools::computeDistToPointsOfInterest(
        landingPoint, 
        std::vector<pcl::PointXYZRGB>({
            rgbCenterPoint,
            highestPoint
        })
    );

    saveToCSV(output_csv_path, curvatures, clusterBB, density, slope, stdDev, distTop, distsOfInterest);

    if(shouldView){
        std::cout << "Viewing" << std::endl;
        pcl_tools::colorSegmentedPoints(ogCloud, pcl::RGB(255,255,255));
        pcl_tools::colorSegmentedPoints(clusterCloud, pcl::RGB(0,255,0));
        pcl_tools::colorSegmentedPoints(surfaceCloud, pcl::RGB(255,0,0));
        // pcl_tools::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{clusterCloud, surfaceCloud});
        pcl_tools::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{segCloud});
    }

    return 0;
}
