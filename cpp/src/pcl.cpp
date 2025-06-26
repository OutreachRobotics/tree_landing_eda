#include "pcl/pcl_tools.hpp"

const int N_NEIGHBORS_SEARCH = 4;
const float DRONE_RADIUS = 1.5;

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
    // pcl::PointCloud<pcl::PointNormal> normalsCloud = extractNormalsPC(*cloud, pcl::PointXYZRGB(0.0, 0.0, 0.0, 255, 255, 255), N_NEIGHBORS_SEARCH);

    pcl::PointXYZRGB min_pt, max_pt;
    pcl::getMinMax3D(*ogCloud, min_pt, max_pt);
    bool isLandingInbound = pcl_tools::checkInboundPoints(min_pt, max_pt, landing_x, landing_y);

    if(!isLandingInbound){
        std::cout << "\n\nWARNING: Saving nan csv line because a point is not inbound\n\n";
        pcl_tools::saveToCSV(output_csv_path);
        return 0;
    }

    pcl::PointXYZRGB rgbCenterPoint(-25.0, -25.0, 0.0, 255, 255, 255); //TODO
    pcl::PointXYZRGB landingPoint(landing_x, landing_y, 0.0, 255, 255, 255);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(*ogCloud));
    pcl_tools::downSamplePC(cloud, DRONE_RADIUS/10.0);
    pcl_tools::extractBiggestCluster(cloud, DRONE_RADIUS/3.0);
    pcl_tools::BoundingBox clusterBB = pcl_tools::getBB(cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr smoothCloud(new pcl::PointCloud<pcl::PointXYZRGB>(*cloud));
    pcl_tools::downSamplePC(smoothCloud, DRONE_RADIUS/3.0);
    pcl_tools::smoothPC(smoothCloud, DRONE_RADIUS/2.0);
    pcl_tools::projectPoint(smoothCloud, rgbCenterPoint);
    pcl_tools::projectPoint(smoothCloud, landingPoint);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr landingCloud = pcl_tools::extractNeighborPC(ogCloud, landingPoint, 2*DRONE_RADIUS);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr extremeCloud = pcl_tools::extractLocalExtremums(smoothCloud, 2*DRONE_RADIUS);

    pcl::PointXYZRGB highestPoint = pcl_tools::getHighestPoint(smoothCloud);

    pcl::PrincipalCurvatures curvatures = pcl_tools::computeCurvature(smoothCloud, landingPoint, 2*DRONE_RADIUS);
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

    pcl_tools::saveToCSV(output_csv_path, curvatures, clusterBB, density, slope, stdDev, distTop, distsOfInterest);

    if(shouldView){
        std::cout << "Viewing" << std::endl;
        pcl_tools::colorSegmentedPoints(cloud, pcl::RGB(255,255,255));
        pcl_tools::colorSegmentedPoints(smoothCloud, pcl::RGB(0,0,255));
        pcl_tools::colorSegmentedPoints(landingCloud, pcl::RGB(255,0,0));
        pcl_tools::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{ogCloud, smoothCloud, landingCloud});
    }

    return 0;
}
