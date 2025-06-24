#include "pcl/pcl_tools.hpp"

const int N_NEIGHBORS_SEARCH = 4;
const float DRONE_RADIUS = 1.5;

int main(int argc, char* argv[])
{
    // std::cout << "Number of args received: " << argc << "\n";

    std::string ply_file_path = "/home/docker/tree_landing_eda/data/inputs/8/rtabmap_cloud.ply";
    std::string output_csv_path = "/home/docker/tree_landing_eda/data/outputs/8/output_pcl.csv";
    float landing_x = -12.50081099;
    float landing_y = 16.76794873;
    float df_x = -12.0;
    float df_y = 16.0;
    bool shouldView = true;

    // Check if the correct number of arguments is provided
    if (argc == 7) {
        // Parse command-line arguments
        ply_file_path = argv[1];
        output_csv_path = argv[2];
        landing_x = std::stof(argv[3]);
        landing_y = std::stof(argv[4]);
        df_x = std::stof(argv[5]);
        df_y = std::stof(argv[6]);
        shouldView = false;
    }
    else if (argc != 1) {
        std::cout << "Usage: " << argv[0] << " <ply_file_path> <landing_x> <landing_y> <center_x> <center_y> <output_csv_path>\n";
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ogCloud = pcl_tools::loadPly(ply_file_path);
    // pcl::PointCloud<pcl::PointNormal> normalsCloud = extractNormalsPC(*cloud, pcl::PointXYZRGB(0.0, 0.0, 0.0, 255, 255, 255));

    pcl::PointXYZRGB min_pt, max_pt;
    pcl::getMinMax3D(*ogCloud, min_pt, max_pt);
    bool isLandingInbound = pcl_tools::checkInboundPoints(min_pt, max_pt, landing_x, landing_y);
    bool isDFInbound = pcl_tools::checkInboundPoints(min_pt, max_pt, df_x, df_y);

    if(!isLandingInbound || !isDFInbound){
        std::cout << "\n\nWARNING: Saving nan csv line because a point is not inbound\n\n";
        pcl_tools::saveToCSV(
            output_csv_path,
            pcl::PrincipalCurvatures(
                std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::quiet_NaN()
            ),
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN(),
            std::vector<float>({
                std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::quiet_NaN()
            }));
        return 0;
    }

    pcl::PointXYZRGB dfCenterPoint(df_x, df_y, 0.0, 255, 255, 255);
    pcl::PointXYZRGB landingPoint(landing_x, landing_y, 0.0, 255, 255, 255);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(*ogCloud));
    pcl_tools::downSamplePC(cloud, 0.1);
    pcl_tools::extractBiggestCluster(cloud, 0.5, 10);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr smoothCloud(new pcl::PointCloud<pcl::PointXYZRGB>(*cloud));
    pcl_tools::downSamplePC(smoothCloud, 0.5);
    pcl_tools::smoothPC(smoothCloud, 2*DRONE_RADIUS);
    pcl_tools::projectPoint(smoothCloud, dfCenterPoint);
    pcl_tools::projectPoint(smoothCloud, landingPoint);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr landingCloud = pcl_tools::extractNeighborPC(ogCloud, landingPoint, DRONE_RADIUS);

    pcl::PointXYZRGB pcCenterPoint = pcl_tools::getHighestPoint(smoothCloud);

    pcl::PrincipalCurvatures curvatures = pcl_tools::computeCurvature(smoothCloud, landingPoint, DRONE_RADIUS);
    float density = pcl_tools::computeDensity(landingCloud, DRONE_RADIUS);
    Eigen::Vector4f coef = pcl_tools::computePlane(landingCloud);
    float slope = pcl_tools::computePlaneAngle(coef);
    float stdDev = pcl_tools::computeStandardDeviation(landingCloud, coef);
    std::vector<float> centerDists = pcl_tools::computeDistToCenters(landingPoint, dfCenterPoint, pcCenterPoint);

    pcl_tools::saveToCSV(output_csv_path, curvatures, density, slope, stdDev, centerDists);

    if(shouldView){
        std::cout << "Viewing" << std::endl;
        pcl_tools::colorSegmentedPoints(cloud, pcl::RGB(255,255,255));
        pcl_tools::colorSegmentedPoints(smoothCloud, pcl::RGB(0,0,255));
        pcl_tools::colorSegmentedPoints(landingCloud, pcl::RGB(255,0,0));
        pcl_tools::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{ogCloud, smoothCloud, landingCloud});
    }

    return 0;
}
