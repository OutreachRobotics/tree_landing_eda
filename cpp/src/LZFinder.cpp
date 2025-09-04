#include "pcl/pcl_tools.hpp"

const float DRONE_RADIUS = 1.5;

int main(int argc, char* argv[])
{
    bool should_view = true;
    std::string ply_file_path = "/home/docker/tree_landing_eda/data/inputs/17/rtabmap_cloud.ply";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ogCloud = pcl_tools::loadPly(ply_file_path);

    const std::vector<float> LANDING_ZONE_FACTORS = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0};
    const int MIN_LZ_POINTS = 20;
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr treeCloud = pcl_tools::computeWatershed(
        segCloud,
        WSHED_DOWNSAMPLE,
        DRONE_RADIUS,
        SMOOTH_FACTOR,
        MEDIAN_KERNEL,
        TOP_HAT_KERNEL,
        TOP_HAT_AMP,
        PACMAN_SOLIDITY,
        should_view
    );

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr gridCloud = pcl_tools::generateGridCloud(treeCloud, DRONE_RADIUS);
    std::vector<std::vector<pcl_tools::Features>> scaledFeaturesList;
    for(const auto& LANDING_ZONE_FACTOR : LANDING_ZONE_FACTORS) {
        scaledFeaturesList.push_back(pcl_tools::computeFeaturesList(treeCloud, gridCloud, DRONE_RADIUS, LANDING_ZONE_FACTOR, MIN_LZ_POINTS));
    }

    if(should_view){
        std::cout << "Viewing" << std::endl;
        // pcl_tools::colorSegmentedPoints(ogCloud, pcl::RGB(255,255,255));
        pcl_tools::colorSegmentedPoints(clusterCloud, pcl::RGB(255,255,0));
        pcl_tools::colorSegmentedPoints(surfaceCloud, pcl::RGB(255,255,255));
        pcl_tools::colorSegmentedPoints(treeCloud, pcl::RGB(255,0,0));
        pcl_tools::colorSegmentedPoints(gridCloud, pcl::RGB(0,255,0));
        // pcl_tools::colorSegmentedPoints(landingSurfaceCloud, pcl::RGB(0,0,255));
        // pcl_tools::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{ogCloud, clusterCloud});
        // pcl_tools::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{surfaceCloud, landingSurfaceCloud});
        pcl_tools::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{surfaceCloud, treeCloud, gridCloud});
        // pcl_tools::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{segCloud});
    }

    return 0;
}
