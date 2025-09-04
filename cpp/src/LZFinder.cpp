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
    std::vector<std::vector<pcl_tools::Features>> scaled_features_list;
    // Eigen::MatrixXd scaled_features_list(LANDING_ZONE_FACTORS.size(), gridCloud->size());
    for(const auto& LANDING_ZONE_FACTOR : LANDING_ZONE_FACTORS) {
        scaled_features_list.push_back(pcl_tools::computeFeaturesList(treeCloud, gridCloud, DRONE_RADIUS, LANDING_ZONE_FACTOR, MIN_LZ_POINTS));
    }

    std::vector<std::vector<pcl_tools::Features>> scaled_features_list_T;
    size_t num_rows = scaled_features_list.size();
    size_t num_cols = scaled_features_list[0].size();
    scaled_features_list_T.resize(num_cols, std::vector<pcl_tools::Features>(num_rows));
    for (size_t i = 0; i < num_rows; ++i) {
        for (size_t j = 0; j < num_cols; ++j) {
            scaled_features_list_T[j][i] = scaled_features_list[i][j]; 
        }
    }

    // struct Features {
    //     pcl::PrincipalCurvatures curvatures;
    //     pcl_tools::BoundingBox treeBB;
    //     float density;
    //     float slope;
    //     float stdDev;
    //     DistsOfInterest distsOfInterest;
    // };

    // struct DistsOfInterest {
    //     float distTop;

    //     float distTreeCenter2D;
    //     float distTreeCenter3D;
    //     float ratioTreeCenter2D;
    //     float ratioTreeCenter3D;

    //     float distTreeHighestPoint2D;
    //     float distTreeHighestPoint3D;
    //     float ratioTreeHighestPoint2D;
    //     float ratioTreeHighestPoint3D;
    // };

    int gridCloudIdx = 0;
    for(const auto& scaled_features : scaled_features_list_T) {
        if(scaled_features[0].distsOfInterest.ratioTreeCenter3D < 0.5) {
            if(scaled_features[1].slope < 30 && scaled_features[1].stdDev < 0.12) {
            //    scaled_features[1].density < 0.1 && scaled_features[1].curvatures. < 0.1) {

                std::cout << "Landing zone found on idx: " << gridCloudIdx << std::endl;
                break;
            }
            else {
                std::cout << "Landing zone candidate failed one of the following tests: " << scaled_features[1].slope << std::endl;
                std::cout << "scaled_features[1].slope < 30: " << scaled_features[1].slope << std::endl;
                std::cout << "scaled_features[1].stdDev < 0.12: " << scaled_features[1].stdDev << std::endl;
            }
        }
        else {
            std::cout << "No landing zone found within ratioTreeCenter3D < 0.5" << std::endl;
            break;
        }
        ++gridCloudIdx;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr selectedLZ(new pcl::PointCloud<pcl::PointXYZRGB>(*treeCloud));
    pcl_tools::extractNeighborCirclePC(selectedLZ, gridCloud->points[gridCloudIdx], DRONE_RADIUS);

    if(should_view){
        std::cout << "Viewing" << std::endl;
        // pcl_tools::colorSegmentedPoints(ogCloud, pcl::RGB(255,255,255));
        pcl_tools::colorSegmentedPoints(clusterCloud, pcl::RGB(255,255,0));
        pcl_tools::colorSegmentedPoints(surfaceCloud, pcl::RGB(255,255,255));
        pcl_tools::colorSegmentedPoints(treeCloud, pcl::RGB(255,0,0));
        pcl_tools::colorSegmentedPoints(gridCloud, pcl::RGB(0,0,255));
        pcl_tools::colorSegmentedPoints(selectedLZ, pcl::RGB(0,255,0));
        // pcl_tools::colorSegmentedPoints(landingSurfaceCloud, pcl::RGB(0,0,255));
        // pcl_tools::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{ogCloud, clusterCloud});
        // pcl_tools::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{surfaceCloud, landingSurfaceCloud});
        pcl_tools::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{surfaceCloud, treeCloud, gridCloud, selectedLZ});
        // pcl_tools::view(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{segCloud});
    }

    return 0;
}
