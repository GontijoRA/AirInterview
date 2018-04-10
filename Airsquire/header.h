/************************************************************************************************************************
Header file
Airsquire task - Cylinder Detection - Round 2

Roberto Almeida Gontijo
Place: Belo Horizonte - Minas Gerais - Brazil
Email: robertoalmeidagontijo@hotmail.com

/****************************************************** Libraries *******************************************************/
#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/boost.h>
#include <pcl/geometry/polygon_operations.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>

using namespace std;
typedef pcl::PointXYZRGB PointT;

/****************************************************** Prototypes *******************************************************/
void printCloud(pcl::PointCloud<PointT>::Ptr pcl_cloud, std::string name);
void passThrough( pcl::PointCloud<PointT>::Ptr cloud_filtered, pcl::PointCloud<PointT>::Ptr cloud);
void estimateNormals_And_Tree (pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::search::KdTree<PointT>::Ptr tree, pcl::PointCloud<PointT>::Ptr cloud_filtered);
void organizedMultiPlaneSegmentation(std::vector< pcl::PlanarRegion< PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > &regions,
				     std::vector< pcl::PointIndices > &inlier_indices,
				     pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
				     pcl::PointCloud<PointT>::Ptr cloud_filtered);
pcl::PointIndices::Ptr inliers_AllPlanes(std::vector< pcl::PointIndices > &inlier_indices);
pcl::PointCloud<PointT>::Ptr removePoints (pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointIndices::Ptr planes_ind);
void euclideanClusters( std::vector< pcl::PointCloud<PointT>::Ptr > &clusters, pcl::search::KdTree<PointT>::Ptr tree, pcl::PointCloud<PointT>::Ptr cloud2);
bool cluster_ContainCyl(pcl::PointCloud<PointT>::Ptr cloud_cylinder, std::vector< pcl::PointCloud<PointT>::Ptr > &clusters, size_t i);

