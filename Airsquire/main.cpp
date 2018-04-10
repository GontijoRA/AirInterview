/************************************************************************************************************************
Main file
Airsquire task - Cylinder Detection - Round 2

Roberto Almeida Gontijo
Place: Belo Horizonte - Minas Gerais - Brazil
Email: robertoalmeidagontijo@hotmail.com

*************************************************************************************************************************/

#include "header.h"

// Main 
int main (int argc, char** argv){
	std::string p1,p2, in_pcd;
	p1 = argv[1]; // Receive input file name (e.g., *.pcd) 
	p2 = argv[2];
	in_pcd = p1 + p2;

        // 1) Removing outliers and computing normals (Same beginning steps of PCL Cylinder model segmentation tutorial)
	// a) Read the input cloud
	pcl::PCDReader reader;
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
	reader.read (in_pcd, *cloud); 
	
	// b) Build a passthrough filter to remove spurious NaNs	
	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
	passThrough(cloud_filtered, cloud);

	// c) Estimate point normals
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
	estimateNormals_And_Tree (cloud_normals, tree, cloud_filtered);
	
	// 2) Segmentation of planes based on depth information
	std::vector< pcl::PlanarRegion< PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
	std::vector< pcl::PointIndices > inlier_indices;
	organizedMultiPlaneSegmentation(regions, inlier_indices, cloud_normals, cloud_filtered);

	printCloud (cloud, "input cloud");
	printCloud (cloud_filtered, "less outliers");

	// 3) Removing Planes
		// Note: Planes must be removed from the data because walls, desks and many other objects built to support things are planes
	pcl::PointIndices::Ptr in_AllPlanes = inliers_AllPlanes(inlier_indices);
	pcl::PointCloud<PointT>::Ptr cloud2 = removePoints(cloud_filtered, in_AllPlanes);
	printCloud (cloud2, "noPlanes");	
	
	// 4) Euclidean Clustering
	std::vector< pcl::PointCloud<PointT>::Ptr > clusters;
	euclideanClusters(clusters, tree, cloud2);

	// 5) For each cluster	
	pcl::PCDWriter writer;
	for (int i = 0; i < clusters.size(); i++){
		// Evaluate if a cluster has a cylinder	
		pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT>);
		bool containCyl = cluster_ContainCyl(cloud_cylinder, clusters, i);
		
		// The cluster has a cylinder
		if(containCyl){
			// Cylinder name			
			size_t lastindex = p2.find_last_of("."); 
			std::string rawname = p2.substr(0, lastindex); 
			//cout<<"rawname: "<<rawname<<endl;
	
			// Write the cylinder
			std::string str = rawname + "_subCloud_" + boost::lexical_cast<std::string>(i) + ".pcd";
			printCloud (cloud_cylinder, str);
			writer.write (str, *cloud_cylinder, false);
		}
	}		
}
