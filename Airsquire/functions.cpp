/************************************************************************************************************************
Implemented functions
Airsquire task - Cylinder Detection - Round 2

Roberto Almeida Gontijo
Place: Belo Horizonte - Minas Gerais - Brazil
Email: robertoalmeidagontijo@hotmail.com

*************************************************************************************************************************/
#include "header.h"

//Print a point cloud
void printCloud(pcl::PointCloud<PointT>::Ptr pcl_cloud, std::string name){
	pcl::visualization::PCLVisualizer viewer (name);
	pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(pcl_cloud);
	viewer.addPointCloud<PointT> (pcl_cloud, rgb, "sample cloud");
	viewer.setBackgroundColor (0, 0, 0, 0);
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	//    viewer.addCoordinateSystem (1.0);
	//    viewer.initCameraParameters (); 

	while (!viewer.wasStopped ()) { // Display the visualiser until any key is pressed
		viewer.spinOnce ();
	}
}

// PassThrough to an input cloud
void passThrough( pcl::PointCloud<PointT>::Ptr cloud_filtered, pcl::PointCloud<PointT>::Ptr cloud){
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (cloud); 	
	pass.setKeepOrganized(1); // To keep the cloud organized after filter
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0, 1.5);
	pass.filter (*cloud_filtered); 
}

// Estimate cloud normals
void estimateNormals_And_Tree  (pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
				pcl::search::KdTree<PointT>::Ptr tree,
			        pcl::PointCloud<PointT>::Ptr cloud_filtered){	
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	ne.setSearchMethod (tree);
	ne.setInputCloud (cloud_filtered);
	ne.setKSearch (400); //400 closer neighbors
	ne.compute (*cloud_normals);
}

// Segmentation of planes based on depth information
void organizedMultiPlaneSegmentation(std::vector< pcl::PlanarRegion< PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > &regions,
				     std::vector< pcl::PointIndices > &inlier_indices,
				     pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
				     pcl::PointCloud<PointT>::Ptr cloud_filtered){

	pcl::OrganizedMultiPlaneSegmentation< PointT, pcl::Normal, pcl::Label > mps;
	mps.setMinInliers (200);
	mps.setAngularThreshold (0.017453 * 2.0); // 2 degrees
	mps.setDistanceThreshold (0.03); // 3cm
	mps.setInputNormals (cloud_normals);

	mps.setInputCloud (cloud_filtered);
	
	std::vector< pcl::ModelCoefficients > model_coefficients;
	pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
	std::vector< pcl::PointIndices > label_indices;
	std::vector< pcl::PointIndices > boundary_indices;
	mps.segmentAndRefine(regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
}

// Build a cloud with all planes
pcl::PointIndices::Ptr inliers_AllPlanes(std::vector< pcl::PointIndices > &inlier_indices){
	pcl::PointIndices::Ptr ptr_plane (new pcl::PointIndices());
	for (size_t i = 0; i < inlier_indices.size(); i++){ //For each flat region
		for (size_t j = 0; j < inlier_indices[i].indices.size(); j++){ // For each inlier indice
			ptr_plane->indices.push_back(inlier_indices[i].indices[j]); // Create a vector with all planes
		}
	}
	return ptr_plane;
}

// Removing specific points (e.g., inliers of planes)
pcl::PointCloud<PointT>::Ptr removePoints (pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointIndices::Ptr planes_ind){
	pcl::PointCloud<PointT>::Ptr cloud_noPlanes (new pcl::PointCloud<PointT>);
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (cloud_in);
   	extract.setIndices (planes_ind);
    	extract.setNegative (true);
    	extract.filter (*cloud_noPlanes);
	return cloud_noPlanes;
}

// Creating clusters based on distance among points (Euclidean Clusters)
void euclideanClusters( std::vector< pcl::PointCloud<PointT>::Ptr > &clusters, 
			pcl::search::KdTree<PointT>::Ptr tree,
 			pcl::PointCloud<PointT>::Ptr cloud2){
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (640*480);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud2);
	ec.extract (cluster_indices);

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
		pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
			cloud_cluster->points.push_back (cloud2->points[*pit]); 
		}
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		
		clusters.push_back(cloud_cluster);
	}
}

// Evaluate if a cluster has a cylinder. Return 0 if the subcloud does not contain cylinder. Return 1 otherwise.
bool cluster_ContainCyl(pcl::PointCloud<PointT>::Ptr cloud_cylinder, 
			std::vector< pcl::PointCloud<PointT>::Ptr > &clusters, 
			size_t i){
	// Detect and extract cylinder in one cluster
	std::vector< int > indices;
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr cluster_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<PointT>::Ptr cluster_tree (new pcl::search::KdTree<PointT> ());
	ne.setSearchMethod (cluster_tree);
	ne.setInputCloud (clusters[i]);
	ne.setKSearch (400);
	ne.compute (*cluster_normals);

	//Verify if a cluster has a cylinder
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_CYLINDER);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight (0.1);
	seg.setMaxIterations (10000);
	seg.setDistanceThreshold (0.02); //2cm
	seg.setRadiusLimits (0, 0.1);
	seg.setInputCloud (clusters[i]);
	seg.setInputNormals (cluster_normals);	
		
	//Detect cylinder
	pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
	seg.segment (*inliers_cylinder, *coefficients_cylinder);

	//Extract just the cylinder from the cloud
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (clusters[i]);
	extract.setIndices (inliers_cylinder);
	extract.setNegative (false);
	extract.filter (*cloud_cylinder);
		
	if (cloud_cylinder->width > 100){ //Cylinder cloud must have at least a minimum number of poits
		return true;
	}
	return false;	
}
