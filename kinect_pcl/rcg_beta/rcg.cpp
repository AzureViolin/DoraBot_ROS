#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <unistd.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/shot_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/transforms.h>


float z_pass_min_ (-10.00f);
float z_pass_max_ (10.00f);
float y_pass_min_ (-10.00f);
float y_pass_max_ (10.00f);
float x_pass_min_ (-10.00f);
float x_pass_max_ (10.00f);
float sample_size_ (0.01f);


float descr_rad_ (0.02f);

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::Normal NormalType;
typedef pcl::SHOT352 DescriptorType;

// PCL viewer //
pcl::visualization::PCLVisualizer viewer("PCL Viewer");
boost::mutex cloud_mutex;
pcl::SHOTEstimationOMP<PointT, NormalType, DescriptorType> descr_est;
pcl::NormalEstimationOMP<PointT, NormalType> norm_est;


void
parseCommandLine (int argc, char *argv[])
{
  //General parameters
  pcl::console::parse_argument (argc, argv, "--z_max", z_pass_max_);
  pcl::console::parse_argument (argc, argv, "--z_min", z_pass_min_);
  pcl::console::parse_argument (argc, argv, "--y_max", y_pass_max_);
  pcl::console::parse_argument (argc, argv, "--y_min", y_pass_min_);
  pcl::console::parse_argument (argc, argv, "--x_max", x_pass_max_);
  pcl::console::parse_argument (argc, argv, "--x_min", x_pass_min_); 
  pcl::console::parse_argument (argc, argv, "--sample_size", sample_size_);
}

void cloud_cb_ (const PointCloudT::ConstPtr &callback_cloud, PointCloudT::Ptr& cloud,
    bool* new_cloud_available_flag)
{
  cloud_mutex.lock ();    // for not overwriting the point cloud from another thread
  *cloud = *callback_cloud;
  *new_cloud_available_flag = true;
  cloud_mutex.unlock ();
}


pcl::CorrespondencesPtr cloud_corr_ (pcl::PointCloud<DescriptorType>::Ptr model_des , PointCloudT::Ptr& scene)
{
	
	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());
    pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());

	
	//  Compute Normals
	norm_est.setInputCloud (scene);
	norm_est.compute (*scene_normals);

	descr_est.setInputCloud (scene);
	descr_est.setInputNormals (scene_normals);
	descr_est.setSearchSurface (scene);
	descr_est.compute (*scene_descriptors);

	//
	//  Find Model-Scene Correspondences with KdTree
	//
	pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

	pcl::KdTreeFLANN<DescriptorType> match_search;
	match_search.setInputCloud (model_des);

	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	for (size_t i = 0; i < scene_descriptors->size (); ++i)
	{
		std::vector<int> neigh_indices (1);
		std::vector<float> neigh_sqr_dists (1);
		if (!pcl_isfinite (scene_descriptors->at(i).descriptor[0])) //skipping NaNs
		{
		 	continue;
		}
		int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
		if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
		{
		pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
		model_scene_corrs->push_back (corr);
		}
	}
	std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;
	return model_scene_corrs;
}





int main (int argc, char** argv)
{
	
	/* read cloud points from kinect*/
	// Read Kinect live stream:
	pcl::PCDWriter writer;
	pcl::PCDReader reader;
	PointCloudT::Ptr model (new PointCloudT);
    reader.read ("model.pcd", *model); // Remember to download the file first!
	pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
	pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
	norm_est.setKSearch (10);
	norm_est.setInputCloud (model);
	norm_est.compute (*model_normals);
	descr_est.setRadiusSearch (descr_rad_);
	descr_est.setInputCloud (model);
	descr_est.setInputNormals (model_normals);
	descr_est.setSearchSurface (model);
	descr_est.compute (*model_descriptors);

	parseCommandLine (argc, argv);

	PointCloudT::Ptr cloud (new PointCloudT);
	bool new_cloud_available_flag = false;
	pcl::Grabber* interface = new pcl::OpenNIGrabber();
	boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f = boost::bind (&cloud_cb_, _1, cloud, &new_cloud_available_flag);
	interface->registerCallback (f);
	interface->start ();

	// Wait for the first frame:
	while(!new_cloud_available_flag) 
		boost::this_thread::sleep(boost::posix_time::milliseconds(1));
	new_cloud_available_flag = false;

	cloud_mutex.lock ();    // for not overwriting the point cloud
	// Display pointcloud:
	viewer.addPointCloud<PointT> (cloud, "input_cloud");
	cloud_mutex.unlock ();
	viewer.spinOnce ();  	
	viewer.removeAllPointClouds();
	


	/*apply pass through filter*/
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	//pass.setFilterLimitsNegative (true);
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (z_pass_min_, z_pass_max_);
	pass.filter (*cloud_filtered);
	pass.setInputCloud (cloud_filtered);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (y_pass_min_, y_pass_max_);
	pass.filter (*cloud_filtered);
	pass.setInputCloud (cloud_filtered);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (x_pass_min_, x_pass_max_);
	pass.filter (*cloud_filtered);

	viewer.addPointCloud<PointT> (cloud_filtered, "input_cloud");
	viewer.spinOnce ();  	
	viewer.removeAllPointClouds();
	cloud = cloud_filtered;
	


	/*down sample the cloud*/
	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	/*
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud (cloud);
	vg.setLeafSize (sample_size_, sample_size_, sample_size_);
	vg.filter (*cloud_filtered);

	viewer.addPointCloud<PointT> (cloud_filtered, "input_cloud");
	viewer.spinOnce ();
	viewer.removeAllPointClouds();
	cloud = cloud_filtered;*/


	
	/*PLANE MODEL FILTER*/
	// Create the segmentation object for the planar model and set all the parameters
	
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.01);

	int i=0, nr_points = (int) cloud->points.size ();

    PointCloudT::Ptr cloud_f (new PointCloudT);
	while (cloud->points.size () > 0.5 * nr_points)
	{
		viewer.removeAllPointClouds();
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
		  std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
		  break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (cloud);
		extract.setIndices (inliers);
		extract.setNegative (false);

		// Get the points associated with the planar surface
		extract.filter (*cloud_plane);

		// Remove the planar inliers, extract the rest
		extract.setNegative (true);
		extract.filter (*cloud_f);
		
		viewer.addPointCloud<PointT> (cloud_f, "input_cloud");
		viewer.spinOnce ();
		//boost::this_thread::sleep(boost::posix_time::millisec(5000));
		std::cout << "***************************************************************************" << std::endl;
		*cloud = *cloud_f;
		
	}
	viewer.removeAllPointClouds();
	




	/*EUCLIDIAN CLUSTER*/
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);

	int j = 0;
	float max_rate = 0.0;
	pcl::CorrespondencesPtr max_corr (new pcl::Correspondences ());
	PointCloudT::Ptr max_cloud (new PointCloudT);
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
		  cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: "<< j << ":"<< cloud_cluster->points.size () << " data points." << std::endl;
		pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler (cloud_cluster, j*20, 255-j*20, j*20);
		std::stringstream ss;
    	ss << "cloud_cluster_" << j << ".pcd";
		//writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
		viewer.addPointCloud (cloud_cluster, color_handler, ss.str() );
		pcl::CorrespondencesPtr tmp_corr = cloud_corr_( model_descriptors , cloud_cluster);
		int corr_count = tmp_corr->size ();
		float corr_rate = corr_count * 1.0 / cloud_cluster->points.size ();
		if (corr_rate > max_rate)
		{
			max_rate = corr_rate;
			max_corr = tmp_corr;
			max_cloud = cloud_cluster;
		}
		std::cout << "corr rate :"<< j << " : "<< corr_rate << std::endl;
		j++;
	}
	
	std::cout << "max_corr_rate :"<< max_corr->size()<< " : "<< max_rate << std::endl;
	

	
	//viewer.removeAllPointClouds();
	
	
	//display result
	pcl::PointCloud<PointT>::Ptr off_scene_model (new pcl::PointCloud<PointT> ());
	pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
    pcl::visualization::PointCloudColorHandlerCustom<PointT> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
    viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
	
	for (size_t j = 0; j < max_corr->size(); ++j)
    {
        std::stringstream ss_line;
        ss_line << "correspondence_line" << "_" << j;
        PointT& model_point = off_scene_model->at (max_corr->at(j).index_query);
        PointT& scene_point = max_cloud->at (max_corr->at(j).index_match);
        //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
        viewer.addLine<PointT, PointT> (model_point, scene_point, 0, 255, 255, ss_line.str ());
    }


	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
	}
	return (0);
}



