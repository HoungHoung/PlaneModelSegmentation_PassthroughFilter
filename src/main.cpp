#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <string>

using namespace pcl;
using namespace std;

int user_data;

//	PCD to PLY convertor
int PCDtoPLYconvertor(string & input_filename)
{
	pcl::PCLPointCloud2 cloud;
	if (pcl::io::loadPCDFile(input_filename, cloud) == -1)
	{
		cout << "Error: cannot load the PCD file!!!" << endl;
		return -1;
	}
	PLYWriter writer;
	writer.write(".//Data//PLY//output.ply", cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true, true);
	return 0;
}

// PLY to PCD convertor
int PLYtoPCDconvertor(string& input_filename)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPLYFile(input_filename, *cloud) == -1)
	{
		cout << "Error: cannot load the PLY file!!!" << endl;
		return -1;
	}
	pcl::io::savePCDFile(".//Data//PCD//output.pcd", *cloud);
	return 0;
}

//	Visualization PCD point cloud data
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);

	//FIXME: possible race condition here:
	user_data++;
}

int VisualizationPCD(string& input_filename)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_filename, *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file PCD \n");
		return (-1);
	}
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(cloud);
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	viewer.runOnVisualizationThread(viewerPsycho);
	while (!viewer.wasStopped())
	{
		user_data++;
	}
	return 0;
}

//	Visualization PLY point cloud data
int VisualizationPLY(string& input_filename)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(input_filename, *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file PLY \n");
		return (-1);
	}
	pcl::visualization::PCLVisualizer viewer("Simple visualizing window");
	viewer.addPointCloud(cloud, "cloud");
	viewer.spinOnce();
	viewer.removePointCloud("cloud");
	return 0;
}


int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	reader.read(".//Data//PCD//output.pcd",*cloud);
	std::cerr << "Point cloud data: " << cloud->points.size() << " points" << std::endl;

	//	Pass through filter
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pt;
	pt.setInputCloud(cloud);
	pt.setFilterFieldName("y");
	pt.setFilterLimits(-5.0, 0.8);
	pt.filter(*cloud_source_filtered);

	// Segmentation
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.15);

	seg.setInputCloud(cloud_source_filtered);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return (-1);
	}

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud_source_filtered);
	extract.setIndices(inliers);

	extract.setNegative(true);
	extract.filter(*cloud_filtered);

	pt.setInputCloud(cloud_filtered);
	pt.setFilterFieldName("x");
	pt.setFilterLimits(-0.67, 0.43);
	pt.filter(*cloud_filtered);

	pcl::visualization::CloudViewer viewer("Filtered");
	viewer.showCloud(cloud_filtered);
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	while (!viewer.wasStopped()) 
	{

	}

	/*string input_filename = "3dpoints_ground.pcd";
	VisualizationPCD(input_filename);*/
	return 0;
}