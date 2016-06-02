#include "fitcubes.h"

#include <iostream>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/centroid.h>
#include <pcl/registration/icp.h>

void removeOutliers(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cube) {
	/*
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBNormal> sor;
	sor.setInputCloud(cube);
	sor.setMeanK(100);
	sor.setStddevMulThresh(0.1);
	sor.filter(*cube);
	*/

	pcl::RadiusOutlierRemoval<pcl::PointXYZRGBNormal> ror;
	ror.setInputCloud(cube);
	ror.setRadiusSearch(8.0);
	ror.setMinNeighborsInRadius(5);
	ror.filter(*cube);
}

void cutOffY(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cube, float cutoff) {
	pcl::ConditionAnd<pcl::PointXYZRGBNormal>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZRGBNormal>());
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGBNormal>::ConstPtr(
		new pcl::FieldComparison<pcl::PointXYZRGBNormal>("y", pcl::ComparisonOps::GT, cutoff))
	);
	pcl::ConditionalRemoval<pcl::PointXYZRGBNormal> condrem;
	condrem.setCondition(range_cond);
	condrem.setInputCloud(cube);
	condrem.setKeepOrganized(true);
	condrem.filter(*cube);
}

void colorCube(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cube, unsigned int r, unsigned int g, unsigned int b) {
	for(unsigned int i = 0; i < cube->points.size(); i++) {
		cube->points[i].r = r;
		cube->points[i].g = g;
		cube->points[i].b = b;
	}
}

void doICP(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr source, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target) {
	// Important for ICP to work!
	source->is_dense = false;

	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*source, *source, indices);

	pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
	icp.setInputSource(source);
	icp.setInputTarget(target);

	icp.align(*source);
	std::cout << "has converged: " << icp.hasConverged() << std::endl;
	std::cout << "ICP score: " << icp.getFitnessScore() << std::endl;
}

void buildCube(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cube, float size, unsigned int numPoints) {
	cube->points.resize(numPoints);

	unsigned int pointsPerPlane = numPoints / 6;
	unsigned int pointsPerLine = floor(sqrt(pointsPerPlane));
	std::cout << "Num points: " << numPoints << std::endl;
	std::cout << "Points per plane: " << pointsPerPlane << std::endl;
	std::cout << "Points per line: " << pointsPerLine << std::endl;

	for(unsigned int i = 0; i < numPoints; i++) {
		unsigned int j = i % pointsPerPlane;
		float relSize = size / pointsPerLine;
		float a = size/2;
		float b = ((j % pointsPerLine) * relSize) - (size/2);
		float c = ((j / pointsPerLine) * relSize) - (size/2);

		if(i < pointsPerPlane) {
			cube->points[i].x = a;
			cube->points[i].y = b;
			cube->points[i].z = c;
		} else if(i >= pointsPerPlane && i < 2*pointsPerPlane) {
			cube->points[i].x = -1*a;
			cube->points[i].y = b;
			cube->points[i].z = c;
		} else if(i >= 2*pointsPerPlane && i < 3*pointsPerPlane) {
			cube->points[i].x = b;
			cube->points[i].y = a;
			cube->points[i].z = c;
		} else if(i >= 3*pointsPerPlane && i < 4*pointsPerPlane) {
			cube->points[i].x = b;
			cube->points[i].y = -1*a;
			cube->points[i].z = c;
		} else if(i >= 4*pointsPerPlane && i < 5*pointsPerPlane) {
			cube->points[i].x = b;
			cube->points[i].y = c;
			cube->points[i].z = a;
		} else {
			cube->points[i].x = b;
			cube->points[i].y = c;
			cube->points[i].z = -1*a;
		}


		cube->points[i].r = 0;
		cube->points[i].g = 0;
		cube->points[i].b = 255;
	}
}

int fitcubes() {
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr center1(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	if(pcl::io::loadOBJFile("res/CalinBox/Center_Box_0/obj/Center_Box_0.000001.obj", *center1) == -1) {
		PCL_ERROR("Couldn't read file!\n");
		return -1;
	}

	cutOffY(center1, 430.0);
	removeOutliers(center1);
	//colorCube(center1, 255,0,0);

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr down1(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	if(pcl::io::loadOBJFile("res/CalinBox/Down_Box_0/obj/Down_Box_0.000001.obj", *down1) == -1) {
		PCL_ERROR("Couldn't read file!\n");
		return -1;
	}

	cutOffY(down1, 190.0);
	removeOutliers(down1);

	/*
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr east1(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	if(pcl::io::loadOBJFile("res/CalinBox/East_Box_0/obj/East_Box_0.000001.obj", *east1) == -1) {
		PCL_ERROR("Couldn't read file!\n");
		return -1;
	}

	cutOffY(east1, 300.0);
	removeOutliers(east1);
	colorCube(east1, 0,255,0);
	*/

	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*center1, centroid);
	pcl::demeanPointCloud(*center1, centroid, *center1);
	
	pcl::compute3DCentroid(*down1, centroid);
	pcl::demeanPointCloud(*down1, centroid, *down1);

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr generated(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	generated->points.resize(center1->points.size());
	buildCube(generated, 150, center1->points.size());

	doICP(center1, generated);
	doICP(down1, generated);
	for(unsigned int i = 0; i < center1->points.size(); i++) {
		if(center1->points[i].x > 75 || center1->points[i].x < -75 || center1->points[i].y > 75 || center1->points[i].y < -75 || center1->points[i].z > 75 || center1->points[i].z < -75) {
			center1->points[i].r = 255;
		}
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Simple Cloud Viewer"));

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb1(center1);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb2(down1);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb3(generated);
	viewer->addPointCloud<pcl::PointXYZRGBNormal>(center1, rgb1, "Cube centered");

	//viewer->addPointCloudNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(center1,center1,1,2.0f, "Cube centered Normals");
	viewer->addPointCloud<pcl::PointXYZRGBNormal>(down1, "Cube down");
	viewer->addPointCloud<pcl::PointXYZRGBNormal>(generated, rgb3, "Cube generated");
	//viewer->addPointCloudNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(down1,down1,1,2.0f, "Cube down Normals");
	//viewer->addPointCloud<pcl::PointXYZRGBNormal>(east1, rgb2, "Cube east");

	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	while(!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	
	return 0;
}
