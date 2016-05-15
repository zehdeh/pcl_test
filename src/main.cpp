#include <iostream>
#include <vector>
#include <string>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>

void PointCloud2Vector3d (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data) {
  for (unsigned i = 0; i < cloud->size (); i++) {
    Point &p = cloud->at (i);
    if (!pcl_isnan (p.x) && !pcl_isnan (p.y) && !pcl_isnan (p.z))
      data.push_back (Eigen::Vector3d (p.x, p.y, p.z));
  }
}

int main(int argc, char** argv) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);

	if(pcl::io::loadPCDFile<pcl::PointXYZ>("res/ism_test_cat.pcd", *input) == -1) {
		PCL_ERROR("Couldn't read file!\n");
		return -1;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	uint8_t r(255), g(15), b(15);
	for(size_t i = 0; i < input->points.size(); ++i) {
		pcl::PointXYZRGB point;
		point.x = input->points[i].x;
		point.y = input->points[i].y;
		point.z = input->points[i].z;
		r = input->points[i].z*2;
		b = 100 + (input->points[i].x)*20;

		uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
		point.rgb = *reinterpret_cast<float*>(&rgb);
		
		cloud->points.push_back(point);
	}

	pcl::on_nurbs::NurbsDataSurface data;
	PointCloud2Vector3d(cloud, data.interior);

	ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCAAboundingBox(3, &data);
	pcl::on_nurbs::FittingSurface fit(&data, nurbs);

	pcl::PolygonMesh mesh;
	pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::Vertices> mesh_vertices;
	std::string mesh_id = "mesh_nurbs";
	pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(fit.m_nurbs, mesh, mesh_resolution);

	pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer.addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();

	viewer.addPolygonMesh(mesh, mesh_id);
	//viewer.showCloud(cloud);
	while(!viewer.wasStopped()) {
		viewer.spinOnce(100);
	}

	return 0;
}
