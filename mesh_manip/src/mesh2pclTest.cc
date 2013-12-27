/*
 * mesh2pclTest.cc
 *
 *  Created on: Dec 18, 2013
 *      Author: isura
 */

#include <vector>


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Polygon.h>

// PCL specific includes
#include <pcl/Vertices.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/mls.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

ros::Publisher pub_cloud;
ros::Publisher pub_mesh;

//Eigen::Vector3d pointToEigenVector( pcl::PointXYZ & p )
//{
//  return Eigen::Vector3d( p.x, p.y, p.z);
//}

double sqMtosqCm( const double & area_in_m )
{
  return area_in_m*10000.0;
}

// sensors per sq cm
double sensorDensity( const double & sensors,
                      const double & area )
{
  return sensors/area;
}

//Eigen::Vector3f funFoo( const Eigen::Vector3f & p1,
//                        const Eigen::Vector3f & p2,
//                        const Eigen::Vector3f & p3 )
//{
//  return Eigen::Vector3f::Zero();
//}

std::vector< ::pcl::Vertices>
getNewPolygonVertices( const std::vector< ::pcl::Vertices> & in_poly,
                       const int index,
                       const int center_vertex )
{

  std::vector< ::pcl::Vertices>  out_poly;
  out_poly.resize(3);

  out_poly[0].vertices.resize(3);
  out_poly[0].vertices[0] = in_poly[index].vertices[0];
  out_poly[0].vertices[1] = in_poly[index].vertices[1];
  out_poly[0].vertices[2] = center_vertex;

  out_poly[1].vertices.resize(3);
  out_poly[1].vertices[0] = in_poly[index].vertices[1];
  out_poly[1].vertices[1] = in_poly[index].vertices[2];
  out_poly[1].vertices[2] = center_vertex;

  out_poly[2].vertices.resize(3);
  out_poly[2].vertices[0] = in_poly[index].vertices[2];
  out_poly[2].vertices[1] = in_poly[index].vertices[0];
  out_poly[2].vertices[2] = center_vertex;

  return out_poly;
}

std::vector< ::pcl::Vertices>
upsamplePolygon( const std::vector< ::pcl::Vertices> & in_poly,
                       pcl::PointCloud<pcl::PointXYZ> & in_cloud   )
{


  pcl::PointCloud<pcl::PointXYZ> temp_cloud;
  temp_cloud.points.resize(3);

  Eigen::Vector4d center;
  center = Eigen::Vector4d::Zero();

  pcl::PointXYZ point;

  std::vector< ::pcl::Vertices> new_polygon;

  for( unsigned int i = 0; i < in_poly.size(); i++ )
  {
    in_poly[i].vertices[0];
    in_poly[i].vertices[1];
    in_poly[i].vertices[2];

    temp_cloud.points[0] = in_cloud.points[ in_poly[i].vertices[0] ];
    temp_cloud.points[1] = in_cloud.points[ in_poly[i].vertices[1] ];
    temp_cloud.points[2] = in_cloud.points[ in_poly[i].vertices[2] ];

    pcl::compute3DCentroid( temp_cloud, center );

    point.x = center(0);
    point.y = center(1);
    point.z = center(2);

    in_cloud.points.push_back( point );

    std::vector< ::pcl::Vertices> temp_polygon;
    temp_polygon = getNewPolygonVertices( in_poly, i, ( in_cloud.size() -1) );

    new_polygon.push_back( temp_polygon[0] );
    new_polygon.push_back( temp_polygon[1] );
    new_polygon.push_back( temp_polygon[2] );
  }

  return new_polygon;

}

int
main( int argc, char** argv )
{
	// Initialize ROS
	ros::init (argc, argv, "mesh2pclTest");
	ros::NodeHandle nh;

	const std::string meshFileName = "test.stl";

	pcl::PolygonMesh testMesh;
	pcl::PCLPointCloud2 processedMesh;

	sensor_msgs::PointCloud2 cloud_msg;
	pcl_msgs::PolygonMesh mesh_msg;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> cloud_small;

//	pcl::io::loadPolygonFileSTL( mescloud_smallhFileName, testMesh );


	testMesh.polygons.resize(1);
	testMesh.polygons[0].vertices.resize(3);
	testMesh.polygons[0].vertices[0] = 0;
	testMesh.polygons[0].vertices[1] = 1;
	testMesh.polygons[0].vertices[2] = 2;

	cloud_small.points.resize(3);
	cloud_small.points[0].x = -0.5;
	cloud_small.points[0].y = 0;
	cloud_small.points[0].z = -0.5;

	cloud_small.points[1].x = 0.5;
	cloud_small.points[1].y = 0;
	cloud_small.points[1].z = 0.5;

	cloud_small.points[2].x = 0;
	cloud_small.points[2].y = 0.5;
	cloud_small.points[2].z = 0;


	std::cout << "\n Area : " << pcl::calculatePolygonArea( cloud_small )
	                          << " in sq cm : " << sqMtosqCm(pcl::calculatePolygonArea( cloud_small ))
	                          << " sensor density : " << sensorDensity( 10, sqMtosqCm(pcl::calculatePolygonArea( cloud_small )) )  << "\n\n";

	std::vector< ::pcl::Vertices> new_polygon;

//	Eigen::Vector4d center;
//	pcl::compute3DCentroid( cloud_small, center );
//	pcl_conversions::fromPCL(testMesh, mesh_msg);
//        pcl::PointXYZ point;
//        point.x = center(0);
//        point.y = center(1);
//        point.z = center(2);
//        cloud_small.points.push_back( point );
//	std::cout << "\n Center : " << center.transpose() << "\n\n";

//	new_polygon = getNewPolygonVertices( testMesh.polygons, 0,  99 );


	new_polygon = upsamplePolygon( testMesh.polygons, cloud_small );
	new_polygon = upsamplePolygon( new_polygon, cloud_small );
	new_polygon = upsamplePolygon( new_polygon, cloud_small );


//	pcl::PointCloud<pcl::PointXYZ> hull;
//        std::vector<pcl::Vertices> polygons;
//
//        pcl::ConvexHull<pcl::PointXYZ> chull;
//        chull.setInputCloud (cloud);
//        chull.reconstruct (hull, polygons);


//      ROS_INFO_STREAM("No of polygons: " << new_polygon.size() );
////      ROS_INFO_STREAM("No of points  : " << testMesh.cloud.data.size() );
//      ROS_INFO_STREAM("No of points in polygon : " << new_polygon[0].vertices.size(); );
//      ROS_INFO_STREAM("Vertices of polygon 1 : " << new_polygon[0].vertices[0] << " | "<< new_polygon[0].vertices[1] << " |  "<< new_polygon[0].vertices[2]);
//      ROS_INFO_STREAM("Vertices of polygon 2 : " << new_polygon[1].vertices[0] << " | "<< new_polygon[1].vertices[1] << " |  "<< new_polygon[1].vertices[2]);
//      ROS_INFO_STREAM("Vertices of polygon 3 : " << new_polygon[2].vertices[0] << " | "<< new_polygon[2].vertices[1] << " |  "<< new_polygon[2].vertices[2]);

//	ROS_INFO_STREAM("No of polygons: " << mesh_msg.polygons.size() );
//	ROS_INFO_STREAM("No of points  : " << testMesh.cloud.data.size() );
//	ROS_INFO_STREAM("No of points in polygon : " << testMesh.polygons[0].vertices.size(); );
//	ROS_INFO_STREAM("xyz of polygon : " << testMesh.polygons[1000].vertices[0] << "  "<< testMesh.polygons[1000].vertices[1] << "  "<< testMesh.polygons[1000].vertices[2]);

//	pcl::fromPCLPointCloud2(testMesh.cloud, *cloud);

//	*cloud_filtered = cloud_small;

//	// Create the filtering object
//	pcl::PassThrough<pcl::PointXYZ> pass;
//	pass.setInputCloud (cloud);
//	pass.setFilterFieldName ("z");
//	pass.setFilterLimits (0.0, 1.0);
//	//pass.setFilterLimitsNegative (true);
//	pass.filter (*cloud_filtered);
//
//	// Create the filtering object
//	pcl::VoxelGrid<pcl::PointXYZ> sor;
//	sor.setInputCloud (cloud);
//	sor.setLeafSize (0.01f, 0.01f, 0.01f);
//	sor.filter (*cloud_filtered);


//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
//	mls.setComputeNormals(false);
//	mls.setInputCloud(cloud);
//	mls.setPolynomialFit(false);
//	mls.setSearchMethod(tree);
//	mls.setSearchRadius(0.1);
//	mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::VOXEL_GRID_DILATION);
//	mls.setDilationVoxelSize(0.003);
//	mls.process(*cloud_filtered);

//	 // Normal estimation*
//	  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//	  pcl::PointCloud<pcl::Normal>0::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//	  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//	  tree->setInputCloud (cloud);
//	  n.setInputCloud (cloud);
//	  n.setSearchMethod (tree);
//	  n.setKSearch (20);
//	  n.compute (*normals);
//	  //* normals should not contain the point normals + surface curvatures
//
//	  // Concatenate the XYZ and normal fields*
//	  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcsqMtosqCml::PointCloud<pcl::PointNormal>);
//	  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
//	  //* cloud_with_normals = cloud + normals
//
//	  // Create search tree*
//	  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
//	  tree2->setInputCloud (cloud_with_normals);
//
//	  // Initialize objects
//	  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//	  pcl::PolygonMesh triangles;
//
//	  // Set the maximum distance between connected points (maximum edge length)
//	  gp3.setSearchRadius (0.025);
//
//	  // Set typical values for the parameters
//	  gp3.setMu (2.5);
//	  gp3.setMaximumNearestNeighbors (100);
//	  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
//	  gp3.setMinimumAngle(M_PI/18); // 10 degrees
//	  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
//	  gp3.setNormalConsistency(false);
//pcl_mesh_msg
//	  // Get result
//	  gp3.setInputCloud (cloud_with_normals);
//	  gp3.setSearchMethod (tree2);
//	  gp3.reconstruct (triangles);

	pcl::toROSMsg(cloud_small, cloud_msg);

//	  pcl_conversions::fromPCL(triangles.cloud, output);


	cloud_msg.header.frame_id = "base_link";
	mesh_msg.header.frame_id = "base_link";


	// Create a ROS publisher for the output point cloud
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("mesh2pcl/cloud", 1);
    pub_mesh  = nh.advertise<pcl_msgs::PolygonMesh> ("mesh2pcl/mesh", 1);


    while(ros::ok())
    {
    	// Publish the data
        pub_cloud.publish (cloud_msg);
        pub_mesh.publish (mesh_msg);
    }


    // Spin
    ros::spin ();
}
