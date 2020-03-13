#include <iostream>
#include <sstream>
#include <pcl/ModelCoefficients.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/extract_indices.h>
#include "plyRead.h"
//-rf 提取最大平面--地面 重新定义坐标系，删除地面以下0.11m的所有点(已经取消)。
//-f提取最大平面
//-sf提取球。
boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

int
main(int argc, char** argv)
{
  // initialize PointClouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

// Fill in the cloud data
  pcl::PCDReader reader;
  plyRead pR;
  // Replace the path below with the path where you saved your file
//  reader.read ("table_scene_lms400.pcd", *cloud); // Remember to download the file first!
	//pcl::io::loadPLYFile<pcl::PointXYZ> ("cloud.ply", *(cloud));
	std::stringstream aa;
	aa << argv[1] << ".ply";
	pcl::io::loadPLYFile<pcl::PointXYZ> (aa.str().c_str(), *(cloud));
	std::cout << "detetion plane " << aa.str().c_str() << endl;
/*
  // populate our PointCloud with points
  cloud->width    = 500;
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    if (pcl::console::find_argument (argc, argv, "-s") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0)
    {
      cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
      cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
      if (i % 5 == 0)
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
      else if(i % 2 == 0)
        cloud->points[i].z =  sqrt( 1 - (cloud->points[i].x * cloud->points[i].x)
                                      - (cloud->points[i].y * cloud->points[i].y));
      else
        cloud->points[i].z =  - sqrt( 1 - (cloud->points[i].x * cloud->points[i].x)
                                        - (cloud->points[i].y * cloud->points[i].y));
    }
    else
    {
      cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
      cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
      if( i % 2 == 0)
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
      else
        cloud->points[i].z = -1 * (cloud->points[i].x + cloud->points[i].y);
    }
  }
*/
  std::vector<int> inliers;

  // created RandomSampleConsensus object and compute the appropriated model
  pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
    model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));

  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));
  if(pcl::console::find_argument (argc, argv, "-f") >= 0 || pcl::console::find_argument (argc, argv, "-rf") >= 0)
  {

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
	  float temp = 0;
	  if(pcl::console::parse (argc, argv, "-p",temp)> 0)
	  {
		  ;
	  }
	  else
	  {
		  temp = 0.02;
	  }
    ransac.setDistanceThreshold (temp);
    ransac.computeModel();
    ransac.getInliers(inliers);
	Eigen::VectorXf model_coefficients;
    ransac.getModelCoefficients(model_coefficients);
    cout << model_coefficients;
    if(pcl::console::find_argument (argc, argv, "-rf") >= 0)
    {
    	//只是对kinfu建立的点云的形式，也就是z指向前方的情况进行坐标变幻
    	float p = model_coefficients[3];
    	Eigen::Vector3f n;
    	n[0] = model_coefficients[0];
    	n[1] = model_coefficients[1];
    	n[2] = model_coefficients[2];
    	Eigen::Vector3f Po;
    	Po[0] = -p*model_coefficients[0];
    	Po[1] = -p*model_coefficients[1];
    	Po[2] = -p*model_coefficients[2];
/*    	Eigen::Vector3f Px,Py,Pz;
    	Px[0] = 0;
    	Px[1] = -p/model_coefficients[1];
    	Px[2] = 0;*/
    	Eigen::Vector3f Iz,Ix,Iy;
    	if( p > 0)
    	{
    		Iz = n;
    	}
    	else
    	{
    		Iz = -1*n;
    	}
    	//定义x轴为原来z轴在平面上的投影
    	Eigen::Vector3f IZ(0,0,1);
    	Iy = Iz.cross(IZ);
    	Iy.normalize();
    	Ix = Iy.cross(Iz);
/*    	Ix = Px - Po;
    	Ix.normalize();
    	Iy = Iz.cross(Ix);*/

/*    	Px = Po + Ix;
    	Py = Po + Iy;
    	Pz = Po + Iz;*/

    	Eigen::Matrix3f A,invA;
    	for(int i= 0;i < 3;i++)
    	{
    		A(i,0) = Ix[i];
    		A(i,1) = Iy[i];
    		A(i,2) = Iz[i];
    	}
    	invA = A.inverse();

    	for (size_t i = 0; i < cloud->points.size (); ++i)
    	{
    		Eigen::Vector3f temp;
    		temp[0] = cloud->points[i].x;
    		temp[1] = cloud->points[i].y;
    		temp[2] = cloud->points[i].z;
    		temp = temp - Po;
    		temp = invA*temp;
    		cloud->points[i].x = temp[0];
    		cloud->points[i].y = temp[1];
    		cloud->points[i].z = temp[2];
    	}
    	//保存旋转code
    	std::fstream out3;
    	char numStr[9][100];
    	for(int i = 0;i < 3;i++)
    	{
    		for(int j =0 ;j < 3;j++)
    		{
        		if(invA(i,j) >= 0 && j!=0)
        		{
        			sprintf(numStr[i*3 +j],"+ %f",invA(i,j));
        		}
        		else
        		{
        			sprintf(numStr[i*3 +j],"%f",invA(i,j));
        		}
    		}

    	}


    	Eigen::Vector3f Iz2,Ix2,Iy2;
    	if( p < 0)
    	{
    		Iy2 = n;
    	}
    	else
    	{
    		Iy2 = -1*n;
    	}
    	//定义x轴为原来z轴在平面上的投影
    	Eigen::Vector3f IZ2(0,0,1);
    	Ix2 = Iy2.cross(IZ2);
    	Ix2.normalize();
    	Iz2 = Ix2.cross(Iy2);
/*    	Ix = Px - Po;
    	Ix.normalize();
    	Iy = Iz.cross(Ix);*/

/*    	Px = Po + Ix;
    	Py = Po + Iy;
    	Pz = Po + Iz;*/

    	Eigen::Matrix3f A2,invA2;
    	for(int i= 0;i < 3;i++)
    	{
    		A2(i,0) = Ix2[i];
    		A2(i,1) = Iy2[i];
    		A2(i,2) = Iz2[i];
    	}
    	invA2 = A2.inverse();
    	for(int i = 0;i < 3;i++)
    	{
    		for(int j =0 ;j < 3;j++)
    		{
        		if(invA2(i,j) >= 0 && j!=0)
        		{
        			sprintf(numStr[i*3 +j],"+ %f",invA2(i,j));
        		}
        		else
        		{
        			sprintf(numStr[i*3 +j],"%f",invA2(i,j));
        		}
    		}

    	}
    	out3.open("planecode.txt",std::ios::out);
		out3 << "tempA[0] = ("<<numStr[0]<<"* vcurr[0]"<<numStr[1]<<"* vcurr[1]"<<numStr[2]<<" * vcurr[2]);"<<endl;
		out3 << "tempA[1] = ("<<numStr[3]<<"* vcurr[0]"<<numStr[4]<<"* vcurr[1]);"<<endl;
		out3 << "tempA[2] = ("<<numStr[6]<<"* vcurr[0]"<<numStr[7]<<"* vcurr[1]"<<numStr[8]<<" * vcurr[2]);"<<endl;
//		tempA[1] = (-0.999192 * vcurr[0] + 0.0401852 * vcurr[1]);
//		tempA[2] = (-0.0400957 * vcurr[0] - 0.996966 * vcurr[1] - 0.0667137 * vcurr[2]);
		out3.close();
    	out3.open("planecode2.txt",std::ios::out); //for kinfu
    	for(int i= 0;i < 3;i++)
    	{
		
out3 << "R(" << i << ",0) = " << numStr[i*3] << ';'<<"R(" << i << ",1) = " << numStr[i*3+1] << ';' <<"R(" << i << ",2) = " << numStr[i*3+2] << ';'<< endl;

    	}
//		for(int i = 0;i < 9;i++)
//{
//out3 << "R.data(" << i << ") = " << numStr[i] << ';' << endl;
//}
		//out3 << "R.data[0] = "<<numStr(0]<<";R.data(1] = "<<numStr[1]<<";R.data[2] = "<<numStr[2]<<";"<<endl;
		//out3 << "R.data[3] = "<<numStr[3]<<";R.data[4] = "<<numStr[4]<<";R.data[5] = "<<numStr[5]<<";"<<endl;
		//out3 << "R.data[6] = "<<numStr[6]<<";R.data[7] = "<<numStr[7]<<";R.data[8] = "<<numStr[8]<<";"<<endl;
//		tempA[1] = (-0.999192 * vcurr[0] + 0.0401852 * vcurr[1]);
//		tempA[2] = (-0.0400957 * vcurr[0] - 0.996966 * vcurr[1] - 0.0667137 * vcurr[2]);
		out3.close();
    	//保存旧坐标到新坐标的变幻矩阵，平移分量位相反
    	std::fstream out1;//file is closed on destruction
		std::stringstream cc;
		cc<<argv[1]<<"_plane_T.txt";
		out1.open(cc.str().c_str(),std::ios::out); //TODO: Errormessage
		out1 << invA << endl;
		out1 << std::abs(p)<<endl;
		out1 << Po << endl;

		////////////////////////
    	//删除地面一下多余点。
    	//std::vector<int> outliers;

    	pcl::PointIndices::Ptr outliers (new pcl::PointIndices);
    	for (size_t i = 0; i < cloud->points.size (); ++i)
		{
			Eigen::Vector3f temp;
			temp[2] = cloud->points[i].z;
			if(temp[2] < -0.011)
			{
				outliers->indices.push_back(i);
			}
		}
    	pcl::ExtractIndices<pcl::PointXYZ> extract;
    	extract.setInputCloud (cloud);
		extract.setIndices (outliers);
		extract.setNegative (true);
		extract.filter (*cloud); //


    	//移动到box角点
    	Eigen::Vector4f min_pt;
    	Eigen::Vector4f max_pt;
    	pcl::getMinMax3D(*cloud,min_pt,max_pt);
    	cout << "min_z" << min_pt[2];
    	Eigen::Vector3f Poo;
    	for(size_t i = 0; i < 2; ++i)  //just move xy
    	{
    		Poo[i] = min_pt[i];
    	}
    	for (size_t i = 0; i < cloud->points.size (); ++i)
    	{
    		Eigen::Vector3f temp;
    		temp[0] = cloud->points[i].x;
    		temp[1] = cloud->points[i].y;
    		temp[2] = cloud->points[i].z;
    		temp = temp - Poo;
    		cloud->points[i].x = temp[0];
    		cloud->points[i].y = temp[1];
    		cloud->points[i].z = temp[2];
    	}
    	out1 << Poo <<endl;
		out1.close();
		std::stringstream bb;
		bb << argv[1] << "_t.ply";
		if(pcl::console::find_argument (argc, argv, "-m") >= 0)
    	{
    		pR.savePly(aa.str().c_str(),bb.str().c_str(), *(cloud));
    	}
    	else
    	{
    		pcl::io::savePLYFile<pcl::PointXYZ> (bb.str().c_str(), *(cloud));
    	}
    }

  }
  else if (pcl::console::find_argument (argc, argv, "-sf") >= 0 )
  {
	model_s->setRadiusLimits(0.05,0.13);
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
    ransac.setMaxIterations(5000000);
    ransac.setDistanceThreshold (.05);
    ransac.computeModel();
    ransac.getInliers(inliers);
    Eigen::VectorXf model_coefficients;
    ransac.getModelCoefficients(model_coefficients);
    cout << model_coefficients;
  }
  
  // copies all inliers of the model computed to another PointCloud
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
  pcl::io::savePLYFile<pcl::PointXYZ> ("ground.ply", *(final));
  // creates the visualization object and adds either our orignial cloud or all of the inliers
  // depending on the command line arguments specified.
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  if (pcl::console::find_argument (argc, argv, "-f") >= 0 ||pcl::console::find_argument (argc, argv, "-rf") >= 0|| pcl::console::find_argument (argc, argv, "-sf") >= 0)
    viewer = simpleVis(final);
  else
    viewer = simpleVis(cloud);
  /*
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  */
  return 0;
 }

