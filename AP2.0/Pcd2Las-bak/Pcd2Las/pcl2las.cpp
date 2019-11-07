//#include "stdafx.h"
#include <iostream>
#include <io.h>
#include <string>
#include <vector>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include "laswriter.hpp"
#include "lasreader.hpp"

#include "gdal_priv.h" 
#include "ogrsf_frmts.h" 

using namespace std;

void getAllFiles(string path, vector<string>& files, string fileType);
void writeLasFromPointCloud3(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const char* strOutLasName);

int main()
{
	string pcd_path = "E:\\1_code\\DataPcd"; // "E:/1_code/jiang/data1.pcd"; F:\0926_kbd_test_bag\00OutPCD
	string las_path = "E:\\1_code\\DataLas"; // "E:/1_code/jiang/data1.pcd";
	vector<string> allFiles;
	getAllFiles(pcd_path, allFiles, ".pcd");

	for(int i=0; i<allFiles.size(); i++) //i<allFiles.size()
	{		
		string strPCD = allFiles[i];
		cout << endl << endl << "---" << i << "---" << endl;
		//cout << "CurPcdFile string: " << strPCD << endl;

		auto name_iter = std::find(strPCD.crbegin(), strPCD.crend(), '.');
		string lasNamestr = string(strPCD.cbegin(), name_iter.base());
		lasNamestr += "las";
		//lasNamestr = las_path + lasNamestr;	
		//cout << "lasNamestr string: " << lasNamestr << endl;

		char* lasName = new char[strlen(lasNamestr.c_str()) + 1];
		strcpy(lasName, lasNamestr.c_str());
		cout << "lasName char: " << lasName << endl;
		//lasName char: E:\1_code\DataPcd\100_No.1 - 01 - 001_Lidar_HESAIGT40_TuGuan1_1.las

		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

		if (-1 == pcl::io::loadPCDFile<pcl::PointXYZI>(strPCD, *cloud))
		{
			cout << "load pcd failed!" << endl; 
			return -1;
		}
		else
			cout << "load pcd success!" << endl;
		
		writeLasFromPointCloud3(cloud, lasName);

		//while (!viewer->wasStopped())	/* Show the point cloud */
		//{
		//	viewer->spinOnce(100);        //updates the screen loop                                                   
		//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		//}

	}
	cout << "convert pcd2las number: " << allFiles.size() << endl;

	return 0;
}

void getAllFiles(string path, vector<string>& files, string fileType)
{	
	long long hFile = 0;
	struct _finddata_t fileinfo;
	string p;

	if ((hFile = _findfirst(p.assign(path).append("\\*" + fileType).c_str(), &fileinfo)) != -1) 
	{
		do 
		{
			files.push_back(p.assign(path).append("\\").append(fileinfo.name));
		} 
		while (_findnext(hFile, &fileinfo) == 0); 

		_findclose(hFile);
	}
}

void writeLasFromPointCloud3(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const char* strOutLasName)
{

	LASwriteOpener laswriteopener;
	laswriteopener.set_file_name(strOutLasName);	

	char *pszWKT;
	OGRSpatialReference oSRS;
	oSRS.SetWellKnownGeogCS("WGS84");
	oSRS.exportToWkt(&pszWKT);
	
	LASheader lasheader;
	lasheader.x_scale_factor = 0.0001;
	lasheader.y_scale_factor = 0.0001;
	lasheader.z_scale_factor = 0.0001;
	lasheader.x_offset = (int)(cloud->points[0].x);
	lasheader.y_offset = (int)(cloud->points[0].y);
	lasheader.z_offset = (int)(cloud->points[0].z);
	cout << fixed << "lasheader.x_offset = " << lasheader.x_offset << endl;
	cout << fixed << "lasheader.y_offset = " << lasheader.y_offset << endl;
	cout << fixed << "lasheader.z_offset = " << lasheader.z_offset << endl;

	lasheader.set_geo_wkt_ogc_cs(strlen(pszWKT), pszWKT);
	lasheader.point_data_format = 3;
	lasheader.point_data_record_length = 28+6;

	LASpoint laspoint;	// init point 
	laspoint.init(&lasheader, lasheader.point_data_format, lasheader.point_data_record_length, 0);

	LASwriter* laswriter = laswriteopener.open(&lasheader); // open laswriter
	if (laswriter == 0)
	{
		fprintf(stderr, "ERROR: could not open laswriter\n");
		return ;
	}
	fprintf(stderr, "writing points to '%s'.\n", laswriteopener.get_file_name());	
		
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		double x =( cloud->points[i].x - lasheader.x_offset) / lasheader.x_scale_factor;
		double y =( cloud->points[i].y - lasheader.y_offset) / lasheader.y_scale_factor;
		double z =( cloud->points[i].z - lasheader.z_offset) / lasheader.z_scale_factor;
		
		laspoint.set_X( (I32)(x) );
		laspoint.set_Y( (I32)(y) );
		laspoint.set_Z( (I32)(z) );
		laspoint.set_intensity((U16)(cloud->points[i].intensity));

		laspoint.set_gps_time(0.0006*i);
		laspoint.set_classification(9);
		laspoint.set_point_source_ID(0);

		laswriter->write_point(&laspoint);
		laswriter->update_inventory(&laspoint);	

		if (i < 1)
		{
			cout << "---PCD x y z---" << endl;
			std::cout << fixed << x << "," << y << "," << z << std::endl;
			cout << "---LAS x y z---" << endl;
			std::cout << fixed << laspoint.get_X() << "," << laspoint.get_Y() << "," << laspoint.get_Z() << std::endl;
		}
		
	}
	cout << "point number: " << cloud->points.size() << endl;
	laswriter->update_header(&lasheader, TRUE);	
	delete laswriter;	
}

//double MinX = cloud->points[0].x;
//double MinY = cloud->points[0].y;
//double MinZ = cloud->points[0].z;

/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("cloud viewer"));
viewer->setBackgroundColor(0.1, 0.1, 0.1);

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color(cloud, 0, 255, 0);
viewer->addPointCloud<pcl::PointXYZI>(cloud, single_color, "sample cloud");

viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud"); //Set the rendering properties

viewer->addCoordinateSystem(1.0); //Adds 3D axes describing a coordinate system to screen at 0,0,0
viewer->initCameraParameters();  //Initialize camera parameters with some default values.

viewer->spinOnce(100);*/


