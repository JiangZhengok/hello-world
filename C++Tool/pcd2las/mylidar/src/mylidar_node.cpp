#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
//#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>

#include <liblas/liblas.hpp>
#include <fstream>
#include <iostream>
using namespace std;

pcl::visualization::CloudViewer viewer("Cloud Viewer");
char* SavePath = "/home/jiang/catkin_ws/src/mylidar/pcd2las";

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);
void writeLasFromPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr pcdCloud, const char* strOutLasName);

int main (int argc, char** argv)
{ 
  ros::init (argc, argv, "mylidar");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe ("pandar_points", 1, cloud_cb); //pandar_points  rslidar_points
  ros::Rate loop_rate(100);

  ros::spin (); 
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  ros::Time CurRosTime = input->header.stamp;
  double CurTime = (double)CurRosTime.sec + (double)CurRosTime.nsec/1000000.0L;

  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*input,*cloud);    
  viewer.showCloud(cloud);  
  //pcl::PCDWriter writer;
  //writer.write<pcl::PointXYZ> ("ros_to_PCL.pcd", *cloud, false);

  char pcdPathName[256];
  sprintf(pcdPathName, "%s/%lf.pcd", SavePath ,CurTime);
  pcl::io::savePCDFileASCII(pcdPathName, *cloud);

  char lasPathName[256];
  sprintf(lasPathName, "%s/%lf.las", SavePath ,CurTime);
  writeLasFromPointCloud(cloud, lasPathName);
}

void writeLasFromPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr pcdCloud, const char* strOutLasName)
{
//    std::ifstream ifs;
//    ifs.open("/home/jiang/catkin_ws/src/mylidar/TuGuan.las", std::ios::in | std::ios::binary);
//    if (!ifs.is_open())
//    {
//        std::cout << "无法打开.las" << std::endl;
//        return;
//    }
//    liblas::ReaderFactory readerFactory;
//    liblas::Reader reader = readerFactory.CreateWithStream(ifs);

    //std::cout << "点云总数:" << pcdCloud->points.size() << std::endl;
    std::ios::openmode m = std::ios::out | std::ios::in | std::ios::binary | std::ios::ate;
    std::ofstream ofs;
    if (!liblas::Create(ofs, strOutLasName))
    {
        std::cout << "无法创建.las" << std::endl;
        return;
    }
    ofs.close();
    std::ofstream * ofs2 = new std::ofstream(strOutLasName, m);
    if (!ofs2->is_open())
    {
        std::cout << "打不开.las" << std::endl;
        return;
    }

    liblas::Header header;          //liblas::Header header = reader.GetHeader();
    header.SetVersionMajor(1);
    header.SetVersionMinor(2);
    //header.SetDataFormatId(liblas::PointFormatName::ePointFormat3);
    header.SetScale(0.003, 0.003, 0.003);

    header.SetDataFormatId(liblas::ePointFormat1); // Time only
// Set coordinate system using GDAL support
    liblas::SpatialReference srs;
    srs.SetFromUserInput("EPSG:4326");

    header.SetSRS(srs);

    liblas::Writer writer(*ofs2, header); //写liblas
    liblas::Point point(&header);

    double minPt[3] = { 9999999, 9999999, 9999999 };
    double maxPt[3] = { 0, 0, 0 };
    double pt[3] = {0};

    for (size_t i = 0; i < pcdCloud->points.size(); i++)
    {
        double x = pcdCloud->points[i].x;
        double y = pcdCloud->points[i].y;
        double z = pcdCloud->points[i].z;
        point.SetX(x);
        point.SetY(y);
        point.SetZ(z);      //point.SetCoordinates(x, y, z);

        uint16_t intensity = pcdCloud->points[i].intensity;
        point.SetIntensity(intensity);

//        uint32_t red = 100;
//        uint32_t green = 100;
//        uint32_t blue = 100;
//        liblas::Color pointColor(red, green, blue);
//        point.SetColor(pointColor);
//
//        double time = 1234.5678;
//        point.SetTime(time);
//
//        point.SetClassification(1);
//
//        point.SetPointSourceID(0);


//        uint32_t red = (uint32_t)pcdCloud->points[i].r;
//        uint32_t green = (uint32_t)pcdCloud->points[i].g;
//        uint32_t blue = (uint32_t)pcdCloud->points[i].b;
//        liblas::Color pointColor(red, green, blue);
//        point.SetColor(pointColor);

        writer.WritePoint(point);
        //std::cout << x << "," << y << "," << z << std::endl;
    }

    header.SetPointRecordsCount(pcdCloud->points.size());
    header.SetPointRecordsByReturnCount(0, pcdCloud->points.size());
    header.SetMax(maxPt[0], maxPt[1], maxPt[2]);
    header.SetMin(minPt[0], minPt[1], minPt[2]);
    writer.SetHeader(header);
    writer.WriteHeader();


    ofs2->close();
}

/*
void writeLasFromPointCloud(const char* strInputPointCloudName, const char* strOutLasName)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader pcdreader;
    pcdreader.read(strInputPointCloudName, *cloud);

    std::cout << "总数:" << cloud->points.size() << std::endl;

    std::ios::openmode m = std::ios::out | std::ios::in | std::ios::binary | std::ios::ate;

    std::ofstream ofs;
    if (!liblas::Create(ofs, strOutLasName))
    {
      std::cout << "无法创建.las" << std::endl;
      return;
    }
    ofs.close();

    std::ofstream * ofs2 = new std::ofstream(strOutLasName, m);
    if (!ofs2->is_open())
    {
      std::cout << "打不开.las" << std::endl;
      return;
    }
    else
    std::cout << "能打开.las" << std::endl;

    liblas::Header header;
    liblas::Writer writer(*ofs2, header);
    liblas::Point point(&header);

    for (size_t i = 0; i < cloud->points.size(); i++)
    {
    double x = cloud->points[i].x;
    double y = cloud->points[i].y;
    double z = cloud->points[i].z;
    point.SetX(x);
    point.SetY(y);
    point.SetZ(z);

    writer.WritePoint(point);
    std::cout << x << "," << y << "," << z << std::endl;
    }
}*/