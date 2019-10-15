#include <iostream>
#include <stdio.h>
#include <fstream>
#include <unistd.h>
#include <dirent.h>
#include <cassert>
#include <string>
#include <sstream>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Eigen/Core>
//#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
bool SaveLasTxt = true;
bool SaveGlobalPCD = true;

struct CamPose
{
    double t = 0.0; double x = 0.0; double y = 0.0; double z = 0.0;
    double roll = 0.0; double pitch = 0.0; double yaw = 0.0;
};

typedef struct POSInfo
{
    int SensorID = 0;
    int SensorBrand = 0;

    int INStatus = 0;
    int POSStatus = 0;

    double Speed_x = 0.0;
    double Speed_y = 0.0;
    double Speed_z = 0.0;
    double Speed = 0.0;

    double x = 0.0;  // dLon;  easting;
    double y = 0.0;  // dLat;  northing;

    double x_proj = 0.0;  //UTM
    double y_proj = 0.0;
    double z = 0.0;  // z m

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    double Time = 0.0;
} POSInfo;

//pcl::visualization::CloudViewer viewer("Cloud Viewer");
char* SavePath = "/media/jiang/X032/0926_kbd_test_bag/00OutPCD";
char* RawPCDPath = "/media/jiang/X032/0926_kbd_test_bag/00RawPCD";
char* CamPoseFile_path = "/home/jiang/1_code/AP2.0/output/1570604619.400730_OutPose.txt";

Eigen::Matrix3d euler2RotationMatrix(const double roll, const double pitch, const double yaw);
Eigen::Quaterniond euler2Quaterniond(const double roll, const double pitch, const double yaw);
bool translationFunc(pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud, const double CloudTime);
void ReadCamPose(vector<CamPose> &vCam_Pose, const char* FilePath);
vector<string> getFiles(string cate_dir);
void initCaliParam();
bool Cmp(CamPose P1, CamPose P2);
double string2num(string str);

const Eigen::Matrix3d R_il = euler2RotationMatrix (-0.65000, -0.88000, 182.28000); //lidar -> imu
const Eigen::Vector3d t_il(-0.20500, 1.13600, 1.8920);
Eigen::Isometry3d T_il = Eigen::Isometry3d::Identity();

//const Eigen::Matrix3d R_ic = euler2RotationMatrix (0,       90.85,    179.3);      //rear -> imu
//const Eigen::Vector3d t_ic(-0.2200,  -0.020,  1.441);
//Eigen::Isometry3d T_ic = Eigen::Isometry3d::Identity();

vector<CamPose> vCamPose;
vector<string> RawPCDfiles;
vector<string> PCDTimes;
int walkID = 0;
int findCount = 0, NofindCount = 0;

int main (int argc, char** argv)
{
    initCaliParam();
    ReadCamPose(vCamPose, CamPoseFile_path);
    RawPCDfiles = getFiles(RawPCDPath);
    for (int i=0; i<RawPCDfiles.size(); i++)
    {
        string ss = RawPCDfiles[i].substr(0, RawPCDfiles[i].length() - 4);
        PCDTimes.push_back (ss);
    }
//    for (int i=0; i<RawPCDfiles.size(); i++)
//        cout << RawPCDfiles[i] << endl;
//    for (int i=0; i<PCDTimes.size(); i++)
//        cout << PCDTimes[i] << endl;
    for (int i=0; i<RawPCDfiles.size(); i++)
    {
        cout << endl << endl;
        cout << "--" << i << "--PCD Name: " << RawPCDfiles[i] << endl;
        cout << "--" << i << "--PCD Time: " << PCDTimes[i] << endl;
        pcl::PointCloud<pcl::PointXYZI>::Ptr RawCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr GlobalCloud (new pcl::PointCloud<pcl::PointXYZI>);

        char CurRawPCDName[256];
        sprintf(CurRawPCDName, "%s/%s", RawPCDPath, RawPCDfiles[i].c_str());
        cout << "CurRawPCDName = " << CurRawPCDName << endl;
        if (-1 == pcl::io::loadPCDFile<pcl::PointXYZI>(CurRawPCDName, *RawCloud))
        {
            cout << "不能载入pcd文件" << endl;
            return -1;
        }
        //viewer.showCloud(RawCloud);
        GlobalCloud->width = RawCloud->width;
        GlobalCloud->height = RawCloud->height;
        GlobalCloud->points.resize(RawCloud->points.size());

        if( translationFunc(RawCloud, GlobalCloud, string2num(PCDTimes[i])) )
        {
            //viewer.showCloud(GlobalCloud);
            if (SaveGlobalPCD)
            {
                char pcdPathName[256];
                sprintf(pcdPathName, "%s/%lf.pcd", SavePath ,string2num(PCDTimes[i]));
                pcl::io::savePCDFileASCII(pcdPathName, *GlobalCloud);
            }
        }
    }

    cout << endl << endl << "---findCount = " << findCount << endl;
    cout << "---NofindCount = " << NofindCount << endl;
    return 0;
}

bool MatchFunc(const double CloudTime, int LeftID, int RightID)
{
    
}
bool translationFunc(pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud, const double CloudTime)
{
    int LeftID = 0, RightID = 0;
    bool findFlag = false;
    if (walkID < vCamPose.size() )
    {
        for (int i=walkID; i<vCamPose.size()-1; i++)
        {
            cout << fixed << "walkID = " << walkID << endl;
            if (CloudTime >= vCamPose[i].t && CloudTime < vCamPose[i+1].t)
            {
                cout << "---匹配成功---" << endl;
                findCount++;
                cout << fixed << "findCount = " << findCount << endl;
                cout << fixed << "vCamPose[i].t = " << vCamPose[i].t << endl;
                cout << fixed << "vCamPose[i+1].t = " << vCamPose[i+1].t << endl;
                LeftID = i;
                RightID = i+1;
                walkID = i;
                cout << fixed << "CloudTime = " << CloudTime << endl;
                findFlag = true;
                if (SaveLasTxt)
                {
                    POSInfo lasPos;
                    lasPos.x_proj = vCamPose[i].x;
                    lasPos.y_proj = vCamPose[i].y;
                    lasPos.z      = vCamPose[i].z;
                    lasPos.roll    = vCamPose[i].roll;
                    lasPos.pitch  = vCamPose[i].pitch;
                    lasPos.yaw    = vCamPose[i].yaw;

                    FILE *fpinDst = NULL;
                    char POSFilePath[256];
                    sprintf(POSFilePath, "%s/%lf.txt", SavePath, CloudTime);
                    cout << "POSFilePath = " << POSFilePath << endl;

                    if ((fpinDst = fopen(POSFilePath, "wb")) != NULL)
                    {
                        fwrite(&lasPos, 1, sizeof(POSInfo), fpinDst);
                        fclose(fpinDst);
                    }
                }
                break;
            }
        }
        if (findFlag)
        {
            cout << "---匹配到的车体(imu)位姿： " << "(" << LeftID << ")" << endl;
            cout << fixed << vCamPose[LeftID].t << " " << vCamPose[LeftID].x << " " << vCamPose[LeftID].y << " " << vCamPose[LeftID].z << " "
                 << vCamPose[LeftID].roll << " " << vCamPose[LeftID].pitch << " " << vCamPose[LeftID].yaw << endl;

            if (vCamPose[LeftID].yaw > 180)   vCamPose[LeftID].yaw -= 360;
            if (vCamPose[RightID].yaw > 180)   vCamPose[RightID].yaw -= 360;

            cout << "vCamPose[LeftID].yaw = " << vCamPose[LeftID].yaw << endl;
            cout << "vCamPose[RightID].yaw = " << vCamPose[RightID].yaw << endl;

            double CurRoll = (vCamPose[LeftID].roll + vCamPose[RightID].roll)/2;
            double CurPitch = (vCamPose[LeftID].pitch + vCamPose[RightID].pitch)/2;
            double CurYaw = (vCamPose[LeftID].yaw + vCamPose[RightID].yaw)/2;
            double Curtx = (vCamPose[LeftID].x + vCamPose[RightID].x)/2;
            double Curty = (vCamPose[LeftID].y + vCamPose[RightID].y)/2;
            double Curtz = (vCamPose[LeftID].z + vCamPose[RightID].z)/2;

            cout << "CurRoll = " << CurRoll << ", " << "CurPitch = " << CurPitch << ", " << "CurYaw = " << CurYaw << endl;

            Eigen::Matrix3d R_wi = euler2RotationMatrix (CurRoll, CurPitch, CurYaw);
            Eigen::Vector3d t_wi (Curtx, Curty, Curtz);

            Eigen::Isometry3d T_wi = Eigen::Isometry3d::Identity();
            T_wi.rotate(R_wi);
            T_wi.pretranslate(t_wi);

            Eigen::Isometry3d T_wl = T_wi * T_il;

            for (int i=0; i<inCloud->points.size(); i++)
            {
                Eigen::Vector3d tmpPoint( inCloud->points[i].x, inCloud->points[i].y, inCloud->points[i].z );
                tmpPoint = T_wl * tmpPoint;
                outCloud->points[i].x = tmpPoint.x();
                outCloud->points[i].y = tmpPoint.y();
                outCloud->points[i].z = tmpPoint.z();
                outCloud->points[i].intensity = inCloud->points[i].intensity;

                if(i<1)
                {
                    cout << "---点云相对坐标(x, y, z, intensity): " << endl;
                    cout << fixed << inCloud->points[i].x << " " << inCloud->points[i].y << " " << inCloud->points[i].z << " " << inCloud->points[i].intensity << endl;
                    cout << "---点云绝对坐标(x, y, z, intensity): " << endl;
                    cout << fixed << outCloud->points[i].x << " " << outCloud->points[i].y << " " << outCloud->points[i].z << " " << outCloud->points[i].intensity << endl;
                }
            }
            return true;
        }
        else
        {
            cout << "---匹配失败---" << endl;
            NofindCount++;
            cout << "---NofindCount = " << NofindCount << endl;
            return false;
        }
    }
    return  false;
}

void initCaliParam()
{
    T_il.rotate(R_il);
    T_il.pretranslate(t_il);
    cout << "---Lidar到imu的外参： " << endl;
    cout << "T_il = " << endl << T_il.matrix() << endl;

//    T_ic.rotate(R_ic);
//    T_ic.pretranslate(t_ic);
//    cout << "---imu到Camera的外参： " << endl;
//    cout << "T_ic = " << endl << T_ic.matrix() << endl;

//    T_lc = T_il.inverse() * T_ic;
//    cout << "---Camera到Lidar的外参： " << endl;
//    cout << "T_lc = " << endl << T_lc.matrix() << endl;
}

bool Cmp(CamPose P1, CamPose P2)
{
    return P1.t < P2.t;
}

void ReadCamPose(vector<CamPose> &vCam_Pose, const char* FilePath)
{
    ifstream CamPoseFile(FilePath);
    if(CamPoseFile.is_open())
    {
        cout << "---打开CamPoseFile文件路径： " << endl << FilePath << endl;
        while (!CamPoseFile.eof())
        {
            CamPose CurPose;
            CamPoseFile >> CurPose.t >> CurPose.x >> CurPose.y >> CurPose.z >> CurPose.roll >> CurPose.pitch
                        >> CurPose.yaw;
            vCam_Pose.push_back(CurPose);
        }
//        for (int i=0; i<20; i++)
//            cout << fixed << "vCam_Pose[i].t = " << vCam_Pose[i].t << endl;
        //std::sort(vCam_Pose.begin(), vCam_Pose.end(), Cmp);
//        for (int i=0; i<20; i++)
//            cout << endl << fixed << "vCam_Pose[i].t = " << vCam_Pose[i].t << endl;
    }
    else
        cout << "---CamPoseFile 打开失败" << endl;

    CamPoseFile.close();
}

vector<string> getFiles(string cate_dir)
{
    vector<string> files;//存放文件名
    DIR *dir;
	struct dirent *ptr;
	char base[1000];

	if ((dir=opendir(cate_dir.c_str())) == NULL)
    {
        perror("Open dir error...");
        exit(1);
    }

	while ((ptr=readdir(dir)) != NULL)
	{
		if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
		        continue;
		else if(ptr->d_type == 8)    ///file
			//printf("d_name:%s/%s\n",basePath,ptr->d_name);
			files.push_back(ptr->d_name);
		else if(ptr->d_type == 10)    ///link file
			//printf("d_name:%s/%s\n",basePath,ptr->d_name);
			continue;
		else if(ptr->d_type == 4)    ///dir
		{
			files.push_back(ptr->d_name);
			/*
		        memset(base,'\0',sizeof(base));
		        strcpy(base,basePath);
		        strcat(base,"/");
		        strcat(base,ptr->d_nSame);
		        readFileList(base);
			*/
		}
	}
	closedir(dir);
    sort(files.begin(), files.end());
    return files;
}

Eigen::Matrix3d euler2RotationMatrix(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
    Eigen::Matrix3d R = q.toRotationMatrix();
    //Eigen::Matrix3d R = q.matrix();
//    cout << "---Euler2RotationMatrix result is:" <<endl;
//    cout << "R = " << endl << R << endl<<endl;
    return R;
}

Eigen::Quaterniond euler2Quaterniond(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
    cout << "---euler2Quaterniond result is:" <<endl;
    cout << "q = " << endl << q.coeffs() << endl<<endl;
    return q;
}

double string2num(string str)
{
    double num;
    stringstream ss;
    ss << str;
    ss >> num;
    return num;
}



