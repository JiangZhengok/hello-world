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
bool SaveLasTxt = true;   //false true
bool SaveGlobalPCD = false;

struct CamPose
{
    double t = 0.0; double x = 0.0; double y = 0.0; double z = 0.0;
    double roll = 0.0; double pitch = 0.0; double yaw = 0.0;
};

struct SlamPose
{
    double t = 0.0; double x = 0.0; double y = 0.0; double z = 0.0;
    Eigen::Matrix4d T = Eigen::Matrix4d::Ones();
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
char* SlamPoseFile_path = "/home/jiang/1_code/AP2.0/output/1571017781.997743_VSLAM_alone.txt";

//#define SAVE_RPY
#ifdef SAVE_RPY
FILE* pRPY = NULL;
char* SaveRPYPath = "/home/jiang/1_code/AP2.0/output";
#endif

Eigen::Matrix3d euler2RotationMatrix(const double roll, const double pitch, double yaw);
Eigen::Matrix3d getMatrixPose(const double roll, const double pitch, double yaw);
Eigen::Quaterniond euler2Quaterniond(const double roll, const double pitch, const double yaw);
bool MatchFunc( const vector<SlamPose> VisonPose, const vector<double> PcdTimes );
void TranslationFunc(pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud, const SlamPose pose);
void ReadVelPose(vector<CamPose> &vCam_Pose, const char* FilePath);
void ReadVslamPose(vector<SlamPose> &Vslam_Pose, const char* FilePath);
vector<string> getFiles(string cate_dir);
void initCaliParam();
bool Cmp(CamPose P1, CamPose P2);
double string2num(string str);

const Eigen::Matrix3d R_il = euler2RotationMatrix (-0.65000, -0.88000, 182.28000); //lidar -> imu
const Eigen::Vector3d t_il(-0.20500, 1.13600, 1.8920);
Eigen::Isometry3d T_il = Eigen::Isometry3d::Identity();

vector<CamPose> vCamPose;
vector<SlamPose> vSlamPose;
vector<string> RawPCDfiles;
vector<string> PCDTimes;
vector<double> dPCDTimes;
//<int> MatchPCDID;
pair< vector<SlamPose>, vector<double> > vMatch;
int walkID = 0;
int findCount = 0, NofindCount = 0;
const double  DEG_TO_ARC = 0.0174532925199433;
const double  ARC_TO_DEG = 57.2957795;

int main (int argc, char** argv)
{
#ifdef SAVE_RPY
    struct timeval curr_time;
	gettimeofday(&curr_time, NULL);
	double CurT = ((double)curr_time.tv_sec + (double)curr_time.tv_usec / 1000000.0L);//0LL
	char CurTime[256];
    sprintf(CurTime, "%s/%lf_RPY.txt", SaveRPYPath, CurT);
	pRPY = fopen(CurTime, "wb+");
#endif

    initCaliParam();
    ReadVslamPose(vSlamPose, SlamPoseFile_path);
    RawPCDfiles = getFiles(RawPCDPath);
    for (int i=0; i<RawPCDfiles.size(); i++)
    {
        string ss = RawPCDfiles[i].substr(0, RawPCDfiles[i].length() - 4);
        PCDTimes.push_back (ss);
    }
    for (int i=0; i<PCDTimes.size(); i++)
    {
        dPCDTimes.push_back(string2num(PCDTimes[i]));
    }
//    for (int i=0; i<RawPCDfiles.size(); i++)
//        cout << RawPCDfiles[i] << endl;
//    for (int i=0; i<PCDTimes.size(); i++)
//        cout << PCDTimes[i] << endl;

    if ( MatchFunc(vSlamPose, dPCDTimes) )
    {
        int n = vMatch.first.size();
        for (int i=0; i<n; i++)
        {
            cout << endl << endl;
            cout << "---正在转换： " << i << "/" << n << endl;
            pcl::PointCloud<pcl::PointXYZI>::Ptr RawCloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr GlobalCloud (new pcl::PointCloud<pcl::PointXYZI>);
            char CurRawPCDName[256];
            sprintf(CurRawPCDName, "%s/%lf%s.pcd", RawPCDPath,  vMatch.second[i], "000");
            cout << "CurRawPCDName = " << CurRawPCDName << endl;
            if (-1 == pcl::io::loadPCDFile<pcl::PointXYZI>(CurRawPCDName, *RawCloud))
            {
                cout << "不能载入pcd文件" << endl;
                return -1;
            }
            //viewer.showCloud(GlobalCloud);
            GlobalCloud->width = RawCloud->width;
            GlobalCloud->height = RawCloud->height;
            GlobalCloud->points.resize(RawCloud->points.size());

            TranslationFunc(RawCloud, GlobalCloud, vMatch.first[i]) ;
            if (SaveGlobalPCD)
            {
                char pcdPathName[256];
                sprintf(pcdPathName, "%s/%lf.pcd", SavePath ,vMatch.first[i].t);
                pcl::io::savePCDFileASCII(pcdPathName, *GlobalCloud);
            }
        }
    }

    cout << endl << endl << "---findCount = " << findCount << endl;
    cout << "---NofindCount = " << NofindCount << endl;
#ifdef SAVE_RPY
    fclose(pRPY);
	pRPY = NULL;
#endif
    return 0;
}

bool MatchFunc( const vector<SlamPose> VisonPose, const vector<double> PcdTimes )
{
    bool findFlag = false;
    for (int i=0; i<VisonPose.size(); i++)
    {
        for (int j=0; j<PcdTimes.size(); j++)
        {
            if  (VisonPose[i].t >= dPCDTimes[j] && VisonPose[i].t < dPCDTimes[j + 1])
            {
                cout << "---匹配成功---" << endl;
                findCount++;
                cout << fixed << "findCount = " << findCount << endl;
                cout << fixed << "dPCDTimes[j].t = " << dPCDTimes[j] << endl;
                cout << fixed << "dPCDTimes[j+1].t = " << dPCDTimes[j + 1] << endl;
                vMatch.first.push_back(VisonPose[i]);
                vMatch.second.push_back(PcdTimes[j]);
                cout << fixed << "vSlamTime = " << VisonPose[i].t << endl;
                findFlag = true;
                break;
            }
        }
    }
    if (findFlag)
        return true;
    else
    {
        cout << "---匹配失败---" << endl;
        NofindCount++;
        cout << "---NofindCount = " << NofindCount << endl;
        return false;
    }
}

void TranslationFunc(pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud, const SlamPose pose)
{
    double Curtx    = pose.x ;
    double Curty    = pose.y ;
    double Curtz    = pose.z ;
    Eigen::Matrix4d CurT4d = pose.T;

    Eigen::Matrix3d R_wi = CurT4d.block(0,0,3,3);
    Eigen::Vector3d t_wi (Curtx, Curty, Curtz);  //Eigen::Vector3d t_wi (CurT4d(0,3), CurT4d(1,3), CurT4d(2,3));

    Eigen::Vector3d euler_angles = R_wi.eulerAngles ( 2,1,0 ); //ZYX, yaw,pitch,roll
    double vyaw = euler_angles(0)   ;//* ARC_TO_DEG;
    double vpitch = euler_angles(1) ;//* ARC_TO_DEG;
    double vroll = euler_angles(2)  ;//* ARC_TO_DEG;
    vyaw = vyaw - 3.5 * DEG_TO_ARC;
    Eigen::AngleAxisd rollAngle(vroll, Eigen::Vector3d::UnitX());   //eulerAngle(2)
    Eigen::AngleAxisd pitchAngle(vpitch, Eigen::Vector3d::UnitY()); //eulerAngle(1)
    Eigen::AngleAxisd yawAngle(vyaw, Eigen::Vector3d::UnitZ());      //eulerAngle(0)

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = yawAngle*pitchAngle*rollAngle;
    R_wi = rotation_matrix;

    if (SaveLasTxt)
    {
        POSInfo lasPos;
        lasPos.x_proj = pose.x;
        lasPos.y_proj = pose.y;
        lasPos.z = pose.z;

        double d_roll  = vroll*ARC_TO_DEG;
        double d_pitch = vpitch*ARC_TO_DEG;
        double d_yaw   = vyaw*ARC_TO_DEG;
        if (d_yaw<0) d_yaw += 360;

        lasPos.roll = d_roll;
        lasPos.pitch = d_pitch;
        lasPos.yaw = d_yaw;

        FILE *fpinDst = NULL;
        char POSFilePath[256];
        sprintf(POSFilePath, "%s/%lf.txt", SavePath, pose.t);
        cout << "POSFilePath = " << POSFilePath << endl;

        if ((fpinDst = fopen(POSFilePath, "wb")) != NULL)
        {
            fwrite(&lasPos, 1, sizeof(POSInfo), fpinDst);
            fclose(fpinDst);
        }
    }
#ifdef SAVE_RPY
    fprintf(pRPY, "%lf %lf %lf\n", roll, pitch, yaw);
#endif

    Eigen::Isometry3d T_wi = Eigen::Isometry3d::Identity();
    T_wi.rotate(R_wi);
    T_wi.pretranslate(t_wi);
    //cout << "---变换矩阵(T_wi)： " << endl << T_wi.matrix() << endl;

    Eigen::Isometry3d T_wl = T_wi * T_il;
    //cout << "---变换矩阵(T_il)： " << endl << T_il.matrix() << endl;
    //cout << "---变换矩阵(T_wl)： " << endl << T_wl.matrix() << endl;

//    Eigen::AngleAxisd rotation_vector ( 3*DEG_TO_ARC, Eigen::Vector3d ( 0,0,1 ) );
//    Eigen::Matrix3d temR = rotation_vector.matrix();

//    for (int i=0; i<inCloud->points.size(); i++)
//    {
//        Eigen::Vector3d tmpPoint(inCloud->points[i].x, inCloud->points[i].y, inCloud->points[i].z);
//        tmpPoint = T_wl * tmpPoint;
//        outCloud->points[i].x = tmpPoint.x();
//        outCloud->points[i].y = tmpPoint.y();
//        outCloud->points[i].z = tmpPoint.z();
//        outCloud->points[i].intensity = inCloud->points[i].intensity;
//
//        if (i == (inCloud->points.size()) / 2)
//        {
//            cout << "---点云相对坐标(x, y, z, intensity): " << endl;
//            cout << fixed << inCloud->points[i].x << " " << inCloud->points[i].y << " " << inCloud->points[i].z
//                 << " " << inCloud->points[i].intensity << endl;
//            cout << "---点云绝对坐标(x, y, z, intensity): " << endl;
//            cout << fixed << outCloud->points[i].x << " " << outCloud->points[i].y << " "
//                 << outCloud->points[i].z << " " << outCloud->points[i].intensity << endl;
//        }
//    }

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

void ReadVelPose(vector<CamPose> &vCam_Pose, const char* FilePath)
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

void ReadVslamPose(vector<SlamPose> &Vslam_Pose, const char* FilePath)
{
    ifstream VslamPoseFile(FilePath);
    if(VslamPoseFile.is_open())
    {
        cout << "---打开Vslam_Pose文件路径： " << endl << FilePath << endl;
        while (!VslamPoseFile.eof())
        {
            SlamPose CurSlamPose;
            double d11, d12, d13, d14, d21, d22, d23, d24, d31, d32, d33, d34;
            VslamPoseFile >> CurSlamPose.t >> CurSlamPose.x >> CurSlamPose.y >> CurSlamPose.z >>
            d11 >> d12 >> d13 >> d14 >> d21 >> d22 >> d23 >> d24 >> d31 >> d32 >> d33 >> d34;
            CurSlamPose.T << d11, d12, d13, d14,
                             d21, d22, d23, d24,
                             d31, d32, d33, d34,
                             0,   0,   0,   1;
            Vslam_Pose.push_back(CurSlamPose);
        }
        //        for (int i=0; i<20; i++)
        //            cout << fixed << "vCam_Pose[i].t = " << vCam_Pose[i].t << endl;
        //std::sort(vCam_Pose.begin(), vCam_Pose.end(), Cmp);
        //        for (int i=0; i<20; i++)
        //            cout << endl << fixed << "vCam_Pose[i].t = " << vCam_Pose[i].t << endl;
    }
    else
    cout << "---Vslam_Pose 打开失败" << endl;

    VslamPoseFile.close();
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

Eigen::Matrix3d euler2RotationMatrix(const double roll, const double pitch, double yaw)
{
    double dRoll = roll * DEG_TO_ARC;
    double dPitch = pitch * DEG_TO_ARC;
    //yaw += 6.0;
    if (yaw > 180)   yaw -= 360;
    if (yaw > 180)   yaw -= 360;
    double dYaw = yaw * DEG_TO_ARC;

    Eigen::AngleAxisd rollAngle(dRoll, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(dPitch, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yawAngle(dYaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3d R = q.toRotationMatrix();

//    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
//
//    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
//    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
//
//    Eigen::Matrix3d rotation_matrix;
//    rotation_matrix = yawAngle*pitchAngle*rollAngle;

//    Eigen::Matrix3d rotation_matrix3;
//    rotation_matrix3 = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()) *
//                       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
//                       Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitX());
    //Eigen::Matrix3d R = q.matrix();
//    cout << "---Euler2RotationMatrix result is:" <<endl;
//    cout << "R = " << endl << R << endl<<endl;
    return R;
    //return rotation_matrix;
}

Eigen::Matrix3d getMatrixPose(const double roll, const double pitch, double yaw)
{
    //double PI_enlarge = M_PI / 180.0;
    const double  DEG_TO_ARC = 0.0174532925199433;
    double dRoll = roll * DEG_TO_ARC;
    double dPitch = pitch * DEG_TO_ARC;
    //yaw += 20.0;
    double dYaw = yaw * DEG_TO_ARC;

    Eigen::Matrix3d PosMatrix = Eigen::Matrix3d::Identity();
    PosMatrix(0, 0) = cos(dPitch)*cos(dYaw);
    PosMatrix(0, 1) = sin(dRoll)*sin(dPitch)*cos(dYaw) - cos(dRoll)*sin(dYaw);
    PosMatrix(0, 2) = cos(dRoll)*sin(dPitch)*cos(dYaw) + sin(dRoll)*sin(dYaw);

    PosMatrix(1, 0) = cos(dPitch)*sin(dYaw);
    PosMatrix(1, 1) = sin(dRoll)*sin(dPitch)*sin(dYaw) + cos(dRoll)*cos(dYaw);
    PosMatrix(1, 2) = cos(dRoll)*sin(dPitch)*sin(dYaw) - sin(dRoll)*cos(dYaw);

    PosMatrix(2, 0) = -sin(dPitch);
    PosMatrix(2, 1) = sin(dRoll)*cos(dPitch);
    PosMatrix(2, 2) = cos(dRoll)*cos(dPitch);

    return PosMatrix;
}


Eigen::Quaterniond euler2Quaterniond(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
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





