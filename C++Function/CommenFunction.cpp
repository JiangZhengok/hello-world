/**** 1 **** 写入数据到文件，文件名由当前时间命名 **/
/**** 2 **** 读取文件内容 **/
/**** 3 **** 获取制定目录下的所有文件名 **/
/**** 4 **** 文件名去除后缀，获取时间戳 **/
/**** 5 **** 旋转矩阵到欧拉角 欧拉角到旋转矩阵 欧拉角到四元数 **/
/**** 6 **** string转double **/
/**** 7 **** 二分查找有序数组 **/
/**** 8 **** 四舍五入到小数点后2位 **/
/**** 9 **** GPS周和周内秒时间转UTC时间, 单位s **/
/**** 10 *** 四元数球面线性插值 **/
/**** 11 *** opencv特征均匀提取与匹配 **/
/**** 12 *** 读取文件内容, 逗号间隔的数据 **/
/**** 13 *** opencv图像去畸变，lable点去畸变 **/
/**** 14 *** opencv 2d_2d求位姿，RANSAC去除误匹配 **/


/*********************************** 1 *************************************
 * 写入数据到文件，文件名由当前时间命名
 **/
#include <iostream>
#include <sys/time.h>

#define SAVE_DATA

#ifdef SAVE_DATA
    FILE* pData = nullptr;
    char* SavePath = "/home/jiang/1_code/AP2.0/output";
#endif

#ifdef SAVE_DATA
    struct timeval curr_time;
    gettimeofday(&curr_time, nullptr);
    double CurT = ((double)curr_time.tv_sec + (double)curr_time.tv_usec / 1000000.0L);
    char FileName[256];
    sprintf(FileName, "%s/%lf_RPY.txt", SavePath, CurT);
    pData = fopen(FileName, "wb+");
#endif

#ifdef SAVE_DATA
    fclose(pData);
	pData = nullptr;
#endif

#ifdef SAVE_RPY
    fprintf(pData, "%lf %lf %lf\n", roll, pitch, yaw);
#endif

/*********************************** 2 *************************************
 * 读取文件内容, 空格间隔的数据
 **/
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

			if(CamPoseFile.fail())
				break;

            vCam_Pose.push_back(CurPose);
        }
    }
    else
        cout << "---CamPoseFile 打开失败" << endl;

    CamPoseFile.close();
}

/*********************************** 3 *************************************
 * 获取制定目录下的所有文件名
 * 输入：目录名，eg: char* RawPCDPath = "/media/jiang/X032/0926_kbd_test_bag/00RawPCD"
 * 输出：文件名，
 **/
vector<string> getFiles(string cate_dir)
{
    vector<string> files;
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
		if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)
		        continue;
		else if(ptr->d_type == 8)
			files.push_back(ptr->d_name);
		else if(ptr->d_type == 10)
			continue;
		else if(ptr->d_type == 4)
		{
			files.push_back(ptr->d_name);
		}
	}
	closedir(dir);
    sort(files.begin(), files.end());
    return files;
}

/*********************************** 4 *************************************
 * 文件名去除后缀，获取时间戳
 **/
for (int i=0; i<RawPCDfiles.size(); i++)
    {
        string ss = RawPCDfiles[i].substr(0, RawPCDfiles[i].length() - 4);
        PCDTimes.push_back (ss);
    }

/*********************************** 5 *************************************
 * 旋转矩阵到欧拉角（弧度，±π）
 * 欧拉角到旋转矩阵  （一步步推导由欧拉角到旋转矩阵的计算过程 https://blog.csdn.net/D_XingGuang/article/details/97148669）
 * 欧拉角到四元数
 **/
const double  DEG_TO_ARC = 0.0174532925199433; //角度 > 弧度
const double  ARC_TO_DEG = 57.2957795;         //弧度 > 角度

Eigen::Vector3d euler_angles = R_wi.eulerAngles ( 2,1,0 ); //ZYX, yaw,pitch,roll
double yaw = euler_angles(0)   ; //* ARC_TO_DEG;
double pitch = euler_angles(1) ; //* ARC_TO_DEG;
double roll = euler_angles(2)  ; //* ARC_TO_DEG;

Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());   //eulerAngle(2)
Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY()); //eulerAngle(1)
Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());     //eulerAngle(0)
//Eigen::Matrix3d rotation_matrix = yawAngle * pitchAngle * rollAngle;
Eigen::Quaterniond            q = yawAngle * pitchAngle * rollAngle;

Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d theta) //theta rpy顺序
{
    Eigen::Matrix3d R_x=Eigen::AngleAxisd(theta(0),Eigen::Vector3d(1,0,0)).toRotationMatrix();
    Eigen::Matrix3d R_y=Eigen::AngleAxisd(theta(1),Eigen::Vector3d(0,1,0)).toRotationMatrix();
    Eigen::Matrix3d R_z=Eigen::AngleAxisd(theta(2),Eigen::Vector3d(0,0,1)).toRotationMatrix();
    return R_z*R_y*R_x;
}

//********通过角度计算*******
Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d &theta) //rpy
{
    Eigen::Matrix3d R_x;    // 计算旋转矩阵的X分量
    R_x <<
        1,              0,               0,
        0,  cos(theta[0]),  -sin(theta[0]),
        0,  sin(theta[0]),   cos(theta[0]);

    Eigen::Matrix3d R_y;    // 计算旋转矩阵的Y分量
    R_y <<
        cos(theta[1]),   0, sin(theta[1]),
                    0,   1,             0,
        -sin(theta[1]),  0, cos(theta[1]);

    Eigen::Matrix3d R_z;    // 计算旋转矩阵的Z分量
    R_z <<
        cos(theta[2]), -sin(theta[2]), 0,
        sin(theta[2]),  cos(theta[2]), 0,
                    0,              0, 1;
    Eigen::Matrix3d R = R_z * R_y * R_x;
    return R;
}

/** 功能： 通过给定的旋转矩阵计算对应的欧拉角**/
bool isRotationMatirx(Eigen::Matrix3d R)
{
    int err=1e-6;//判断R是否奇异
    Eigen::Matrix3d shouldIdenity;
    shouldIdenity=R*R.transpose();
    Eigen::Matrix3d I=Eigen::Matrix3d::Identity();
    return (shouldIdenity-I).norm()<err?true:false;
}

Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R)
{
    assert(isRotationMatrix(R));
    float sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
    bool singular = sy < 1e-6;
    float x, y, z;
    if (!singular)
    {
        x = atan2( R(2,1), R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2( R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    return Eigen::Vector3d(x, y, z);
}

/*********************************** 6 *************************************
 * string转double
 **/
double string2num(string str)
{
    double num;
    stringstream ss;
    ss << str;
    ss >> num;
    return num;
}

/*********************************** 7 *************************************
/**
 * 二分查找有序数组
 **/
int binarySearch(int Array[],int key,int n) {

    int left=0;         //记录范围左边界
    int right = n - 1;  //记录范围右边界
    int mid;            //范围是否不为空
    while (left<=right)
    {
        mid = (right + left) / 2;
        if (Array[mid]==key)
        {
            return mid;         //查找成功返回
        }
        else if (Array[mid]>key)
        {
            right = mid - 1;    //继续在右半边中查找
        }
        else
        {
            left = mid + 1;     //继续在左半边中查找
        }
    }
    return -1;                  //当left>right时表示查找区间为空，查找失败
}


/*********************************** 8 *************************************
/**
 * 四舍五入到小数点后2位
 **/
float func(float a)
{
    return (int)(a*100+0.5)/100.0;
    //return floor(a*100+0.5)/100.0;
}

/*********************************** 9 *************************************
/**
 * GPS周和周内秒时间转UTC时间, 单位s
 **/
#define GPS2UTC_leapSecond = 18;         //GPS-UTC=18s (Year:2017)
#define GPS2UNIX_Second = 315964800;    //GPS 1980 UNIX 1970 
double  GPS2UNIX(int GpsWeek, double GpsSecond)
{
	return GpsWeek*86400.0*7.0 + GpsSecond + GPS2UNIX_Second - GPS2UTC_leapSecond;
};

/*********************************** 10 *************************************
/**
 * 四元数球面线性插值
 **/
https://www.cnblogs.com/21207-iHome/p/6952004.html

/*********************************** 11 *************************************
/** 
 * opencv特征均匀提取与匹配
 **/

void find_surf_matches(const cv::Mat& img1, const cv::Mat& img2,
                                       std::vector<cv::KeyPoint>& keyPoint_obj,
                                       std::vector<cv::KeyPoint>& keyPoint_scene,
                                       std::vector< cv::DMatch >& goodMatch)
{
    //SURF特征提取
    int minHessian = 200;
    Ptr<SURF> detector = SURF::create(minHessian);
    
    Mat descriptor_obj, descriptor_scene;
    detector->detectAndCompute(img1, Mat(), keyPoint_obj, descriptor_obj, false);
    detector->detectAndCompute(img2, Mat(), keyPoint_scene, descriptor_scene, false);
    
    cv::Ptr<cv::DescriptorMatcher> surfDescriptorMatcher = cv::Ptr<cv::DescriptorMatcher>(new cv::BFMatcher(cv::NORM_L2, false));
    std::vector<std::vector<cv::DMatch>> candidateFwdMatches;
    surfDescriptorMatcher->knnMatch(descriptor_obj, descriptor_scene, candidateFwdMatches, 5);
    
    std::vector<std::vector<cv::DMatch>> candidateRevMatches;
    surfDescriptorMatcher->knnMatch(descriptor_scene, descriptor_obj, candidateRevMatches, 5);
    cout << "candidateFwdMatches size: " << candidateFwdMatches.size() << ", candidateRevMatches size: " << candidateRevMatches.size() << endl;

    float k_maxDistanceRatio = 0.7;
    std::vector<std::vector<cv::DMatch>> fwdMatches(candidateFwdMatches.size());
    for (size_t i = 0; i < candidateFwdMatches.size(); ++i)
    {
        std::vector<cv::DMatch> &match = candidateFwdMatches.at(i);

        if (match.size() < 2)
        {
            continue;
        }

        float distanceRatio = match.at(0).distance / match.at(1).distance;

        if (distanceRatio < k_maxDistanceRatio)
        {
            fwdMatches.at(i).push_back(match.at(0));
        }
    }

    std::vector<std::vector<cv::DMatch>> revMatches(candidateRevMatches.size());
    for (size_t i = 0; i < candidateRevMatches.size(); ++i)
    {
        std::vector<cv::DMatch> &match = candidateRevMatches.at(i);

        if (match.size() < 2)
        {
            continue;
        }

        float distanceRatio = match.at(0).distance / match.at(1).distance;

        if (distanceRatio < k_maxDistanceRatio)
        {
            revMatches.at(i).push_back(match.at(0));
        }
    }

    // cross-check
    for (size_t i = 0; i < fwdMatches.size(); ++i)
    {
        if (fwdMatches.at(i).empty())
        {
            continue;
        }

        cv::DMatch &fwdMatch = fwdMatches.at(i).at(0);

        if (revMatches.at(fwdMatch.trainIdx).empty())
        {
            continue;
        }

        cv::DMatch &revMatch = revMatches.at(fwdMatch.trainIdx).at(0);

        if (fwdMatch.queryIdx == revMatch.trainIdx &&
            fwdMatch.trainIdx == revMatch.queryIdx)
        {
            cv::DMatch match;
            match.queryIdx = fwdMatch.queryIdx;
            match.trainIdx = revMatch.queryIdx;
            match.distance = fwdMatch.distance;
            goodMatch.push_back(match);
        }
    }
    cout << "matches total length: " << goodMatch.size();

    //将匹配关系表示出来
//    Mat matchesImg;
//    drawMatches(img1, keyPoint_obj, img2, keyPoint_scene, goodMatch, matchesImg, Scalar::all(-1), Scalar::all(-1),
//                vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
//    cv::namedWindow("goodMatches", CV_WINDOW_NORMAL);
//    imshow("goodMatches",matchesImg);
////    imwrite("/home/jiang/2_data/MonoDistance/image_write/NoRansac.png", matchesImg);
// 	cv::waitKey(0);
}

/*********************************** 12 *************************************
 * 读取文件内容, 逗号间隔的数据 
 **/
void ReadLable(vector<ImgLable> &vImgLable， const string &FilePath)
{
    char buffer[1000];
    ifstream LableFile(FilePath.c_str());

    if(LableFile.is_open())
    {
        //cout << "---打开CamPoseFile文件路径： " << file << endl << endl;
        while (!LableFile.eof())
        {
            LableFile.getline(buffer, 1000);

            int bValid, nType1, tracked_id;            
            float fMaxX, fMaxY, fMinX, fMinY, fProb;
            
            sscanf(buffer,"%d,%d,%d,%f,%f,%f,%f,%f", 
                            &bValid, &nType1, &tracked_id, &fMaxX, &fMaxY, &fMinX, &fMinY);

            if(LableFile.fail())
                break;

            ImgLable tmpLable;
            
            tmpLable.nType = nType1;
            tmpLable.bValid = bValid;
            tmpLable.tracked_id = tracked_id;           

            tmpLable.fMaxX = fMaxX;    tmpLable.fMaxY = fMaxY;
            tmpLable.fMinX = fMinX;    tmpLable.fMinY = fMinY;           

            vImgLable.emplace_back(tmpLable);
        }
    }
    else
        cerr << "---LableFile 打开失败" << file << endl << endl;

    LableFile.close();   
    
}

/*********************************** 13 *************************************
* opencv图像去畸变，lable点去畸变
**/
void Monodistance::UndistortImageAndLable(const cv::Mat &img, cv::Mat &imgCalib, vector<ImgLable> &vlable)
{
    Mat frame;

    Mat map1, map2;
    Size imageSize;
    imageSize = img.size();
    initUndistortRectifyMap(mK, mD, Mat(),
                            getOptimalNewCameraMatrix(mK, mD, imageSize, 1, imageSize, 0),
                            imageSize, CV_16SC2, map1, map2);

    remap(img, imgCalib, map1, map2, INTER_LINEAR);
//    imshow("Origianl", img);
//    imshow("Calibration", imgCalib);
//    imwrite("c:\\Users\\yangg\\Desktop\\立体匹配\\1左矫正.png", imgCalib);
//    cv::waitKey(0);

    int rows1 = map1.rows;
    int cols1 = map1.cols;
    int n = vlable.size();
    for(int i=0; i<n; i++)
    {
        ImgLable CurLable = vlable[i];
        cv::Point2f vLablePoint1, vLablePoint2;
        cv::Point2f vLablePointUn1(0.0, 0.0), vLablePointUn2(0.0, 0.0);

        vLablePoint1 = cv::Point2f(CurLable.fMinX, CurLable.fMinY);
        vLablePoint2 = cv::Point2f(CurLable.fMaxX, CurLable.fMaxY);

// initUndistortRectifyMap() 这个函数计算出来的 map1 是CV_16SC2类型的两通道变量，
// 而且函数计算的 map1 是根据理想图像计算出来的每一个像素点对应的畸变后的x方向点和y方向点，
// 即map1的像素（x，y）是理想点(去畸变后的点)，map1的通道[0]代表理想点x发生畸变后的x'点，
// map1的通道[1]代表理想点y发生畸变后的y'点，而你的（1344,756）是畸变后的（x',y'）点，
// 如果你想要的找到（1344,756）对应的理想点，要通过查表法查找map1(i,j)[0]=1344，map1(i,j)[1]=756,
// 此时的(i,j)是（1344,756）对应的理想点，而此函数求出来的map2实际是没什么用的，当map1类型是CV_32FC1时，
// 此函数计算出的map1为对应的x方向映射，map2为对应的y方向映射，但是查找理想点还是要用我刚刚说的查表方法查。
        for (int v = 0; v < rows1; v++)
        {
            for (int u = 0; u < cols1; u++)
            {
                // typedef cv::Vec<int16_t, 2> Vec2myi;
                if (abs(map1.at<Vec2myi>(v, u)[0] - (int)vLablePoint1.x) < 2 &&
                    abs(map1.at<Vec2myi>(v, u)[1] - (int)vLablePoint1.y) < 2 )
                {
                    vLablePointUn1.x = (float)u;
                    vLablePointUn1.y = (float)v;
                    break;
                }
                else if (abs(map1.at<Vec2myi>(v, u)[0] - (int)vLablePoint2.x) < 2 &&
                         abs(map1.at<Vec2myi>(v, u)[1] - (int)vLablePoint2.y) < 2)
                {
                    vLablePointUn2.x = (float)u;
                    vLablePointUn2.y = (float)v;
                    break;
                }
            }
        }

        vlable[i].fMinX = vLablePointUn1.x;
        vlable[i].fMinY = vLablePointUn1.y;
        vlable[i].fMaxX = vLablePointUn2.x;
        vlable[i].fMaxY = vLablePointUn2.y;
    }

}

/*********************************** 14 *************************************
* opencv 2d_2d求位姿，RANSAC去除误匹配
**/
void Pose2d2dAndRansac (
    const cv::Mat &img1,
    const cv::Mat &img2,
    const std::vector<cv::KeyPoint>& keypoints_1,
    const std::vector<cv::KeyPoint>& keypoints_2,
    const std::vector<cv::DMatch>& matches,
    cv::Mat& R, cv::Mat& t )
{    
    //-- 把匹配点转换为vector<Point2f>的形式
    vector<cv::Point2f> points1， points2; 
    vector <KeyPoint> RAN_KP1, RAN_KP2;

    for ( int i = 0; i < ( int ) matches.size(); i++ )
    {
        //cout << "[matches[i].queryIdx: " << matches[i].queryIdx << endl;
        points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
        points2.push_back ( keypoints_2[matches[i].trainIdx].pt );

        RAN_KP1.push_back ( keypoints_1[matches[i].queryIdx] );
        RAN_KP2.push_back ( keypoints_2[matches[i].trainIdx] );

    }   

    //-- 计算本质矩阵
    cv::Mat essential_matrix;
    cv::Mat mask;  //mask中大于零的点代表内点，等于零的点代表外点
    vector<uchar> RansacStatus;

    essential_matrix = findEssentialMat(points1, points2, mK, CV_RANSAC, 0.999, 3.0, mask); // 3.0->25 4.0->19
    if (essential_matrix.empty())
        cerr << "essential_matrix is empty." << endl;

    for(int i=0; i<mask.size().height; i++)
        RansacStatus.emplace_back(mask.at<uchar>(i,0));

    // cout << "RansacStatus.size(): " << RansacStatus.size() << endl;

    int feasible_count=0, unfeasible_count=0;
    for(int i=0; i<RansacStatus.size(); i++)
    {
        if(RansacStatus[i] == 0)
            unfeasible_count++;
        else
            feasible_count++;
    }

    cout << "内点数量： " << feasible_count << endl;
    cout << "外点数量： " << points1.size() - feasible_count << endl;

    //对于RANSAC而言，outlier数量大于50%时，结果是不可靠的
    float rito = (float)feasible_count / points1.size();
        cout << "内点占比： " << rito << endl;
    if (rito < 0.6)
        cerr << "外点数量大于40%" << endl;

    cout << fixed << "essential_matrix is "<< endl << essential_matrix << endl << endl;

    //显示RANSAC后的匹配情况
   vector <KeyPoint> RR_KP1, RR_KP2;
   vector<cv::DMatch> tmpMatches(matches);
   vector <DMatch> RR_matches;
   int index = 0;
   for (size_t i = 0; i < tmpMatches.size(); i++)
   {
       if (RansacStatus[i] != 0)
       {
           RR_KP1.push_back(RAN_KP1[i]);
           RR_KP2.push_back(RAN_KP2[i]);
           tmpMatches[i].queryIdx = index;
           tmpMatches[i].trainIdx = index;
           RR_matches.push_back(tmpMatches[i]);
           index++;
       }
   }
   cout << "RANSAC后匹配点数" <<RR_matches.size()<< endl;
   Mat img_RR_matches;
   drawMatches(img1, RR_KP1, img2, RR_KP2, RR_matches, img_RR_matches);
   imshow("After RANSAC",img_RR_matches);
// imwrite("/home/jiang/2_data/MonoDistance/image_write/ransac.png", img_RR_matches);

    //-- 从本质矩阵中恢复旋转和平移信息.
    int pass_count = recoverPose ( essential_matrix, points1, points2, mK, R, t, mask); 
    
    if (((double)pass_count) / feasible_count < 0.7)    //同时位于两个相机前方的点的数量要足够大
        cerr << "同时位于两个相机前方的点的数量太少" << endl;

    cout << fixed << "R is "<< endl << R << endl << endl;
    cout << fixed << "t is "<< endl << t << endl;

    cv::waitKey(0);
}
