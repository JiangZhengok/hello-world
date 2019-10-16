
/*********************************** 1 *************************************
 * 写入数据到文件，文件名由当前时间命名
 **/
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

/*********************************** 2 *************************************
 * 读取文件内容
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
 * 欧拉角到旋转矩阵
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

Eigen::Matrix3d rotation_matrix = yawAngle * pitchAngle * rollAngle;
Eigen::Quaterniond            q = yawAngle * pitchAngle * rollAngle;

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