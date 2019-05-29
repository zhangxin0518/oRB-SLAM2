/**
* This file is part of ORB-SLAM2.
* 单目 相机 kitti数据集
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>    //时间
#include<iomanip>

#include<opencv2/core/core.hpp>
#include"System.h"

using namespace std;

// 读取图片目录  根据序列文件 返回文件名字符串容器 和 对于时间戳序列
void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps);


int main(int argc, char **argv)
{
    // 用法： ./mono_kitti 词典路径 配置文件路径 数据集
    if(argc != 4)
    {                            
        cerr << endl << "用法: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // 加载序列文件，得到 图片文件路径 和 对应的序列与时间戳
    vector<string> vstrImageFilenames;         // 图片文件名
    vector<double> vTimestamps;                // 图片时间戳
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);
    int nImages = vstrImageFilenames.size();   // 图片数量

    // 创建SLAM系统对象 完成了 字典读取 跟踪线程 建图线程 回环检测线程 可视化线程 的初始化启动
    // 初始化系统 传入字典 配置文件路径
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // 跟踪线程 每一帧图像跟踪处理的时间
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "开始处理图片序列 ..." << endl;
    cout << "总图片数量: " << nImages << endl << endl;

    // 追踪线程主循环============================================================================
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // 循环读取图片序列内的图像数据
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        // 循环读取图像对应的时间戳
        double tframe = vTimestamps[ni];
        if(im.empty())
        {
            cerr << endl << "未能成功载入图像: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

		// 时间记录开始
	    #ifdef COMPILEDWITHC11
		    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
	    #else
		    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
	    #endif

        // 将图像和时间戳传给SLAM系统，进行单目追踪线程处理
        SLAM.TrackMonocular(im,tframe);

	    // 时间记录结束
	    #ifdef COMPILEDWITHC11
		    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
	    #else
		    std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
	    #endif

        // 单目跟踪一帧图像耗费的时间
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
	 
	    // 保持单目跟踪处理的时间
        vTimesTrack[ni]=ttrack;

        // 两帧时间戳之差
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)// 跟踪时间小于图像帧率时间，休息一会
        {
            usleep((T-ttrack)*1e6);
        }
    }

    // 关闭所有系统线程
    SLAM.Shutdown();

    // 每一帧跟踪时间排序
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni]; // 跟踪总时间
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;// 每一帧跟踪的时间中值 
    cout << "mean tracking time: " << totaltime/nImages << endl;       // 每一帧跟踪的时间均值

    // 保存相机轨迹
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");    

    return 0;
}


// 根据图片序列文件 生成 图片文件路径 容器 和 其 对应时间戳 容器 
void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
	ifstream fTimes;
	string strPathTimeFile = strPathToSequence + "/times.txt";
	fTimes.open(strPathTimeFile.c_str());//打开文件
	while(!fTimes.eof())//到文件末尾
	{
	    string s;
	    getline(fTimes,s);//每一行
	    if(!s.empty())
	    {
		stringstream ss;
		ss << s;
		double t;
		ss >> t;//时间戳
		vTimestamps.push_back(t);// 存入时间戳容器
	    }
	}

	string strPrefixLeft = strPathToSequence + "/image_0/";//图片父目录

	const int nTimes = vTimestamps.size();//总数量
	vstrImageFilenames.resize(nTimes);

	for(int i=0; i<nTimes; i++)
	{
	    stringstream ss;
	    ss << setfill('0') << setw(6) << i;// 宽度6位 填充0 
	    vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";// 图片文件完整路径
	}
}
