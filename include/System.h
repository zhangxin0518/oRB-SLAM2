/**
* This file is part of ORB-SLAM2.
*
*/


#ifndef SYSTEM_H
#define SYSTEM_H

#include<string>				//字符串
#include<thread>				// 线程
#include<opencv2/core/core.hpp>	// opencv

// user 
#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;

class System
{
    public:
	// 传感器枚举类型  输入传感器类型
	enum eSensor{
	    MONOCULAR=0,	// 单目0
	    STEREO=1,    	// 双目1
	    RGBD=2	    	// 深度2
	};

    public:
    // 初始化SLAM系统，启动 建图、闭环检测 和 可视化 线程 
	System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true);

	// Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
	// 双目跟踪 图像必须同步且矫正 返回相机位姿 
	cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

	// Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
	// Input depthmap: Float (CV_32F).
	// 深度跟踪 深度图必须关联RGB图 返回相机位姿
	cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);

	// Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
	// 单目跟踪  返回相机位姿
	cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

	// 仅仅实现相机追踪线程，关闭局部建图线程
	void ActivateLocalizationMode();
	// This resumes local mapping thread and performs SLAM again.
	// 实现相机追踪 和 局部建图 的 SLAM过程
	void DeactivateLocalizationMode();

	// Returns true if there have been a big map change (loop closure, global BA)
	// since last call to this function
	bool MapChanged();

	// Reset the system (clear map)
	void Reset();

	// 全部线程被要求完成并结束
	// This function must be called before saving the trajectory.
	void Shutdown();

	// Save camera trajectory in the TUM RGB-D dataset format.
	// Only for stereo and RGB-D. This method does not work for monocular.
	// Call first Shutdown()
	// See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
	// 保存相机 位姿
	void SaveTrajectoryTUM(const string &filename);

	// Save keyframe poses in the TUM RGB-D dataset format.
	// This method works for all sensor input.
	// Call first Shutdown()
	// See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
	void SaveKeyFrameTrajectoryTUM(const string &filename);

	// Save camera trajectory in the KITTI dataset format.
	// Only for stereo and RGB-D. This method does not work for monocular.
	// Call first Shutdown()
	// See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
	void SaveTrajectoryKITTI(const string &filename);

	// TODO: Save/Load functions
	// SaveMap(const string &filename);
	// LoadMap(const string &filename);

	// Information from most recent processed frame
	// You can call this right after TrackMonocular (or stereo or RGBD)
	int GetTrackingState();
	std::vector<MapPoint*> GetTrackedMapPoints();
	std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

    private:
	// 枚举变量  输入相机类型 单目 双目 深度
	eSensor mSensor;

	// 词典对象指针 用于 地点识别 特征匹配 orb特征
	ORBVocabulary* mpVocabulary;

	// 关键帧 数据库 对象指针  用于 地点识别 定位 回环检测
	KeyFrameDatabase* mpKeyFrameDatabase;

	// 地图对象指针 存储 关键帧 和 地图点
	Map* mpMap;

	// 跟踪对象指针 接收一帧图像并计算相机位姿 决定何时植入新的关键帧，创建新地图点， 追踪失败的重定位
	Tracking* mpTracker;

	// 局部建图对象指针 管理局部地图 完成局部BA优化
	LocalMapping* mpLocalMapper;

	// 回环检测对象指针 姿态图优化 全局BA优化
	LoopClosing* mpLoopCloser;

	// 可视化对象指针 可视化地图与图像位姿
	Viewer* mpViewer;

	// 画关键帧对象指针
	FrameDrawer* mpFrameDrawer;

	// 画地图对象指针
	MapDrawer* mpMapDrawer;

	// 系统线程：局部建图、闭环检测 与 可视化
	// 追踪线程 在主函数里执行
	std::thread* mptLocalMapping; // 建图线程指针
	std::thread* mptLoopClosing;  // 闭环检测线程指针
	std::thread* mptViewer;	      // 可视化线程指针

	// 线程重启标志
	std::mutex mMutexReset;
	bool mbReset;

    // 使用std::mutex创建互斥量，通过调用成员函数lock()进行上锁，unlock()进行解锁。但不方便的是需要记住锁后要在函数出口再次调用unlock()解锁. 
    // 因此可以用std::lock_guard,其会在构造的时候提供已锁的互斥量，并在析构的时候进行解锁，从而保证自动管理。
	std::mutex mMutexMode;
	bool mbActivateLocalizationMode;	// 跟踪 + 定位
	bool mbDeactivateLocalizationMode;	// 跟踪 + 建图

	// 跟踪线程的状态
	int mTrackingState;
	std::vector<MapPoint*> mTrackedMapPoints;
	std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
	std::mutex mMutexState;

    };

}// namespace ORB_SLAM

#endif // SYSTEM_H
