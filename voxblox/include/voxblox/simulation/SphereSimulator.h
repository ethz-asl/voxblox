#ifndef SPHERESIMULATOR_H_
#define SPHERESIMULATOR_H_

//C++
//#include <ctime>
#include <random>
#include <cmath>
#include <memory>

//C++
#include <opencv2/opencv.hpp>
#include <mutex>
#include <thread>
#include <queue>
#include <algorithm>

//#include <pcl/registration/icp.h>
//#include <tf/tf.h>
//#include <tf/transform_listener.h>

//#include <geometry_msgs/Point.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <visualization_msgs/Marker.h>


//Thirdpary

using namespace std;

namespace SphereSim{

//forward decs
//...
//--------------

class DataSimulator
{
public:
    typedef shared_ptr<random_device> rdevptr;
    typedef shared_ptr<mt19937> mtwistptr;
    typedef shared_ptr<normal_distribution<double>> ndistptr;
    class UniqueIdDispenser;
//    typedef boost::shared_ptr<UniqueIdDispenser> uidptr;

	class UniqueIdDispenser
	{
	public:
	    UniqueIdDispenser() : mLastId(-1) {}

	    int operator()()
	    {
		return this->GetId();
	    }

	    size_t GetId()
	    {
		unique_lock<mutex> lock(mMutexId);
		++mLastId;
		return mLastId;
	    }

	    size_t GetLastId()
	    {
		unique_lock<mutex> lock(mMutexId);
		return mLastId;
	    }

	private:
	    size_t mLastId;
	    mutex mMutexId;
	};

public:
    DataSimulator(double dMu, double dSigma, string path_to_params);

    void Sphere(cv::Mat& points3D, double rad, size_t n);

private:

    void CreateSphere(cv::Mat& data, double rad, size_t nval);

    //Infrastructure
    UniqueIdDispenser mUID;
//    ros::NodeHandle mNh;
//    ros::Publisher mPubPcl;

    string mPathToParams;

    //Distribution Params
    double mdMu;
    double mdSigma;

    //Distributions
    rdevptr pRDev;
    mtwistptr pMTwist;
    ndistptr pDist;
};

} //end ns

#endif
