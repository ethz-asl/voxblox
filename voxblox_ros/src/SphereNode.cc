#include <voxblox/simulation/SphereSimulator.h>

#include <opencv2/opencv.hpp>

//ROS
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "Sphere Simulator");

    if(argc != 2)
    {
        cerr << endl << "Usage: rosrun cslam clientnode path_to_params" << endl;
        ros::shutdown();
        return 1;
    }    

    ros::NodeHandle Nh;
    ros::Publisher PubPcl;
    PubPcl = Nh.advertise<sensor_msgs::PointCloud2>("Sphere",1);

    //Load Params

    cv::FileStorage fParams(argv[1], cv::FileStorage::READ);
    double mu = fParams["Sim.Mu"];
    double sig = fParams["Sim.Sigma"];
    int n = fParams["Sim.NumOfPoints"];
    double rad = fParams["Sim.Radius"];

    cout << "***Simulator Params***" << endl;
    cout << "Mean: " << mu << endl;
    cout << "Sigma: " << sig << endl;
    cout << "#Points: " << n << endl;
    cout << "Radius: " << rad << endl;
    cout << "********************" << endl;

    SphereSim::DataSimulator Sim(mu,sig,argv[1]);
    cv::Mat sphere;
    Sim.Sphere(sphere,rad,n);

    pcl::PointCloud<pcl::PointXYZRGB> Cloud;

    uint32_t rgb = ((uint32_t)0 << 16 | (uint32_t)0 << 8 | (uint32_t)0);

    //calculate coordinates
    for(int it=0;it<sphere.rows;++it)
    {
        pcl::PointXYZRGB p;
        p.x = sphere.at<double>(it,1);
        p.y = sphere.at<double>(it,2);
        p.z = sphere.at<double>(it,3);
        p.rgb = *reinterpret_cast<float*>(&rgb);
        Cloud.points.push_back(p);
    }

    sensor_msgs::PointCloud2 pclMsg;

    if(!Cloud.points.empty())
    {
//        sensor_msgs::PointCloud2 pclMsg;
        pcl::toROSMsg(Cloud,pclMsg);
        pclMsg.header.frame_id = "world";
        pclMsg.header.stamp = ros::Time::now();
        cout << "Publishing sphere with " << Cloud.points.size() << " points" << endl;
//        for(int it=0;it<1000;++it)
//        {
//            PubPcl.publish(pclMsg);
//            usleep(10000);
//        }
//        usleep(100000);
    }

    ros::Rate r(1);
    while(ros::ok())
    {
        if(!Cloud.points.empty())
            PubPcl.publish(pclMsg);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
