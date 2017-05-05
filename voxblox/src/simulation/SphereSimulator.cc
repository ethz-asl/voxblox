#include <voxblox/simulation/SphereSimulator.h>

namespace SphereSim{

DataSimulator::DataSimulator(double dMu, double dSigma, string path_to_params)
    : mdMu(dMu),mdSigma(dSigma),mPathToParams(path_to_params)
{
    pRDev.reset(new random_device());
    pMTwist.reset(new mt19937(pRDev->operator()()));
    pDist.reset(new normal_distribution<double>(mdMu,mdSigma));

//    ros::Publisher mPubPcl = mNh.advertise<sensor_msgs::PointCloud2>("Sphere",1);
}

void DataSimulator::Sphere(cv::Mat &data, double rad, size_t n)
{
//    Load Params

//    cv::FileStorage fParams(mPathToParams, cv::FileStorage::READ);
//    int n = fParams["Sim.NumOfPoints"];
//    double rad = fParams["Sim.Radius"];

//    cout << "***Simulator Params***" << endl;
//    cout << "Mean: " << mdMu << endl;
//    cout << "Sigma: " << mdSigma << endl;
//    cout << "#Points: " << n << endl;
//    cout << "Radius: " << rad << endl;
//    cout << "********************" << endl;

    //make sphere
    this->CreateSphere(data,rad,n);
}

void DataSimulator::CreateSphere(cv::Mat &data, double rad, size_t nval)
{
    data = cv::Mat(0,4,6);

    int n = nval -2; //1 for top and bottom point
    int s = round(sqrt(n-1)); //sphere slices
    double step = 2*rad / ((double)s+1);

    {
        //top point
        cv::Mat p(1,4,6,0.0);
        p.at<double>(0,0) = (double)mUID();
        p.at<double>(0,1) = 0.0;
        p.at<double>(0,2) = 0.0;
        p.at<double>(0,3) = rad;
        data.push_back(p);
    }

    double SumA = 0;
    for(int it=1;it<=s;++it)
    {
        double z = rad - (double)it * step;
        SumA += 2*M_PI*sqrt(rad*rad - z*z);
    }

    for(int it=1;it<=s;++it)
    {
        double zi = rad - (double)it*step;
        double ri = sqrt(rad*rad - zi*zi);
        double Ai = 2*M_PI*ri;
        int ni = round(n*Ai/SumA);
        double stepi = 2*M_PI/(double)ni;

        for(int it2=0;it2<ni;++it2)
        {
            double a;
            if(it%2 == 0)
                a = (double)it2*stepi;
            else
                a = (double)(it2+0.5)*stepi;

            double x,y,z;

            x = ri*cos(a) + pDist->operator()(*pMTwist);
            y = ri*sin(a) + pDist->operator()(*pMTwist);
            z = zi + pDist->operator()(*pMTwist);

            cv::Mat p(1,4,6,0.0);
            p.at<double>(0,0) = (double)mUID();
            p.at<double>(0,1) = x;
            p.at<double>(0,2) = y;
            p.at<double>(0,3) = z;
            data.push_back(p);
        }
    }

    {
        cv::Mat p(1,4,6,0.0);
        p.at<double>(0,0) = (double)mUID();
        p.at<double>(0,1) = 0.0;
        p.at<double>(0,2) = 0.0;
        p.at<double>(0,3) = -rad;
        data.push_back(p);
    }

    cout << "Created sphere with " << data.rows << " points" << endl;
}

//void DataSimulator::SimulateSphere()
//{
//    //Load Params

//    cv::FileStorage fParams(mPathToParams, cv::FileStorage::READ);
//    int n = fParams["Sim.NumOfPoints"];
//    double rad = fParams["Sim.Radius"];

//    cout << "***Simulator Params***" << endl;
//    cout << "Mean: " << mu << endl;
//    cout << "Sigma: " << sig << endl;
//    cout << "#Points: " << n << endl;
//    cout << "Radius: " << rad << endl;
//    cout << "********************" << endl;

//    DataSimulator Sim(mu,sig);
//    Sim.Spheres(Sphere,rad,n);

//    pcl::PointCloud<pcl::PointXYZRGB> Cloud;

//    uint32_t rgbRed = (((uint32_t)255) << 16 | ((uint32_t)0) << 8 | ((uint32_t)0));
//    uint32_t rgbOrange = (((uint32_t)255) << 16 | ((uint32_t)128) << 8 | ((uint32_t)0));
//    uint32_t rgbCyan = ((uint32_t)0 << 16 | (uint32_t)255 << 8 | (uint32_t)255);
//    uint32_t rgbMagenta = ((uint32_t)255 << 16 | (uint32_t)0 << 8 | (uint32_t)255);
//    uint32_t rgbGreen = ((uint32_t)0 << 16 | (uint32_t)255 << 8 | (uint32_t)0);
//    uint32_t rgbBlack = ((uint32_t)0 << 16 | (uint32_t)0 << 8 | (uint32_t)0);
//    uint32_t rgbBlue = ((uint32_t)0 << 16 | (uint32_t)0 << 8 | (uint32_t)255);

//    //calculate coordinates
//    for(int it=0;it<Data.rows;++it)
//    {
//        pcl::PointXYZRGB p;
//        p.x = Data.at<double>(it,1);
//        p.y = Data.at<double>(it,2);
//        p.z = Data.at<double>(it,3);
//        p.rgb = *reinterpret_cast<float*>(&rgbBlack);
//        Cloud.points.push_back(p);
//    }

//    if(!Cloud.points.empty())
//    {
//        sensor_msgs::PointCloud2 pclMsg;
//        pcl::toROSMsg(Cloud,pclMsg);
//        pclMsg.header.frame_id = "world";
//        pclMsg.header.stamp = ros::Time::now();
//        mPubPcl.publish(pclMsg);
//        usleep(10000);
//        mPubPcl.publish(pclMsg);
//        usleep(10000);
//        mPubPcl.publish(pclMsg);
//        usleep(10000);
//        mPubPcl.publish(pclMsg);
//        usleep(10000);
//        mPubPcl.publish(pclMsg);
//        usleep(100000);
//    }
//}

} //end namespace
