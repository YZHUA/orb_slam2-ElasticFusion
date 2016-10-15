#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <OpenNI.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <System.h>   // orb_slam2

using namespace std;
using namespace openni;
using namespace cv;

void showdevice(){
    // 获取设备信息
    Array<DeviceInfo> aDeviceList;
    OpenNI::enumerateDevices(&aDeviceList);

    cout << "电脑上连接着 " << aDeviceList.getSize() << " 个体感设备." << endl;

    for (int i = 0; i < aDeviceList.getSize(); ++i)
    {
        cout << "设备 " << i << endl;
        const DeviceInfo& rDevInfo = aDeviceList[i];
        cout << "设备名： " << rDevInfo.getName() << endl;
        cout << "设备Id： " << rDevInfo.getUsbProductId() << endl;
        cout << "供应商名： " << rDevInfo.getVendor() << endl;
        cout << "供应商Id: " << rDevInfo.getUsbVendorId() << endl;
        cout << "设备URI: " << rDevInfo.getUri() << endl;

    }
}

Status initstream(Status& rc, Device& xtion, VideoStream& streamDepth, VideoStream& streamColor,int &total,char *oniFilePath)
{
    rc = STATUS_OK;

    //初始化OpenNI环境
    openni::OpenNI::initialize();

    //声明设备并打开oni文件
    xtion.open(oniFilePath);


    //声明控制对象，这对视频流的控制起到了关键作用
    openni::PlaybackControl* pController = xtion.getPlaybackControl();
    //验证是否有彩色传感器（是否有彩色视频）和建立与设备想关联的视频流
    if(xtion.hasSensor(openni::SENSOR_COLOR))
    {
        if(streamColor.create( xtion, openni::SENSOR_COLOR ) == openni::STATUS_OK )
        {
            cout<<"建立视频流成功"<<endl;
        }
        else
        {
            cerr<<"ERROR: 建立视频流没有成功"<<endl;
        }
    }
    else
    {
        cerr << "ERROR: 该设备没有彩色传感器" << endl;
        //system("pause");
        //return -1;
    }
    //DEPTH
    if(xtion.hasSensor(openni::SENSOR_DEPTH))
    {
        if(streamDepth.create( xtion, openni::SENSOR_DEPTH ) == openni::STATUS_OK )
        {
            cout<<"建立视频流成功"<<endl;
        }
        else
        {
            cerr<<"ERROR: 建立视频流没有成功"<<endl;
        }
    }
    else
    {
        cerr << "ERROR: 该设备没有彩色传感器" << endl;
        //system("pause");
        //return -1;
    }

    /*// 图像模式注册,彩色图与深度图对齐
    if (xtion.isImageRegistrationModeSupported(
        IMAGE_REGISTRATION_DEPTH_TO_COLOR))
    {
        xtion.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    }  */
    total = pController->getNumberOfFrames(streamColor);
    pController->setSpeed(-1);
    return rc;
}

int main(int argc, char **argv)
{
    //argc=4;
    //argv[3]="/home/yzh/orbslam/gaox/ORB_SLAM2_modified/test.oni";
    if(argc != 4)
    {
        cerr << endl << "Usage: ./rgbd_cc_offline path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    // 创建ORB_SLAM系统. (参数1：ORB词袋文件  参数2：xtion参数文件)
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    cout << endl << "-------" << endl;
    cout << "Openning Xtion XXX ..." << endl;

    Status rc = STATUS_OK;
    // 初始化OpenNI环境
    OpenNI::initialize();
    showdevice();
    // 声明并打开Device设备。
    Device xtion;
    //const char * deviceURL = openni::ANY_DEVICE;  //设备名
    //rc = xtion.open(deviceURL);


    VideoStream streamDepth;
    VideoStream streamColor;
    int total_frames=0;
    if(initstream(rc, xtion, streamDepth, streamColor,total_frames,argv[3]) == STATUS_OK)     // 初始化数据流
        cout << "Open Xtion Successfully!"<<endl;
    else
    {
        cout << "Open Xtion Failed!"<<endl;
        return 0;
    }

    streamColor.start();
    streamDepth.start();
    // Main loop
    cv::Mat imRGB, imD;
    bool continueornot = true;
    // 循环读取数据流信息并保存在VideoFrameRef中
    VideoFrameRef  frameDepth;
    VideoFrameRef  frameColor;
    namedWindow("RGB Image", CV_WINDOW_AUTOSIZE);
    for (double index = 1.0; continueornot; index+=1.0)
    {
        //cout<<index<<endl;
        rc = streamDepth.readFrame(&frameDepth);
        if (rc == STATUS_OK)
        {
            imD = cv::Mat(frameDepth.getHeight(), frameDepth.getWidth(), CV_16UC1, (void*)frameDepth.getData());   //获取深度图
        }
        rc = streamColor.readFrame(&frameColor);
        if (rc == STATUS_OK)
        {
            const Mat tImageRGB(frameColor.getHeight(), frameColor.getWidth(), CV_8UC3, (void*)frameColor.getData());   //获取彩色图
            cvtColor(tImageRGB, imRGB, CV_RGB2BGR);
            imshow("RGB Image",imRGB);
        }
        SLAM.TrackRGBD( imRGB, imD,  index);   // ORB_SLAM处理深度图和彩色图
        char c  = cv::waitKey(5);
        switch(c)
        {
        case 'q':
        case 27:         //退出
            continueornot = false;
            break;
        case 'p':         //暂停
            cv::waitKey(0);
            break;
        default:
            break;
        }
        if(index>total_frames)
            continueornot = false;
    }
    streamColor.destroy();
    streamDepth.destroy();
    // Stop all threads
    SLAM.Shutdown();
    SLAM.SaveTrajectoryTUM("trajectory.txt");
    SLAM.SaveTrajectoryKITTI("trajectory1.txt");

    cv::destroyAllWindows();
    return 0;
}

