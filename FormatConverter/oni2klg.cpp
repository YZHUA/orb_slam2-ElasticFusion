
#include <iostream>  
#include <OpenNI.h>  
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>

using namespace std;  
  
int main(int argc,char **argv)
{
    if(argc!=3){
        printf("error using FormatConverter!!\nexample usage:\nFormatConverter test.oni test.klg\n");
        return -1;
    }
    //定义oni文件中视频的总帧数以及得到的图片的保存目录  
    int total = 0;  
      
    //初始化OpenNI环境  
    openni::OpenNI::initialize();  
  
    //声明设备并打开oni文件  
    openni::Device fromonifile;  
    fromonifile.open(argv[1]);
  
    //声明控制对象，这对视频流的控制起到了关键作用  
    openni::PlaybackControl* pController = fromonifile.getPlaybackControl();  
  
    //声明视频流对象以及帧对象  
    openni::VideoStream streamColor;  
    openni::VideoFrameRef frameColor;  
    openni::VideoStream streamDepth;
    openni::VideoFrameRef frameDepth;
    openni::VideoStream streamIR;
    openni::VideoFrameRef frameIR;
  
    int COLOR_status=0,DEPTH_status=0,IR_status=0;
    //验证是否有彩色传感器（是否有彩色视频）和建立与设备想关联的视频流  
    if(fromonifile.hasSensor(openni::SENSOR_COLOR))  
    {
        COLOR_status=1;
        if(streamColor.create( fromonifile, openni::SENSOR_COLOR ) == openni::STATUS_OK )  
        {  
            cout<<"建立视频流成功"<<endl;  
        }  
        else  
        {  
            cerr<<"ERROR: 建立视频流没有成功"<<endl;  
            system("pause");  
            return -1;  
        }  
    }  
    else
    {  
        cerr << "ERROR: 该设备没有彩色传感器" << endl;
        //system("pause");
        //return -1;
    }  
    //DEPTH
    if(fromonifile.hasSensor(openni::SENSOR_DEPTH))
    {
        DEPTH_status=1;
        if(streamDepth.create( fromonifile, openni::SENSOR_DEPTH ) == openni::STATUS_OK )
        {
            cout<<"建立视频流成功"<<endl;
        }
        else
        {
            cerr<<"ERROR: 建立视频流没有成功"<<endl;
            system("pause");
            return -1;
        }
    }
    else
    {
        cerr << "ERROR: 该设备没有彩色传感器" << endl;
        //system("pause");
        //return -1;
    }
    //IR
    if(fromonifile.hasSensor(openni::SENSOR_IR))
    {
        IR_status=1;
        if(streamIR.create( fromonifile, openni::SENSOR_IR ) == openni::STATUS_OK )
        {
            cout<<"建立视频流成功"<<endl;
        }
        else
        {
            cerr<<"ERROR: 建立视频流没有成功"<<endl;
            system("pause");
            return -1;
        }
    }
    else
    {
        cerr << "ERROR: 该设备没有彩色传感器" << endl;
        //system("pause");
        //return -1;
    }
  
    //建立显示窗口  
    if(COLOR_status==1)
        cv::namedWindow("Color");
    if(DEPTH_status==1)
        cv::namedWindow("Depth");
    if(IR_status==1)
        cv::namedWindow("IR");
  
    //获取总的视频帧数并将该设备的速度设为-1以便能留出足够的时间对每一帧进行处理、显示和保存  
    total = pController->getNumberOfFrames(streamColor);  
    pController->setSpeed(-1);  

    FILE *fp=fopen(argv[2],"w");
    fwrite( &total, sizeof(int32_t  ), 1, fp );
    int iMaxDepth = streamDepth.getMaxPixelValue();
    //开启视频流  
    streamColor.start();
    streamDepth.start();
    int num_pixs1=640*480*2;int num_pixs2=640*480*3;
    for (long int i = 1;i <= total; ++ i)
    {  
        fwrite( &i, sizeof( int64_t ), 1, fp );
        fwrite( &num_pixs1, sizeof( int32_t ), 1, fp );
        fwrite( &num_pixs2, sizeof( int32_t ), 1, fp );

        if(DEPTH_status==1){
            //读取视频流的当前帧
            streamDepth.readFrame(&frameDepth);
            //将帧保存到Mat中并且将其转换到BGR模式，因为在OpenCV中图片的模式是BGR
            cv::Mat GrayImg(frameDepth.getHeight(), frameDepth.getWidth(), CV_16UC1, (void*)frameDepth.getData());
            cv::Mat mScaledDepth;
            char *tmp=(char*)frameDepth.getData();
            GrayImg.convertTo( mScaledDepth, CV_8U, 255.0 / iMaxDepth );
            fwrite( tmp, sizeof( char ), num_pixs1, fp );
            //fout.write(tmp, sizeof(char) * num_pixs1);
            //显示当前帧
            cv::imshow("Depth",GrayImg);
        }

        if(COLOR_status==1){
            //读取视频流的当前帧
            streamColor.readFrame(&frameColor);

            cout<<"当前正在读的帧数是："<<frameColor.getFrameIndex()<<endl;

            //将帧保存到Mat中并且将其转换到BGR模式，因为在OpenCV中图片的模式是BGR
            cv::Mat rgbImg(frameColor.getHeight(), frameColor.getWidth(), CV_8UC3, (void*)frameColor.getData());
            cv::Mat bgrImg;
            cvtColor(rgbImg, bgrImg, CV_RGB2BGR);
            char *tmp=(char*)frameColor.getData();
            fwrite( tmp, sizeof( char ), num_pixs2, fp );
            //显示当前帧
            cv::imshow("Color",bgrImg);
        }

        if(IR_status==1){
            streamIR.readFrame(&frameIR);
            //将帧保存到Mat中并且将其转换到BGR模式，因为在OpenCV中图片的模式是BGR
            cv::Mat GrayImg1(frameIR.getHeight(), frameIR.getWidth(), CV_8UC1, (void*)frameIR.getData());
            //显示当前帧
            cv::imshow("IR",GrayImg1);
        }
  
       /* //将每一帧按顺序帧保存到图片目录下
        char imagefullname[255];  
        char imagenum[50];  
        sprintf(imagenum,"//%03d.png",i);
        strcpy(imagefullname,imagefile);  
        strcat(imagefullname,imagenum);  
        cv::imwrite(imagefullname,bgrImg);  */
  

        if (cv::waitKey(30) == 27)  
        {  
            break;  
        }  
    }  

    fclose(fp);
    //销毁显示窗口  
    cv::destroyWindow("Image");   
  
    //关闭视频流  
    streamColor.destroy();  
  
    //关闭设备  
    fromonifile.close();  
  
    //关闭OpenNI  
    openni::OpenNI::shutdown();  
      
    return 0;  
} 
