#include <iostream>
#include <opencv2\highgui\highgui.hpp>
#include <string>
#include <Kinect.h>

using   namespace   std;
using   namespace   cv;

const   string  get_name(int n);    //此函数判断出关节点的名字
int main(void)
{
	IKinectSensor   * mySensor = nullptr;
	GetDefaultKinectSensor(&mySensor);
	mySensor->Open();

	int myBodyCount = 0;
	IBodyFrameSource    * myBodySource = nullptr;
	IBodyFrameReader    * myBodyReader = nullptr;
	mySensor->get_BodyFrameSource(&myBodySource);
	myBodySource->OpenReader(&myBodyReader);
	myBodySource->get_BodyCount(&myBodyCount);

	IDepthFrameSource   * myDepthSource = nullptr;
	IDepthFrameReader   * myDepthReader = nullptr;
	mySensor->get_DepthFrameSource(&myDepthSource);
	myDepthSource->OpenReader(&myDepthReader);

	int height = 0, width = 0;
	IFrameDescription   * myDescription = nullptr;;
	myDepthSource->get_FrameDescription(&myDescription);
	myDescription->get_Height(&height);
	myDescription->get_Width(&width);   //以上为准备好深度数据和骨骼数据的Reader

	IBodyFrame  * myBodyFrame = nullptr;
	IDepthFrame * myDepthFrame = nullptr;
	Mat img16(height, width, CV_16UC1); //为显示深度图像做准备
	Mat img8(height, width, CV_8UC1);

	while (1)
	{
		while (myDepthReader->AcquireLatestFrame(&myDepthFrame) != S_OK);
		myDepthFrame->CopyFrameDataToArray(width * height, (UINT16 *)img16.data);
		img16.convertTo(img8, CV_8UC1, 255.0 / 4500);
		imshow("Depth Img", img8);  //深度图像的转化及显示

		while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != S_OK);

		int myBodyCount = 0;
		IBody   ** bodyArr = nullptr;
		myBodySource->get_BodyCount(&myBodyCount);
		bodyArr = new IBody *[myBodyCount];
		for (int i = 0; i < myBodyCount; i++)   //bodyArr的初始化
			bodyArr[i] = nullptr;

		myBodyFrame->GetAndRefreshBodyData(myBodyCount, bodyArr);
		for (int i = 0; i < myBodyCount; i++)   //遍历6个人(可能用不完)
		{
			BOOLEAN     result = false;
			if (bodyArr[i]->get_IsTracked(&result) == S_OK && result)   //判断此人是否被侦测到
			{
				cout << "Body " << i << " tracked!" << endl;

				int count = 0;
				Joint   jointArr[JointType_Count];
				bodyArr[i]->GetJoints(JointType_Count, jointArr);    //获取此人的关节数据
				for (int j = 0; j < JointType_Count; j++)
				{
					if (jointArr[j].TrackingState != TrackingState_Tracked) //将确定侦测到的关节显示出来
						continue;
					string  rt = get_name(jointArr[j].JointType);   //获取关节的名字
					if (rt != "NULL")   //输出关节信息
					{
						count++;
						cout << "   " << rt << " tracked" << endl;
						if (rt == "Right thumb")
							cout << "       Right thumb at " << jointArr[j].Position.X << "," << jointArr[j].Position.Y << "," << jointArr[j].Position.Z << endl;
					}
				}
				cout << count << " joints tracked" << endl << endl;
			}
		}
		myDepthFrame->Release();
		myBodyFrame->Release();
		delete[] bodyArr;

		if (waitKey(30) == VK_ESCAPE)
			break;
		Sleep(1000);    //为避免数据刷太快，每秒钟更新一次
	}
	myBodyReader->Release();
	myDepthReader->Release();
	myBodySource->Release();
	myDepthSource->Release();
	mySensor->Close();
	mySensor->Release();

	return  0;
}

const   string  get_name(int n)
{
	switch (n)
	{
	case    2:return    "Neck"; break;
	case    3:return    "Head"; break;
	case    4:return    "Left shoulder"; break;
	case    8:return    "Right shoulder"; break;
	case    7:return    "Left hand"; break;
	case    11:return   "Right hand"; break;
	case    22:return   "Left thumb"; break;
	case    24:return   "Right thumb"; break;
	default:return "NULL";
	}
}