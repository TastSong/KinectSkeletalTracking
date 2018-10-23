#include <iostream>
#include <opencv2\highgui\highgui.hpp>
#include <string>
#include <Kinect.h>

using   namespace   std;
using   namespace   cv;

const   string  get_name(int n);    //�˺����жϳ��ؽڵ������
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
	myDescription->get_Width(&width);   //����Ϊ׼����������ݺ͹������ݵ�Reader

	IBodyFrame  * myBodyFrame = nullptr;
	IDepthFrame * myDepthFrame = nullptr;
	Mat img16(height, width, CV_16UC1); //Ϊ��ʾ���ͼ����׼��
	Mat img8(height, width, CV_8UC1);

	while (1)
	{
		while (myDepthReader->AcquireLatestFrame(&myDepthFrame) != S_OK);
		myDepthFrame->CopyFrameDataToArray(width * height, (UINT16 *)img16.data);
		img16.convertTo(img8, CV_8UC1, 255.0 / 4500);
		imshow("Depth Img", img8);  //���ͼ���ת������ʾ

		while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != S_OK);

		int myBodyCount = 0;
		IBody   ** bodyArr = nullptr;
		myBodySource->get_BodyCount(&myBodyCount);
		bodyArr = new IBody *[myBodyCount];
		for (int i = 0; i < myBodyCount; i++)   //bodyArr�ĳ�ʼ��
			bodyArr[i] = nullptr;

		myBodyFrame->GetAndRefreshBodyData(myBodyCount, bodyArr);
		for (int i = 0; i < myBodyCount; i++)   //����6����(�����ò���)
		{
			BOOLEAN     result = false;
			if (bodyArr[i]->get_IsTracked(&result) == S_OK && result)   //�жϴ����Ƿ���⵽
			{
				cout << "Body " << i << " tracked!" << endl;

				int count = 0;
				Joint   jointArr[JointType_Count];
				bodyArr[i]->GetJoints(JointType_Count, jointArr);    //��ȡ���˵Ĺؽ�����
				for (int j = 0; j < JointType_Count; j++)
				{
					if (jointArr[j].TrackingState != TrackingState_Tracked) //��ȷ����⵽�Ĺؽ���ʾ����
						continue;
					string  rt = get_name(jointArr[j].JointType);   //��ȡ�ؽڵ�����
					if (rt != "NULL")   //����ؽ���Ϣ
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
		Sleep(1000);    //Ϊ��������ˢ̫�죬ÿ���Ӹ���һ��
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