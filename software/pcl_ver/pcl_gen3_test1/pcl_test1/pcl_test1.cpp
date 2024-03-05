#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h> 

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>//kdtree搜索

#include <fstream>  
#include <string>  
#include <vector> 

#include "deviceKinematics.h"
#include "SerialPort.h"

#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

using namespace std;
//RED X axis, GREEN Y axis, BLUE z axis

char portName[] = "\\\\.\\COM3";
char portName1[] = "\\\\.\\COM4";
int dataNum = 17;
int dataNumSent = 30;
SerialPort *deviceSerial;
SerialPort *deviceSerial1;

struct hipVector
{
	double codX;
	double codY;
	double codZ;
	double length;
};

struct pointOffset
{
	double offsetX;
	double offsetY;
	double offsetZ;
};

int readPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double multipleK, const char *fileName, pointOffset *offset)
{
	struct point3D
	{
		double x;  //mm world coordinate x  
		double y;  //mm world coordinate y  
		double z;  //mm world coordinate z  
	};

	int pointNumRead;
	FILE *pointTxt;
	point3D pointRead;
	vector<point3D> pointCombine;
	pointTxt = fopen(fileName, "r");
	if (pointTxt)
	{
		double a, b, c;//文件点法向量，不使用
		while (fscanf(pointTxt, "%lf %lf %lf %lf %lf %lf", &pointRead.x, &pointRead.y, &pointRead.z, &a, &b, &c) != EOF)
			pointCombine.push_back(pointRead);
	}
	else
		cout << "点云文件加载失败" << endl;
	pointNumRead = pointCombine.size();
	cout << "文件包含点：" << pointNumRead << endl;

	cloud->width = pointNumRead;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width);
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = pointCombine[i].x * multipleK + offset->offsetX;
		cloud->points[i].y = pointCombine[i].y * multipleK + offset->offsetY;
		cloud->points[i].z = pointCombine[i].z * multipleK + offset->offsetZ;
	}
	return 0;
}

int serialWrite(int *data)
{
	char *buffer = new char[dataNumSent] {'\0'};
	sprintf_s(buffer, dataNumSent, "h%d %d %di", *(data), *(data + 1), *(data + 2));
	int state= deviceSerial1->writeSerialPort(buffer, strlen(buffer));
	cout << buffer << endl << endl;
	if (state)
		cout << "data sent" << endl;
	else
		cout << "data sent fail" << endl;
	return 0;
}

int serialRead(int *data)
{
	int readNewData = 0;
	while (!readNewData)
	{
		char *buffer = new char[dataNum];
		int state = deviceSerial->readSerialPort(buffer, dataNum);
		//cout << "buffer "<<buffer <<"r "<<state<< endl;
		char *start = buffer;
		int count = 0;
		int countLen = 0;
		int readFlag = 0;
		char input[30] = { '\0' };

		while (*start != 'a'&&count < dataNum)
		{
			start++;
			count++;
		}
		while (countLen < dataNum - count)
		{
			if (*start == 'e')
			{
				readFlag = 1;
				start -= countLen;
				break;
			}
			start++;
			countLen++;
		}
		if (readFlag)
		{
			readNewData = 1;
			for (int n = 0; n < countLen; n++)
				*(input + n) = *(start + n);
			//cout <<"input" <<input << endl;
			sscanf_s(input, "a%d %d %de", data, data + 1, data + 2);
			printf("%d,  %d,  %d\n", *data, *(data + 1), *(data + 2));
		}
		delete[]buffer;
	}
	return 0;
}


int main()
{
	deviceSerial = new SerialPort(portName);
	deviceSerial1 = new SerialPort(portName1);

	pointOffset offset;

	vector<hipVector> hipEndBall;//末端距离范围内力向量组

	double hipK = 0.4;//弹性系数
	double motorK = 40;//电机K

	//srand((unsigned int)time(NULL));

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSearch(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDeviceEnd(new pcl::PointCloud<pcl::PointXYZ>);

	offset.offsetX = 1.5;
	offset.offsetY = 1.5;
	offset.offsetZ = 1.5;

	readPointCloud(cloud, 500, "horse.xyz", &offset);
	//readPointCloud(cloud, 500, "hand.xyz", &offset);

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;//kdtree初始化
	kdtree.setInputCloud(cloud);//导入被搜索点云
	pcl::PointXYZ searchPoint;//末端参照点

	vector<int> pointIdSearch;//搜寻点标号
	vector<float> pointDistance;//搜寻点距离

	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	vtkObject::GlobalWarningDisplayOff();//关闭vtk弹窗警告

	deviceKinematics hapticDevice;//定义触反馈设备类
	hapticDevice.initLinkL(22.5/10,230/10,210/10);//mm

	double radius = 3;//搜索半径
	int pointSearchedNum = 0;

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloudSearch_color_handler(cloudSearch, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloudDeviceEnd_color_handler(cloudDeviceEnd, 0, 255, 0);
	//搜索后点颜色修改
	viewer.addPointCloud(cloud, "cloud");
	viewer.addPointCloud(cloudSearch, cloudSearch_color_handler, "cloudSearch");//选中点染色
	viewer.addPointCloud(cloudDeviceEnd, cloudDeviceEnd_color_handler, "cloudDeviceEnd");

	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloudSearch");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20, "cloudDeviceEnd");

	viewer.setBackgroundColor(0, 0, 0);
	viewer.addCoordinateSystem(20.0, "coordinate1", 0);//添加坐标系统

	//double fp1 = 0, fp2 = 0, fp3 = 0;
	double dataRead[3] = { 3.1415f/4,0,-3.1415f / 6 };//串口读取数据，设备角度数据
	double dataSent[3] = { 0 };//串口发送数据，电机电流
	int read_temp[3] = { 0 };
	int sent_temp[3] = { 255,-10,10 };

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();

		if (deviceSerial->isConnected())//Serial event begin
		{
			serialRead(hapticDevice.rawAngle);//read device rad data
			//cout << "read raw" << endl;
			hapticDevice.rawToAngle();
			//cout << hapticDevice.rawAngle[0] << " " << hapticDevice.rawAngle[1] << " " << hapticDevice.rawAngle[2] << endl;
			//cout << hapticDevice.deviceAngle[0] << " " << hapticDevice.deviceAngle[1] << " " << hapticDevice.deviceAngle[2] << endl;
			serialWrite(hapticDevice.motorTorque);
			//serialWrite(temp);
		}

		/*cout << hapticDevice.deviceRad.radArm1 << endl;
		cout << hapticDevice.deviceRad.radArm2 << endl;
		cout << hapticDevice.deviceRad.radBase << endl;*/

		hapticDevice.hapticFk3D(&hapticDevice.deviceCod, &hapticDevice.deviceRad, &hapticDevice.deviceHardware);

		searchPoint.x = hapticDevice.deviceCod.endX;
		searchPoint.y = hapticDevice.deviceCod.endY;
		searchPoint.z = hapticDevice.deviceCod.endZ;

		//cout <<"end pos"<< hapticDevice.deviceCod.endX << endl;

		cloudDeviceEnd->points.clear();
		cloudDeviceEnd->points.push_back(searchPoint);

		pointIdSearch.clear();
		pointDistance.clear();//清空暂存vector类
		hipVector hipVectorTem;
		hipEndBall.clear();//清空暂存vector类

		//pointSearchedNum = kdtree.nearestKSearch(searchPoint, Kin, pointIdSearch, pointDistance);
		pointSearchedNum = kdtree.radiusSearch(searchPoint, radius, pointIdSearch, pointDistance);
		//cout << "找到点数目：" << pointSearchedNum << endl;

		cloudSearch->points.clear();
		//cout << "nums x ori " << cloudSearch->points.size() << endl << endl;

		for (int n = 0; n < pointSearchedNum; n++)
		{
			int pointNum = pointIdSearch[n];
			cloudSearch->points.push_back(cloud->points[pointNum]);
		}//搜索后的点导入新点云

		if (pointSearchedNum)//在末端范围内有点情况下
		{
			double sumX = 0, sumY = 0, sumZ = 0;
			for (int n = 0; n < pointSearchedNum; n++)
			{
				hipVectorTem.codX = searchPoint.x - cloudSearch->points[n].x;
				hipVectorTem.codY = searchPoint.y - cloudSearch->points[n].y;
				hipVectorTem.codZ = searchPoint.z - cloudSearch->points[n].z;
				hipVectorTem.length = sqrt(pow(hipVectorTem.codX, 2) + pow(hipVectorTem.codY, 2) + pow(hipVectorTem.codZ, 2));
				sumX += hipVectorTem.codX;
				sumY += hipVectorTem.codY;
				sumZ += hipVectorTem.codZ;
				hipEndBall.push_back(hipVectorTem);//储存计算完毕的向量结构
			}

			double hipVectorLength = sqrt(pow(sumX, 2) + pow(sumY, 2) + pow(sumZ, 2));
			//hapticDevice.hipForceVector.vHip.force = hipK * hipVectorLength;
			hapticDevice.endForce.vx = sumX * hipK;
			hapticDevice.endForce.vy = sumY * hipK;
			hapticDevice.endForce.vz = sumZ * hipK;

			//cout << "vforce " << hapticDevice.endForce.vx << " " << hapticDevice.endForce.vy << " " << hapticDevice.endForce.vz << endl;

			/*cout << "x:" << hapticDevice.hipForceVector.vHip.vx << "  ";
			cout << "y:" << hapticDevice.hipForceVector.vHip.vy << "  ";
			cout << "z:" << hapticDevice.hipForceVector.vHip.vz << "  ";
			cout << "f:" << hapticDevice.hipForceVector.vHip.force << endl;*/
		}
		else//末端范围内无点情况下
		{
			hapticDevice.endForce.vx = 0;
			hapticDevice.endForce.vy = 0;
			hapticDevice.endForce.vz = 0;
		}

		//逆向运动学
		hapticDevice.hapticID3D(&hapticDevice.deviceCod, &hapticDevice.deviceRad, &hapticDevice.deviceHardware, &hapticDevice.endForce, &hapticDevice.deviceTorque);
		//cout << "vtorque " << hapticDevice.deviceTorque.baseT << " " << hapticDevice.deviceTorque.arm1T << " " << hapticDevice.deviceTorque.arm2T << endl;

		hapticDevice.rawToTorque(motorK);
		//cout << "Mtorque " << hapticDevice.motorTorque[0] << " " << hapticDevice.motorTorque[1] << " " << hapticDevice.motorTorque[2] << endl;

		viewer.updatePointCloud(cloudSearch, cloudSearch_color_handler, "cloudSearch");//更新选中点云
		viewer.updatePointCloud(cloudDeviceEnd, cloudDeviceEnd_color_handler, "cloudDeviceEnd");//更新设备末端点
	}
	system("pause");
	return 0;
}
