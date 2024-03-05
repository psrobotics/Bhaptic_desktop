#include "deviceKinematics.h"
#include <stdlib.h>
#include <cmath>
#include <algorithm>
#include <iostream>

using namespace std;

int deviceKinematics::initLinkL(double l1, double l2, double l3)
{
	deviceHardware.lengthBase = l1;
	deviceHardware.lengthArm1 = l2;
	deviceHardware.lengthArm2 = l3;
	return 0;
}

int deviceKinematics::hapticFk3D(hapticCod *result, hapticRad *rad, hapticHardware *device1)
{
	double l1 = device1->lengthBase;
	double l2 = device1->lengthArm1;
	double l3 = device1->lengthArm2;

	double a1 = rad->radBase;
	double a2 = rad->radArm1;
	double a3 = rad->radArm2;

	double x1 = l2 * cos(a1)*cos(a2) + l3 * cos(a1)*cos(a2)*cos(a3) - l3 * cos(a1)*sin(a2)*sin(a3);
	double y1 = l2 * cos(a2)*sin(a1) + l3 * cos(a2)*cos(a3)*sin(a1) - l3 * sin(a1)*sin(a2)*sin(a3);
	double z1 = l1 + l2 * sin(a2) + l3 * cos(a2)*sin(a3) + l3 * cos(a3)*sin(a2);

	result->endX = x1;
	result->endY = y1;
	result->endZ = z1;

	return 0;
}

int deviceKinematics::hapticID3D(hapticCod *cod, hapticRad *rad, hapticHardware *device1, forceV *force, hapticTorque *torque)
{

	double l1 = device1->lengthBase; double l2 = device1->lengthArm1; double l3 = device1->lengthArm2;

	double a1 = rad->radBase; double a2 = rad->radArm1; double a3 = rad->radArm2;

	double fx = force->vx; double fy = force->vy; double fz = force->vz;

	double t1 = (fy*cos(a1)) / (l2*pow(cos(a1), 2)*cos(a2) + l2 * cos(a2)*pow(sin(a1), 2) + l3 * pow(cos(a1), 2)*cos(a2)*cos(a3) + l3 * cos(a2)*cos(a3)*pow(sin(a1), 2) - l3 * pow(cos(a1), 2)*sin(a2)*sin(a3) - l3 * pow(sin(a1), 2)*sin(a2)*sin(a3)) - (fx*sin(a1)) / (l2*pow(cos(a1), 2)*cos(a2) + l2 * cos(a2)*pow(sin(a1), 2) + l3 * pow(cos(a1), 2)*cos(a2)*cos(a3) + l3 * cos(a2)*cos(a3)*pow(sin(a1), 2) - l3 * pow(cos(a1), 2)*sin(a2)*sin(a3) - l3 * pow(sin(a1), 2)*sin(a2)*sin(a3));
	double t2 = (fz*(cos(a2)*sin(a3) + cos(a3)*sin(a2))) / (l2*pow(cos(a2),2)*sin(a3) + l2 * pow(sin(a2),2)*sin(a3)) - (fx*cos(a1)*(sin(a2)*sin(a3) - cos(a2)*cos(a3))) / (l2*pow(sin(a1), 2)*pow(sin(a2), 2)*sin(a3) + l2 * pow(cos(a1), 2)*pow(cos(a2), 2)*sin(a3) + l2 * pow(cos(a1), 2)*pow(sin(a2), 2)*sin(a3) + l2 * pow(cos(a2), 2)*pow(sin(a1), 2)*sin(a3)) - (fy*sin(a1)*(sin(a2)*sin(a3) - cos(a2)*cos(a3))) / (l2*pow(sin(a1), 2)*pow(sin(a2), 2)*sin(a3) + l2 * pow(cos(a1), 2)*pow(cos(a2), 2)*sin(a3) + l2 * pow(cos(a1), 2)*pow(sin(a2), 2)*sin(a3) + l2 * pow(cos(a2), 2)*pow(sin(a1), 2)*sin(a3));
	double t3 = -(fx*(l2*cos(a1)*cos(a2) + l3 * cos(a1)*cos(a2)*cos(a3) - l3 * cos(a1)*sin(a2)*sin(a3))) / (l2*l3*pow(cos(a1), 2)*pow(cos(a2), 2)*sin(a3) + l2 * l3*pow(cos(a1), 2)*pow(sin(a2), 2)*sin(a3) + l2 * l3*pow(cos(a2), 2)*pow(sin(a1), 2)*sin(a3) + l2 * l3*pow(sin(a1), 2)*pow(sin(a2), 2)*sin(a3)) - (fy*(l2*cos(a2)*sin(a1) + l3 * cos(a2)*cos(a3)*sin(a1) - l3 * sin(a1)*sin(a2)*sin(a3))) / (l2*l3*pow(cos(a1), 2)*pow(cos(a2), 2)*sin(a3) + l2 * l3*pow(cos(a1), 2)*pow(sin(a2), 2)*sin(a3) + l2 * l3*pow(cos(a2), 2)*pow(sin(a1), 2)*sin(a3) + l2 * l3*pow(sin(a1), 2)*pow(sin(a2), 2)*sin(a3)) - (fz*(l2*sin(a2) + l3 * cos(a2)*sin(a3) + l3 * cos(a3)*sin(a2))) / (l2*l3*pow(cos(a2), 2)*sin(a3) + l2 * l3*pow(sin(a2), 2)*sin(a3));

	torque->baseT = t1; torque->arm1T = t2; torque->arm2T = t3;
	return 0;
}

int deviceKinematics::rawToAngle()
{
	double angleTemp[3];
	for (int n = 0; n < 3; n++)
		angleTemp[n] = (rawAngle[n] / 4096.0f) * 360;
	deviceAngle[0] = angleTemp[0];
	deviceAngle[1] = angleTemp[1];
	deviceAngle[2] = angleTemp[1] + angleTemp[2];
	
	deviceAngle[0] += offsetAngle[0];
	deviceAngle[1] = offsetAngle[1] - deviceAngle[1];
	deviceAngle[2] += offsetAngle[2];

	deviceRad.radBase = -1*(deviceAngle[0] / 180 * PI);
	deviceRad.radArm1 = deviceAngle[1] / 180 * PI;
	deviceRad.radArm2 = deviceAngle[2] / 180 * PI;

	return 0;
}

int deviceKinematics::rawToTorque(double motorK)
{
	motorTorque[0] = int(deviceTorque.baseT*motorK);
	motorTorque[1] = int(deviceTorque.arm1T*motorK);
	motorTorque[2] = int(deviceTorque.arm2T*motorK);

	for (int n = 0; n < 3; n++)
	{
		if (motorTorque[n] > 255)
			motorTorque[n] = 255;
		if (motorTorque[n] < -255)
			motorTorque[n] = -255;
	}

	return 0;
}

int deviceKinematics::printData(hapticCod *cod, hapticRad *rad)
{
	cout << "设备末端坐标为" << endl;
	cout << "x: " << cod->endX << "  y: " << cod->endY << "  z: " << cod->endZ << endl;
	cout << "设备关节角度为" << endl;
	cout << "base: " << rad->radBase;
	cout << "  arm1: " << rad->radArm1;
	cout << "  arm2: " << rad->radArm2 << endl;
	cout << endl;
	return 0;
}


