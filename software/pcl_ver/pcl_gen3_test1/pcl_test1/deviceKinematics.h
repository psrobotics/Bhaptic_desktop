#ifndef deviceKinematics_H
#define deviceKinematics_H

class deviceKinematics
{
private:
	struct hapticCod
	{
		double endX;
		double endY;
		double endZ;
	};
	typedef struct hapticCod hapticCod;

	struct hapticRad
	{
		double radBase;
		double radArm1;
		double radArm2;
	};
	typedef struct hapticRad hapticRad;

	struct hapticHardware
	{
		double lengthBase;
		double lengthArm1;
		double lengthArm2;
	};
	typedef hapticHardware hapticHardware;

	struct forceV
	{
		double vx;
		double vy;
		double vz;
	};
	typedef forceV forceV;
	//力向量
	

	struct hapticTorque
	{
		double baseT;
		double arm1T;
		double arm2T;
	};
	typedef hapticTorque hapticTorque;

	//设备分界力向量组
	const double PI = 3.1415926535;

public:

	hapticCod deviceCod;
	hapticRad deviceRad;
	hapticHardware deviceHardware;
	forceV endForce;
	hapticTorque deviceTorque;
	int rawAngle[3];
	double offsetAngle[3] = { -119.883,264.551,-1 * (474.346 -21) };
	double deviceAngle[3];

	int motorTorque[3] = { 0,0,0 };

	int initLinkL(double l1, double l2, double l3);
	int printData(hapticCod *cod, hapticRad *rad);
	int hapticFk3D(hapticCod *result, hapticRad *rad, hapticHardware *device1);
	int hapticID3D(hapticCod *cod, hapticRad *rad, hapticHardware *device1, forceV *force, hapticTorque *torque);
	int rawToAngle();
	int rawToTorque(double motorK);
};

#endif