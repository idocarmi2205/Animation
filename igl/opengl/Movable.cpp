#include "Movable.h"
#include "Movable.h"
#include "Movable.h"
#include "Movable.h"
#include "Movable.h"
#include "Movable.h"
#include <iostream>
Movable::Movable()
{
	Tout = Eigen::Affine3d::Identity();
	Tin = Eigen::Affine3d::Identity();
	/////////////////////////////////////////
	A1 = Eigen::MatrixXd(buildZEuler(0));
	//printA1();

	A2 = buildXEuler(0);
	A3 = buildZEuler(0);
	//printA1();

	//////////////////////////////////////////

}

Movable::Movable(const Movable& mov)
{
	Tout = mov.Tout;
	Tin = mov.Tin;
}

Eigen::Matrix4f Movable::MakeTransScale()
{
	return (Tout.matrix() * Tin.matrix()).cast<float>();
}

Eigen::Matrix4d Movable::MakeTransScaled()
{
	return (Tout.matrix() * Tin.matrix());
}

Eigen::Matrix4d Movable::MakeTransd()
{
	Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
	mat.col(3) << Tin.translation(), 1;

	return (Tout.matrix() * mat);
}


void Movable::MyTranslate(Eigen::Vector3d amt, bool preRotation)
{

	if (preRotation)
		Tout.pretranslate(amt);
	else
		Tout.translate(amt);
}

void Movable::TranslateInSystem(Eigen::Matrix3d rot, Eigen::Vector3d amt)
{
	Tout.pretranslate(rot.transpose() * amt);
}
//angle in radians
void Movable::MyRotate(Eigen::Vector3d rotAxis, double angle)
{
	Tout.rotate(Eigen::AngleAxisd(angle, rotAxis.normalized()));
}

void Movable::RotateInSystem(Eigen::Vector3d rotAxis, double angle)
{
	Tout.rotate(Eigen::AngleAxisd(angle, Tout.rotation().transpose() * (rotAxis.normalized())));
}

void Movable::MyRotate(const Eigen::Matrix3d& rot)
{
	Tout.rotate(rot);
}

void Movable::MyScale(Eigen::Vector3d amt)
{
	Tin.scale(amt);
}

void Movable::SetCenter(Eigen::Vector3d amt) {
	Tin.translate(-amt);
	Tout.translate(amt);
}

Eigen::Vector3d Movable::GetCenter() {
	return -Tin.translation();
}

void Movable::RotateZByEuler(double delta)
{
	//printA1();
	Eigen::Matrix3d deltaRot = buildZEuler(delta);
	//A1 = buildZEuler(0);
	//A2 = buildXEuler(0);
	//A3 = buildZEuler(0);
	//printf("A1 before rot:\n %f \t %f\t%f\t\n%f \t %f\t%f\t\n%f \t %f\t%f\t\n", A1(0, 0), (A1)(0, 1), (A1)(0, 2), (A1)(1, 0), (A1)(1, 1), (A1)(1, 2), (A1)(2, 0), (A1)(2, 1), (A1)(2, 2));
	A1 = Eigen::Matrix3d((A1)*deltaRot);
	//printf("A1 after rot:\n %f \t %f\t%f\t\n%f \t %f\t%f\t\n%f \t %f\t%f\t\n", (A1)(0, 0), (A1)(0, 1), (A1)(0, 2), (A1)(1, 0), (A1)(1, 1), (A1)(1, 2), (A1)(2, 0), (A1)(2, 1), (A1)(2, 2));
	Eigen::Matrix3d A = (A1) * (A2) * (A3);
	MyRotate(A);
	deltaRot = GetRotation();
	printf("rotation after rot:\n %f \t %f\t%f\t\n%f \t %f\t%f\t\n%f \t %f\t%f\t\n", deltaRot(0, 0), deltaRot(0, 1), deltaRot(0, 2), deltaRot(1, 0), deltaRot(1, 1), deltaRot(1, 2), deltaRot(2, 0), deltaRot(2, 1), deltaRot(2, 2));

}



void Movable::RotateXByEuler(double delta)
{
	//Eigen::Matrix3d deltaRot = buildZEuler(delta);
	//A1 = A1 * deltaRot;
	//A3 = A3 * deltaRot;

	Eigen::Matrix3d deltaRot = Eigen::Matrix3d(buildXEuler(delta));
	A2 = Eigen::Matrix3d((A2)*deltaRot);

	MyRotate(A1 * A2 * A3);
}

Eigen::Matrix3d Movable::buildZEuler(double delta)
{
	Eigen::Matrix3d z(3, 3);
	z <<
		cos(delta), -sin(delta), 0,
		sin(delta), cos(delta), 0,
		0, 0, 1;
	//printf("ZZZZ:\n%f \t %f\t%f\t\n%f \t %f\t%f\t\n%f \t %f\t%f\t\n", z(0, 0), z(0, 1), z(0, 2), z(1, 0), z(1, 1), z(1, 2), z(2, 0), z(2, 1), z(2, 2));

	return z;
}

Eigen::Matrix3d Movable::buildXEuler(double delta)
{
	Eigen::Matrix3d x(3, 3);
	x <<
		1, 0, 0,
		0, cos(delta), -sin(delta),
		0, sin(delta), cos(delta);

	return x;
}

void Movable::initEuler()
{
	//A1 = buildZEuler(0);
	//A2 = buildXEuler(0);
	//A3 = buildZEuler(0);
}

void Movable::printA1()
{
	printf("A1:\n %f \t %f\t%f\t\n%f \t %f\t%f\t\n%f \t %f\t%f\t\n", (A1)(0, 0), (A1)(0, 1), (A1)(0, 2), (A1)(1, 0), (A1)(1, 1), (A1)(1, 2), (A1)(2, 0), (A1)(2, 1), (A1)(2, 2));

}





























//void Movable::TranslateInSystem(Eigen::Matrix4d Mat, Eigen::Vector3d amt, bool preRotation)
//{
//	Eigen::Vector3d v = Mat.transpose().block<3, 3>(0, 0) * amt; //transpose instead of inverse
//	MyTranslate(v, preRotation);
//}
//
//void Movable::RotateInSystem(Eigen::Matrix4d Mat, Eigen::Vector3d rotAxis, double angle)
//{
//	Eigen::Vector3d v = Mat.transpose().block<3, 3>(0, 0) * rotAxis; //transpose instead of inverse
//	MyRotate(v.normalized(), angle);
//}
//
//
//void Movable::SetCenterOfRotation(Eigen::Vector3d amt)
//{
//	Tout.pretranslate(Tout.rotation().matrix().block<3, 3>(0, 0) * amt);
//	Tin.translate(-amt);
//}
//
//Eigen::Vector3d Movable::GetCenterOfRotation()
//{
//	return Tin.translation();
//}
