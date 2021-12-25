#pragma once
#include <Eigen/core>
#include <Eigen/Geometry>
#include <Eigen/dense>


class Movable
{
public:
	Movable();
	Movable(const Movable& mov);
	Eigen::Matrix4f MakeTransScale();
	Eigen::Matrix4d MakeTransd();
	Eigen::Matrix4d MakeTransScaled();
	Eigen::Matrix3d GetRotation() { return Tout.rotation(); }
	void MyTranslate(Eigen::Vector3d amt, bool preRotation);
	void TranslateInSystem(Eigen::Matrix3d rot,Eigen::Vector3d amt);
	void MyRotate(Eigen::Vector3d rotAxis, double angle);
	void RotateInSystem(Eigen::Vector3d rotAxis, double angle);
	void MyRotate(const Eigen::Matrix3d &rot);
	void MyScale(Eigen::Vector3d amt);
	void SetCenter(Eigen::Vector3d amt);
	Eigen::Vector3d GetCenter();
	Eigen::Matrix3d GetRotation() const{ return Tout.rotation().matrix(); }

	void RotateZByEuler(double delta);
	void RotateXByEuler(double delta);
	Eigen::Matrix3d buildZEuler(double delta);
	Eigen::Matrix3d buildXEuler(double delta);
	void initEuler();
	void printA1();




	virtual ~Movable() {}
private:
	Eigen::Affine3d Tout,Tin;
	Eigen::Matrix3d A1, A2, A3;
};

