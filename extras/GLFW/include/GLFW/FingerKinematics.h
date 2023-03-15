//
// Created by SuYuan on 2022/10/15
//


#ifndef _FINGER_KINEMATICS_
#define _FINGER_KINEMATICS_

#include "Eigen/Eigen"
#include <math.h>
#include <iostream>
#define PI 3.1415926

namespace chai3d
{
	namespace FingerMath
	{

		Eigen::Matrix4f T_10;		//1 relative 0
		Eigen::Matrix4f T_21;		//2 relative 1
		Eigen::Matrix4f T_32;
		Eigen::Matrix4f T_43;
		Eigen::Matrix4f T_54;
		Eigen::Matrix4f T_65;
		Eigen::Matrix4f T_tip_6;

		float L1;
		float L2;
		float L3;
		float L4;
		float L5;

		Eigen::Matrix4f T_60;
		Eigen::Matrix4f T_tip_0;
		Eigen::Matrix4f T_tip_W;

		Eigen::Matrix4f T_0W;					//device frame{0} relative world frame{W}
		Eigen::Matrix4f T_finger_0W;			//finger frame{0} relative world frame{W}
		Eigen::Matrix4f T_finger_0W_inv;		//inverse
		Eigen::Matrix4f T_finger_tip_0;

		Eigen::Matrix3f Rx;
		Eigen::Matrix3f Ry;
		Eigen::Matrix3f Rz;

		Eigen::Matrix3f Rxyz;

		//calculate single finger joint angles
		float e1;
		float d1;
		float d2;
		float a;
		float b;

		Eigen::Vector4f finger_joint_angle;

		Eigen::Matrix3f getRx(float theta) {
			Rx(0, 0) = 1;
			Rx(0, 1) = 0;
			Rx(0, 2) = 0;
			Rx(1, 0) = 0;
			Rx(1, 1) = cos(theta * PI / 180);
			Rx(1, 2) = (-1) * sin(theta * PI / 180);
			Rx(2, 0) = 0;
			Rx(2, 1) = sin(theta * PI / 180);
			Rx(2, 2) = cos(theta * PI / 180);
			return Rx;
		}

		Eigen::Matrix3f getRy(float theta) {
			Ry(0, 0) = cos(theta * PI / 180);
			Ry(0, 1) = 0;
			Ry(0, 2) = sin(theta * PI / 180);
			Ry(1, 0) = 0;
			Ry(1, 1) = 1;
			Ry(1, 2) = 0;
			Ry(2, 0) = (-1) * sin(theta * PI / 180);
			Ry(2, 1) = 0;
			Ry(2, 2) = cos(theta * PI / 180);
			return Ry;
		}

		Eigen::Matrix3f getRz(float theta) {
			Rz(0, 0) = cos(theta * PI / 180);
			Rz(0, 1) = (-1) * sin(theta * PI / 180);
			Rz(0, 2) = 0;
			Rz(1, 0) = sin(theta * PI / 180);
			Rz(1, 1) = cos(theta * PI / 180);
			Rz(1, 2) = 0;
			Rz(2, 0) = 0;
			Rz(2, 1) = 0;
			Rz(2, 2) = 1;
			return Rz;
		}

		Eigen::Matrix4f getTransMatrix(float& theta1, float& theta2, float& theta3, float& theta4, float& theta5, float& theta6)
		{
			//
			theta1 = (-1) * theta1;
			theta2 = (-1) * theta2;
			theta3 = (-1) * theta3;
			theta4 = (-1) * theta4;
			theta5 = theta5;
			theta6 = (-1) * theta6;

			//
			theta1 = theta1 - 78.97;
			theta2 = theta2 + 180;
			theta3 = theta3 + 90;
			theta4 = theta4;
			theta5 = theta5 - 90;
			theta6 = theta6 - 70;

			//
			L1 = 74.01;
			L2 = 33.30;
			L3 = 43.80;
			L5 = 12.99;

			//
			T_10(0, 0) = cos(theta1 * PI / 180);
			T_10(0, 1) = (-1) * sin(theta1 * PI / 180);
			T_10(0, 2) = 0;
			T_10(0, 3) = 0;
			T_10(1, 0) = sin(theta1 * PI / 180) * cos(0 * PI / 180);
			T_10(1, 1) = cos(theta1 * PI / 180) * cos(0 * PI / 180);
			T_10(1, 2) = (-1) * sin(0 * PI / 180);
			T_10(1, 3) = (-1) * sin(0 * PI / 180) * 0;
			T_10(2, 0) = sin(theta1 * PI / 180) * sin(0 * PI / 180);
			T_10(2, 1) = cos(theta1 * PI / 180) * sin(0 * PI / 180);
			T_10(2, 2) = cos(0 * PI / 180);
			T_10(2, 3) = cos(0 * PI / 180) * 0;
			T_10(3, 0) = 0;
			T_10(3, 1) = 0;
			T_10(3, 2) = 0;
			T_10(3, 3) = 1;
			//
			T_21(0, 0) = cos(theta2 * PI / 180);
			T_21(0, 1) = (-1) * sin(theta2 * PI / 180);
			T_21(0, 2) = 0;
			T_21(0, 3) = L1;											//73.9
			T_21(1, 0) = sin(theta2 * PI / 180) * cos(0 * PI / 180);
			T_21(1, 1) = cos(theta2 * PI / 180) * cos(0 * PI / 180);
			T_21(1, 2) = (-1) * sin(0 * PI / 180);
			T_21(1, 3) = (-1) * sin(0 * PI / 180) * 0;
			T_21(2, 0) = sin(theta2 * PI / 180) * sin(0 * PI / 180);
			T_21(2, 1) = cos(theta2 * PI / 180) * sin(0 * PI / 180);
			T_21(2, 2) = cos(0 * PI / 180);
			T_21(2, 3) = cos(0 * PI / 180) * 0;
			T_21(3, 0) = 0;
			T_21(3, 1) = 0;
			T_21(3, 2) = 0;
			T_21(3, 3) = 1;
			//
			T_32(0, 0) = cos(theta3 * PI / 180);
			T_32(0, 1) = (-1) * sin(theta3 * PI / 180);
			T_32(0, 2) = 0;
			T_32(0, 3) = 0;
			T_32(1, 0) = sin(theta3 * PI / 180) * cos(90 * PI / 180);
			T_32(1, 1) = cos(theta3 * PI / 180) * cos(90 * PI / 180);
			T_32(1, 2) = (-1) * sin(90 * PI / 180);
			T_32(1, 3) = (-1) * sin(90 * PI / 180) * L2;
			T_32(2, 0) = sin(theta3 * PI / 180) * sin(90 * PI / 180);
			T_32(2, 1) = cos(theta3 * PI / 180) * sin(90 * PI / 180);
			T_32(2, 2) = cos(90 * PI / 180);
			T_32(2, 3) = cos(90 * PI / 180) * L2;
			T_32(3, 0) = 0;
			T_32(3, 1) = 0;
			T_32(3, 2) = 0;
			T_32(3, 3) = 1;
			//
			T_43(0, 0) = cos(theta4 * PI / 180);
			T_43(0, 1) = (-1) * sin(theta4 * PI / 180);
			T_43(0, 2) = 0;
			T_43(0, 3) = 0;
			T_43(1, 0) = sin(theta4 * PI / 180) * cos(60 * PI / 180);
			T_43(1, 1) = cos(theta4 * PI / 180) * cos(60 * PI / 180);
			T_43(1, 2) = (-1) * sin(60 * PI / 180);
			T_43(1, 3) = (-1) * sin(60 * PI / 180) * L3;
			T_43(2, 0) = sin(theta4 * PI / 180) * sin(60 * PI / 180);
			T_43(2, 1) = cos(theta4 * PI / 180) * sin(60 * PI / 180);
			T_43(2, 2) = cos(60 * PI / 180);
			T_43(2, 3) = cos(60 * PI / 180) * L3;
			T_43(3, 0) = 0;
			T_43(3, 1) = 0;
			T_43(3, 2) = 0;
			T_43(3, 3) = 1;
			//
			T_54(0, 0) = cos(theta5 * PI / 180);
			T_54(0, 1) = (-1) * sin(theta5 * PI / 180);
			T_54(0, 2) = 0;
			T_54(0, 3) = 0;
			T_54(1, 0) = sin(theta5 * PI / 180) * cos((-1) * 90 * PI / 180);
			T_54(1, 1) = cos(theta5 * PI / 180) * cos((-1) * 90 * PI / 180);
			T_54(1, 2) = (-1) * sin((-1) * 90 * PI / 180);
			T_54(1, 3) = (-1) * sin((-1) * 90 * PI / 180) * 0;
			T_54(2, 0) = sin(theta5 * PI / 180) * sin((-1) * 90 * PI / 180);
			T_54(2, 1) = cos(theta5 * PI / 180) * sin((-1) * 90 * PI / 180);
			T_54(2, 2) = cos((-1) * 90 * PI / 180);
			T_54(2, 3) = cos((-1) * 90 * PI / 180) * 0;
			T_54(3, 0) = 0;
			T_54(3, 1) = 0;
			T_54(3, 2) = 0;
			T_54(3, 3) = 1;
			//
			T_65(0, 0) = cos(theta6 * PI / 180);
			T_65(0, 1) = (-1) * sin(theta6 * PI / 180);
			T_65(0, 2) = 0;
			T_65(0, 3) = L5;
			T_65(1, 0) = sin(theta6 * PI / 180) * cos((-1) * 90 * PI / 180);
			T_65(1, 1) = cos(theta6 * PI / 180) * cos((-1) * 90 * PI / 180);
			T_65(1, 2) = (-1) * sin((-1) * 90 * PI / 180);
			T_65(1, 3) = (-1) * sin((-1) * 90 * PI / 180) * 0;
			T_65(2, 0) = sin(theta6 * PI / 180) * sin((-1) * 90 * PI / 180);
			T_65(2, 1) = cos(theta6 * PI / 180) * sin((-1) * 90 * PI / 180);
			T_65(2, 2) = cos((-1) * 90 * PI / 180);
			T_65(2, 3) = cos((-1) * 90 * PI / 180) * 0;
			T_65(3, 0) = 0;
			T_65(3, 1) = 0;
			T_65(3, 2) = 0;
			T_65(3, 3) = 1;

			//
			T_tip_6(0, 0) = 1;		T_tip_6(0, 1) = 0;		T_tip_6(0, 2) = 0;		T_tip_6(0, 3) = 20;
			T_tip_6(1, 0) = 0;		T_tip_6(1, 1) = 0;		T_tip_6(1, 2) = -1;		T_tip_6(1, 3) = 28;
			T_tip_6(2, 0) = 0;		T_tip_6(2, 1) = 1;		T_tip_6(2, 2) = 0;		T_tip_6(2, 3) = 0;
			T_tip_6(3, 0) = 0;		T_tip_6(3, 1) = 0;		T_tip_6(3, 2) = 0;		T_tip_6(3, 3) = 1;

			//
			T_60 = T_10 * T_21 * T_32 * T_43 * T_54 * T_65;
			T_tip_0 = T_60 * T_tip_6;

			return T_tip_0;
		}



		Eigen::Matrix4f getF1TransMatrix(float& theta1, float& theta2, float& theta3, float& theta4, float& theta5, float& theta6)
		{
			
			T_tip_0 = getTransMatrix(theta1, theta2, theta3, theta4, theta5, theta6);
			//std::cout << T_tip_0(0, 3) << "," << T_tip_0(1, 3) << "," << T_tip_0(2, 3) << std::endl;

			Rx(0, 0) = 1;
			Rx(0, 1) = 0;
			Rx(0, 2) = 0;
			Rx(1, 0) = 0;
			Rx(1, 1) = cos((-1) * 180 * PI / 180);
			Rx(1, 2) = (-1) * sin((-1) * 180 * PI / 180);
			Rx(2, 0) = 0;
			Rx(2, 1) = sin((-1) * 180 * PI / 180);
			Rx(2, 2) = cos((-1) * 180 * PI / 180);

			//
			Ry(0, 0) = cos((-1) * 45 * PI / 180);
			Ry(0, 1) = 0;
			Ry(0, 2) = sin((-1) * 45 * PI / 180);
			Ry(1, 0) = 0;
			Ry(1, 1) = 1;
			Ry(1, 2) = 0;
			Ry(2, 0) = (-1) * sin((-1) * 45 * PI / 180);
			Ry(2, 1) = 0;
			Ry(2, 2) = cos((-1) * 45 * PI / 180);

			//
			Rxyz = Rx * Ry;

			//
			T_0W(0, 0) = Rxyz(0, 0);	T_0W(0, 1) = Rxyz(0, 1);	T_0W(0, 2) = Rxyz(0, 2);	T_0W(0, 3) = (-1) * 5.50;
			T_0W(1, 0) = Rxyz(1, 0);	T_0W(1, 1) = Rxyz(1, 1);	T_0W(1, 2) = Rxyz(1, 2);	T_0W(1, 3) = 46.5;
			T_0W(2, 0) = Rxyz(2, 0);	T_0W(2, 1) = Rxyz(2, 1);	T_0W(2, 2) = Rxyz(2, 2);	T_0W(2, 3) = (-1) * 28.00;		//已查验
			T_0W(3, 0) = 0;				T_0W(3, 1) = 0;				T_0W(3, 2) = 0;				T_0W(3, 3) = 1;

			//
			T_tip_W = T_0W * T_tip_0;

			return T_tip_W;

		}

		Eigen::Matrix4f getF2TransMatrix(float& theta1, float& theta2, float& theta3, float& theta4, float& theta5, float& theta6)
		{
			
			T_tip_0 = getTransMatrix(theta1, theta2, theta3, theta4, theta5, theta6);

			//check T_tip_0
			//std::cout << T_tip_0(0, 3) << "," << T_tip_0(1, 3) << "," << T_tip_0(2, 3) << std::endl;

			Rx = getRx(-90);

			Rxyz = Rx;

			//
			T_0W(0, 0) = Rxyz(0, 0);	T_0W(0, 1) = Rxyz(0, 1);	T_0W(0, 2) = Rxyz(0, 2);	T_0W(0, 3) = 54.00;		//已检查 实物坐标54
			T_0W(1, 0) = Rxyz(1, 0);	T_0W(1, 1) = Rxyz(1, 1);	T_0W(1, 2) = Rxyz(1, 2);	T_0W(1, 3) = 21.00;		//已检查 实物坐标21
			T_0W(2, 0) = Rxyz(2, 0);	T_0W(2, 1) = Rxyz(2, 1);	T_0W(2, 2) = Rxyz(2, 2);	T_0W(2, 3) = 18.00;		//已检查 实物坐标18
			T_0W(3, 0) = 0;				T_0W(3, 1) = 0;				T_0W(3, 2) = 0;				T_0W(3, 3) = 1;

			//
			T_tip_W = T_0W * T_tip_0;

			return T_tip_W;

		}

		Eigen::Matrix4f getF3TransMatrix(float& theta1, float& theta2, float& theta3, float& theta4, float& theta5, float& theta6)
		{
			//
			T_tip_0 = getTransMatrix(theta1, theta2, theta3, theta4, theta5, theta6);
			//std::cout << T_tip_0(0, 3) << "," << T_tip_0(1, 3) << "," << T_tip_0(2, 3) << std::endl;

			//
			Rx = getRx(-90);

			Rxyz = Rx;

			//
			T_0W(0, 0) = Rxyz(0, 0);	T_0W(0, 1) = Rxyz(0, 1);	T_0W(0, 2) = Rxyz(0, 2);	T_0W(0, 3) = 54.00;
			T_0W(1, 0) = Rxyz(1, 0);	T_0W(1, 1) = Rxyz(1, 1);	T_0W(1, 2) = Rxyz(1, 2);	T_0W(1, 3) = (-1) * 24.00;		 //不窜动-24，攒动-25，误差1mm
			T_0W(2, 0) = Rxyz(2, 0);	T_0W(2, 1) = Rxyz(2, 1);	T_0W(2, 2) = Rxyz(2, 2);	T_0W(2, 3) = 18.00;
			T_0W(3, 0) = 0;				T_0W(3, 1) = 0;				T_0W(3, 2) = 0;				T_0W(3, 3) = 1;

			//
			T_tip_W = T_0W * T_tip_0;

			return T_tip_W;

		}






		//
		// Created by YuanSu on 01/02/23.
		// Hand model kinematic solution.
		//

		Eigen::Vector3f aux_xyz;
		
		///index finger	
		Eigen::Vector3f index_1_motion(float &theta1 , float &theta2)	
		{

			aux_xyz(0) = 0 + 0.45 * sin(theta2 * PI / 180);
			aux_xyz(1) = 0.23 + 0.45 * sin(theta1 * PI / 180);
			aux_xyz(2) = -0.05 + 0.45 * cos(theta2 * PI / 180);		
			return aux_xyz;
		}
		Eigen::Vector3f index_2_motion(float& theta1, float& theta2, float& theta3)
		{
	
			aux_xyz(0) = 0 + 0.45 * sin(theta2 * PI / 180) + 0.28 * sin((theta2 + theta3) * PI / 180);
			aux_xyz(1) = 0.23 + (0.45 + 0.28) * sin(theta1 * PI / 180);
			aux_xyz(2) = -0.05 + 0.45 * cos(theta2 * PI / 180) + 0.28 * cos((theta2 + theta3) * PI / 180);
		
			return aux_xyz;
		}
		Eigen::Vector3f index_3_motion(float&theta1, float& theta2, float& theta3, float& theta4)
		{

			aux_xyz(0) = 0 + 0.45 * sin(theta2 * PI / 180) + 0.28 * sin((theta2 + theta3) * PI / 180) + 0.21 * sin((theta2 + theta3 + theta4) * PI / 180);
			aux_xyz(1) = 0.23 + (0.45 + 0.28 + 0.21) * sin(theta1 * PI / 180);
			aux_xyz(2) = -0.05 + 0.45 * cos(theta2 * PI / 180) + 0.28 * cos((theta2 + theta3) * PI / 180) + 0.21 * cos((theta2 + theta3 + theta4) * PI / 180);
			
			return aux_xyz;
		}

		///middle finger
		Eigen::Vector3f middle_1_motion(float& theta1, float& theta2)
		{						
			aux_xyz(0) = 0 + 0.48 * sin(theta2 * PI / 180);
			aux_xyz(1) = 0 + 0.48 * sin(theta1 * PI / 180);;
			aux_xyz(2) = 0 + 0.48 * cos(theta2 * PI / 180);
							
			return aux_xyz;
		}
		Eigen::Vector3f middle_2_motion(float& theta1, float& theta2, float& theta3)
		{							
			aux_xyz(0) = 0 + 0.48 * sin(theta2 * PI / 180) + 0.32 * sin((theta2 + theta3) * PI / 180);
			aux_xyz(1) = 0 + (0.48 + 0.32) * sin(theta1 * PI / 180);
			aux_xyz(2) = 0 + 0.48 * cos(theta2 * PI / 180) + 0.32 * cos((theta2 + theta3) * PI / 180);
				
			return aux_xyz;
		}
		Eigen::Vector3f middle_3_motion(float& theta1, float& theta2, float& theta3, float& theta4)
		{
			aux_xyz(0) = 0 + 0.48 * sin(theta2 * PI / 180) + 0.32 * sin((theta2 + theta3) * PI / 180) + 0.21 * sin((theta2 + theta3 + theta4) * PI / 180);
			aux_xyz(1) = 0 + (0.48 + 0.32 + 0.21) * sin(theta1 * PI / 180);
			aux_xyz(2) = 0 + 0.48 * cos(theta2 * PI / 180) + 0.32 * cos((theta2 + theta3) * PI / 180) + 0.21 * cos((theta2 + theta3 + theta4) * PI / 180);
							
			return aux_xyz;
		}

		/*
		Eigen::Vector3f index_1_motion(float& theta2)
		{

			aux_xyz(0) = 0 + 0.45 * sin(theta2 * PI / 180);
			aux_xyz(1) = 0.27;
			aux_xyz(2) = -0.05 + 0.45 * cos(theta2 * PI / 180);
			return aux_xyz;
		}
		Eigen::Vector3f index_2_motion(float& theta2, float& theta3)
		{

			aux_xyz(0) = 0 + 0.45 * sin(theta2 * PI / 180) + 0.28 * sin((theta2 + theta3) * PI / 180);
			aux_xyz(1) = 0.28;
			aux_xyz(2) = -0.05 + 0.45 * cos(theta2 * PI / 180) + 0.28 * cos((theta2 + theta3) * PI / 180);

			return aux_xyz;
		}
		Eigen::Vector3f index_3_motion(float& theta2, float& theta3, float& theta4)
		{

			aux_xyz(0) = 0 + 0.45 * sin(theta2 * PI / 180) + 0.28 * sin((theta2 + theta3) * PI / 180) + 0.21 * sin((theta2 + theta3 + theta4) * PI / 180);
			aux_xyz(1) = 0.24;
			aux_xyz(2) = -0.05 + 0.45 * cos(theta2 * PI / 180) + 0.28 * cos((theta2 + theta3) * PI / 180) + 0.21 * cos((theta2 + theta3 + theta4) * PI / 180);

			return aux_xyz;
		}

		///middle finger
		Eigen::Vector3f middle_1_motion(float& theta2)
		{
			aux_xyz(0) = 0 + 0.48 * sin(theta2 * PI / 180);
			aux_xyz(1) = 0;
			aux_xyz(2) = 0 + 0.48 * cos(theta2 * PI / 180);

			return aux_xyz;
		}
		Eigen::Vector3f middle_2_motion(float& theta2, float& theta3)
		{
			aux_xyz(0) = 0 + 0.48 * sin(theta2 * PI / 180) + 0.32 * sin((theta2 + theta3) * PI / 180);
			aux_xyz(1) = 0;
			aux_xyz(2) = 0 + 0.48 * cos(theta2 * PI / 180) + 0.32 * cos((theta2 + theta3) * PI / 180);

			return aux_xyz;
		}
		Eigen::Vector3f middle_3_motion(float& theta2, float& theta3, float& theta4)
		{
			aux_xyz(0) = 0 + 0.48 * sin(theta2 * PI / 180) + 0.32 * sin((theta2 + theta3) * PI / 180) + 0.21 * sin((theta2 + theta3 + theta4) * PI / 180);
			aux_xyz(1) = 0;
			aux_xyz(2) = 0 + 0.48 * cos(theta2 * PI / 180) + 0.32 * cos((theta2 + theta3) * PI / 180) + 0.21 * cos((theta2 + theta3 + theta4) * PI / 180);

			return aux_xyz;
		}
		*/
		

		//
		Eigen::Vector3f thumb_1_motion(float& theta2)
		{	
			Ry = getRy(-90);
			Rz = getRz(45);
			Rx = getRx(-45);

			Rxyz = Ry * Rz * Rx;

			Eigen::Matrix4f T0W;
			T0W(0, 0) = Rxyz(0, 0);		T0W(0, 1) = Rxyz(0, 1);		T0W(0, 2) = Rxyz(0, 2);		T0W(0, 3) = 0;
			T0W(1, 0) = Rxyz(1, 0);		T0W(1, 1) = Rxyz(1, 1);		T0W(1, 2) = Rxyz(1, 2);		T0W(1, 3) = 0.19;
			T0W(2, 0) = Rxyz(2, 0);		T0W(2, 1) = Rxyz(2, 1);		T0W(2, 2) = Rxyz(2, 2);		T0W(2, 3) = -0.72;
			T0W(3, 0) = 0;				T0W(3, 1) = 0;				T0W(3, 2) = 0;				T0W(3, 3) = 1;
			Eigen::Matrix4f TP0;
			TP0(0, 0) = 1;				TP0(0, 1) = 0;				TP0(0, 2) = 0;				TP0(0, 3) = 0 + sqrt(0.2 * 0.2 + 0.32 * 0.32)*cos(theta2*PI/180);
			TP0(1, 0) = 0;				TP0(1, 1) = 1;				TP0(1, 2) = 0;				TP0(1, 3) = 0;
			TP0(2, 0) = 0;				TP0(2, 1) = 0;				TP0(2, 2) = 1;				TP0(2, 3) = 0 - sqrt(0.2 * 0.2 + 0.32 * 0.32)*sin(theta2*PI/180);
			TP0(3, 0) = 0;				TP0(3, 1) = 0;				TP0(3, 2) = 0;				TP0(3, 3) = 1;

			Eigen::Matrix4f TPW;
			TPW = T0W * TP0;

			aux_xyz(0) = TPW(0,3);
			aux_xyz(1) = TPW(1,3);
			aux_xyz(2) = TPW(2,3);

			return aux_xyz;
		}
		Eigen::Vector3f thumb_2_motion(float& theta2, float& theta3)
		{
			Ry = getRy(-90);
			Rz = getRz(45);
			Rx = getRx(-45);

			Rxyz = Ry * Rz * Rx;

			Eigen::Matrix4f T0W;
			T0W(0, 0) = Rxyz(0, 0);		T0W(0, 1) = Rxyz(0, 1);		T0W(0, 2) = Rxyz(0, 2);		T0W(0, 3) = 0;
			T0W(1, 0) = Rxyz(1, 0);		T0W(1, 1) = Rxyz(1, 1);		T0W(1, 2) = Rxyz(1, 2);		T0W(1, 3) = 0.19;
			T0W(2, 0) = Rxyz(2, 0);		T0W(2, 1) = Rxyz(2, 1);		T0W(2, 2) = Rxyz(2, 2);		T0W(2, 3) = -0.72;
			T0W(3, 0) = 0;				T0W(3, 1) = 0;				T0W(3, 2) = 0;				T0W(3, 3) = 1;
			Eigen::Matrix4f TP0;
			TP0(0, 0) = 1;				TP0(0, 1) = 0;				TP0(0, 2) = 0;				TP0(0, 3) = 0 + sqrt(0.2 * 0.2 + 0.32 * 0.32) * cos(theta2 * PI / 180) + sqrt(0.14 * 0.14 + 0.34 * 0.34) * cos((theta2 + theta3) * PI / 180);
			TP0(1, 0) = 0;				TP0(1, 1) = 1;				TP0(1, 2) = 0;				TP0(1, 3) = 0;
			TP0(2, 0) = 0;				TP0(2, 1) = 0;				TP0(2, 2) = 1;				TP0(2, 3) = 0 - sqrt(0.2 * 0.2 + 0.32 * 0.32) * sin(theta2 * PI / 180) - sqrt(0.14 * 0.14 + 0.34 * 0.34) * sin((theta2 + theta3) * PI / 180);
			TP0(3, 0) = 0;				TP0(3, 1) = 0;				TP0(3, 2) = 0;				TP0(3, 3) = 1;

			Eigen::Matrix4f TPW;
			TPW = T0W * TP0;

			aux_xyz(0) = TPW(0, 3);
			aux_xyz(1) = TPW(1, 3);
			aux_xyz(2) = TPW(2, 3);

			return aux_xyz;
		}
		Eigen::Vector3f thumb_3_motion(float& theta2, float& theta3, float& theta4)
		{
			Ry = getRy(-90);
			Rz = getRz(45);
			Rx = getRx(-45);

			Rxyz = Ry * Rz * Rx;

			Eigen::Matrix4f T0W;
			T0W(0, 0) = Rxyz(0, 0);		T0W(0, 1) = Rxyz(0, 1);		T0W(0, 2) = Rxyz(0, 2);		T0W(0, 3) = 0;
			T0W(1, 0) = Rxyz(1, 0);		T0W(1, 1) = Rxyz(1, 1);		T0W(1, 2) = Rxyz(1, 2);		T0W(1, 3) = 0.19;
			T0W(2, 0) = Rxyz(2, 0);		T0W(2, 1) = Rxyz(2, 1);		T0W(2, 2) = Rxyz(2, 2);		T0W(2, 3) = -0.72;
			T0W(3, 0) = 0;				T0W(3, 1) = 0;				T0W(3, 2) = 0;				T0W(3, 3) = 1;
			Eigen::Matrix4f TP0;
			TP0(0, 0) = 1;				TP0(0, 1) = 0;				TP0(0, 2) = 0;				TP0(0, 3) = 0 + sqrt(0.2 * 0.2 + 0.32 * 0.32) * cos(theta2 * PI / 180) + sqrt(0.14 * 0.14 + 0.34 * 0.34) * cos((theta2 + theta3) * PI / 180)+ sqrt(0.11 * 0.11 + 0.3 * 0.3) * cos((theta2 + theta3 + theta4) * PI / 180);
			TP0(1, 0) = 0;				TP0(1, 1) = 1;				TP0(1, 2) = 0;				TP0(1, 3) = 0;
			TP0(2, 0) = 0;				TP0(2, 1) = 0;				TP0(2, 2) = 1;				TP0(2, 3) = 0 - sqrt(0.2 * 0.2 + 0.32 * 0.32) * sin(theta2 * PI / 180) - sqrt(0.14 * 0.14 + 0.34 * 0.34) * sin((theta2 + theta3) * PI / 180)- sqrt(0.11 * 0.11 + 0.3 * 0.3) * sin((theta2+theta3+theta4)*PI/180);
			TP0(3, 0) = 0;				TP0(3, 1) = 0;				TP0(3, 2) = 0;				TP0(3, 3) = 1;

			Eigen::Matrix4f TPW;
			TPW = T0W * TP0;

			aux_xyz(0) = TPW(0, 3);
			aux_xyz(1) = TPW(1, 3);
			aux_xyz(2) = TPW(2, 3);

			return aux_xyz;
		}





		Eigen::Matrix4f auxF1TransMatrix(float& theta1, float& theta2, float& theta3, float& theta4, float& theta5, float& theta6)
		{
			//for simple calculate thumb

			T_tip_0 = getTransMatrix(theta1, theta2, theta3, theta4, theta5, theta6);
			
			Rx(0, 0) = 1;
			Rx(0, 1) = 0;
			Rx(0, 2) = 0;
			Rx(1, 0) = 0;
			Rx(1, 1) = cos(-90 * PI / 180);
			Rx(1, 2) = (-1) * sin(-90 * PI / 180);
			Rx(2, 0) = 0;
			Rx(2, 1) = sin(-90 * PI / 180);
			Rx(2, 2) = cos(-90 * PI / 180);

			Rxyz = Rx;

			//
			T_0W(0, 0) = Rxyz(0, 0);	T_0W(0, 1) = Rxyz(0, 1);	T_0W(0, 2) = Rxyz(0, 2);	T_0W(0, 3) = 0;
			T_0W(1, 0) = Rxyz(1, 0);	T_0W(1, 1) = Rxyz(1, 1);	T_0W(1, 2) = Rxyz(1, 2);	T_0W(1, 3) = 0;
			T_0W(2, 0) = Rxyz(2, 0);	T_0W(2, 1) = Rxyz(2, 1);	T_0W(2, 2) = Rxyz(2, 2);	T_0W(2, 3) = 0;		//已查验
			T_0W(3, 0) = 0;				T_0W(3, 1) = 0;				T_0W(3, 2) = 0;				T_0W(3, 3) = 1;

			//
			T_tip_W = T_0W*T_tip_0;	//此处world为运动链根部
			
			return T_tip_W;

		}


	}
}

#endif // !_FINGER_KINEMATICS_

