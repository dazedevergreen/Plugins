#pragma once
#ifndef DaZu_Robot_h
#define DaZu_Robot_h

#include <LancetHansRobot.h>
#include<iostream>
#include<vector>
#include <array>
#include <atomic>
#include <mutex>
#include <thread>
#include <map>
#include <QLabel>
#include <QTimer>
#include <QObject>
#include <QThread>
#include <QWidget>
#include "ToolFunction.h"
#include <QTextBrowser>

class DaZuRobot 
{

public:

	/**
	 * @brief  ���캯��
	 * @param  1��hostname ip��ַ  2��nPort �˿ں�
	 * @return [return type]
	 * @author zzx
	 * @date   2024/9/2
	 */
	DaZuRobot();
	~DaZuRobot();

public:
	ToolFunction Utility;
	ToolFunction * UtilityFunction = &Utility;
	bool Connect(QTextBrowser* browser);
	bool PowerOn(QTextBrowser* browser);
	bool PowerOff(QTextBrowser* browser);
	bool StartFreeGrag(QTextBrowser* browser);
	bool StopFreeGrag(QTextBrowser* browser);
	bool Stop(QTextBrowser* browser);
	bool Reset(QTextBrowser* browser);

	bool SetFlangetoTCP(QTextBrowser* browser, std::array<double, 6> TCP);
	bool SetFlangetoTCP(QTextBrowser* browser, std::array<double, 6> TCP, string TCP_Name);

	bool ConfigFlangetoTCP(QTextBrowser* browser, std::array<double, 6> TCP, string TCP_Name);
	bool SetBasetoUCS(QTextBrowser* browser, std::array<double, 6> UCS);
	bool ConfigBasetoUCS(QTextBrowser* browser, std::array<double, 6> UCS, string UCS_Name);

	//��ǰTCP�������
	bool SetVelocity_dAcc_dRadius(QTextBrowser* browser, double Velocity, double dAcc, double dRadius);
	bool isMoving(QTextBrowser* browser);
	bool isnSaftyGuard(QTextBrowser* browser);

	
	bool SetMoveToolMotion(QTextBrowser* browser, bool toolmotion);

	std::array<double, 6> CaculateTargetRel(QTextBrowser* browser, std::array<double, 6> Distance);
	bool RelMoveXplus(QTextBrowser* browser, double Distance);
	bool RelMoveYplus(QTextBrowser* browser, double Distance);
	bool RelMoveZplus(QTextBrowser* browser, double Distance);
	bool RelMoveRxplus(QTextBrowser* browser, double Distance);
	bool RelMoveRyplus(QTextBrowser* browser, double Distance);
	bool RelMoveRzplus(QTextBrowser* browser, double Distance);
	bool RelMoveXmin(QTextBrowser* browser, double Distance);
	bool RelMoveYmin(QTextBrowser* browser, double Distance);
	bool RelMoveZmin(QTextBrowser* browser, double Distance);
	bool RelMoveRxmin(QTextBrowser* browser, double Distance);
	bool RelMoveRymin(QTextBrowser* browser, double Distance);
	bool RelMoveRzmin(QTextBrowser* browser, double Distance);


	//flangetoTCP
	std::array<double, 6> ReadFlangetoTCP(QTextBrowser* browser);
	//basetoTCP
	std::array<double, 6> ReadBasetoTCP(QTextBrowser* browser);
	std::array<double, 6> ReadBasetoTCP(QTextBrowser* browser, string sTcpName);
	std::array<double, 6> ReadBasetoFlange(QTextBrowser* browser);
	std::array<double, 6> ReadTargetBasetoTCP(QTextBrowser* browser);
	std::array<double, 6> ReadJoint(QTextBrowser* browser);
	std::array<double, 6> ReadTargetJoint(QTextBrowser* browser);
	std::array<double, 6> ReadUCS(QTextBrowser* browser);
	std::array<double, 6> ReadUCS(QTextBrowser* browser, string UCSName);
	std::array<double, 6> ReadFTCabData(QTextBrowser* browser);

	std::array<double, 6> GetInverseKin(QTextBrowser* browser, std::array<double, 6> TargetPoint);
	//�ؽ��˶�
	bool MoveJ(QTextBrowser* browser, std::array<double, 6> Target);
	bool MoveJ(QTextBrowser* browser, std::array<double, 6> Target, bool isUseJoint);
	//�ռ��˶�
	bool MoveP(QTextBrowser* browser, std::array<double, 6> TargetPoint);

	bool SetForceFreeDrive(QTextBrowser* browser, int nMode);
	bool SetForceFreeDriveFreedom(QTextBrowser* browser, std::array<double, 6> Freedom);
	bool SetForceFTFreeDriveFaztor(QTextBrowser* browser, double linerfaztor, double anglefaztor);
	bool SetForceFreeDriveVelocity(QTextBrowser* browser, double Maxlinervelocity, double Maxanglevelocity);
	bool SetForceToolCoordinateMotion(QTextBrowser* browser, int nMode);
	bool SetForceZero(QTextBrowser* browser);


	void TranslateAndRotateMatrix(vtkTransform* matrix, double dX, double dY, double dZ, double dRx, double dRy, double dRz);
	void TranslateAndRotateMatrix(vtkTransform* matrix, std::array<double, 6> TCP);

	Eigen::Vector3d GetAulerAngle(vtkMatrix4x4* matrix);
	Eigen::Vector3d GetAulerAngle(double matrix[16]);
	Eigen::Vector3d GetAulerAngle(vtkTransform* matrix);

private:



	//һЩĬ�ϲ������޸�
private:
	unsigned int boxID = 0;
	unsigned int rbtID = 0;
	unsigned short int nPort = 10003;
	std::string hostname = "192.168.0.10";
	std::string strCmdID = "0";

	int nIsSeek = 0;
	int nIOBit = 0;
	int nIOState = 0;
	double dVelocity = 10;
	//�ٶ���ҪС�ڼ��ٶ� 
	double dAcc = 15;
	double dRadius = 30;
	bool ToolMotion = 1;

	std::map<int, std::string> NumtoMoveType = {
	   {0, "x"}, {1, "y"}, {2, "z"}, {3, "rx"}, {4, "ry"}, {5, "rz"}
	};

	std::array<double, 6> zero{ 0, 0, 0, 0, 0, 0 };


	//NumtoMoveType.insert(std::make_pair(6, "zz"));

	/*std::map<int, std::string> TCP_List = {
	   {0, "TCP"}, {1, "TCP_TOOL_1"}, {2, "TCP_TOOL_2"}, {3, "TCP_TOOL_3"}, {4, "TCP_TOOL_4"}, {5, "TCP_TOOL_5"}
	};

	std::map<int, std::string> UCS_List = {
	   {0, "Base"}, {1, "UCS_1"}, {2, "UCS_2"}, {3, "UCS_3"}, {4, "UCS_4"}, {5, "UCS_5"}
	};*/

	/*std::vector<std::string> TCP_Name{ "TCP_1" };

	std::vector<std::string> TCP_Name{ "TCP_1" };*/

	/*std::string TCP_NAME = "TCP_Tool";
	std::string UCS_NAME = "UCS";*/

};


#endif DaZu_Robot_h