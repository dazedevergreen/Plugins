#pragma once
#ifndef JakaRobot_h
#define JakaRobot_h

#include "LancetJakaRobot.h"
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

class JakaRobot
{

public:

	/**
	 * @brief  构造函数
	 * @param  1：hostname ip地址  2：nPort 端口号
	 * @return [return type]
	 * @author zzx
	 * @date   2024/9/2
	 */
	JakaRobot();
	~JakaRobot();

public:
	ToolFunction Utility;
	ToolFunction* UtilityFunction = &Utility;
	JAKAZuRobot m_JakaRobot;
	bool Connect(QTextBrowser* browser);
	bool PowerOn(QTextBrowser* browser);
	bool PowerOff(QTextBrowser* browser);
	bool StartFreeGrag(QTextBrowser* browser);
	bool StopFreeGrag(QTextBrowser* browser);
	bool Stop(QTextBrowser* browser);

	bool SetFlangetoTCP(QTextBrowser* browser, std::array<double, 6> TCP);
	bool SetFlangetoTCP(QTextBrowser* browser, std::array<double, 6> TCP, std::string TCP_Name);

	bool ConfigFlangetoTCP(QTextBrowser* browser, std::array<double, 6> TCP, std::string TCP_Name);
	bool SetBasetoUCS(QTextBrowser* browser, std::array<double, 6> UCS);
	bool ConfigBasetoUCS(QTextBrowser* browser, std::array<double, 6> UCS, std::string UCS_Name);

	//当前TCP下求逆解
	bool SetVelocity_dAcc_dRadius(QTextBrowser* browser, double Velocity, double dAcc, double dRadius);
	bool isMoving(QTextBrowser* browser);
	bool isnSaftyGuard(QTextBrowser* browser);


	bool SetMoveToolMotion(QTextBrowser* browser, bool toolmotion);
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
	std::array<double, 6> ReadBasetoTCP(QTextBrowser* browser, std::string sTcpName);
	std::array<double, 6> ReadBasetoFlange(QTextBrowser* browser);
	std::array<double, 6> ReadTargetBasetoTCP(QTextBrowser* browser);
	std::array<double, 6> ReadJoint(QTextBrowser* browser);
	std::array<double, 6> ReadTargetJoint(QTextBrowser* browser);
	std::array<double, 6> ReadUCS(QTextBrowser* browser);
	std::array<double, 6> ReadUCS(QTextBrowser* browser, std::string UCSName);

	std::array<double, 6> GetInverseKin(QTextBrowser* browser, std::array<double, 6> TargetPoint);
	//关节运动
	bool MoveJ(QTextBrowser* browser, std::array<double, 6> Target);
	bool MoveJ(QTextBrowser* browser, std::array<double, 6> Target, bool isUseJoint);
	//空间运动
	bool MoveP(QTextBrowser* browser, std::array<double, 6> TargetPoint);

	bool SetForceFreeDrive(QTextBrowser* browser, int nMode);
	bool SetForceFreeDriveFreedom(QTextBrowser* browser, std::array<double, 6> Freedom);
	bool SetForceFTFreeDriveFaztor(QTextBrowser* browser, double linerfaztor, double anglefaztor);
	bool SetForceFreeDriveVelocity(QTextBrowser* browser, double Maxlinervelocity, double Maxanglevelocity);
	bool SetForceToolCoordinateMotion(QTextBrowser* browser, int nMode);

private:



	//一些默认参数勿修改
private:
	unsigned int boxID = 0;
	unsigned int rbtID = 0;
	unsigned short int nPort = 10003;
	const char* hostname = "10.5.5.100";
	std::string strCmdID = "0";

	int nIsSeek = 0;
	int nIOBit = 0;
	int nIOState = 0;
	double dVelocity = 10;
	//速度需要小于加速度 
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


#endif JakaRobot_h