#include "JakaRobot.h"
#include "vtkMath.h"

JakaRobot::JakaRobot() {
	m_JakaRobot.login_in(hostname);
	std::cout << "RObot IP is:" << this->hostname << "  " << "Robot port is" << this->nPort << std::endl;
}

JakaRobot::~JakaRobot() {

}


bool JakaRobot::Connect(QTextBrowser* browser)
{
	m_JakaRobot.login_in(hostname);
	m_JakaRobot.power_on();
	return true;
}

bool JakaRobot::PowerOn(QTextBrowser* browser) {
	m_JakaRobot.enable_robot();
	return true;
}

bool JakaRobot::PowerOff(QTextBrowser* browser) {
	m_JakaRobot.disable_robot();
	return true;
}

bool JakaRobot::StartFreeGrag(QTextBrowser* browser) {
	m_JakaRobot.drag_mode_enable(TRUE);
	return true;
}

bool JakaRobot::StopFreeGrag(QTextBrowser* browser) {
	m_JakaRobot.drag_mode_enable(FALSE);
	return true;
}

bool JakaRobot::Stop(QTextBrowser* browser) {
	
	m_JakaRobot.motion_abort();
	return true;
	
}

bool JakaRobot::SetFlangetoTCP(QTextBrowser* browser, std::array<double, 6> TCP) {
	//int nRet = HRIF_SetTCP(boxID, rbtID, TCP[0], TCP[1], TCP[2], TCP[3], TCP[4], TCP[5]);
	//UtilityFunction->PrintError(browser, nRet, "Set tcp");
	//UtilityFunction->PrintTCP(browser, nRet, TCP);
	//if (!nRet)
	//{
		return true;
	//}
	//else
	//{
	//	return false;
	//}
}

bool JakaRobot::SetFlangetoTCP(QTextBrowser* browser, std::array<double, 6> TCP, std::string TCP_Name) {
	//int nRet = HRIF_SetTCPByName(boxID, rbtID, TCP_Name);
	//this->SetFlangetoTCP(browser, TCP);
	//UtilityFunction->PrintError(browser, nRet, "Set tcp by name");
	//if (!nRet)
	//{
		return true;
	//}
	//else
	//{
	//	return false;
	//}
}

bool JakaRobot::ConfigFlangetoTCP(QTextBrowser* browser, std::array<double, 6> TCP, std::string TCP_Name) {
	//int nRet = HRIF_ConfigTCP(boxID, rbtID, TCP_Name, TCP[0], TCP[1], TCP[2], TCP[3], TCP[4], TCP[5]);
	//UtilityFunction->PrintError(browser, nRet, "ConfigTCP by " + TCP_Name);
	//UtilityFunction->PrintTCP(browser, nRet, TCP);
	//this->SetFlangetoTCP(browser, TCP, TCP_Name);
	//this->SetMoveToolMotion(browser, 1);
	//this->SetMoveToolMotion(browser, 0);
	//this->SetMoveToolMotion(browser, 1);
	//this->SetFlangetoTCP(browser, TCP, TCP_Name);
	//if (!nRet)
	//{
		return true;
	//}
	//else
	//{
	//	return false;
	//}

}

bool JakaRobot::ConfigBasetoUCS(QTextBrowser* browser, std::array<double, 6> UCS, std::string UCS_Name) {
	//int nRet = HRIF_ConfigUCS(boxID, rbtID, UCS_Name, UCS[0], UCS[1], UCS[2], UCS[3], UCS[4], UCS[5]);
	//UtilityFunction->PrintError(browser, nRet, "ConfigUCS by " + UCS_Name);
	//UtilityFunction->PrintUCS(browser, nRet, UCS);
	//nRet = HRIF_SetUCSByName(boxID, rbtID, UCS_Name);
	//UtilityFunction->PrintError(browser, nRet, "SetUCSByName");
	//this->SetBasetoUCS(browser, UCS);
	//if (!nRet)
	//{
		return true;
	//}
	//else
	//{
	//	return false;
	//}
}

bool JakaRobot::SetBasetoUCS(QTextBrowser* browser, std::array<double, 6> UCS) {
	//int nRet = HRIF_SetUCS(boxID, rbtID, UCS[0], UCS[1], UCS[2], UCS[3], UCS[4], UCS[5]);
	//UtilityFunction->PrintError(browser, nRet, "SetUCS ");
	//if (!nRet)
	//{
		return true;
	//}
	//else
	//{
	//	return false;
	//}
}


bool JakaRobot::SetVelocity_dAcc_dRadius(QTextBrowser* browser, double Velocity, double dAcc, double dRadius) {
	//if (Velocity > dAcc)
	//{
	//	std::cout << "input is error Velocity must be less than dAcc" << std::endl;
	//	return false;
	//}
	//else
	//{
	//	std::cout << "set succeed" << std::endl;
	//	this->dAcc = dAcc;
	//	this->dVelocity = Velocity;
	//	this->dRadius = dRadius;
		return true;
	//}
}

bool JakaRobot::isMoving(QTextBrowser* browser) {
	//int nMovingState = 0; int nEnableState = 0; int nErrorState = 0; int nErrorCode = 0;
	//int nErrorAxis = 0; int nBreaking = 0; int nPause = 0; int nEmergencyStop = 0;
	//int nSaftyGuard = 0; int nElectrify = 0; int nIsConnectToBox = 0; int nBlendingDone = 0; int nInPos = 0;
	//// 读取状态
	//int nRet = HRIF_ReadRobotState(0, 0, nMovingState, nEnableState, nErrorState, nErrorCode, nErrorAxis,
	//	nBreaking, nPause, nEmergencyStop, nSaftyGuard, nElectrify, nIsConnectToBox, nBlendingDone, nInPos);
	//if (!nRet)
	//{
	//	//运动完成
	//	return nMovingState;
	//}
	//else
	//{
	//	std::cout << "ismoving model is error please check " << std::endl;
	//	UtilityFunction->PrintError(browser, nRet, "Judge is moving done Error");
		return false;
	//}
}

bool JakaRobot::isnSaftyGuard(QTextBrowser* browser) {
	//int nMovingState = 0; int nEnableState = 0; int nErrorState = 0; int nErrorCode = 0;
	//int nErrorAxis = 0; int nBreaking = 0; int nPause = 0; int nEmergencyStop = 0;
	//int nSaftyGuard = 0; int nElectrify = 0; int nIsConnectToBox = 0; int nBlendingDone = 0; int nInPos = 0;
	//// 读取状态
	//int nRet = HRIF_ReadRobotState(0, 0, nMovingState, nEnableState, nErrorState, nErrorCode, nErrorAxis,
	//	nBreaking, nPause, nEmergencyStop, nSaftyGuard, nElectrify, nIsConnectToBox, nBlendingDone, nInPos);
	//if (!nRet)
	//{
	//	return nSaftyGuard;
	//}
	//else
	//{
	//	std::cout << "ismoving model is error please check " << std::endl;
	//	UtilityFunction->PrintError(browser, nRet, "Judge is moving done Error");
		return false;
	//}
}

//1是
bool JakaRobot::SetMoveToolMotion(QTextBrowser* browser, bool toolmotion) {
	//this->ToolMotion = toolmotion;
	//int nRet = HRIF_SetToolMotion(boxID, rbtID, this->ToolMotion);
	//UtilityFunction->PrintError(browser, nRet, "Set toolmotion :");
	//if (!nRet)
	//{
		return true;
	//}
	//else
	//{
	//	return false;
	//}
}

bool JakaRobot::RelMoveXplus(QTextBrowser* browser, double Distance) {
	CartesianPose cart{ {Distance, 0, 0}, {0,0, 0} };
	m_JakaRobot.linear_move(&cart, INCR, TRUE, 10);
	return true;
}
bool JakaRobot::RelMoveYplus(QTextBrowser* browser, double Distance) {
	CartesianPose cart{ {0, Distance, 0}, {0,0, 0} };
	m_JakaRobot.linear_move(&cart, INCR, TRUE, 10);
	return true;
}
bool JakaRobot::RelMoveZplus(QTextBrowser* browser, double Distance) {
	CartesianPose cart{ { 0, 0,Distance}, {0,0, 0} };
	m_JakaRobot.linear_move(&cart, INCR, TRUE, 10);
	return true;
}
bool JakaRobot::RelMoveRxplus(QTextBrowser* browser, double Distance) {
	CartesianPose cart{ {0, 0, 0}, {Distance,0, 0} };
	m_JakaRobot.linear_move(&cart, INCR, TRUE, 10);
	return true;
}
bool JakaRobot::RelMoveRyplus(QTextBrowser* browser, double Distance) {
	CartesianPose cart{ {0, 0, 0}, {0,Distance, 0} };
	m_JakaRobot.linear_move(&cart, INCR, TRUE, 10);
	return true;
}
bool JakaRobot::RelMoveRzplus(QTextBrowser* browser, double Distance) {
	CartesianPose cart{ {0, 0, 0}, {0,0, Distance} };
	m_JakaRobot.linear_move(&cart, INCR, TRUE, 10);
	return true;
}
bool JakaRobot::RelMoveXmin(QTextBrowser* browser, double Distance) {
	CartesianPose cart{ {-Distance, 0, 0}, {0,0, 0} };
	m_JakaRobot.linear_move(&cart, INCR, TRUE, 10);
	return true;
}
bool JakaRobot::RelMoveYmin(QTextBrowser* browser, double Distance) {
	CartesianPose cart{ {0, -Distance, 0}, {0,0, 0} };
	m_JakaRobot.linear_move(&cart, INCR, TRUE, 10);
	return true;
}
bool JakaRobot::RelMoveZmin(QTextBrowser* browser, double Distance) {
	CartesianPose cart{ {0, 0, -Distance}, {0,0, 0} };
	m_JakaRobot.linear_move(&cart, INCR, TRUE, 10);
	return true;
}
bool JakaRobot::RelMoveRxmin(QTextBrowser* browser, double Distance) {
	CartesianPose cart{ {0, 0, 0}, {-Distance,0, 0} };
	m_JakaRobot.linear_move(&cart, INCR, TRUE, 10);
	return true;
}
bool JakaRobot::RelMoveRymin(QTextBrowser* browser, double Distance) {
	CartesianPose cart{ {0, 0, 0}, {0,-Distance, 0} };
	m_JakaRobot.linear_move(&cart, INCR, TRUE, 10);
	return true;
}
bool JakaRobot::RelMoveRzmin(QTextBrowser* browser, double Distance) {
	CartesianPose cart{ {0, 0, 0}, {0,0, -Distance} };
	m_JakaRobot.linear_move(&cart, INCR, TRUE, 10);
	return true;
}

//flangetoTCP
std::array<double, 6> JakaRobot::ReadFlangetoTCP(QTextBrowser* browser) {
	std::array<double, 6> FlangeToTCP{ 0 };
	//int nRet = HRIF_ReadCurTCP(boxID, rbtID, FlangeToTCP[0], FlangeToTCP[1], FlangeToTCP[2], FlangeToTCP[3], FlangeToTCP[4], FlangeToTCP[5]);
	//UtilityFunction->PrintError(browser, nRet, "ReadFlangetoTCP :");
	//UtilityFunction->PrintTCP(browser, nRet, FlangeToTCP);

	return FlangeToTCP;
}
//basetoTCP
std::array<double, 6> JakaRobot::ReadBasetoTCP(QTextBrowser* browser) {
	std::array<double, 6> BaseToTCP{ 0 };
	//int nRet = HRIF_ReadActTcpPos(boxID, rbtID, BaseToTCP[0], BaseToTCP[1], BaseToTCP[2], BaseToTCP[3], BaseToTCP[4], BaseToTCP[5]);
	//UtilityFunction->PrintError(browser, nRet, "ReadBaseToTCP :");
	//UtilityFunction->PrintTCP(browser, nRet, BaseToTCP);

	return BaseToTCP;
}

std::array<double, 6> JakaRobot::ReadBasetoTCP(QTextBrowser* browser, std::string sTcpName) {
	std::array<double, 6> BaseToTCP{ 0 };

	//int nRet = HRIF_ReadTCPByName(boxID, rbtID, sTcpName, BaseToTCP[0], BaseToTCP[1], BaseToTCP[2], BaseToTCP[3], BaseToTCP[4], BaseToTCP[5]);
	//UtilityFunction->PrintError(browser, nRet, "ReadBaseToTCP :" + sTcpName);
	//UtilityFunction->PrintTCP(browser, nRet, BaseToTCP);

	return BaseToTCP;
}

std::array<double, 6> JakaRobot::ReadBasetoFlange(QTextBrowser* browser) {
	//std::array<double, 6> BasetoTCP = this->ReadBasetoTCP(browser);
	//std::array<double, 6> FlangetoTCP = this->ReadFlangetoTCP(browser);
	//auto Trans_BasetoTCP = vtkTransform::New();
	//auto Trans_FlangetoTCP = vtkTransform::New();
	//auto vtk_BasetoTCP = vtkMatrix4x4::New();
	//auto vtk_TCPtoFlange = vtkMatrix4x4::New();
	//UtilityFunction->TranslateAndRotateMatrix(Trans_BasetoTCP, BasetoTCP);
	//UtilityFunction->TranslateAndRotateMatrix(Trans_FlangetoTCP, FlangetoTCP);
	//vtk_BasetoTCP->DeepCopy(Trans_BasetoTCP->GetMatrix());
	//vtk_TCPtoFlange->DeepCopy(Trans_FlangetoTCP->GetMatrix());
	//vtk_TCPtoFlange->Invert();

	//auto Trans_BaseToFlange = vtkTransform::New();
	//UtilityFunction->MultiplyMatrices(Trans_BaseToFlange, vtk_BasetoTCP, vtk_TCPtoFlange);

	//auto vtk_BasetoFlange = vtkMatrix4x4::New();
	//vtk_BasetoFlange->DeepCopy(Trans_BaseToFlange->GetMatrix());

	//Eigen::Vector3d Angle = UtilityFunction->GetAulerAngle(vtk_BasetoFlange);
	std::array<double, 6> BasetoFlange;
	//std::array<double, 6> BasetoFlange{ vtk_BasetoFlange->GetElement(0, 3), vtk_BasetoFlange->GetElement(1, 3),
	//	vtk_BasetoFlange->GetElement(2, 3), vtkMath::DegreesFromRadians(Angle(2)),vtkMath::DegreesFromRadians(Angle(1)),vtkMath::DegreesFromRadians(Angle(0)) };

	//UtilityFunction->PrintTCP(browser, 0, BasetoFlange);

	return BasetoFlange;

}

std::array<double, 6> JakaRobot::ReadTargetBasetoTCP(QTextBrowser* browser) {
	std::array<double, 6> TargetBaseToTCP{ 0 };
	//int nRet = HRIF_ReadCmdTcpPos(boxID, rbtID, TargetBaseToTCP[0], TargetBaseToTCP[1], TargetBaseToTCP[2],
	//	TargetBaseToTCP[3], TargetBaseToTCP[4], TargetBaseToTCP[5]);
	//UtilityFunction->PrintError(browser, nRet, "ReadBaseToTCP :");
	//UtilityFunction->PrintTCP(browser, nRet, TargetBaseToTCP);

	return TargetBaseToTCP;
}

std::array<double, 6> JakaRobot::ReadJoint(QTextBrowser* browser) {
	std::array<double, 6> Joint{ 0 };
	//int nRet = HRIF_ReadActJointPos(boxID, rbtID, Joint[0], Joint[1], Joint[2], Joint[3], Joint[4], Joint[5]);
	//UtilityFunction->PrintError(browser, nRet, "ReadBaseToTCP :");
	//UtilityFunction->PrintJoint(browser, nRet, Joint);

	return Joint;
}

std::array<double, 6> JakaRobot::ReadTargetJoint(QTextBrowser* browser) {
	std::array<double, 6> Joint{ 0 };
	//int nRet = HRIF_ReadCmdJointPos(boxID, rbtID, Joint[0], Joint[1], Joint[2], Joint[3], Joint[4], Joint[5]);
	//UtilityFunction->PrintError(browser, nRet, "ReadBaseToTCP :");
	//UtilityFunction->PrintJoint(browser, nRet, Joint);

	return Joint;
}

std::array<double, 6> JakaRobot::ReadUCS(QTextBrowser* browser) {
	std::array<double, 6> UCS{ 0 };

	//int nRet = HRIF_ReadCurUCS(boxID, rbtID, UCS[0], UCS[1], UCS[2], UCS[3], UCS[4], UCS[5]);
	//UtilityFunction->PrintError(browser, nRet, "ReadBaseToTCP :");
	//UtilityFunction->PrintUCS(browser, nRet, UCS);

	return UCS;
}

std::array<double, 6> JakaRobot::ReadUCS(QTextBrowser* browser, std::string UCSName) {
	std::array<double, 6> UCS{ 0 };

	//int nRet = HRIF_ReadUCSByName(boxID, rbtID, UCSName, UCS[0], UCS[1], UCS[2], UCS[3], UCS[4], UCS[5]);
	//UtilityFunction->PrintError(browser, nRet, "ReadBaseToTCP :" + UCSName);
	//UtilityFunction->PrintUCS(browser, nRet, UCS);

	return UCS;
}

std::array<double, 6> JakaRobot::GetInverseKin(QTextBrowser* browser, std::array<double, 6> TargetPoint) {

	std::array<double, 6> UCS = this->ReadUCS(browser, "Base");
	std::array<double, 6> FlangetoTCP = this->ReadFlangetoTCP(browser);
	std::array<double, 6> CurrentJoint = this->ReadJoint(browser);
	std::array<double, 6> TargetJoint{ 0,0,0,0,0,0 };

	//int nRet = HRIF_GetInverseKin(boxID, rbtID, TargetPoint[0], TargetPoint[1], TargetPoint[2], TargetPoint[3], TargetPoint[4], TargetPoint[5],
	//	FlangetoTCP[0], FlangetoTCP[1], FlangetoTCP[2], FlangetoTCP[3], FlangetoTCP[4], FlangetoTCP[5],
	//	UCS[0], UCS[1], UCS[2], UCS[3], UCS[4], UCS[5],
	//	CurrentJoint[0], CurrentJoint[1], CurrentJoint[2], CurrentJoint[3], CurrentJoint[4], CurrentJoint[5],
	//	TargetJoint[0], TargetJoint[1], TargetJoint[2], TargetJoint[3], TargetJoint[4], TargetJoint[5]);
	//UtilityFunction->PrintError(browser, nRet, "GetInverseKin :");
	//UtilityFunction->PrintJoint(browser, nRet, TargetJoint);

	return TargetJoint;
}
//关节运动
bool JakaRobot::MoveJ(QTextBrowser* browser, std::array<double, 6> Target) {

	//int nMoveType = 0;
	//int isUseJoint = 0;

	//std::array<double, 6> TargetJoint = this->GetInverseKin(browser, Target);
	//std::array<double, 6> UCS = this->ReadUCS(browser, "Base");
	//std::array<double, 6> FlangetoTCP = this->ReadFlangetoTCP(browser);

	//int nRet = HRIF_WayPointEx(boxID, rbtID, nMoveType, Target[0], Target[1], Target[2], Target[3], Target[4], Target[5],
	//	TargetJoint[0], TargetJoint[1], TargetJoint[2], TargetJoint[3], TargetJoint[4], TargetJoint[5],
	//	FlangetoTCP[0], FlangetoTCP[1], FlangetoTCP[2], FlangetoTCP[3], FlangetoTCP[4], FlangetoTCP[5],
	//	UCS[0], UCS[1], UCS[2], UCS[3], UCS[4], UCS[5],
	//	dVelocity, dAcc, dRadius, isUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
	//UtilityFunction->PrintError(browser, nRet, "MoveJ");
	//if (!nRet)
	//{
		return true;
	//}
	//else
	//{
	//	return false;
	//}
}

bool JakaRobot::MoveJ(QTextBrowser* browser, std::array<double, 6> Target, bool isUseJoint) {

	/*int nMoveType = 0;

	std::array<double, 6> TargetJoint = this->GetInverseKin(browser, Target);
	std::array<double, 6> UCS = this->ReadUCS(browser, "Base");
	std::array<double, 6> FlangetoTCP = this->ReadFlangetoTCP(browser);
	if (isUseJoint) {
		int nRet = HRIF_WayPointEx(boxID, rbtID, nMoveType, zero[0], zero[1], zero[2], zero[3], zero[4], zero[5],
			Target[0], Target[1], Target[2], Target[3], Target[4], Target[5],
			zero[0], zero[1], zero[2], zero[3], zero[4], zero[5],
			zero[0], zero[1], zero[2], zero[3], zero[4], zero[5],
			dVelocity, dAcc, dRadius, isUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
		UtilityFunction->PrintError(browser, nRet, "MoveJ");
		if (!nRet)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else {
		int nRet = HRIF_WayPointEx(boxID, rbtID, nMoveType, Target[0], Target[1], Target[2], Target[3], Target[4], Target[5],
			TargetJoint[0], TargetJoint[1], TargetJoint[2], TargetJoint[3], TargetJoint[4], TargetJoint[5],
			FlangetoTCP[0], FlangetoTCP[1], FlangetoTCP[2], FlangetoTCP[3], FlangetoTCP[4], FlangetoTCP[5],
			UCS[0], UCS[1], UCS[2], UCS[3], UCS[4], UCS[5],
			dVelocity, dAcc, dRadius, isUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
		UtilityFunction->PrintError(browser, nRet, "MoveJ");
		if (!nRet)
		{
			*/return true;/*
		}
		else
		{
			return false;
		}
	}*/

}

//空间直线运动
bool JakaRobot::MoveP(QTextBrowser* browser, std::array<double, 6> TargetPoint) {
	//int nIsUseJoint = 0;
	//int nMoveType = 1;

	//std::array<double, 6> TargetJoint = this->GetInverseKin(browser, TargetPoint);
	//std::array<double, 6> UCS = this->ReadUCS(browser, "Base");
	//std::array<double, 6> FlangetoTCP = this->ReadFlangetoTCP(browser);

	//int nRet = HRIF_WayPointEx(boxID, rbtID, nMoveType, TargetPoint[0], TargetPoint[1], TargetPoint[2], TargetPoint[3], TargetPoint[4], TargetPoint[5],
	//	TargetJoint[0], TargetJoint[1], TargetJoint[2], TargetJoint[3], TargetJoint[4], TargetJoint[5],
	//	FlangetoTCP[0], FlangetoTCP[1], FlangetoTCP[2], FlangetoTCP[3], FlangetoTCP[4], FlangetoTCP[5],
	//	UCS[0], UCS[1], UCS[2], UCS[3], UCS[4], UCS[5],
	//	dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
	//UtilityFunction->PrintError(browser, nRet, "MoveL");
	//if (!nRet)
	//{
		return true;
	//}
	//else
	//{
	//	return false;
	//}
}

bool JakaRobot::SetForceFreeDrive(QTextBrowser* browser, int nMode) {
	//int nRet = HRIF_SetForceFreeDriveMode(boxID, rbtID, nMode);
	//UtilityFunction->PrintError(browser, nRet, "SetFreeDriveMode");
	//if (!nRet)
	//{
		return true;
	//}
	//else
	//{
	//	return false;
	//}
}

bool JakaRobot::SetForceFreeDriveFreedom(QTextBrowser* browser, std::array<double, 6> Freedom) {
	//int nRet = HRIF_SetFreeDriveMotionFreedom(0, 0, Freedom[0], Freedom[1], Freedom[2], Freedom[3], Freedom[4], Freedom[5]);
	//UtilityFunction->PrintError(browser, nRet, "SetFreeDriveFreedom");
	//if (!nRet)
	//{
		return true;
	//}
	//else
	//{
	//	return false;
	//}
}

bool JakaRobot::SetForceFTFreeDriveFaztor(QTextBrowser* browser, double linerfaztor, double anglefaztor) {
	//int nRet = HRIF_SetFTFreeFactor(0, 0, linerfaztor, anglefaztor);
	//UtilityFunction->PrintError(browser, nRet, "SetForceFTFreeDriveFaztor");
	//if (!nRet)
	//{
		return true;
	//}
	//else
	//{
	//	return false;
	//}
}

bool JakaRobot::SetForceFreeDriveVelocity(QTextBrowser* browser, double Maxlinervelocity, double Maxanglevelocity) {
	//int nRet = HRIF_SetMaxFreeDriveVel(0, 0, Maxlinervelocity, Maxanglevelocity);
	//UtilityFunction->PrintError(browser, nRet, "SetForceFreeDriveVelocity");
	int nRet = 0;
	if (!nRet)
	{
		return true;
	}
	else
	{
		return false;
	}

}

bool JakaRobot::SetForceToolCoordinateMotion(QTextBrowser* browser, int nMode) {
	//int nRet = HRIF_SetForceToolCoordinateMotion(0, 0, nMode);
	//UtilityFunction->PrintError(browser, nRet, "SetForceToolCoordinateMotion");
	//if (!nRet)
	//{
		return true;
	//}
	//else
	//{
	//	return false;
	//}
}
