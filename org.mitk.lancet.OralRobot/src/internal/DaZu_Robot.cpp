#include "Dazu_Robot.h"
#include "vtkMath.h"

DaZuRobot::DaZuRobot() {
	std::cout << "RObot IP is:" << this->hostname << "  " << "Robot port is" << this->nPort << std::endl;
}

DaZuRobot::~DaZuRobot() {

}

void DaZuRobot::TranslateAndRotateMatrix(vtkTransform* matrix, double dX, double dY, double dZ, double dRx, double dRy, double dRz) {
	vtkNew<vtkMatrix4x4> I;
	I->Identity();

	matrix->PreMultiply();
	matrix->SetMatrix(I);
	matrix->Translate(dX, dY, dZ);
	matrix->RotateZ(dRz);
	matrix->RotateY(dRy);
	matrix->RotateX(dRx);
	matrix->Update();
}

void DaZuRobot::TranslateAndRotateMatrix(vtkTransform* matrix, std::array<double, 6> TCP) {
	vtkNew<vtkMatrix4x4> I;
	I->Identity();

	matrix->PreMultiply();
	matrix->SetMatrix(I);
	matrix->Translate(TCP[0], TCP[1], TCP[2]);
	matrix->RotateZ(TCP[5]);
	matrix->RotateY(TCP[4]);
	matrix->RotateX(TCP[3]);
	matrix->Update();
}

Eigen::Vector3d DaZuRobot::GetAulerAngle(vtkMatrix4x4* matrix) {

	Eigen::Matrix3d Re;

	Re << matrix->GetElement(0, 0), matrix->GetElement(0, 1), matrix->GetElement(0, 2),
		matrix->GetElement(1, 0), matrix->GetElement(1, 1), matrix->GetElement(1, 2),
		matrix->GetElement(2, 0), matrix->GetElement(2, 1), matrix->GetElement(2, 2);

	// 欧拉角顺序zyx
	Eigen::Vector3d eulerAngle = Re.eulerAngles(2, 1, 0);
	return eulerAngle;
}

Eigen::Vector3d DaZuRobot::GetAulerAngle(double matrix[16]) {
	Eigen::Matrix3d Re;

	Re << matrix[0], matrix[1], matrix[2],
		matrix[4], matrix[5], matrix[6],
		matrix[8], matrix[9], matrix[10];

	// 欧拉角顺序zyx
	Eigen::Vector3d eulerAngle = Re.eulerAngles(2, 1, 0);
	return eulerAngle;
}

Eigen::Vector3d DaZuRobot::GetAulerAngle(vtkTransform* matrix) {
	Eigen::Matrix3d Re;
	auto Matrix = vtkMatrix4x4::New();
	Matrix->DeepCopy(matrix->GetMatrix());
	Re << Matrix->GetElement(0, 0), Matrix->GetElement(0, 1), Matrix->GetElement(0, 2),
		Matrix->GetElement(1, 0), Matrix->GetElement(1, 1), Matrix->GetElement(1, 2),
		Matrix->GetElement(2, 0), Matrix->GetElement(2, 1), Matrix->GetElement(2, 2);

	// 欧拉角顺序zyx
	Eigen::Vector3d eulerAngle = Re.eulerAngles(2, 1, 0);

	return eulerAngle;
}

bool DaZuRobot::Connect(QTextBrowser* browser)
{
	int nRet = HRIF_Connect(boxID, hostname.c_str(), nPort);
	UtilityFunction->PrintError(browser, nRet, "Connect");

	if (HRIF_IsConnected(0))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool DaZuRobot::PowerOn(QTextBrowser* browser) {
	int nRet = HRIF_GrpEnable(boxID, rbtID);
	UtilityFunction->PrintError(browser, nRet, "Power ON");
	if (!nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool DaZuRobot::PowerOff(QTextBrowser* browser) {
	int nRet = HRIF_GrpDisable(boxID, rbtID);
	UtilityFunction->PrintError(browser, nRet, "Power Off");
	if (!nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool DaZuRobot::StartFreeGrag(QTextBrowser* browser) {
	int nRet = HRIF_GrpOpenFreeDriver(boxID, rbtID);
	UtilityFunction->PrintError(browser, nRet, "StartFreeGrag");
	if (!nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool DaZuRobot::StopFreeGrag(QTextBrowser* browser) {
	int nRet = HRIF_GrpCloseFreeDriver(boxID, rbtID);
	UtilityFunction->PrintError(browser, nRet, "StopFreeGrag");
	if (!nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool DaZuRobot::Stop(QTextBrowser* browser) {
	int nRet = HRIF_GrpStop(boxID, rbtID);
	UtilityFunction->PrintError(browser, nRet, "Stop Robot");
	if (!nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool DaZuRobot::SetFlangetoTCP(QTextBrowser* browser, std::array<double, 6> TCP) {
	int nRet = HRIF_SetTCP(boxID, rbtID, TCP[0], TCP[1], TCP[2], TCP[3], TCP[4], TCP[5]);
	UtilityFunction->PrintError(browser, nRet, "Set tcp");
	UtilityFunction->PrintTCP(browser, nRet, TCP);
	if (!nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool DaZuRobot::SetFlangetoTCP(QTextBrowser* browser, std::array<double, 6> TCP, string TCP_Name) {
	int nRet = HRIF_SetTCPByName(boxID, rbtID, TCP_Name);
	this->SetFlangetoTCP(browser, TCP);
	UtilityFunction->PrintError(browser, nRet, "Set tcp by name");
	if (!nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool DaZuRobot::ConfigFlangetoTCP(QTextBrowser* browser, std::array<double, 6> TCP, string TCP_Name) {
	int nRet = HRIF_ConfigTCP(boxID, rbtID, TCP_Name, TCP[0], TCP[1], TCP[2], TCP[3], TCP[4], TCP[5]);
	UtilityFunction->PrintError(browser, nRet, "ConfigTCP by " + TCP_Name);
	UtilityFunction->PrintTCP(browser, nRet, TCP);
	this->SetFlangetoTCP(browser, TCP, TCP_Name);
	this->SetMoveToolMotion(browser, 1);
	this->SetMoveToolMotion(browser, 0);
	this->SetMoveToolMotion(browser, 1);
	this->SetFlangetoTCP(browser, TCP, TCP_Name);
	if (!nRet)
	{
		return true;
	}
	else
	{
		return false;
	}

}

bool DaZuRobot::ConfigBasetoUCS(QTextBrowser* browser, std::array<double, 6> UCS, string UCS_Name) {
	int nRet = HRIF_ConfigUCS(boxID, rbtID, UCS_Name, UCS[0], UCS[1], UCS[2], UCS[3], UCS[4], UCS[5]);
	UtilityFunction->PrintError(browser, nRet, "ConfigUCS by " + UCS_Name);
	UtilityFunction->PrintUCS(browser, nRet, UCS);
	nRet = HRIF_SetUCSByName(boxID, rbtID, UCS_Name);
	UtilityFunction->PrintError(browser, nRet, "SetUCSByName");
	this->SetBasetoUCS(browser, UCS);
	if (!nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool DaZuRobot::SetBasetoUCS(QTextBrowser* browser, std::array<double, 6> UCS) {
	int nRet = HRIF_SetUCS(boxID, rbtID, UCS[0], UCS[1], UCS[2], UCS[3], UCS[4], UCS[5]);
	UtilityFunction->PrintError(browser, nRet, "SetUCS ");
	if (!nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}


bool DaZuRobot::SetVelocity_dAcc_dRadius(QTextBrowser* browser, double Velocity, double dAcc, double dRadius) {
	if (Velocity > dAcc)
	{
		std::cout << "input is error Velocity must be less than dAcc" << std::endl;
		return false;
	}
	else
	{
		std::cout << "set succeed" << std::endl;
		this->dAcc = dAcc;
		this->dVelocity = Velocity;
		this->dRadius = dRadius;
		return true;
	}
}

bool DaZuRobot::isMoving(QTextBrowser* browser) {
	int nMovingState = 0; int nEnableState = 0; int nErrorState = 0; int nErrorCode = 0;
	int nErrorAxis = 0; int nBreaking = 0; int nPause = 0; int nEmergencyStop = 0;
	int nSaftyGuard = 0; int nElectrify = 0; int nIsConnectToBox = 0; int nBlendingDone = 0; int nInPos = 0;
	// 读取状态
	int nRet = HRIF_ReadRobotState(0, 0, nMovingState, nEnableState, nErrorState, nErrorCode, nErrorAxis,
		nBreaking, nPause, nEmergencyStop, nSaftyGuard, nElectrify, nIsConnectToBox, nBlendingDone, nInPos);
	if (!nRet)
	{
		//运动完成
		return nMovingState;
	}
	else
	{
		std::cout << "ismoving model is error please check " << std::endl;
		UtilityFunction->PrintError(browser, nRet, "Judge is moving done Error");
		return false;
	}
}

bool DaZuRobot::isnSaftyGuard(QTextBrowser* browser) {
	int nMovingState = 0; int nEnableState = 0; int nErrorState = 0; int nErrorCode = 0;
	int nErrorAxis = 0; int nBreaking = 0; int nPause = 0; int nEmergencyStop = 0;
	int nSaftyGuard = 0; int nElectrify = 0; int nIsConnectToBox = 0; int nBlendingDone = 0; int nInPos = 0;
	// 读取状态
	int nRet = HRIF_ReadRobotState(0, 0, nMovingState, nEnableState, nErrorState, nErrorCode, nErrorAxis,
		nBreaking, nPause, nEmergencyStop, nSaftyGuard, nElectrify, nIsConnectToBox, nBlendingDone, nInPos);
	if (!nRet)
	{
		return nSaftyGuard;
	}
	else
	{
		std::cout << "ismoving model is error please check " << std::endl;
		UtilityFunction->PrintError(browser, nRet, "Judge is moving done Error");
		return false;
	}
}

//1是
bool DaZuRobot::SetMoveToolMotion(QTextBrowser* browser, bool toolmotion) {
	this->ToolMotion = toolmotion;
	int nRet = HRIF_SetToolMotion(boxID, rbtID, this->ToolMotion);
	UtilityFunction->PrintError(browser, nRet, "Set toolmotion :");
	if (!nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}

std::array<double, 6> DaZuRobot::CaculateTargetRel(QTextBrowser* browser, std::array<double, 6> Distance) {
	std::array<double, 6> TCP = this->ReadBasetoTCP(browser);
	auto trans_BasetoTCP = vtkTransform::New();
	this->TranslateAndRotateMatrix(trans_BasetoTCP, TCP);
	if (this->ToolMotion) {
		trans_BasetoTCP->Translate(Distance[0], Distance[1], Distance[2]);
		trans_BasetoTCP->RotateZ(Distance[5]);
		trans_BasetoTCP->RotateY(Distance[4]);
		trans_BasetoTCP->RotateX(Distance[3]);
	}
	else {
		trans_BasetoTCP->PostMultiply();
		trans_BasetoTCP->RotateX(Distance[5]);
		trans_BasetoTCP->RotateY(Distance[4]);
		trans_BasetoTCP->RotateZ(Distance[3]);
		trans_BasetoTCP->Translate(Distance[0], Distance[1], Distance[2]);
	}
	trans_BasetoTCP->Update();

	auto vtk_BasetoTarget = vtkMatrix4x4::New();
	vtk_BasetoTarget->DeepCopy(trans_BasetoTCP->GetMatrix());

	Eigen::Vector3d Angle = this->GetAulerAngle(trans_BasetoTCP);
	std::array<double, 6> Target{ vtk_BasetoTarget->GetElement(0,3), vtk_BasetoTarget->GetElement(1,3) , vtk_BasetoTarget->GetElement(2,3),
		vtkMath::DegreesFromRadians(Angle(2)),vtkMath::DegreesFromRadians(Angle(1)),vtkMath::DegreesFromRadians(Angle(0)) };
	return Target;
}

bool DaZuRobot::RelMoveXplus(QTextBrowser* browser, double Distance) {
	std::array<double, 6> Dis{ Distance, 0,0,0,0,0 };

	std::array<double, 6> Target = this->CaculateTargetRel(browser, Dis);

	bool nRet = this->MoveJ(browser, Target);
	UtilityFunction->PrintError(browser, !nRet, "Rel Move : xplus");

	if (nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}
bool DaZuRobot::RelMoveYplus(QTextBrowser* browser, double Distance) {
	std::array<double, 6> Dis{ 0, Distance,0,0,0,0 };

	std::array<double, 6> Target = this->CaculateTargetRel(browser, Dis);
	bool nRet = this->MoveJ(browser, Target);
	UtilityFunction->PrintError(browser, !nRet, "Rel Move : yplus");

	if (nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool DaZuRobot::RelMoveZplus(QTextBrowser* browser, double Distance) {
	std::array<double, 6> Dis{ 0, 0, Distance,0,0,0 };

	std::array<double, 6> Target = this->CaculateTargetRel(browser, Dis);
	bool nRet = this->MoveJ(browser, Target);
	UtilityFunction->PrintError(browser, !nRet, "Rel Move : zplus");
	if (nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}
bool DaZuRobot::RelMoveRxplus(QTextBrowser* browser, double Distance) {
	//定义运动坐标类型  0-当前选择的用户坐标运动   1-按Tool坐标运动
	std::array<double, 6> Dis{ 0, 0, 0, Distance,0,0 };

	std::array<double, 6> Target = this->CaculateTargetRel(browser, Dis);
	bool nRet = this->MoveJ(browser, Target);
	UtilityFunction->PrintError(browser, !nRet, "Rel Move : Rxplus");
	if (nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}
bool DaZuRobot::RelMoveRyplus(QTextBrowser* browser, double Distance) {
	std::array<double, 6> Dis{ 0, 0, 0,0,Distance,0 };

	std::array<double, 6> Target = this->CaculateTargetRel(browser, Dis);
	bool nRet = this->MoveJ(browser, Target);
	UtilityFunction->PrintError(browser, !nRet, "Rel Move : Ryplus");
	if (nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}
bool DaZuRobot::RelMoveRzplus(QTextBrowser* browser, double Distance) {
	std::array<double, 6> Dis{ 0, 0, 0,0,0,Distance };

	std::array<double, 6> Target = this->CaculateTargetRel(browser, Dis);
	bool nRet = this->MoveJ(browser, Target);
	UtilityFunction->PrintError(browser, !nRet, "Rel Move : Rzplus");
	if (nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}
bool DaZuRobot::RelMoveXmin(QTextBrowser* browser, double Distance) {
	std::array<double, 6> Dis{ -Distance, 0, 0,0,0, 0 };

	std::array<double, 6> Target = this->CaculateTargetRel(browser, Dis);
	bool nRet = this->MoveJ(browser, Target);
	UtilityFunction->PrintError(browser, !nRet, "Rel Move : Rzplus");
	if (nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}
bool DaZuRobot::RelMoveYmin(QTextBrowser* browser, double Distance) {
	std::array<double, 6> Dis{ 0, -Distance, 0,0,0,0 };

	std::array<double, 6> Target = this->CaculateTargetRel(browser, Dis);
	bool nRet = this->MoveJ(browser, Target);
	UtilityFunction->PrintError(browser, !nRet, "Rel Move : Rzplus");
	if (nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}
bool DaZuRobot::RelMoveZmin(QTextBrowser* browser, double Distance) {
	std::array<double, 6> Dis{ 0, 0, -Distance,0,0,0 };

	std::array<double, 6> Target = this->CaculateTargetRel(browser, Dis);
	bool nRet = this->MoveJ(browser, Target);
	UtilityFunction->PrintError(browser, !nRet, "Rel Move : Rzplus");
	if (nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}
bool DaZuRobot::RelMoveRxmin(QTextBrowser* browser, double Distance) {
	std::array<double, 6> Dis{ 0,0, 0, -Distance,0,0 };

	std::array<double, 6> Target = this->CaculateTargetRel(browser, Dis);
	bool nRet = this->MoveJ(browser, Target);
	UtilityFunction->PrintError(browser, !nRet, "Rel Move : Rzplus");
	if (nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}
bool DaZuRobot::RelMoveRymin(QTextBrowser* browser, double Distance) {
	std::array<double, 6> Dis{ 0,0, 0, 0,-Distance,0 };

	std::array<double, 6> Target = this->CaculateTargetRel(browser, Dis);
	bool nRet = this->MoveJ(browser, Target);
	UtilityFunction->PrintError(browser, !nRet, "Rel Move : Rzplus");
	if (nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}
bool DaZuRobot::RelMoveRzmin(QTextBrowser* browser, double Distance) {
	std::array<double, 6> Dis{ 0,0, 0,0,0, -Distance };

	std::array<double, 6> Target = this->CaculateTargetRel(browser, Dis);
	bool nRet = this->MoveJ(browser, Target);
	UtilityFunction->PrintError(browser, !nRet, "Rel Move : Rzplus");
	if (nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}

//flangetoTCP
std::array<double, 6> DaZuRobot::ReadFlangetoTCP(QTextBrowser* browser) {
	std::array<double, 6> FlangeToTCP{ 0 };
	int nRet = HRIF_ReadCurTCP(boxID, rbtID, FlangeToTCP[0], FlangeToTCP[1], FlangeToTCP[2], FlangeToTCP[3], FlangeToTCP[4], FlangeToTCP[5]);
	UtilityFunction->PrintError(browser, nRet, "ReadFlangetoTCP :");
	UtilityFunction->PrintTCP(browser, nRet, FlangeToTCP);

	return FlangeToTCP;
}
//basetoTCP
std::array<double, 6> DaZuRobot::ReadBasetoTCP(QTextBrowser* browser) {
	std::array<double, 6> BaseToTCP{ 0 };
	int nRet = HRIF_ReadActTcpPos(boxID, rbtID, BaseToTCP[0], BaseToTCP[1], BaseToTCP[2], BaseToTCP[3], BaseToTCP[4], BaseToTCP[5]);
	UtilityFunction->PrintError(browser, nRet, "ReadBaseToTCP :");
	UtilityFunction->PrintTCP(browser, nRet, BaseToTCP);

	return BaseToTCP;
}

std::array<double, 6> DaZuRobot::ReadBasetoTCP(QTextBrowser* browser, string sTcpName) {
	std::array<double, 6> BaseToTCP{ 0 };

	int nRet = HRIF_ReadTCPByName(boxID, rbtID, sTcpName, BaseToTCP[0], BaseToTCP[1], BaseToTCP[2], BaseToTCP[3], BaseToTCP[4], BaseToTCP[5]);
	UtilityFunction->PrintError(browser, nRet, "ReadBaseToTCP :" + sTcpName);
	UtilityFunction->PrintTCP(browser, nRet, BaseToTCP);

	return BaseToTCP;
}

std::array<double, 6> DaZuRobot::ReadBasetoFlange(QTextBrowser* browser) {
	std::array<double, 6> BasetoTCP = this->ReadBasetoTCP(browser);
	std::array<double, 6> FlangetoTCP = this->ReadFlangetoTCP(browser);
	auto Trans_BasetoTCP = vtkTransform::New();
	auto Trans_FlangetoTCP = vtkTransform::New();
	auto vtk_BasetoTCP = vtkMatrix4x4::New();
	auto vtk_TCPtoFlange = vtkMatrix4x4::New();
	this->TranslateAndRotateMatrix(Trans_BasetoTCP, BasetoTCP);
	this->TranslateAndRotateMatrix(Trans_FlangetoTCP, FlangetoTCP);
	vtk_BasetoTCP->DeepCopy(Trans_BasetoTCP->GetMatrix());
	vtk_TCPtoFlange->DeepCopy(Trans_FlangetoTCP->GetMatrix());
	vtk_TCPtoFlange->Invert();

	auto Trans_BaseToFlange = vtkTransform::New();
	UtilityFunction->MultiplyMatrices(Trans_BaseToFlange, vtk_BasetoTCP, vtk_TCPtoFlange);

	auto vtk_BasetoFlange = vtkMatrix4x4::New();
	vtk_BasetoFlange->DeepCopy(Trans_BaseToFlange->GetMatrix());

	Eigen::Vector3d Angle = this->GetAulerAngle(vtk_BasetoFlange);

	std::array<double, 6> BasetoFlange{ vtk_BasetoFlange->GetElement(0, 3), vtk_BasetoFlange->GetElement(1, 3),
		vtk_BasetoFlange->GetElement(2, 3), vtkMath::DegreesFromRadians(Angle(2)),vtkMath::DegreesFromRadians(Angle(1)),vtkMath::DegreesFromRadians(Angle(0)) };

	UtilityFunction->PrintTCP(browser, 0, BasetoFlange);

	return BasetoFlange;

}

std::array<double, 6> DaZuRobot::ReadTargetBasetoTCP(QTextBrowser* browser) {
	std::array<double, 6> TargetBaseToTCP{ 0 };
	int nRet = HRIF_ReadCmdTcpPos(boxID, rbtID, TargetBaseToTCP[0], TargetBaseToTCP[1], TargetBaseToTCP[2],
		TargetBaseToTCP[3], TargetBaseToTCP[4], TargetBaseToTCP[5]);
	UtilityFunction->PrintError(browser, nRet, "ReadBaseToTCP :");
	UtilityFunction->PrintTCP(browser, nRet, TargetBaseToTCP);

	return TargetBaseToTCP;
}

std::array<double, 6> DaZuRobot::ReadJoint(QTextBrowser* browser) {
	std::array<double, 6> Joint{ 0 };
	int nRet = HRIF_ReadActJointPos(boxID, rbtID, Joint[0], Joint[1], Joint[2], Joint[3], Joint[4], Joint[5]);
	UtilityFunction->PrintError(browser, nRet, "ReadBaseToTCP :");
	UtilityFunction->PrintJoint(browser, nRet, Joint);

	return Joint;
}

std::array<double, 6> DaZuRobot::ReadTargetJoint(QTextBrowser* browser) {
	std::array<double, 6> Joint{ 0 };
	int nRet = HRIF_ReadCmdJointPos(boxID, rbtID, Joint[0], Joint[1], Joint[2], Joint[3], Joint[4], Joint[5]);
	UtilityFunction->PrintError(browser, nRet, "ReadBaseToTCP :");
	UtilityFunction->PrintJoint(browser, nRet, Joint);

	return Joint;
}

std::array<double, 6> DaZuRobot::ReadUCS(QTextBrowser* browser) {
	std::array<double, 6> UCS{ 0 };

	int nRet = HRIF_ReadCurUCS(boxID, rbtID, UCS[0], UCS[1], UCS[2], UCS[3], UCS[4], UCS[5]);
	UtilityFunction->PrintError(browser, nRet, "ReadBaseToTCP :");
	UtilityFunction->PrintUCS(browser, nRet, UCS);

	return UCS;
}

std::array<double, 6> DaZuRobot::ReadUCS(QTextBrowser* browser, string UCSName) {
	std::array<double, 6> UCS{ 0 };

	int nRet = HRIF_ReadUCSByName(boxID, rbtID, UCSName, UCS[0], UCS[1], UCS[2], UCS[3], UCS[4], UCS[5]);
	UtilityFunction->PrintError(browser, nRet, "ReadBaseToTCP :" + UCSName);
	UtilityFunction->PrintUCS(browser, nRet, UCS);

	return UCS;
}

std::array<double, 6> DaZuRobot::GetInverseKin(QTextBrowser* browser, std::array<double, 6> TargetPoint) {

	std::array<double, 6> UCS = this->ReadUCS(browser, "Base");
	std::array<double, 6> FlangetoTCP = this->ReadFlangetoTCP(browser);
	std::array<double, 6> CurrentJoint = this->ReadJoint(browser);
	std::array<double, 6> TargetJoint{ 0,0,0,0,0,0 };

	int nRet = HRIF_GetInverseKin(boxID, rbtID, TargetPoint[0], TargetPoint[1], TargetPoint[2], TargetPoint[3], TargetPoint[4], TargetPoint[5],
		FlangetoTCP[0], FlangetoTCP[1], FlangetoTCP[2], FlangetoTCP[3], FlangetoTCP[4], FlangetoTCP[5],
		UCS[0], UCS[1], UCS[2], UCS[3], UCS[4], UCS[5],
		CurrentJoint[0], CurrentJoint[1], CurrentJoint[2], CurrentJoint[3], CurrentJoint[4], CurrentJoint[5],
		TargetJoint[0], TargetJoint[1], TargetJoint[2], TargetJoint[3], TargetJoint[4], TargetJoint[5]);
	UtilityFunction->PrintError(browser, nRet, "GetInverseKin :");
	UtilityFunction->PrintJoint(browser, nRet, TargetJoint);

	return TargetJoint;
}
//关节运动
bool DaZuRobot::MoveJ(QTextBrowser* browser, std::array<double, 6> Target) {

	int nMoveType = 0;
	int isUseJoint = 0;

	std::array<double, 6> TargetJoint = this->GetInverseKin(browser, Target);
	std::array<double, 6> UCS = this->ReadUCS(browser, "Base");
	std::array<double, 6> FlangetoTCP = this->ReadFlangetoTCP(browser);

	int nRet = HRIF_WayPointEx(boxID, rbtID, nMoveType, Target[0], Target[1], Target[2], Target[3], Target[4], Target[5],
		TargetJoint[0], TargetJoint[1], TargetJoint[2], TargetJoint[3], TargetJoint[4], TargetJoint[5],
		FlangetoTCP[0], FlangetoTCP[1], FlangetoTCP[2], FlangetoTCP[3], FlangetoTCP[4], FlangetoTCP[5],
		UCS[0], UCS[1], UCS[2], UCS[3], UCS[4], UCS[5],
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

bool DaZuRobot::MoveJ(QTextBrowser* browser, std::array<double, 6> Target, bool isUseJoint) {

	int nMoveType = 0;

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
			return true;
		}
		else
		{
			return false;
		}
	}

}

//空间直线运动
bool DaZuRobot::MoveP(QTextBrowser* browser, std::array<double, 6> TargetPoint) {
	int nIsUseJoint = 0;
	int nMoveType = 1;

	std::array<double, 6> TargetJoint = this->GetInverseKin(browser, TargetPoint);
	std::array<double, 6> UCS = this->ReadUCS(browser, "Base");
	std::array<double, 6> FlangetoTCP = this->ReadFlangetoTCP(browser);

	int nRet = HRIF_WayPointEx(boxID, rbtID, nMoveType, TargetPoint[0], TargetPoint[1], TargetPoint[2], TargetPoint[3], TargetPoint[4], TargetPoint[5],
		TargetJoint[0], TargetJoint[1], TargetJoint[2], TargetJoint[3], TargetJoint[4], TargetJoint[5],
		FlangetoTCP[0], FlangetoTCP[1], FlangetoTCP[2], FlangetoTCP[3], FlangetoTCP[4], FlangetoTCP[5],
		UCS[0], UCS[1], UCS[2], UCS[3], UCS[4], UCS[5],
		dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
	UtilityFunction->PrintError(browser, nRet, "MoveL");
	if (!nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool DaZuRobot::SetForceFreeDrive(QTextBrowser* browser, int nMode) {
	int nRet = HRIF_SetForceFreeDriveMode(boxID, rbtID, nMode);
	UtilityFunction->PrintError(browser, nRet, "SetFreeDriveMode");
	if (!nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool DaZuRobot::SetForceFreeDriveFreedom(QTextBrowser* browser, std::array<double, 6> Freedom) {
	int nRet = HRIF_SetFreeDriveMotionFreedom(0, 0, Freedom[0], Freedom[1], Freedom[2], Freedom[3], Freedom[4], Freedom[5]);
	UtilityFunction->PrintError(browser, nRet, "SetFreeDriveFreedom");
	if (!nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool DaZuRobot::SetForceFTFreeDriveFaztor(QTextBrowser* browser, double linerfaztor, double anglefaztor) {
	int nRet = HRIF_SetFTFreeFactor(0, 0, linerfaztor, anglefaztor);
	UtilityFunction->PrintError(browser, nRet, "SetForceFTFreeDriveFaztor");
	if (!nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool DaZuRobot::SetForceFreeDriveVelocity(QTextBrowser* browser, double Maxlinervelocity, double Maxanglevelocity) {
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

bool DaZuRobot::SetForceToolCoordinateMotion(QTextBrowser* browser, int nMode) {
	int nRet = HRIF_SetForceToolCoordinateMotion(0, 0, nMode);
	UtilityFunction->PrintError(browser, nRet, "SetForceToolCoordinateMotion");
	if (!nRet)
	{
		return true;
	}
	else
	{
		return false;
	}
}