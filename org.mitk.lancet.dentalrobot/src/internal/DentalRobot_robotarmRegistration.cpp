#include "DentalRobot.h"

// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qt
#include  <Qtimer>
#include <QMessageBox>
#include <QInputDialog>
#include <QtWidgets/qfiledialog.h>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QLineEdit>
#include <QKeyEvent>
#include <QApplication>
#include <QShortcut>

// mitk image
#include <mitkImage.h>
#include <mitkAffineTransform3D.h>
#include <mitkMatrixConvert.h>
#include "mitkNodePredicateProperty.h"
#include <mitkDataNode.h>
#include <mitkRenderingManager.h>
#include <mitkBaseRenderer.h>
#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include "lancetTrackingDeviceSourceConfigurator.h"
#include "lancetVegaTrackingDevice.h"
#include "mitkImageToSurfaceFilter.h"
#include "mitkMatrixConvert.h"
#include "mitkNavigationToolStorageDeserializer.h"
#include "mitkPointSet.h"
#include "mitkSurfaceToImageFilter.h"
#include "QmitkDataStorageTreeModel.h"
#include "QmitkRenderWindow.h"
#include "surfaceregistraion.h"
#include <vtkSphere.h>
#include <vtkLandmarkTransform.h>
#include "mitkNodePredicateAnd.h"
#include "mitkNodePredicateDataType.h"
#include "mitkNodePredicateNot.h"
#include "mitkNodePredicateOr.h"
#include "mitkNodePredicateProperty.h"
#include <mitkImageAccessByItk.h>
#include <vtkIterativeClosestPointTransform.h>
#include "PrintDataHelper.h"
#include <iomanip>
#include <mitkPoint.h>
#include <mitkPointSet.h>


//math
#include "windows.h"
#include "direct.h"
#include "io.h"
#include <iostream>
#include <math.h>
#include <algorithm>
#include <thread>
#include <mutex>
#include "stdafx.h"
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <random>
#include <vtkKdTree.h>
#include <vtkTransformPolyDataFilter.h>

//Aimooe Camera
#include "AimPositionAPI.h"
#include "AimPositionDef.h"

//DaZu Robot
#include "HR_Pro.h"

//log
#include <fstream>

//口腔机器人
#include <vtkImageCast.h>
#include <vtkImageIterator.h>
#include <vtkConnectivityFilter.h>

#include <QFileDialog>
#include <vtkAppendPolyData.h>
#include <vtkCamera.h>
#include <vtkCardinalSpline.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkCenterOfMass.h>
#include <vtkCleanPolyData.h>
#include <vtkClipPolyData.h>
#include <vtkConnectivityFilter.h>
#include <vtkFillHolesFilter.h>
#include <vtkImageAppend.h>
#include <vtkImageCast.h>
#include <vtkImageIterator.h>
#include <vtkImplicitPolyDataDistance.h>
#include <vtkOBBTree.h>
#include <vtkPlane.h>
#include <vtkPlanes.h>
#include <vtkPlaneSource.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>
#include <vtkProbeFilter.h>
#include <vtkRendererCollection.h>
#include <vtkSplineFilter.h>
#include <vtkTransformPolyDataFilter.h>
#include <ep/include/vtk-9.1/vtkTransformFilter.h>
#include "mitkSurfaceVtkMapper3D.h"
#include "lancetTrackingDeviceSourceConfigurator.h"
#include "lancetVegaTrackingDevice.h"
#include "leastsquaresfit.h"
#include "mitkImageToSurfaceFilter.h"
#include "mitkMatrixConvert.h"
#include "mitkNavigationToolStorageDeserializer.h"
#include "mitkPointSet.h"
#include "mitkSurfaceToImageFilter.h"
#include "QmitkDataStorageTreeModel.h"
#include "QmitkRenderWindow.h"
#include "surfaceregistraion.h"
#include <vtkSphere.h>
#include <mitkImageAccessByItk.h>

#include "PrintDataHelper.h"

void DentalRobot::setTCPToFlange()
{
	dTcp_X = 0;
	dTcp_Y = 0;
	dTcp_Z = 0;
	dTcp_Rx = 0;
	dTcp_Ry = 0;
	dTcp_Rz = 0;
	int nRet = HRIF_SetTCP(0, 0, dTcp_X, dTcp_Y, dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz);
	PrintResult(m_Controls.textBrowser, nRet, "Set Flange");
}

void DentalRobot::setInitialPoint()
{
	int result = HRIF_ReadActPos(0, 0, g_init_X, g_init_Y, g_init_Z, g_init_Rx, g_init_Ry, g_init_Rz, dJ1_init, dJ2_init, dJ3_init, dJ4_init,
		dJ5_init, dJ6_init, dTcp_X, dTcp_Y, dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz, dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz);
	PrintResult(m_Controls.textBrowser, result, "SetInitialPoint");
}

void DentalRobot::goToInitial()
{
	int nIsUseJoint = 1;
	int nIsSeek = 0;
	int nIOBit = 0;
	int nIOState = 0;
	//int result = HRIF_MoveJ(0, 0, g_init_X, g_init_Y, g_init_Z, g_init_Rx, g_init_Ry, g_init_Rz,
	//	dJ1_init, dJ2_init, dJ3_init, dJ4_init, dJ5_init, dJ6_init, sTcpName, sUcsName, dVelocity, dAcc, dRadius,
	//	nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
	int result = HRIF_WayPoint(0, 0, nMoveType, g_init_X, g_init_Y, g_init_Z, g_init_Rx, g_init_Ry, g_init_Rz,
		dJ1_init, dJ2_init, dJ3_init, dJ4_init, dJ5_init, dJ6_init, sTcpName, sUcsName, dVelocity, dAcc, dRadius,
		nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
	PrintResult(m_Controls.textBrowser, result, "Goto Initial Position");
}



void DentalRobot::captureRobot()
{
	m_Controls.textBrowser->append("captureRobot");
	if (m_IndexOfRobotCapture < 5) //The first five translations, 
	{
		m_IndexOfRobotCapture++;
		std::cout << "m_IndexOfRobotCapture: " << m_IndexOfRobotCapture << std::endl;
		m_Controls.lineEdit_collectedRoboPose->setText(QString::number(m_IndexOfRobotCapture));
		CapturePose(true);
	}
	else if (m_IndexOfRobotCapture < 10) //the last five rotations
	{
		m_IndexOfRobotCapture++;
		m_Controls.lineEdit_collectedRoboPose->setText(QString::number(m_IndexOfRobotCapture));
		std::cout << "m_IndexOfRobotCapture: " << m_IndexOfRobotCapture << std::endl;
		CapturePose(false);
	}
	else
	{
		//MITK_INFO << "OnRobotCapture finish: " << m_IndexOfRobotCapture;
		vtkNew<vtkMatrix4x4> robotEndToFlangeMatrix;
		m_RobotRegistration.GetTCPmatrix(robotEndToFlangeMatrix);

		vtkMatrix4x4* matrix4x4 = vtkMatrix4x4::New();
		m_RobotRegistration.GetRegistraionMatrix(matrix4x4);

		double x = robotEndToFlangeMatrix->GetElement(0, 3);
		double y = robotEndToFlangeMatrix->GetElement(1, 3);
		double z = robotEndToFlangeMatrix->GetElement(2, 3);
		std::cout << "X: " << x << std::endl;
		std::cout << "Y: " << y << std::endl;
		std::cout << "Z: " << z << std::endl;

		robotEndToFlangeMatrix->Invert();

		m_Controls.textBrowser->append("Registration RMS: " + QString::number(m_RobotRegistration.RMS()));
		std::cout << "Registration RMS: " << m_RobotRegistration.RMS() << std::endl;

		vtkMatrix4x4* vtkT_BaseToBaseRF = vtkMatrix4x4::New();
		m_RobotRegistration.GetRegistraionMatrix(vtkT_BaseToBaseRF);
		vtkT_BaseToBaseRF->Invert();
		memcpy_s(m_T_BaseToBaseRF, sizeof(double) * 16, vtkT_BaseToBaseRF->GetData(), sizeof(double) * 16);

		vtkMatrix4x4* vtkT_FlangeToEndRF = vtkMatrix4x4::New();
		m_RobotRegistration.GetTCPmatrix(vtkT_FlangeToEndRF);
		memcpy_s(m_T_FlangeToEdnRF, sizeof(double) * 16, vtkT_FlangeToEndRF->GetData(), sizeof(double) * 16);
	}
}

void DentalRobot::CapturePose(bool translationOnly)
{

	//Read TCP get T_BaseToFlanger
	double dX = 0; double dY = 0; double dZ = 0;
	double dRx = 0; double dRy = 0; double dRz = 0;
	int nRet = HRIF_ReadActTcpPos(0, 0, dX, dY, dZ, dRx, dRy, dRz);

	auto tmpTrans = vtkTransform::New();
	tmpTrans->PostMultiply();
	tmpTrans->RotateX(dRx);
	tmpTrans->RotateY(dRy);
	tmpTrans->RotateZ(dRz);


	tmpTrans->Translate(dX, dY, dZ);
	tmpTrans->Update();

	//VTKT_BaseToFlanger
	vtkSmartPointer<vtkMatrix4x4> VTKT_BaseToFlanger = tmpTrans->GetMatrix();
	QVector<double> _vtkMatrix4x4;
	_vtkMatrix4x4 = { VTKT_BaseToFlanger->GetElement(0,0), VTKT_BaseToFlanger->GetElement(0, 1), VTKT_BaseToFlanger->GetElement(0, 2), VTKT_BaseToFlanger->GetElement(0,3),
					  VTKT_BaseToFlanger->GetElement(1, 0),VTKT_BaseToFlanger->GetElement(1, 1), VTKT_BaseToFlanger->GetElement(1, 2), VTKT_BaseToFlanger->GetElement(1,3),
					  VTKT_BaseToFlanger->GetElement(2, 0), VTKT_BaseToFlanger->GetElement(2, 1), VTKT_BaseToFlanger->GetElement(2, 2), VTKT_BaseToFlanger->GetElement(2,3),
					  VTKT_BaseToFlanger->GetElement(3, 0), VTKT_BaseToFlanger->GetElement(3, 1), VTKT_BaseToFlanger->GetElement(3, 2), VTKT_BaseToFlanger->GetElement(3,3)
	};

	auto VTKT_CameratoEndRF = vtkMatrix4x4::New();
	VTKT_CameratoEndRF->DeepCopy(m_T_CamToEndRF);

	auto VTKT_BaseRFToCamera = vtkMatrix4x4::New();
	VTKT_BaseRFToCamera->DeepCopy(m_T_CamToBaseRF);
	VTKT_BaseRFToCamera->Invert();

	//VTKT_BassRFToEndRF
	vtkNew<vtkTransform> tmpTransform;
	tmpTransform->PostMultiply();
	tmpTransform->Identity();
	tmpTransform->SetMatrix(VTKT_CameratoEndRF);
	tmpTransform->Concatenate(VTKT_BaseRFToCamera);
	tmpTransform->Update();
	auto vtkBaseRFtoRoboEndRFMatrix = tmpTransform->GetMatrix();
	//Robotic arm registration
	m_RobotRegistration.AddPoseWithVtkMatrix(VTKT_BaseToFlanger, vtkBaseRFtoRoboEndRFMatrix, translationOnly);
}

void DentalRobot::replaceRegistration()
{
	m_Controls.textBrowser->append("Replace Registration");
	m_RobotRegistration.RemoveAllPose();
	m_IndexOfRobotCapture = 0;
	m_Controls.lineEdit_collectedRoboPose->setText(QString::number(0));

}

void DentalRobot::saveArmMatrix()
{
	std::ofstream robotMatrixFile(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\T_BaseToBaseRF.txt");
	for (int i = 0; i < 16; i++) {
		robotMatrixFile << m_T_BaseToBaseRF[i];
		if (i != 15) {
			robotMatrixFile << ",";
		}
		else {
			robotMatrixFile << ";";
		}
	}
	robotMatrixFile << std::endl;
	robotMatrixFile.close();
	std::ofstream robotMatrixFile1(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\T_FlangeToEndRF.txt");
	for (int i = 0; i < 16; i++) {
		robotMatrixFile1 << m_T_FlangeToEdnRF[i];
		if (i != 15) {
			robotMatrixFile1 << ",";
		}
		else {
			robotMatrixFile1 << ";";
		}
	}
	robotMatrixFile1 << std::endl;
	robotMatrixFile1.close();
	m_Controls.textBrowser->append("saveArmMatrix");
}

void DentalRobot::reuseArmMatrix()
{
	std::ifstream inputFile(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\T_BaseToBaseRF.txt");
	if (inputFile.is_open()) {
		std::string line;
		if (std::getline(inputFile, line)) {
			std::stringstream ss(line);
			std::string token;
			int index = 0;
			while (std::getline(ss, token, ',')) {
				m_T_BaseToBaseRF[index] = std::stod(token);
				index++;
			}
		}
		inputFile.close();
	}
	else {

		m_Controls.textBrowser->append("无法打开文件:T_BaseToBaseRF.txt");
	}
	PrintDataHelper::AppendTextBrowserMatrix(m_Controls.textBrowser, "T_BaseToBaseRF", m_T_BaseToBaseRF);
	PrintArray16ToMatrix("T_BaseToBaseRF", m_T_BaseToBaseRF);

	//导入T_FlangeToEdnRF
	std::ifstream inputFile2(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\T_FlangeToEndRF.txt");
	if (inputFile2.is_open()) {
		std::string line2;
		if (std::getline(inputFile2, line2)) {
			std::stringstream ss2(line2);
			std::string token2;
			int index2 = 0;
			while (std::getline(ss2, token2, ',')) {
				m_T_FlangeToEdnRF[index2] = std::stod(token2);
				index2++;
			}
		}
		inputFile2.close();
	}
	else {
		m_Controls.textBrowser->append("无法打开文件：T_FlangeToEndRF.txt");
	}
	//打印T_FlangeToEdnRF
	PrintDataHelper::AppendTextBrowserMatrix(m_Controls.textBrowser, "T_FlangeToEdnRF", m_T_FlangeToEdnRF);
	PrintArray16ToMatrix("T_FlangeToEdnRF", m_T_FlangeToEdnRF);
}

void DentalRobot::PrintArray16ToMatrix(const std::string& title, double* Array)
{
	std::cout << "+-----------------------------------------------+" << std::endl;
	std::cout << "|               " << title << "              |" << std::endl;
	std::cout << "+-----------+-----------+----------+------------+" << std::endl;
	for (int i = 0; i < 4; i++) {
		std::cout << "|  ";
		for (int j = 0; j < 4; j++) {
			std::cout << std::fixed << std::setprecision(6) << Array[i * 4 + j] << " ";
		}
		std::cout << "|" << std::endl;
	}
	std::cout << "+-----------+-----------+----------+------------+" << std::endl;
}

void DentalRobot::xp()
{
	m_Controls.textBrowser->append("xp");
	unsigned int boxID = 0;
	unsigned int rbtID = 0;
	int nToolMotion = 1;
	int nAxisID = 0;
	int nDirection = 1;
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser->append(QString::number(nMoveRelL));
	m_Controls.textBrowser->append("xp finish");
}

void DentalRobot::xm()
{
	m_Controls.textBrowser->append("xm");

	int nToolMotion = 1;//定义运动坐标类型  0-当前选择的用户坐标运动   1-按Tool坐标运动
	int nAxisID = 0;//定义轴 ID 0:x轴 1：y轴 2：z轴 3：rx轴 4：ry轴 5：rz轴
	int nDirection = 0;//定义运动方向 0：负向 1：正向
	double inputValue;

	inputValue = m_Controls.lineEdit_intuitiveValue->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser->append(QString::fromStdString(valueString));

	int nMoveRelL = HRIF_MoveRelL(0, 0, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser->append("xm finish");
}

void DentalRobot::yp()
{
	m_Controls.textBrowser->append("yp");
	int nToolMotion = 1;
	int nAxisID = 1;
	int nDirection = 1;
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(0, 0, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser->append("yp finish");
}

void DentalRobot::ym()
{
	m_Controls.textBrowser->append("ym");
	int nToolMotion = 1;
	int nAxisID = 1;
	int nDirection = 0;
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(0, 0, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser->append("ym finish");
}

void DentalRobot::zp()
{
	m_Controls.textBrowser->append("zp");
	int nToolMotion = 1;
	int nAxisID = 2;
	int nDirection = 1;
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(0, 0, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser->append("zp finish");
}

void DentalRobot::zm()
{
	m_Controls.textBrowser->append("zm");
	int nToolMotion = 1;
	int nAxisID = 2;
	int nDirection = 0;
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(0, 0, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser->append("zm finish");
}

void DentalRobot::rxp()
{
	m_Controls.textBrowser->append("rxp");
	int nToolMotion = 1;//定义运动坐标类型  0-当前选择的用户坐标运动   1-按Tool坐标运动
	int nAxisID = 3;//定义轴 ID 0:x轴 1：y轴 2：z轴 3：rx轴 4：ry轴 5：rz轴
	int nDirection = 1;//定义运动方向 0：负向 1：正向
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_5->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(0, 0, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser->append("rxp finish");
}

void DentalRobot::rxm()
{
	m_Controls.textBrowser->append("rxm");
	int nToolMotion = 1;//定义运动坐标类型  0-当前选择的用户坐标运动   1-按Tool坐标运动
	int nAxisID = 3;//定义轴 ID 0:x轴 1：y轴 2：z轴 3：rx轴 4：ry轴 5：rz轴
	int nDirection = 0;//定义运动方向 0：负向 1：正向
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_5->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(0, 0, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser->append("rxm finish");
}

void DentalRobot::ryp()
{
	m_Controls.textBrowser->append("ryp");
	int nToolMotion = 1;//定义运动坐标类型  0-当前选择的用户坐标运动   1-按Tool坐标运动
	int nAxisID = 4;//定义轴 ID 0:x轴 1：y轴 2：z轴 3：rx轴 4：ry轴 5：rz轴
	int nDirection = 1;//定义运动方向 0：负向 1：正向
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_5->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(0, 0, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser->append("ryp finish");
}

void DentalRobot::rym()
{
	m_Controls.textBrowser->append("rym");
	int nToolMotion = 1;//定义运动坐标类型  0-当前选择的用户坐标运动   1-按Tool坐标运动
	int nAxisID = 4;//定义轴 ID 0:x轴 1：y轴 2：z轴 3：rx轴 4：ry轴 5：rz轴
	int nDirection = 0;//定义运动方向 0：负向 1：正向
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_5->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(0, 0, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser->append("rym finish");
}

void DentalRobot::rzp()
{
	m_Controls.textBrowser->append("rzp");
	int nToolMotion = 1;//定义运动坐标类型  0-当前选择的用户坐标运动   1-按Tool坐标运动
	int nAxisID = 5;//定义轴 ID 0:x轴 1：y轴 2：z轴 3：rx轴 4：ry轴 5：rz轴
	int nDirection = 1;//定义运动方向 0：负向 1：正向
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_5->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(0, 0, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser->append("rzp finish");
}

void DentalRobot::rzm()
{
	m_Controls.textBrowser->append("rzm");
	int nToolMotion = 1;//定义运动坐标类型  0-当前选择的用户坐标运动   1-按Tool坐标运动
	int nAxisID = 5;//定义轴 ID 0:x轴 1：y轴 2：z轴 3：rx轴 4：ry轴 5：rz轴
	int nDirection = 0;//定义运动方向 0：负向 1：正向
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_5->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(0, 0, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser->append("rzm finish");
}

void DentalRobot::JiontAngleMotion()
{
	double DJ1 = m_Controls.lineEdit_DJ1->text().toDouble();
	double DJ2 = m_Controls.lineEdit_DJ2->text().toDouble();
	double DJ3 = m_Controls.lineEdit_DJ3->text().toDouble();
	double DJ4 = m_Controls.lineEdit_DJ4->text().toDouble();
	double DJ5 = m_Controls.lineEdit_DJ5->text().toDouble();
	double DJ6 = m_Controls.lineEdit_DJ6->text().toDouble();

	m_Controls.textBrowser->append("----------------------------------------------------");
	m_Controls.textBrowser->append("JiontAngle");
	m_Controls.textBrowser->append(QString::number(DJ1));
	m_Controls.textBrowser->append(QString::number(DJ2));
	m_Controls.textBrowser->append(QString::number(DJ3));
	m_Controls.textBrowser->append(QString::number(DJ4));
	m_Controls.textBrowser->append(QString::number(DJ5));
	m_Controls.textBrowser->append(QString::number(DJ6));
	m_Controls.textBrowser->append("----------------------------------------------------");

	int nIsUseJoint = 1;
	int nIsSeek = 0;
	int nIOBit = 0;
	int nIOState = 0;
	int result = HRIF_MoveJ(0, 0, g_init_X, g_init_Y, g_init_Z, g_init_Rx, g_init_Ry, g_init_Rz,
		DJ1, DJ2, DJ3, DJ4, DJ5, DJ6, sTcpName, sUcsName, dVelocity, dAcc, dRadius,
		nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
	PrintResult(m_Controls.textBrowser, result, "JiontMove");
}


//---------------------------------------------------------------------------------------------------------------
/**
 * @brief 点击按钮会去读save文件夹、在文件夹中有log_Point.txt文件的一组关节角，然后进行关节角运动
 * @note 需要在桌面上有save文件夹、在文件夹中有log_Point.txt文件，里面每一行存的都是一组关节角
 */
 //---------------------------------------------------------------------------------------------------------------
void DentalRobot::AutoMoveJ()
{
	//设定一个全局的变量，每次点击后增加一，并把对应的行数的数据拿出来，给关节角，然后运动
	m_Controls.textBrowser->append("Current AutoMoveJ_id: " + QString::number(AutoMoveJ_id));
	// 打开存储机器人角度的文件
	std::ifstream inputFile(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\log_Point.txt");
	std::vector<std::vector<double>> jointAngles;
	std::string line;
	while (std::getline(inputFile, line)) {
		std::replace(line.begin(), line.end(), ',', ' ');
		std::istringstream iss(line);
		std::vector<double> angles((std::istream_iterator<double>(iss)), std::istream_iterator<double>());
		if (angles.size() == 6) {
			jointAngles.push_back(angles);
		}
	}
	inputFile.close();

	if (AutoMoveJ_id < jointAngles.size()) {
		std::vector<double> selectedJointAngles = jointAngles[AutoMoveJ_id];
		double DJ1 = selectedJointAngles[0];
		double DJ2 = selectedJointAngles[1];
		double DJ3 = selectedJointAngles[2];
		double DJ4 = selectedJointAngles[3];
		double DJ5 = selectedJointAngles[4];
		double DJ6 = selectedJointAngles[5];

		m_Controls.textBrowser->append("----------------------------------------------------");
		m_Controls.textBrowser->append("JiontAngle");
		m_Controls.textBrowser->append(QString::number(DJ1));
		m_Controls.textBrowser->append(QString::number(DJ2));
		m_Controls.textBrowser->append(QString::number(DJ3));
		m_Controls.textBrowser->append(QString::number(DJ4));
		m_Controls.textBrowser->append(QString::number(DJ5));
		m_Controls.textBrowser->append(QString::number(DJ6));
		m_Controls.textBrowser->append("----------------------------------------------------");

		int nIsUseJoint = 1;
		int nIsSeek = 0;
		int nIOBit = 0;
		int nIOState = 0;
		int result = HRIF_MoveJ(0, 0, g_init_X, g_init_Y, g_init_Z, g_init_Rx, g_init_Ry, g_init_Rz,
			DJ1, DJ2, DJ3, DJ4, DJ5, DJ6, sTcpName, sUcsName, dVelocity, dAcc, dRadius,
			nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
		PrintResult(m_Controls.textBrowser, result, "Goto Point");
		AutoMoveJ_id++;
	}
	else {
		m_Controls.textBrowser->append("AutoMoveJ_id 超出范围");
		AutoMoveJ_id = 0;
	}

}