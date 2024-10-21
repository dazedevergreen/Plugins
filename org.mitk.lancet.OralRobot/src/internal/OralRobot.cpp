/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qmitk
#include "OralRobot.h"

// Qt
#include <QMessageBox>

// mitk image
#include <mitkImage.h>

// Qt
#include  <Qtimer>
#include <QInputDialog>
#include <QtWidgets/qfiledialog.h>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QLineEdit>
#include <QKeyEvent>
#include <QApplication>
#include <QShortcut>


// mitk image
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
//#include "leastsquaresfit.h"
//#include "mitkGizmo.h"
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
//#include "PrintDataHelper.h"
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

//

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
//#include "leastsquaresfit.h"
//#include "mitkGizmo.h"
//#include "mitkGizmo_noscale.h"
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
using namespace Eigen;
using namespace std;

const std::string OralRobot::VIEW_ID = "org.mitk.views.oralrobot";

void OralRobot::SetFocus()
{
  //m_Controls.buttonPerformImageProcessing->setFocus();
}

void OralRobot::OnSelectionChanged(berry::IWorkbenchPart::Pointer /*source*/,
                                                const QList<mitk::DataNode::Pointer> &nodes)
{
  //// iterate all selected objects, adjust warning visibility
  //foreach (mitk::DataNode::Pointer node, nodes)
  //{
  //  if (node.IsNotNull() && dynamic_cast<mitk::Image *>(node->GetData()))
  //  {
  //    m_Controls.labelWarning->setVisible(false);
  //    m_Controls.buttonPerformImageProcessing->setEnabled(true);
  //    return;
  //  }
  //}

  //m_Controls.labelWarning->setVisible(true);
  //m_Controls.buttonPerformImageProcessing->setEnabled(false);
}



void OralRobot::InitPointSetSelector(QmitkSingleNodeSelectionWidget* widget)
{
	widget->SetDataStorage(GetDataStorage());
	widget->SetNodePredicate(mitk::NodePredicateAnd::New(
		mitk::TNodePredicateDataType<mitk::PointSet>::New(),
		mitk::NodePredicateNot::New(mitk::NodePredicateOr::New(mitk::NodePredicateProperty::New("helper object"),
			mitk::NodePredicateProperty::New("hidden object")))));

	widget->SetSelectionIsOptional(true);
	widget->SetAutoSelectNewNodes(true);
	widget->SetEmptyInfo(QString("Please select a point set"));
	widget->SetPopUpTitel(QString("Select point set"));
}

//---------------------------------------------------------------------------------------------------------------
/**
 * @brief 初始化表面选择器
 * @param widget 单节点选择部件指针
 */
 //---------------------------------------------------------------------------------------------------------------
void OralRobot::InitSurfaceSelector(QmitkSingleNodeSelectionWidget* widget)
{
	widget->SetDataStorage(GetDataStorage());
	widget->SetNodePredicate(mitk::NodePredicateAnd::New(
		mitk::TNodePredicateDataType<mitk::Surface>::New(),
		mitk::NodePredicateNot::New(mitk::NodePredicateOr::New(mitk::NodePredicateProperty::New("helper object"),
			mitk::NodePredicateProperty::New("hidden object")))));

	widget->SetSelectionIsOptional(true);
	widget->SetAutoSelectNewNodes(true);
	widget->SetEmptyInfo(QString("Please select a surface"));
	widget->SetPopUpTitel(QString("Select surface"));
}


void OralRobot::CreateQtPartControl(QWidget* parent)
{
	// create GUI widgets from the Qt Designer's .ui file

	m_Controls.setupUi(parent);
	InitPointSetSelector(m_Controls.mitkNodeSelectWidget_LandmarkPoint); // Select a line
	InitPointSetSelector(m_Controls.mitkNodeSelectWidget_imageTargetLine); // 口腔机器人选择走位规划点
	InitPointSetSelector(m_Controls.mitkNodeSelectWidget_imageTargetPlane);
	InitPointSetSelector(m_Controls.mitkNodeSelectWidget_ImageTargetLine);

	InitSurfaceSelector(m_Controls.mitkNodeSelectWidget_STLSurface);

	connect(m_Controls.pushButton_GrapStop, &QPushButton::clicked, this, &OralRobot::suddenstop);
	connect(m_Controls.pushButton_ConnectArm, &QPushButton::clicked, this, &OralRobot::connectArm);
	connect(m_Controls.pushButton_ArmPowerOn, &QPushButton::clicked, this, &OralRobot::powerOn);
	connect(m_Controls.pushButton_ToolMotion, &QPushButton::clicked, this, &OralRobot::SetToolMotion);
	connect(m_Controls.pushButton_BaseMotion, &QPushButton::clicked, this, &OralRobot::SetBaseMotion);
	connect(m_Controls.pushButton_RecordInitPosition, &QPushButton::clicked, this, &OralRobot::ReadActPose);
	connect(m_Controls.pushButton_GotoInitPosition, &QPushButton::clicked, this, &OralRobot::Gotoinitpos);
	connect(m_Controls.pushButton_ConnectAim, &QPushButton::clicked, this, &OralRobot::ConnectAim);
	connect(m_Controls.pushButton_AimAcquisition, &QPushButton::clicked, this, &OralRobot::AimAcquisition);
	connect(m_Controls.pushButton_PrintMatrix, &QPushButton::clicked, this, &OralRobot::Printmatrix);
	connect(m_Controls.pushButton_ArmPowerOff, &QPushButton::clicked, this, &OralRobot::powerOff);
	connect(m_Controls.pushButton_OpenFreeDrag, &QPushButton::clicked, this, &OralRobot::OpenFreeDrag);
	connect(m_Controls.pushButton_CloseFreeDrag, &QPushButton::clicked, this, &OralRobot::TurnOffFreeDrag);
	connect(m_Controls.pushButton_SetTCPtoFlange, &QPushButton::clicked, this, &OralRobot::setTCPToFlange);
	connect(m_Controls.pushButton_CaptureRobot, &QPushButton::clicked, this, &OralRobot::CaptureRobot);
	connect(m_Controls.pushButton_SaveRobotMatrix, &QPushButton::clicked, this, &OralRobot::SaveRobotMatrix);
	connect(m_Controls.pushButton_ReuseRobotMatrix, &QPushButton::clicked, this, &OralRobot::ReuseRobotMatrix);
	connect(m_Controls.pushButton_ReCaptureRobot, &QPushButton::clicked, this, &OralRobot::DeleteRobotMatrixCache);
	connect(m_Controls.pushButton_AutoCaptureRobot, &QPushButton::clicked, this, &OralRobot::AutoCaptureRobot);
	connect(m_Controls.pushButton_DemarcateTCP, &QPushButton::clicked, this, &OralRobot::DemarcateTCP);
	connect(m_Controls.pushButton_SaveTCPMatrix, &QPushButton::clicked, this, &OralRobot::SaveTCP);
	connect(m_Controls.pushButton_ReuseTCPMatrix, &QPushButton::clicked, this, &OralRobot::ReuseTCP);

	connect(m_Controls.pushButton_NavigationForceControl, &QPushButton::clicked, this, &OralRobot::StartForceControl);




	connect(m_Controls.pushButton_xp, &QPushButton::clicked, this, &OralRobot::xp);
	connect(m_Controls.pushButton_yp, &QPushButton::clicked, this, &OralRobot::yp);
	connect(m_Controls.pushButton_zp, &QPushButton::clicked, this, &OralRobot::zp);
	connect(m_Controls.pushButton_rxp, &QPushButton::clicked, this, &OralRobot::rxp);
	connect(m_Controls.pushButton_ryp, &QPushButton::clicked, this, &OralRobot::ryp);
	connect(m_Controls.pushButton_rzp, &QPushButton::clicked, this, &OralRobot::rzp);
	connect(m_Controls.pushButton_xm, &QPushButton::clicked, this, &OralRobot::xm);
	connect(m_Controls.pushButton_ym, &QPushButton::clicked, this, &OralRobot::ym);
	connect(m_Controls.pushButton_zm, &QPushButton::clicked, this, &OralRobot::zm);
	connect(m_Controls.pushButton_rxm, &QPushButton::clicked, this, &OralRobot::rxm);
	connect(m_Controls.pushButton_rym, &QPushButton::clicked, this, &OralRobot::rym);
	connect(m_Controls.pushButton_rzm, &QPushButton::clicked, this, &OralRobot::rzm);

	connect(m_Controls.pushButton_Test, &QPushButton::clicked, this, &OralRobot::JustForTest);


}

OralRobot::~OralRobot()
{

}


void OralRobot::suddenstop()
{
	bool nRet = Robot->Stop(m_Controls.textBrowser_Log);
}

void OralRobot::connectArm()
{
	bool nRet = Robot->Connect(m_Controls.textBrowser_Log);
}

void OralRobot::powerOn()
{
	bool nRet = Robot->PowerOn(m_Controls.textBrowser_Log);
}

//---------------------------------------------------------------------------------------------------------------
/**
 * @brief 关闭机械臂电源
 */
 //---------------------------------------------------------------------------------------------------------------
void OralRobot::powerOff()
{
	bool nRet = Robot->PowerOff(m_Controls.textBrowser_Log);
}

void OralRobot::OpenFreeDrag()
{
	bool nRet = Robot->StartFreeGrag(m_Controls.textBrowser_Log);
}

void OralRobot::TurnOffFreeDrag()
{
	bool nRet = Robot->StopFreeGrag(m_Controls.textBrowser_Log);
}

void OralRobot::ConnectAim() {
	T_AIMPOS_DATAPARA mPosDataPara;
	Aim_API_Initial(aimhandle);
	Aim_SetEthernetConnectIP(aimhandle, 192, 168, 31, 10);
	RLT = Aim_ConnectDevice(aimhandle, I_ETHERNET, mPosDataPara);
	m_Controls.textBrowser_Log->append("-------------------------------------------------------------");
	switch (RLT) {
	case AIMOOE_OK:
		m_Controls.textBrowser_Log->append("Aimooe Connect Success");
		std::cout << "connect success";
		break;
	case AIMOOE_CONNECT_ERROR:
		m_Controls.textBrowser_Log->append("Aimooe Connect Faild");
		std::cout << "connect faild";
		break;
	case AIMOOE_READ_FAULT:
		m_Controls.textBrowser_Log->append("Aimooe Read Fault");
		std::cout << "connect faild";
		break;
	case AIMOOE_WRITE_FAULT:
		m_Controls.textBrowser_Log->append("Aimooe Write Fault");
		std::cout << "connect faild";
		break;
	case AIMOOE_ERROR:
		m_Controls.textBrowser_Log->append("Unknown Fault");
		std::cout << "connect faild";
		break;
	}

	QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", "");
	if (filename.isNull()) return;
	filename.append("/");
	std::cout << "The selected folder address :" << filename;
	RLT = Aim_SetToolInfoFilePath(aimhandle, filename.toLatin1().data());

	if (RLT == AIMOOE_OK)
	{
		m_Controls.textBrowser_Log->append("set filenemae success");
		std::cout << "set filenemae success";
	}
	else {
		std::cout << "set filenemae failed";
		m_Controls.textBrowser_Log->append("set filenemae failed");
	}

	int size = 0;
	Aim_GetCountOfToolInfo(aimhandle, size);

	if (size != 0)
	{
		t_ToolBaseInfo* toolarr = new t_ToolBaseInfo[size];

		RLT = Aim_GetAllToolFilesBaseInfo(aimhandle, toolarr);

		if (RLT == AIMOOE_OK)
		{
			for (int i = 0; i < size; i++)
			{
				char* ptool = toolarr[i].name;
				QString toolInfo = QString(ptool);
				m_Controls.textBrowser_Log->append(toolInfo);
			}
		}
		delete[] toolarr;
	}
	else {
		std::cout << "There are no tool identification files in the current directory:";
		m_Controls.textBrowser_Log->append("There are no tool identification files in the current directory:");

	}

	std::cout << "End of connection";
	m_Controls.textBrowser_Log->append("End of connection");

	RLT = AIMOOE_OK;
	m_Controls.textBrowser_Log->append("-------------------------------------------------------------");
}

void OralRobot::AimAcquisition() {
	if (m_AimoeVisualizeTimer == nullptr)
	{
		m_AimoeVisualizeTimer = new QTimer(this);
	}
	connect(m_AimoeVisualizeTimer, SIGNAL(timeout()), this, SLOT(updateCameraData())); //口腔拿数据第二种方式
	m_AimoeVisualizeTimer->start(100);
	m_Controls.textBrowser_Log->append("Get Tool Info Timer Success");
	std::cout << "Get Tool Info Timer Success" << std::endl;
}

void OralRobot::updateCameraData()
{
	RLT = Aim_GetMarkerAndStatusFromHardware(aimhandle, I_ETHERNET, markerSt, statusSt);
	if (RLT == AIMOOE_NOT_REFLASH)
	{
		std::cout << "camera get data failed";
	}
	T_AimToolDataResult* mtoolsrlt = new T_AimToolDataResult;//新建一个值指，将指针清空用于存数据
	mtoolsrlt->next = NULL;
	mtoolsrlt->validflag = false;

	RLT = Aim_FindToolInfo(aimhandle, markerSt, mtoolsrlt, 0);//获取数据
	if (RLT == AIMOOE_OK) {
		do
		{
			//获取数据
			UpdateCameraToToolMatrix(mtoolsrlt, ToolName[0].data(), T_CamToBaseRF, m_Controls.BaseRFLable);
			UpdateCameraToToolMatrix(mtoolsrlt, ToolName[1].data(), T_CamToEndRF, m_Controls.EndRFLable);
			UpdateCameraToToolMatrix(mtoolsrlt, ToolName[2].data(), T_CamToPatientRF, m_Controls.PatinetRFLable);
			UpdateCameraToToolMatrix(mtoolsrlt, ToolName[3].data(), T_CamToProbe, m_Controls.ProbleLable);
			UpdateCameraToToolMatrix(mtoolsrlt, ToolName[4].data(), T_CamToCalibratorRF, m_Controls.TCPRFLable);
			//UpdateCameraToToolMatrix(mtoolsrlt, ToolName[5].data(), T_CamToEndRF, m_Controls.Spine_RobotEndRFDataLabel);

			//获取Spine_Probe数据
			if (strcmp(mtoolsrlt->toolname, "Oral_Probe") == 0)
			{
				if (mtoolsrlt->validflag)
				{
					ProbeTop[0] = mtoolsrlt->tooltip[0];
					ProbeTop[1] = mtoolsrlt->tooltip[1];
					ProbeTop[2] = mtoolsrlt->tooltip[2];
				}
			}

			T_AimToolDataResult* pnext = mtoolsrlt->next;
			delete mtoolsrlt;
			mtoolsrlt = pnext;
		} while (mtoolsrlt != NULL);
	}
	else
	{
		delete mtoolsrlt;
		m_Controls.textBrowser_Log->append("Find Tool Info Faild");
		std::cout << "faild";
	}

}

void OralRobot::UpdateCameraToToolMatrix(T_AimToolDataResult* ToolData, const char* Name, double* Camera2Tool, QLabel* label) {

	float R_tran[3][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };
	float t_tran[3] = { 0.0f, 0.0f, 0.0f };

	if (strcmp(ToolData->toolname, Name) == 0)
	{
		if (ToolData->validflag)
		{
			//获取相机数据
			t_tran[0] = ToolData->Tto[0];
			t_tran[1] = ToolData->Tto[1];
			t_tran[2] = ToolData->Tto[2];
			for (int i = 0; i < 3; ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					R_tran[i][j] = ToolData->Rto[i][j];
				}
			}
			//拼接矩阵
			vtkNew<vtkMatrix4x4> matrix;

			Robot->UtilityFunction->CombineRotationTranslation(R_tran, t_tran, matrix);

			Robot->UtilityFunction->Matrix4x4ToDouble4x4(Camera2Tool, matrix);

			QString str = "x:" + QString::number(ToolData->tooltip[0]) + " "
				+ "y:" + QString::number(ToolData->tooltip[1]) + " "
				+ "z:" + QString::number(ToolData->tooltip[2]);
			label->setText(str);
			label->setStyleSheet("QLabel { color : green; }");

		}
		else
		{
			if (label != nullptr)
			{
				QString str = "x:nan  y:nan   z:nan";
				label->setText(str);
				/*std::cout << str << std::endl;*/
				label->setStyleSheet("QLabel { color : red; }");
			}
		}
	}
}

void OralRobot::Printmatrix() {
	Robot->UtilityFunction->PrintMatrix(m_Controls.textBrowser_Log, "T_CamToEndRF", T_CamToEndRF);//打印T_CamToEndRF
	Robot->UtilityFunction->PrintMatrix(m_Controls.textBrowser_Log, "T_CamToBaseRF", T_CamToBaseRF);//打印T_CamToBaseRF
	Robot->UtilityFunction->PrintMatrix(m_Controls.textBrowser_Log, "T_CamToProbe", T_CamToProbe);//打印T_CamToProbe
	Robot->UtilityFunction->PrintMatrix(m_Controls.textBrowser_Log, "T_CamToPatientRF", T_CamToPatientRF);//打印T_CamToPatientRF
	Robot->UtilityFunction->PrintMatrix(m_Controls.textBrowser_Log, "T_BaseToBaseRF", T_BaseToBaseRF);//打印T_CamToPatientRF
	Robot->UtilityFunction->PrintMatrix(m_Controls.textBrowser_Log, "T_FlangeToEndRF", T_FlangeToEndRF);//打印T_FlangeToEndRF
	Robot->UtilityFunction->PrintMatrix(m_Controls.textBrowser_Log, "T_CameraToCalibratorRF", T_CamToCalibratorRF);//打印T_FlangeToEndRF
	std::array<double, 6> BasetoFlange = Robot->ReadBasetoFlange(m_Controls.textBrowser_Log);
	auto vtk_BasetoFlange = vtkTransform::New();
	Robot->TranslateAndRotateMatrix(vtk_BasetoFlange, BasetoFlange);
	double T_BasetoFlange[16];
	Robot->UtilityFunction->Matrix4x4ToDouble4x4(T_BasetoFlange, vtk_BasetoFlange->GetMatrix());
	Robot->UtilityFunction->PrintMatrix(m_Controls.textBrowser_Log, "T_BasetoFlange", T_BasetoFlange);
}


void OralRobot::SetToolMotion()
{
	bool nRet = Robot->SetMoveToolMotion(m_Controls.textBrowser_Log, 1);
}

void OralRobot::SetBaseMotion() {
	bool nRet = Robot->SetMoveToolMotion(m_Controls.textBrowser_Log, 0);
}

void OralRobot::setTCPToFlange() {
	std::array<double, 6> TCP{ 0, 0, 0, 0, 0, 0 };
	bool nRet = Robot->ConfigFlangetoTCP(m_Controls.textBrowser_Log, TCP, sTcpName);
}

void OralRobot::ReadActPose() {
	Joint_init = Robot->ReadJoint(m_Controls.textBrowser_Log);
}

void OralRobot::Gotoinitpos() {
	bool nRet = Robot->MoveJ(m_Controls.textBrowser_Log, Joint_init, 1);
}

void OralRobot::CaptureRobotPose(bool translationOnly) {
	//basetotool
	std::array<double, 6> BasetoFlange = Robot->ReadBasetoFlange(m_Controls.textBrowser_Log);

	auto Trans_BasetoFlange = vtkTransform::New();
	Robot->TranslateAndRotateMatrix(Trans_BasetoFlange, BasetoFlange);

	auto vtk_BasetoFlange = vtkMatrix4x4::New();
	vtk_BasetoFlange->DeepCopy(Trans_BasetoFlange->GetMatrix());

	auto vtkT_BaseRFtoCamera = vtkMatrix4x4::New();
	vtkT_BaseRFtoCamera->DeepCopy(T_CamToBaseRF);
	vtkT_BaseRFtoCamera->Invert();

	auto vtkT_CamToEndRF = vtkMatrix4x4::New();
	vtkT_CamToEndRF->DeepCopy(T_CamToEndRF);

	auto Transform_1 = vtkTransform::New();
	Robot->UtilityFunction->MultiplyMatrices(Transform_1, vtkT_BaseRFtoCamera, vtkT_CamToEndRF);
	auto vtkT_BaseRFToEndRF = vtkMatrix4x4::New();

	vtkT_BaseRFToEndRF->DeepCopy(Transform_1->GetMatrix());

	Robot->UtilityFunction->PrintMatrix(m_Controls.textBrowser_Log, "vtkT_BaseRFToEndRF", vtkT_BaseRFToEndRF);

	m_RobotRegistration.AddPoseWithVtkMatrix(vtk_BasetoFlange, vtkT_BaseRFToEndRF, translationOnly);
	m_IndexOfRobotCapture++;
	std::cout << "m_IndexOfRobotCapture: " << m_IndexOfRobotCapture << std::endl;
	m_Controls.lineEdit_NumOfRobotPose->setText(QString::number(m_IndexOfRobotCapture));

}

void OralRobot::CaptureRobot() {
	m_Controls.textBrowser_Log->append("captureRobot");
	if (m_IndexOfRobotCapture < 5) //The first five translations, 
	{
		CaptureRobotPose(true);
	}
	else if (m_IndexOfRobotCapture < 10) //the last five rotations
	{
		CaptureRobotPose(false);
	}
	else
	{
		CaptureRobotFinash();
	}
}

void OralRobot::CaptureRobotFinash() {
	vtkMatrix4x4* vtkT_BaseToBaseRF = vtkMatrix4x4::New();
	m_RobotRegistration.GetRegistraionMatrix(vtkT_BaseToBaseRF);
	vtkT_BaseToBaseRF->Invert();
	Robot->UtilityFunction->Matrix4x4ToDouble4x4(T_BaseToBaseRF, vtkT_BaseToBaseRF);

	vtkMatrix4x4* vtkT_FlangeToEndRF = vtkMatrix4x4::New();
	m_RobotRegistration.GetTCPmatrix(vtkT_FlangeToEndRF);
	Robot->UtilityFunction->Matrix4x4ToDouble4x4(T_FlangeToEndRF, vtkT_FlangeToEndRF);

	m_Controls.textBrowser_Log->append("Registration RMS: " + QString::number(m_RobotRegistration.RMS()));
	std::cout << "Registration RMS: " << m_RobotRegistration.RMS() << std::endl;
	m_Controls.lineEdit_RobotRMS->setText(QString::number(m_RobotRegistration.RMS()));
}

void OralRobot::SaveRobotMatrix() {
	Robot->UtilityFunction->SaveFiles(m_Controls.textBrowser_Log, "\\Desktop\\save\\T_BaseToBaseRF.txt", T_BaseToBaseRF);
	Robot->UtilityFunction->SaveFiles(m_Controls.textBrowser_Log, "\\Desktop\\save\\T_FlangeToEndRF.txt", T_FlangeToEndRF);

	m_Controls.textBrowser_Log->append("saveArmMatrix");
}

void OralRobot::ReuseRobotMatrix() {
	Robot->UtilityFunction->ReuseFiles(m_Controls.textBrowser_Log, "\\Desktop\\save\\T_BaseToBaseRF.txt", T_BaseToBaseRF);
	Robot->UtilityFunction->ReuseFiles(m_Controls.textBrowser_Log, "\\Desktop\\save\\T_FlangeToEndRF.txt", T_FlangeToEndRF);

	m_Controls.textBrowser_Log->append("ReuseArmMatrix");
}

void OralRobot::DeleteRobotMatrixCache() {
	m_Controls.textBrowser_Log->append("Replace Registration");
	m_RobotRegistration.RemoveAllPose();
	m_IndexOfRobotCapture = 0;
	m_Controls.lineEdit_NumOfRobotPose->setText(QString::number(m_IndexOfRobotCapture));
}

void OralRobot::AutoCaptureRobot() {

	for (; m_IndexOfRobotCapture < 10; ) {
		automove();
		Robot->UtilityFunction->Sleep(400);
		while (Robot->isMoving(m_Controls.textBrowser_Log)) {
			Robot->UtilityFunction->Sleep(2000);
		}
		Robot->UtilityFunction->Sleep(4000);
		CaptureRobot();
		Robot->UtilityFunction->Sleep(300);
	}
	CaptureRobotFinash();
}

void OralRobot::automove() {
	switch (m_IndexOfRobotCapture) {
	case 0:
		Robot->RelMoveXplus(m_Controls.textBrowser_Log, 50);
		break;
	case 1:
		Robot->RelMoveYplus(m_Controls.textBrowser_Log, 50);
		break;
	case 2:
		Robot->RelMoveXmin(m_Controls.textBrowser_Log, 50);
		break;
	case 3:
		Robot->RelMoveZplus(m_Controls.textBrowser_Log, 50);
		break;
	case 4:
		Robot->RelMoveYmin(m_Controls.textBrowser_Log, 50);
		break;
	case 5:
		Robot->RelMoveRxmin(m_Controls.textBrowser_Log, 15);
		break;
	case 6:
		Robot->RelMoveRxplus(m_Controls.textBrowser_Log, 30);
		break;
	case 7:
		Robot->RelMoveRymin(m_Controls.textBrowser_Log, 15);
		break;
	case 8:
		Robot->RelMoveRyplus(m_Controls.textBrowser_Log, 30);
		break;
	case 9:
		Robot->RelMoveRzplus(m_Controls.textBrowser_Log, 20);
		break;
	default:
		break;
	}
}

void OralRobot::DemarcateTCP() {

	if (GetDataStorage()->GetNamedNode("probe_head_tail_mandible") == nullptr ||
		GetDataStorage()->GetNamedNode("probe_head_tail_maxilla") == nullptr)
	{
		m_Controls.textBrowser_Log->append("probe_head_tail_mandible or probe_head_tail_maxilla is missing!");
		return;
	}

	// probe_head_tail_mandible下颌标定点集
	// probe_head_tail_mandible下颌标定点集

	auto probe_head_tail = mitk::PointSet::New();

	if (m_Controls.radioButton_maxilla->isChecked())
	{
		auto probe_head_tail_maxilla = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("probe_head_tail_maxilla")->GetData());
		if (probe_head_tail_maxilla->GetSize() != 2) {
			m_Controls.textBrowser_Log->append("probe_head_tail_maxilla is problematic!");
			return;
		}
		probe_head_tail = probe_head_tail_maxilla;
	}
	else
	{
		auto probe_head_tail_mandible = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("probe_head_tail_mandible")->GetData());
		if (probe_head_tail_mandible->GetSize() != 2) {
			m_Controls.textBrowser_Log->append("probe_head_tail_mandible is problematic!");
			return;
		}
		probe_head_tail = probe_head_tail_mandible;
	}

	// 计算法兰到EndRF的转换矩阵T_FlangeRFToEndRF
	auto T_EndRFtoCamera = vtkMatrix4x4::New();
	auto T_CameratoCalibratorRF = vtkMatrix4x4::New();
	auto T_FlangetoEndRF = vtkMatrix4x4::New();

	T_EndRFtoCamera->DeepCopy(T_CamToEndRF);
	T_EndRFtoCamera->Invert();
	T_CameratoCalibratorRF->DeepCopy(T_CamToCalibratorRF);
	T_FlangetoEndRF->DeepCopy(T_FlangeToEndRF);

	auto probe_head = probe_head_tail->GetPoint(0);
	auto probe_tail = probe_head_tail->GetPoint(1);

	Eigen::Vector3d z_target;
	z_target[0] = probe_tail[0] - probe_head[0];
	z_target[1] = probe_tail[1] - probe_head[1];
	z_target[2] = probe_tail[2] - probe_head[2];
	z_target.normalize();

	Eigen::Vector3d x_std{ 1, 0, 0 };

	Eigen::Vector3d y_target;
	y_target = z_target.cross(x_std);
	y_target.normalize();

	Eigen::Vector3d x_target;
	x_target = y_target.cross(z_target);
	x_target.normalize();

	float R[3][3]{ {(float)x_target(0), (float)y_target(0), (float)z_target(0)},
		{(float)x_target(1), (float)y_target(1), (float)z_target(1)},
		{(float)x_target(2), (float)y_target(2), (float)z_target(2)}
	};

	float Trans[3] = { (float)probe_head[0], (float)probe_head[1], (float)probe_head[2] };

	auto T_CalibratorRFtoTCP = vtkMatrix4x4::New();

	Robot->UtilityFunction->CombineRotationTranslation(R, Trans, T_CalibratorRFtoTCP);


	auto Trans_FlangeToTCP = vtkTransform::New();
	Robot->UtilityFunction->MultiplyMatrices(Trans_FlangeToTCP, T_FlangetoEndRF, T_EndRFtoCamera, T_CameratoCalibratorRF, T_CalibratorRFtoTCP);

	// Todo: the matrix below should be averaged for a time span before being stored into m_T_handpieceRFtoDrill
	Eigen::Vector3d x_FlangeToTCP;
	x_FlangeToTCP << Trans_FlangeToTCP->GetMatrix()->GetElement(0, 0), Trans_FlangeToTCP->GetMatrix()->GetElement(1, 0), Trans_FlangeToTCP->GetMatrix()->GetElement(2, 0);

	double MinRotAngle = 180 * acos(x_FlangeToTCP.dot(x_std)) / 3.141592654;
	double RotAngle;

	auto vtkT_FlanegToTCP = vtkMatrix4x4::New();
	vtkT_FlanegToTCP->DeepCopy(Trans_FlangeToTCP->GetMatrix());
	for (int i = 0; i < 720; i++) {
		Trans_FlangeToTCP->RotateZ(0.5);
		Trans_FlangeToTCP->Update();
		x_FlangeToTCP << Trans_FlangeToTCP->GetMatrix()->GetElement(0, 0), Trans_FlangeToTCP->GetMatrix()->GetElement(1, 0), Trans_FlangeToTCP->GetMatrix()->GetElement(2, 0);
		RotAngle = 180 * acos(x_FlangeToTCP.dot(x_std)) / 3.141592654;
		if ((MinRotAngle - RotAngle) >= 0) {
			vtkT_FlanegToTCP->DeepCopy(Trans_FlangeToTCP->GetMatrix());
		}
	}

	Robot->UtilityFunction->Matrix4x4ToDouble4x4(T_FlangeToTCP, vtkT_FlanegToTCP);

	ApplyTCP();

}

void OralRobot::SaveTCP() {
	Robot->UtilityFunction->SaveFiles(m_Controls.textBrowser_Log, "\\Desktop\\Test\\save\\T_FlangeToTCP.txt", T_FlangeToTCP);

	m_Controls.textBrowser_Log->append("saveTCPMatrix");
}

void OralRobot::ReuseTCP() {
	Robot->UtilityFunction->ReuseFiles(m_Controls.textBrowser_Log, "\\Desktop\\Test\\save\\T_FlangeToTCP.txt", T_FlangeToTCP);
	ApplyTCP();
	m_Controls.textBrowser_Log->append("ReuseTCPMatrix");

}

void OralRobot::ApplyTCP() {
	auto vtkT_FlanegToTCP = vtkMatrix4x4::New();
	Robot->UtilityFunction->Double4x4ToMatrix4x4(T_FlangeToTCP, vtkT_FlanegToTCP);

	Eigen::Vector3d Angle = Robot->GetAulerAngle(vtkT_FlanegToTCP);

	std::array<double, 6> TCP{ vtkT_FlanegToTCP->GetElement(0, 3), vtkT_FlanegToTCP->GetElement(1, 3), vtkT_FlanegToTCP->GetElement(2, 3),
			vtkMath::DegreesFromRadians(Angle(2)), vtkMath::DegreesFromRadians(Angle(1)), vtkMath::DegreesFromRadians(Angle(0)) };

	Robot->ConfigFlangetoTCP(m_Controls.textBrowser_Log, TCP, sTcpName);

}


void OralRobot::StartForceControl() {
	std::array<double, 6> Freedom{ 0,0,0,0,0,1 };
	Robot->SetForceFreeDriveFreedom(m_Controls.textBrowser_Log, Freedom);
	// 定义力控自由驱动自由度状态
	Robot->SetForceFreeDriveVelocity(m_Controls.textBrowser_Log, 10, 10);
	// 开启力控自由驱动
	Robot->SetForceFreeDrive(m_Controls.textBrowser_Log, 1);
	
	if (DeepProtectionTimer == nullptr)
	{
		DeepProtectionTimer = new QTimer(this);
	}
	connect(DeepProtectionTimer, SIGNAL(timeout()), this, SLOT(DeepProtection()));
	DeepProtectionTimer->start(30);
}

void OralRobot::DeepProtection() {
	std::array<double, 6> TCP = Robot->ReadBasetoTCP(m_Controls.textBrowser_Log);


	auto Trans_BasetoTCP = vtkTransform::New();
	Robot->TranslateAndRotateMatrix(Trans_BasetoTCP, TCP);

	auto vtk_TCPtoBase = vtkMatrix4x4::New();
	vtk_TCPtoBase->DeepCopy(Trans_BasetoTCP->GetMatrix());
	vtk_TCPtoBase->Invert();

	auto Trans_BasetoTarget = vtkTransform::New();
	Robot->TranslateAndRotateMatrix(Trans_BasetoTarget, T_TargetPoint);

	//Base坐标系下的第二个规划点位姿，需要设置成全局变量并且在位置导航的地方进行赋值操作

	auto VTK_BasetoTarget = vtkMatrix4x4::New();
	VTK_BasetoTarget->DeepCopy(Trans_BasetoTarget->GetMatrix());

	auto Trans_TCPtoTarget = vtkTransform::New();
	Robot->UtilityFunction->MultiplyMatrices(Trans_TCPtoTarget, vtk_TCPtoBase, VTK_BasetoTarget);

	auto vtk_TCPtoTarget = vtkMatrix4x4::New();
	vtk_TCPtoTarget->DeepCopy(Trans_TCPtoTarget->GetMatrix());
	//计算欧几里得距离
	double distance = sqrt(pow(vtk_TCPtoTarget->GetElement(0, 3), 2) + pow(vtk_TCPtoTarget->GetElement(1, 3), 2)
		+ pow(vtk_TCPtoTarget->GetElement(2, 3), 2));
	//距离小于0.2mm或者TCPZ轴方向上距离为负就停止力控并延当前TCPZ轴回退2mm，之后人工再次开启力控或自由拖动就行了
	//理论上来说没什么问题，就是怕实际运行的时候出差错
	if (distance < 0.1 || vtk_TCPtoTarget->GetElement(2, 3) < 0) {
		Robot->Stop(m_Controls.textBrowser_Log);
		DeepProtectionTimer->stop();
		
		//软件显示已到达目标深度，其实如果能从机械臂身上把供电捞出去，设置电钻不钻就可以了，也不需要这些stop什么的。
		Robot->UtilityFunction->Sleep(800);
		//页面上点击确定后再开启
		Robot->SetForceFreeDrive(m_Controls.textBrowser_Log, 1);
		Robot->UtilityFunction->Sleep(1000);
		DeepProtectionTimer->start(30);
	}
}

/////////////////////////////////////////////////////////////////////机械臂测试用////////////////////////////////////////////////////////////////////////////
/*============================================================================

机械臂测试用

============================================================================*/

void OralRobot::JustForTest() {
	//Rotate在右边后面乘
	//以前用模长来进行ASIN纯属多余，直接acos（a.dot(b)）就完事了
	//不知道为什么，轴角法要用post后乘，而且要先setMatrix然后在变换，很奇怪啊很奇怪
	//post后乘和pre前乘是可以随意切换的，值是可以从新设的
	//vtktransform不能乱用，只能用一次，因为->getmatrix（）是指针操作，可以用DeepCopy
	//运算过程不会改变过程中的矩阵
	//运算参数里Matrix和Transform的效果是一样的


}


void OralRobot::xp()
{
	Robot->RelMoveXplus(m_Controls.textBrowser_Log, m_Controls.lineEdit_Translate->text().toDouble());
}

void OralRobot::yp()
{
	Robot->RelMoveYplus(m_Controls.textBrowser_Log, m_Controls.lineEdit_Translate->text().toDouble());
}

void OralRobot::zp()
{
	Robot->RelMoveZplus(m_Controls.textBrowser_Log, m_Controls.lineEdit_Translate->text().toDouble());

}

void OralRobot::xm()
{
	Robot->RelMoveXmin(m_Controls.textBrowser_Log, m_Controls.lineEdit_Translate->text().toDouble());

}

void OralRobot::ym()
{
	Robot->RelMoveYmin(m_Controls.textBrowser_Log, m_Controls.lineEdit_Translate->text().toDouble());

}

void OralRobot::zm()
{
	Robot->RelMoveZmin(m_Controls.textBrowser_Log, m_Controls.lineEdit_Translate->text().toDouble());

}

void OralRobot::rxp()
{
	Robot->RelMoveRxplus(m_Controls.textBrowser_Log, m_Controls.lineEdit_Translate->text().toDouble());

}

void OralRobot::ryp()
{
	Robot->RelMoveRyplus(m_Controls.textBrowser_Log, m_Controls.lineEdit_Translate->text().toDouble());

}

void OralRobot::rzp()
{
	Robot->RelMoveRzplus(m_Controls.textBrowser_Log, m_Controls.lineEdit_Translate->text().toDouble());

}

void OralRobot::rxm()
{
	Robot->RelMoveRxmin(m_Controls.textBrowser_Log, m_Controls.lineEdit_Translate->text().toDouble());

}

void OralRobot::rym()
{
	Robot->RelMoveRymin(m_Controls.textBrowser_Log, m_Controls.lineEdit_Translate->text().toDouble());

}

void OralRobot::rzm()
{
	Robot->RelMoveRzmin(m_Controls.textBrowser_Log, m_Controls.lineEdit_Translate->text().toDouble());

}
