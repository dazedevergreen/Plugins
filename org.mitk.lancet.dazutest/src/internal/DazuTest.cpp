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
#include "DazuTest.h"

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

using namespace Eigen;
using namespace std;

const std::string DazuTest::VIEW_ID = "org.mitk.views.DazuTest";

void DazuTest::SetFocus()
{
  //m_Controls.buttonPerformImageProcessing->setFocus();
}

void DazuTest::OnSelectionChanged(berry::IWorkbenchPart::Pointer /*source*/, const QList<mitk::DataNode::Pointer> &nodes)
{

}

void DazuTest::InitPointSetSelector(QmitkSingleNodeSelectionWidget* widget)
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
void DazuTest::InitSurfaceSelector(QmitkSingleNodeSelectionWidget* widget)
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


void DazuTest::CreateQtPartControl(QWidget *parent)
{
  // create GUI widgets from the Qt Designer's .ui file

  m_Controls.setupUi(parent);
  InitPointSetSelector(m_Controls.mitkNodeSelectWidget_LandmarkPoint); // Select a line
  InitPointSetSelector(m_Controls.mitkNodeSelectWidget_imageTargetLine); // 口腔机器人选择走位规划点
  InitPointSetSelector(m_Controls.mitkNodeSelectWidget_imageTargetPlane);
  InitPointSetSelector(m_Controls.mitkNodeSelectWidget_ImageTargetLine);

  InitSurfaceSelector(m_Controls.mitkNodeSelectWidget_STLSurface);

  connect(m_Controls.pushButton_GrapStop, &QPushButton::clicked, this, &DazuTest::suddenstop);
  connect(m_Controls.pushButton_ConnectArm, &QPushButton::clicked, this, &DazuTest::connectArm);
  connect(m_Controls.pushButton_ArmPowerOn, &QPushButton::clicked, this, &DazuTest::powerOn);
  connect(m_Controls.pushButton_ConnectAim, &QPushButton::clicked, this, &DazuTest::ConnectAim);
  connect(m_Controls.pushButton_AimAcquisition, &QPushButton::clicked, this, &DazuTest::AimAcquisition);
  connect(m_Controls.pushButton_PrintMatrix, &QPushButton::clicked, this, &DazuTest::Printmatrix);
  connect(m_Controls.pushButton_Reset, &QPushButton::clicked, this, &DazuTest::Reset);
  connect(m_Controls.pushButton_ArmPowerOff, &QPushButton::clicked, this, &DazuTest::powerOff);
  connect(m_Controls.pushButton_OpenFreeDrag, &QPushButton::clicked, this, &DazuTest::OpenFreeDrag);
  connect(m_Controls.pushButton_CloseFreeDrag, &QPushButton::clicked, this, &DazuTest::TurnOffFreeDrag);
  connect(m_Controls.pushButton_SetTCPtoFlange, &QPushButton::clicked, this, &DazuTest::setTCPToFlange);
  connect(m_Controls.pushButton_CaptureRobot, &QPushButton::clicked, this, &DazuTest::CaptureRobot);
  connect(m_Controls.pushButton_SaveRobotMatrix, &QPushButton::clicked, this, &DazuTest::SaveRobotMatrix);
  connect(m_Controls.pushButton_ReuseRobotMatrix, &QPushButton::clicked, this, &DazuTest::ReuseRobotMatrix);
  connect(m_Controls.pushButton_ReCaptureRobot, &QPushButton::clicked, this, &DazuTest::DeleteRobotMatrixCache);
  connect(m_Controls.pushButton_AutoCaptureRobot, &QPushButton::clicked, this, &DazuTest::AutoCaptureRobot);
  connect(m_Controls.pushButton_DemarcateTCP, &QPushButton::clicked, this, &DazuTest::DemarcateTCP);
  connect(m_Controls.pushButton_SaveTCPMatrix, &QPushButton::clicked, this, &DazuTest::SaveTCP);
  connect(m_Controls.pushButton_ReuseTCPMatrix, &QPushButton::clicked, this, &DazuTest::ReuseTCP);
  connect(m_Controls.pushButton_TrackingPosition, &QPushButton::clicked, this, &DazuTest::TrackingPosition);
  connect(m_Controls.pushButton_NavigationForceControl, &QPushButton::clicked, this, &DazuTest::StartForceControl);





  connect(m_Controls.pushButton_ToolMotion, &QPushButton::clicked, this, &DazuTest::SetToolMotion);
  connect(m_Controls.pushButton_BaseMotion, &QPushButton::clicked, this, &DazuTest::SetBaseMotion);
  connect(m_Controls.pushButton_SetTCP, &QPushButton::clicked, this, &DazuTest::SetTCPbyName);
  connect(m_Controls.pushButton_ConfigTCPbyName, &QPushButton::clicked, this, &DazuTest::ConfigTCP);
  
  connect(m_Controls.pushButton_ReadBasetoFlange, &QPushButton::clicked, this, &DazuTest::ReadBasetoFlange);
  connect(m_Controls.pushButton_RecordInitPosition, &QPushButton::clicked, this, &DazuTest::ReadActPose);
  connect(m_Controls.pushButton_GotoInitPosition, &QPushButton::clicked, this, &DazuTest::gotoinitpos);
  connect(m_Controls.pushButton_ReadActTCPByName, &QPushButton::clicked, this, &DazuTest::ReadActTCPByName);
  connect(m_Controls.pushButton_ReadcmdTCPPose, &QPushButton::clicked, this, &DazuTest::ReadcmdTCPPose);
  connect(m_Controls.pushButton_ReadCurTCP, &QPushButton::clicked, this, &DazuTest::ReadCurTCP);


  connect(m_Controls.pushButton_xp, &QPushButton::clicked, this, &DazuTest::xp);
  connect(m_Controls.pushButton_yp, &QPushButton::clicked, this, &DazuTest::yp);
  connect(m_Controls.pushButton_zp, &QPushButton::clicked, this, &DazuTest::zp);
  connect(m_Controls.pushButton_rxp, &QPushButton::clicked, this, &DazuTest::rxp);
  connect(m_Controls.pushButton_ryp, &QPushButton::clicked, this, &DazuTest::ryp);
  connect(m_Controls.pushButton_rzp, &QPushButton::clicked, this, &DazuTest::rzp);
  connect(m_Controls.pushButton_xm, &QPushButton::clicked, this, &DazuTest::xm);
  connect(m_Controls.pushButton_ym, &QPushButton::clicked, this, &DazuTest::ym);
  connect(m_Controls.pushButton_zm, &QPushButton::clicked, this, &DazuTest::zm);
  connect(m_Controls.pushButton_rxm, &QPushButton::clicked, this, &DazuTest::rxm);
  connect(m_Controls.pushButton_rym, &QPushButton::clicked, this, &DazuTest::rym);
  connect(m_Controls.pushButton_rzm, &QPushButton::clicked, this, &DazuTest::rzm);

  connect(m_Controls.pushButton_MoveJ, &QPushButton::clicked, this, &DazuTest::MoveJ);
  connect(m_Controls.pushButtoN_MoveJJoint, &QPushButton::clicked, this, &DazuTest::MoveJJoint);
  connect(m_Controls.pushButton_MoveL, &QPushButton::clicked, this, &DazuTest::MoveL);


  connect(m_Controls.pushButton_SetForceControl, &QPushButton::clicked, this, &DazuTest::setforcecontrol);
  connect(m_Controls.pushButton_CancelForceControl, &QPushButton::clicked, this, &DazuTest::cancelforcecontrol);
  connect(m_Controls.pushButton_SetForceToolMotion, &QPushButton::clicked, this, &DazuTest::settoolmove);
  connect(m_Controls.pushButton_ClearForce, &QPushButton::clicked, this, &DazuTest::setforcezero);
  connect(m_Controls.pushButton_SetForceVelocites, &QPushButton::clicked, this, &DazuTest::setvelocities);
  connect(m_Controls.pushButton_SetFreedom, &QPushButton::clicked, this, &DazuTest::setfreedom);
  connect(m_Controls.pushButton_SetConstForce, &QPushButton::clicked, this, &DazuTest::setConstantforce);
  connect(m_Controls.pushButton_setObstacleAvoidance, &QPushButton::clicked, this, &DazuTest::setObstacleavoidance);
  connect(m_Controls.pushButton_SetMassParam, &QPushButton::clicked, this, &DazuTest::setmassparam);
  connect(m_Controls.pushButton_SetDampParam, &QPushButton::clicked, this, &DazuTest::setdampparam);
  connect(m_Controls.pushButton_SetStiffParam, &QPushButton::clicked, this, &DazuTest::setstiffparam);
  connect(m_Controls.pushButton_SetGoalForce, &QPushButton::clicked, this, &DazuTest::setgoalforce);
  connect(m_Controls.pushButton_SetForceLimit, &QPushButton::clicked, this, &DazuTest::setforceDataLimits);


  connect(m_Controls.pushButton_SetForceFreeDrive, &QPushButton::clicked, this, &DazuTest::setforcefreedrivemode);
  connect(m_Controls.pushButton_CancelForceFreeDrive, &QPushButton::clicked, this, &DazuTest::canforcefreedrivemode);
  connect(m_Controls.pushButton_ReadForceSensor, &QPushButton::clicked, this, &DazuTest::readforce);
  connect(m_Controls.pushButton_SetForceFreeDriveFreedom, &QPushButton::clicked, this, &DazuTest::setfreedrivefreedom);
  connect(m_Controls.pushButton_SetFTFreeFaztor, &QPushButton::clicked, this, &DazuTest::setFTFreeFaztor);
  connect(m_Controls.pushButton_SetCompensateForce, &QPushButton::clicked, this, &DazuTest::setcompensateforce);
  //connect(m_Controls.pushButton_43, &QPushButton::clicked, this, &DazuTest::setfreedriveforce);
  //connect(m_Controls.pushButton_44, &QPushButton::clicked, this, &DazuTest::setfreefrivevel);
  //connect(m_Controls.pushButton_46, &QPushButton::clicked, this, &DazuTest::setmaxdistance);
  //connect(m_Controls.pushButton_47, &QPushButton::clicked, this, &DazuTest::setdeviationrange);
  connect(m_Controls.pushButton_SetTangentForceBound, &QPushButton::clicked, this, &DazuTest::settangentforcebound);

  connect(m_Controls.pushButton_Test, &QPushButton::clicked, this, &DazuTest::JustForTest);
  connect(m_Controls.pushButton_fps, &QPushButton::clicked, this, &DazuTest::FPS_test);
  connect(m_Controls.pushButton_DealwithData, &QPushButton::clicked, this, &DazuTest::DealwithData);
  connect(m_Controls.pushButton_ConfigUCS, &QPushButton::clicked, this, &DazuTest::ConfigUCS);
  connect(m_Controls.pushButton_delay, &QPushButton::clicked, this, &DazuTest::RobotDelayTest);
  connect(m_Controls.pushButton_dealwithdelaydata, &QPushButton::clicked, this, &DazuTest::DealwithDelayData);
}

DazuTest::DazuTest() {
	init(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\log.txt");
}

DazuTest::DazuTest(const std::string& logPath) {
	init(logPath);
}

DazuTest::~DazuTest() {
	// 恢复原来的 std::cout 缓冲区并释放资源
	std::cout.rdbuf(oldCoutStreambuf);
	delete teeStreambuf;
	if (logFile.is_open()) {
		logFile.close();
	}
}

void DazuTest::init(const std::string& logPath) {
	std::filesystem::path dir = std::filesystem::path(logPath).parent_path();

	if (!std::filesystem::exists(dir)) {
		if (!std::filesystem::create_directories(dir)) {
			std::cerr << "Failed to create directory: " << dir << std::endl;
		}
	}

	// 打开日志文件，以追加模式写入
	logFile.open(logPath, std::ios::out | std::ios::app);
	if (!logFile) {
		std::cerr << "Failed to create log file: " << logPath << std::endl;
		return;
	}

	// 创建 TeeStreambuf 对象，将 std::cout 同时输出到终端和日志文件
	teeStreambuf = new TeeStreambuf(std::cout.rdbuf(), logFile.rdbuf());
	// 保存原始的 std::cout 缓冲区
	oldCoutStreambuf = std::cout.rdbuf(teeStreambuf);
}


void DazuTest::suddenstop()
{
	bool nRet = Robot->Stop(m_Controls.textBrowser_Log);
}

void DazuTest::connectArm()
{
	bool nRet = Robot->Connect(m_Controls.textBrowser_Log);
}

void DazuTest::powerOn()
{
	bool nRet = Robot->PowerOn(m_Controls.textBrowser_Log);
}

void DazuTest::Reset() {
	bool nRet = Robot->Reset(m_Controls.textBrowser_Log);
}

//---------------------------------------------------------------------------------------------------------------
/**
 * @brief 关闭机械臂电源
 */
 //---------------------------------------------------------------------------------------------------------------
void DazuTest::powerOff()
{
	bool nRet = Robot->PowerOff(m_Controls.textBrowser_Log);
}

void DazuTest::OpenFreeDrag()
{
	bool nRet = Robot->StartFreeGrag(m_Controls.textBrowser_Log);
}

void DazuTest::TurnOffFreeDrag()
{
	bool nRet = Robot->StopFreeGrag(m_Controls.textBrowser_Log);
}

void DazuTest::ConnectAim() {
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

void DazuTest::AimAcquisition() {
	if (m_AimoeVisualizeTimer == nullptr)
	{
		m_AimoeVisualizeTimer = new QTimer(this);
	}
	connect(m_AimoeVisualizeTimer, SIGNAL(timeout()), this, SLOT(updateCameraData())); //口腔拿数据第二种方式
	m_AimoeVisualizeTimer->start(100);
	m_Controls.textBrowser_Log->append("Get Tool Info Timer Success");
	std::cout << "Get Tool Info Timer Success"<<std::endl;
}

void DazuTest::updateCameraData()
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
		std::cout << "Find Tool Info Faild" << std::endl;
	}

}

void DazuTest::UpdateCameraToToolMatrix(T_AimToolDataResult* ToolData, const char* Name, double* Camera2Tool, QLabel* label) {

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

void DazuTest::Printmatrix() {
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


void DazuTest::SetToolMotion()
{
	bool nRet = Robot->SetMoveToolMotion(m_Controls.textBrowser_Log, 1);
}

void DazuTest::SetBaseMotion() {
	bool nRet = Robot->SetMoveToolMotion(m_Controls.textBrowser_Log, 0);
}

void DazuTest::setTCPToFlange() {
	std::array<double, 6> TCP{ 0, 0, 0, 0, 0, 0 };
	bool nRet = Robot->ConfigFlangetoTCP(m_Controls.textBrowser_Log, TCP, sTcpName);
}

void DazuTest::ReadActPose() {
	Joint_init = Robot->ReadJoint(m_Controls.textBrowser_Log);
}

void DazuTest::gotoinitpos() {
	bool nRet = Robot->MoveJ(m_Controls.textBrowser_Log, Joint_init, 1);
}

void DazuTest::CaptureRobotPose(bool translationOnly) {
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

void DazuTest::CaptureRobot() {
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

void DazuTest::CaptureRobotFinash() {
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

void DazuTest::SaveRobotMatrix() {
	Robot->UtilityFunction->SaveFiles(m_Controls.textBrowser_Log, "\\Desktop\\save\\T_BaseToBaseRF.txt", T_BaseToBaseRF);
	Robot->UtilityFunction->SaveFiles(m_Controls.textBrowser_Log, "\\Desktop\\save\\T_FlangeToEndRF.txt", T_FlangeToEndRF);

	m_Controls.textBrowser_Log->append("saveArmMatrix");
}

void DazuTest::ReuseRobotMatrix() {
	Robot->UtilityFunction->ReuseFiles(m_Controls.textBrowser_Log, "\\Desktop\\save\\T_BaseToBaseRF.txt", T_BaseToBaseRF);
	Robot->UtilityFunction->ReuseFiles(m_Controls.textBrowser_Log, "\\Desktop\\save\\T_FlangeToEndRF.txt", T_FlangeToEndRF);

	m_Controls.textBrowser_Log->append("ReuseArmMatrix");
}

void DazuTest::DeleteRobotMatrixCache() {
	m_Controls.textBrowser_Log->append("Replace Registration");
	m_RobotRegistration.RemoveAllPose();
	m_IndexOfRobotCapture = 0;
	m_Controls.lineEdit_NumOfRobotPose->setText(QString::number(m_IndexOfRobotCapture));
}

void DazuTest::AutoCaptureRobot() {

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

void DazuTest::automove() {
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
		default :
			break;
	}
}

void DazuTest::DemarcateTCP() {

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

void DazuTest::SaveTCP() {
	Robot->UtilityFunction->SaveFiles(m_Controls.textBrowser_Log, "\\Desktop\\Test\\save\\T_FlangeToTCP.txt", T_FlangeToTCP);

	m_Controls.textBrowser_Log->append("saveTCPMatrix");
}

void DazuTest::ReuseTCP() {
	Robot->UtilityFunction->ReuseFiles(m_Controls.textBrowser_Log, "\\Desktop\\Test\\save\\T_FlangeToTCP.txt", T_FlangeToTCP);
	ApplyTCP();
	m_Controls.textBrowser_Log->append("ReuseTCPMatrix");

}

void DazuTest::ApplyTCP() {
	auto vtkT_FlanegToTCP = vtkMatrix4x4::New();
	Robot->UtilityFunction->Double4x4ToMatrix4x4(T_FlangeToTCP, vtkT_FlanegToTCP);

	Eigen::Vector3d Angle = Robot->GetAulerAngle(vtkT_FlanegToTCP);

	std::array<double, 6> TCP{ vtkT_FlanegToTCP->GetElement(0, 3), vtkT_FlanegToTCP->GetElement(1, 3), vtkT_FlanegToTCP->GetElement(2, 3),
			vtkMath::DegreesFromRadians(Angle[2]), vtkMath::DegreesFromRadians(Angle[1]), vtkMath::DegreesFromRadians(Angle[0]) };

	Robot->ConfigFlangetoTCP(m_Controls.textBrowser_Log, TCP, sTcpName);

}

void DazuTest::ImageBoneLandmarkCapture() {
 
}

void DazuTest::ImageBoneLandmark() {
	
}

void DazuTest::ImageBoneICPCapture() {

}

void DazuTest::ImageBoneICP() {

}

void DazuTest::MixLandmarkandICPImageBone() {

}

void DazuTest::UpdateImageBone() {

}

void DazuTest::CleanRegistrationCacheImageBone() {

}

void DazuTest::SaveRegistrationMatrixImageBone() {

}

void DazuTest::ReuseRegistrationMatrixImageBone() {

}

void DazuTest::StartForceControl() {
	std::array<double, 6> Freedom{ 0,0,0,0,0,1 };
	Robot->SetForceZero(m_Controls.textBrowser_Log);
	Robot->SetForceFreeDriveFreedom(m_Controls.textBrowser_Log, Freedom);
	// 定义力控自由驱动自由度状态
	Robot->SetForceFreeDriveVelocity(m_Controls.textBrowser_Log, 10, 10);
	// 开启力控自由驱动
	Robot->SetForceFreeDrive(m_Controls.textBrowser_Log, 1);

	if (DeepProtectionTimer == nullptr)
	{
		DeepProtectionTimer = new QTimer(this);
		connect(DeepProtectionTimer, SIGNAL(timeout()), this, SLOT(DeepProtection()));
		DeepProtectionTimer->start(30);
	}
}

void DazuTest::DeepProtection() {
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
		std::array<double, 6> Force = Robot->ReadFTCabData(m_Controls.textBrowser_Log);
		if (m_Controls.radioButton_mandible->isChecked() && Force[3] < 0) {
			Robot->SetForceZero(m_Controls.textBrowser_Log);
			Robot->SetForceFreeDrive(m_Controls.textBrowser_Log, 1);
		}
		else if (m_Controls.radioButton_maxilla->isChecked() && Force[3] > 0) {
			Robot->SetForceZero(m_Controls.textBrowser_Log);
			Robot->SetForceFreeDrive(m_Controls.textBrowser_Log, 1);
		} 
		else
		{
			Robot->Stop(m_Controls.textBrowser_Log);
			//DeepProtectionTimer->stop();

			////软件显示已到达目标深度，其实如果能从机械臂身上把供电捞出去，设置电钻不钻就可以了，也不需要这些stop什么的。
			//Robot->UtilityFunction->Sleep(800);
			////页面上点击确定后再开启
			//Robot->SetForceFreeDrive(m_Controls.textBrowser_Log, 1);
			//Robot->UtilityFunction->Sleep(1000);
			//DeepProtectionTimer->start(30);
		}
		
	}
}


/////////////////////////////////////////////////////////////////////机械臂测试用////////////////////////////////////////////////////////////////////////////
/*============================================================================

机械臂测试用

============================================================================*/
void DazuTest::JustForTest() {
	//Rotate在右边后面乘
	//以前用模长来进行ASIN纯属多余，直接acos（a.dot(b)）就完事了
	//不知道为什么，轴角法要用post后乘，而且要先setMatrix然后在变换，很奇怪啊很奇怪
	//post后乘和pre前乘是可以随意切换的，值是可以从新设的
	//vtktransform不能乱用，只能用一次，因为->getmatrix（）是指针操作，可以用DeepCopy
	//运算过程不会改变过程中的矩阵
	//运算参数里Matrix和Transform的效果是一样的
	//eigen是以欧拉角
}

void DazuTest::FPS_test() {

	std::ofstream robotMatrixFile1(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\time.txt");
	QDateTime now;
	while (Robot->isMoving(m_Controls.textBrowser_Log)) {
		
		std::array<double, 6> TCP = Robot->ReadBasetoTCP(m_Controls.textBrowser_Log);
		now = QDateTime::currentDateTime();
		qDebug() << "Current time:" << now;
		for (int i = 0; i < 6; i++) {
			robotMatrixFile1 << TCP[i];
			if (i < 5) {
				robotMatrixFile1 << ",";	
			}
		}
		robotMatrixFile1 << std::endl << now.toString("hh:mm:ss.zzz").toStdString() << std::endl;
	}
	robotMatrixFile1.close();
	//qDebug() << "Nano seconds since start:" << nanoSeconds;

}

void DazuTest::DealwithData() {
	std::vector<std::vector<double>> tcpData;
	std::vector<QDateTime> timeStamps;

	// 读取数据
	std::string filePath = std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\time.txt";
	std::ifstream inFile(filePath);
	if (!inFile) {
		std::cerr << "Failed to open file: " << filePath << std::endl;
	}

	std::string line;
	while (std::getline(inFile, line)) {
		std::vector<double> tcpFrame;
		std::stringstream ss(line);
		std::string value;

		// 解析TCP数据
		while (std::getline(ss, value, ',')) {
			tcpFrame.push_back(std::stod(value));
		}

		// 检查是否读取的是TCP数据（长度为6）
		if (tcpFrame.size() == 6) {
			tcpData.push_back(tcpFrame);
		}
		else if (!tcpFrame.empty()) {
			// 解析时间
			QString qTimeStr = QString::fromStdString(line);
			QDateTime time = QDateTime::fromString(qTimeStr, "hh:mm:ss.zzz");
			timeStamps.push_back(time);
		}
	}

	inFile.close();

	// 检查数据完整性
	if (tcpData.size() != timeStamps.size() || tcpData.size() < 2) {
		std::cerr << "Data mismatch or not enough data to process." << std::endl;
	}

	// 存储通信间隔
	std::vector<qint64> communicationIntervals;

	// 记录最后发生变化的时间
	QDateTime lastUpdatedTime = timeStamps[0];
	std::vector<double> lastUpdatedTCP = tcpData[0];

	// 遍历每一帧TCP数据，比较相邻帧是否发生变化，并记录时间差
	for (size_t i = 1; i < tcpData.size(); ++i) {
		bool isDifferent = false;
		for (size_t j = 0; j < lastUpdatedTCP.size(); ++j) {
			if (lastUpdatedTCP[j] != tcpData[i][j]) {
				isDifferent = true;
				break;
			}
		}

		// 如果当前帧和上一次更新的帧不同
		if (isDifferent) {
			// 计算时间差（以毫秒为单位）
			qint64 timeDiff = lastUpdatedTime.msecsTo(timeStamps[i]);
			communicationIntervals.push_back(timeDiff);

			// 更新最后发生变化的时间和数据
			lastUpdatedTime = timeStamps[i];
			lastUpdatedTCP = tcpData[i];

			std::ofstream robotMatrixFile(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\dtime.txt", std::ios::app);
	
			robotMatrixFile << timeDiff << std::endl;
			robotMatrixFile.close();
		}
	}

	// 计算平均通信间隔
	if (communicationIntervals.empty()) {
		std::cerr << "No data updates detected between frames." << std::endl;
	}

	qint64 totalInterval = 0;
	for (auto interval : communicationIntervals) {
		totalInterval += interval;
	}
	double averageInterval = static_cast<double>(totalInterval) / communicationIntervals.size();

	// 计算帧率（每秒的帧数 = 1000毫秒 / 平均通信间隔）
	double frameRate = 1000.0 / averageInterval;

	// 输出结果
	std::cout << "Average Communication Interval: " << averageInterval << " ms" << std::endl;
	std::cout << "Frame Rate: " << frameRate << " frames per second" << std::endl;
}

void DazuTest::RobotDelayTest() {
	std::ofstream robotMatrixFile1(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\time_delay_before.txt");
	std::ofstream robotMatrixFile2(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\time_delay_final.txt");

	std::array<double, 6> Joint{ 0,0,90,0,90,0 };
	std::array<QDateTime, 2> NOW= GetDelayTime(Joint);
	robotMatrixFile1 << NOW[0].toString("hh:mm:ss.zzz").toStdString() << std::endl;
	robotMatrixFile2 << NOW[1].toString("hh:mm:ss.zzz").toStdString() << std::endl;
	while ((Robot->isMoving(m_Controls.textBrowser_Log))) {
		Robot->UtilityFunction->Sleep(1000);
	}

	Joint = { 0,0,90,0,0,0 };
	NOW = GetDelayTime(Joint);
	robotMatrixFile1 << NOW[0].toString("hh:mm:ss.zzz").toStdString() << std::endl;
	robotMatrixFile2 << NOW[1].toString("hh:mm:ss.zzz").toStdString() << std::endl;
	while ((Robot->isMoving(m_Controls.textBrowser_Log))) {
		Robot->UtilityFunction->Sleep(1000);
	}

	Joint = { 0,0,90,0,90,0 };
	NOW = GetDelayTime(Joint);
	robotMatrixFile1 << NOW[0].toString("hh:mm:ss.zzz").toStdString() << std::endl;
	robotMatrixFile2 << NOW[1].toString("hh:mm:ss.zzz").toStdString() << std::endl;
	while ((Robot->isMoving(m_Controls.textBrowser_Log))) {
		Robot->UtilityFunction->Sleep(1000);
	}
	Joint = { 0,0,90,0,0,0 };
	NOW = GetDelayTime(Joint);
	robotMatrixFile1 << NOW[0].toString("hh:mm:ss.zzz").toStdString() << std::endl;
	robotMatrixFile2 << NOW[1].toString("hh:mm:ss.zzz").toStdString() << std::endl;
	while ((Robot->isMoving(m_Controls.textBrowser_Log))) {
		Robot->UtilityFunction->Sleep(1000);
	}
	Joint = { 0,0,90,0,90,0 };
	NOW = GetDelayTime(Joint);
	robotMatrixFile1 << NOW[0].toString("hh:mm:ss.zzz").toStdString() << std::endl;
	robotMatrixFile2 << NOW[1].toString("hh:mm:ss.zzz").toStdString() << std::endl;
	while ((Robot->isMoving(m_Controls.textBrowser_Log))) {
		Robot->UtilityFunction->Sleep(1000);
	}
	Joint = { 0,0,90,0,0,0 };
	NOW = GetDelayTime(Joint);
	robotMatrixFile1 << NOW[0].toString("hh:mm:ss.zzz").toStdString() << std::endl;
	robotMatrixFile2 << NOW[1].toString("hh:mm:ss.zzz").toStdString() << std::endl;
	
	robotMatrixFile1.close();
}

std::array<QDateTime, 2> DazuTest::GetDelayTime(std::array<double, 6> Joint) {
	QDateTime now1 = QDateTime::currentDateTime();
	Robot->MoveJ(m_Controls.textBrowser_Log, Joint, 1);
	while (!(Robot->isMoving(m_Controls.textBrowser_Log))) {
	}

	QDateTime now2 = QDateTime::currentDateTime();
	std::cout << "time_delay_before time:" << now1.toString("hh:mm:ss.zzz").toStdString() << std::endl;
	std::cout << "time_delay_final time:" << now2.toString("hh:mm:ss.zzz").toStdString() << std::endl;
	std::array<QDateTime, 2> now{ now1, now2 };
	return now;
}

void DazuTest::DealwithDelayData() {

	// 打开保存的文件
	std::ifstream beforeFile(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\time_delay_before.txt");
	std::ifstream finalFile(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\time_delay_final.txt");
	std::ofstream diffFile(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\time_delay_diff.txt");

	// 确保文件被正确打开
	if (!beforeFile.is_open() || !finalFile.is_open() || !diffFile.is_open()) {
		std::cerr << "无法打开文件。" << std::endl;
		return;
	}

	std::string beforeLine, finalLine;

	while (std::getline(beforeFile, beforeLine) && std::getline(finalFile, finalLine)) {
		// 将字符串转换为QDateTime对象
		QDateTime beforeTime = QDateTime::fromString(QString::fromStdString(beforeLine), "hh:mm:ss.zzz");
		QDateTime finalTime = QDateTime::fromString(QString::fromStdString(finalLine), "hh:mm:ss.zzz");

		// 计算时间差
		qint64 timeDiffMs = beforeTime.msecsTo(finalTime);

		// 输出时间差到新文件
		diffFile << timeDiffMs << " ms" << std::endl;
		qDebug() << "Diff time:" << timeDiffMs;
	}

	// 关闭文件
	beforeFile.close();
	finalFile.close();
	diffFile.close();

}







void DazuTest::ConfigUCS() {
	std::array<double, 6> UCS{
		m_Controls.lineEdit_TCP_x->text().toDouble(),
		m_Controls.lineEdit_TCP_y->text().toDouble(),
		m_Controls.lineEdit_TCP_z->text().toDouble(),
		m_Controls.lineEdit_TCP_rx->text().toDouble(),
		m_Controls.lineEdit_TCP_ry->text().toDouble(),
		m_Controls.lineEdit_TCP_rz->text().toDouble()
	};

	std::string UCS_Name = m_Controls.lineEdit_TCPName->text().toStdString();
	Robot->ConfigBasetoUCS(m_Controls.textBrowser_Log, UCS, UCS_Name);

}


void DazuTest::SetTCPbyName() {

	std::array<double, 6> TCP{
		m_Controls.lineEdit_TCP_x->text().toDouble(),
		m_Controls.lineEdit_TCP_y->text().toDouble(),
		m_Controls.lineEdit_TCP_z->text().toDouble(),
		m_Controls.lineEdit_TCP_rx->text().toDouble(),
		m_Controls.lineEdit_TCP_ry->text().toDouble(),
		m_Controls.lineEdit_TCP_rz->text().toDouble()
	};

	std::string TCP_NAME = m_Controls.lineEdit_TCPName->text().toStdString();
	Robot->SetFlangetoTCP(m_Controls.textBrowser_Log, TCP, TCP_NAME);
}

void DazuTest::ConfigTCP() {
	std::array<double, 6> TCP{
		m_Controls.lineEdit_TCP_x->text().toDouble(),
		m_Controls.lineEdit_TCP_y->text().toDouble(),
		m_Controls.lineEdit_TCP_z->text().toDouble(),
		m_Controls.lineEdit_TCP_rx->text().toDouble(),
		m_Controls.lineEdit_TCP_ry->text().toDouble(),
		m_Controls.lineEdit_TCP_rz->text().toDouble()
	};

	std::string TCP_NAME = m_Controls.lineEdit_TCPName->text().toStdString();
	Robot->ConfigFlangetoTCP(m_Controls.textBrowser_Log, TCP, TCP_NAME);
}

void DazuTest::ReadBasetoFlange() {
	std::array<double, 6> TCP;
	TCP = Robot->ReadBasetoFlange(m_Controls.textBrowser_Log);
}

void DazuTest::ReadActTCPByName() {

	std::array<double, 6> TCP;
	std::string TCP_NAME = m_Controls.lineEdit_TCPName->text().toStdString();
	TCP = Robot->ReadBasetoTCP(m_Controls.textBrowser_Log, TCP_NAME);
}

void DazuTest::ReadcmdTCPPose() {
	std::array<double, 6> TCP;
	TCP = Robot->ReadTargetBasetoTCP(m_Controls.textBrowser_Log);
}

void DazuTest::ReadCurTCP() {
	std::array<double, 6> TCP;
	TCP = Robot->ReadFlangetoTCP(m_Controls.textBrowser_Log);
}

void DazuTest::xp()
{
	Robot->RelMoveXplus(m_Controls.textBrowser_Log, m_Controls.lineEdit_Translate->text().toDouble());
}

void DazuTest::yp()
{
	Robot->RelMoveYplus(m_Controls.textBrowser_Log, m_Controls.lineEdit_Translate->text().toDouble());
}

void DazuTest::zp()
{
	Robot->RelMoveZplus(m_Controls.textBrowser_Log, m_Controls.lineEdit_Translate->text().toDouble());

}

void DazuTest::xm()
{
	Robot->RelMoveXmin(m_Controls.textBrowser_Log, m_Controls.lineEdit_Translate->text().toDouble());

}

void DazuTest::ym()
{
	Robot->RelMoveYmin(m_Controls.textBrowser_Log, m_Controls.lineEdit_Translate->text().toDouble());

}

void DazuTest::zm()
{
	Robot->RelMoveZmin(m_Controls.textBrowser_Log, m_Controls.lineEdit_Translate->text().toDouble());

}

void DazuTest::rxp()
{
	Robot->RelMoveRxplus(m_Controls.textBrowser_Log, m_Controls.lineEdit_Rotate->text().toDouble());

}

void DazuTest::ryp()
{
	Robot->RelMoveRyplus(m_Controls.textBrowser_Log, m_Controls.lineEdit_Rotate->text().toDouble());

}

void DazuTest::rzp()
{
	Robot->RelMoveRzplus(m_Controls.textBrowser_Log, m_Controls.lineEdit_Rotate->text().toDouble());

}

void DazuTest::rxm()
{
	Robot->RelMoveRxmin(m_Controls.textBrowser_Log, m_Controls.lineEdit_Rotate->text().toDouble());

}

void DazuTest::rym()
{
	Robot->RelMoveRymin(m_Controls.textBrowser_Log, m_Controls.lineEdit_Rotate->text().toDouble());

}

void DazuTest::rzm()
{
	Robot->RelMoveRzmin(m_Controls.textBrowser_Log, m_Controls.lineEdit_Rotate->text().toDouble());

}

void DazuTest::MoveJ() {

	std::array<double, 6> TargetPoint{
		m_Controls.lineEdit_TargetX->text().toDouble(),
		m_Controls.lineEdit_TargetY->text().toDouble(),
		m_Controls.lineEdit_TargetZ->text().toDouble(),
		m_Controls.lineEdit_TargetRX->text().toDouble(),
		m_Controls.lineEdit_TargetRY->text().toDouble(),
		m_Controls.lineEdit_TargetRZ->text().toDouble()
	};

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	std::string TCP_NAME = m_Controls.lineEdit_TCPName->text().toStdString();
	HRIF_MoveJ(0, 0, TargetPoint[0], TargetPoint[1], TargetPoint[2], TargetPoint[3], TargetPoint[4], TargetPoint[5],0, 0, 90, 0, 0, 0, TCP_NAME, "Base", 10, 12, 30, 1, 0, 0, 0, "0");
	//Robot->MoveJ(m_Controls.textBrowser_Log, TargetPoint);
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void DazuTest::MoveL() {
	std::array<double, 6> TargetPoint{
		m_Controls.lineEdit_TargetX->text().toDouble(),
		m_Controls.lineEdit_TargetY->text().toDouble(),
		m_Controls.lineEdit_TargetZ->text().toDouble(),
		m_Controls.lineEdit_TargetRX->text().toDouble(),
		m_Controls.lineEdit_TargetRY->text().toDouble(),
		m_Controls.lineEdit_TargetRZ->text().toDouble()
	};

	Robot->MoveP(m_Controls.textBrowser_Log, TargetPoint);
}

void DazuTest::MoveJJoint() {
	std::array<double, 6> Joint{
		m_Controls.lineEdit_TargetX->text().toDouble(),
		m_Controls.lineEdit_TargetY->text().toDouble(),
		m_Controls.lineEdit_TargetZ->text().toDouble(),
		m_Controls.lineEdit_TargetRX->text().toDouble(),
		m_Controls.lineEdit_TargetRY->text().toDouble(),
		m_Controls.lineEdit_TargetRZ->text().toDouble()
	};

	Robot->MoveJ(m_Controls.textBrowser_Log, Joint, 1);
}

void DazuTest::setforcecontrol() {
	int boxID = 0;
	int robotID = 0;
	int nState = 1;
	int result = HRIF_SetForceControlState(boxID, robotID, nState);
	if (result) std::cout << result << std::endl;
	else {
		m_Controls.textBrowser_Log->append("start force control");
	}
}

void DazuTest::cancelforcecontrol() {
	int boxID = 0;
	int robotID = 0;
	int nState = 0;
	int result = HRIF_SetForceControlState(boxID, robotID, nState);
	if (result) std::cout << result << std::endl;
	else {
		m_Controls.textBrowser_Log->append("end force control");
	}
}

void DazuTest::settoolmove() {
	int boxID = 0;
	int robotID = 0;
	int ToolMoveType = 1;
	int result = HRIF_SetForceToolCoordinateMotion(boxID, robotID, ToolMoveType);
	if (result) std::cout << result << std::endl;
	else {
		m_Controls.textBrowser_Log->append("move based on tcp");
	}
}

void DazuTest::setforcezero() {
	int boxID = 0;
	int robotID = 0;
	int result = HRIF_SetForceZero(boxID, robotID);
	if (result) std::cout << result << std::endl;
	else {
		m_Controls.textBrowser_Log->append("set force zero");
	}
}

void DazuTest::setvelocities() {
	double dMaxLinearVelocity = 10;
	double dMaxAngularVelocity = 5;
	int boxID = 0;
	int robotID = 0;
	int result = HRIF_SetMaxSearchVelocities(boxID, robotID, dMaxLinearVelocity, dMaxAngularVelocity);
	if (result) std::cout << result << std::endl;
	else {
		m_Controls.textBrowser_Log->append("start force control\nlinear:	" + QString::number(dMaxLinearVelocity) + "angular:	" + QString::number(dMaxAngularVelocity));
	}
}

void DazuTest::setfreedom() {
	int x = m_Controls.lineEdit_ForceX->text().toInt();
	int y = m_Controls.lineEdit_ForceY->text().toInt();
	int z = m_Controls.lineEdit_ForceZ->text().toInt();
	int rx = m_Controls.lineEdit_ForceRx->text().toInt();
	int ry = m_Controls.lineEdit_ForceRy->text().toInt();
	int rz = m_Controls.lineEdit_ForceRz->text().toInt();
	int boxID = 0;
	int robotID = 0;
	int result = HRIF_SetControlFreedom(boxID, robotID, x, y, z, rx, ry, rz);
	if (result) std::cout << result << std::endl;
	else {
		m_Controls.textBrowser_Log->append("freedom:\nX:	" + QString::number(x) + "		Y:	" + QString::number(y) + "		Z:	" + QString::number(z) + "  \nRX:	" +
			QString::number(rx) + "		RY:	" + QString::number(ry) + "		RZ:	" + QString::number(rz));
	}
}

void DazuTest::setConstantforce() {
	int nState = 0;
	int boxID = 0;
	int robotID = 0;
	int result = HRIF_SetForceControlStrategy(boxID, robotID, nState);
	if (result) std::cout << result << std::endl;
	else {
		m_Controls.textBrowser_Log->append("constant force");
	}
}

void DazuTest::setObstacleavoidance() {
	int nState = 2;
	int boxID = 0;
	int robotID = 0;
	int result = HRIF_SetForceControlStrategy(boxID, robotID, nState);
	if (result) std::cout << result << std::endl;
	else {
		m_Controls.textBrowser_Log->append("Crossing barriers");
	}
}

void DazuTest::setmassparam() {

	int dXMass = m_Controls.lineEdit_ForceX->text().toInt();
	int dYMass = m_Controls.lineEdit_ForceY->text().toDouble();
	int dZMass = m_Controls.lineEdit_ForceZ->text().toDouble();
	int dRxMass = m_Controls.lineEdit_ForceRx->text().toDouble();
	int dRyMass = m_Controls.lineEdit_ForceRy->text().toDouble();
	int dRzMass = m_Controls.lineEdit_ForceRz->text().toDouble();
	int boxID = 0;
	int robotID = 0;
	int result = HRIF_SetMassParams(boxID, robotID, dXMass, dYMass, dZMass, dRxMass, dRyMass, dRzMass);
	if (result) std::cout << result << std::endl;
	else {
		m_Controls.textBrowser_Log->append("massparam:\nX:	" + QString::number(dXMass) + "		Y:	" + QString::number(dYMass) + "		Z:	" + QString::number(dZMass) + "  \nRX:	" +
			QString::number(dRxMass) + "		RY:	" + QString::number(dRyMass) + "		RZ:	" + QString::number(dRzMass));
	}
}

void DazuTest::setdampparam() {
	int dXDamp = m_Controls.lineEdit_ForceX->text().toDouble();
	int dYDamp = m_Controls.lineEdit_ForceY->text().toDouble();
	int dZDamp = m_Controls.lineEdit_ForceZ->text().toDouble();
	int dRxDamp = m_Controls.lineEdit_ForceRx->text().toDouble();
	int dRyDamp = m_Controls.lineEdit_ForceRy->text().toDouble();
	int dRzDamp = m_Controls.lineEdit_ForceRz->text().toDouble();
	int boxID = 0;
	int robotID = 0;
	int result = HRIF_SetDampParams(boxID, robotID, dXDamp, dYDamp, dZDamp, dRxDamp, dRyDamp, dRzDamp);
	if (result) std::cout << result << std::endl;
	else {
		m_Controls.textBrowser_Log->append("Dampparam:\nX:	" + QString::number(dXDamp) + "		Y:	" + QString::number(dYDamp) + "		Z:	" + QString::number(dZDamp) + "  \nRX:	" +
			QString::number(dRxDamp) + "		RY:	" + QString::number(dRyDamp) + "		RZ:	" + QString::number(dRzDamp));
	}
}

void DazuTest::setstiffparam() {
	int dXStiff = m_Controls.lineEdit_ForceX->text().toDouble();
	int dYStiff = m_Controls.lineEdit_ForceY->text().toDouble();
	int dZStiff = m_Controls.lineEdit_ForceZ->text().toDouble();
	int dRxStiff = m_Controls.lineEdit_ForceRx->text().toDouble();
	int dRyStiff = m_Controls.lineEdit_ForceRy->text().toDouble();
	int dRzStiff = m_Controls.lineEdit_ForceRz->text().toDouble();
	int boxID = 0;
	int robotID = 0;
	int result = HRIF_SetStiffParams(boxID, robotID, dXStiff, dYStiff, dZStiff, dRxStiff, dRyStiff, dRzStiff);
	if (result) std::cout << result << std::endl;
	else {
		m_Controls.textBrowser_Log->append("Stiffparam:\nX:	" + QString::number(dXStiff) + "		Y:	" + QString::number(dYStiff) + "		Z:	" + QString::number(dZStiff) + "  \nRX:	" +
			QString::number(dRxStiff) + "		RY:	" + QString::number(dRyStiff) + "		RZ:	" + QString::number(dRzStiff));
	}
}

void DazuTest::setgoalforce() {
	int dXForce = m_Controls.lineEdit_ForceX->text().toDouble();
	int dYForce = m_Controls.lineEdit_ForceY->text().toDouble();
	int dZForce = m_Controls.lineEdit_ForceZ->text().toDouble();
	int dRxForce = m_Controls.lineEdit_ForceRx->text().toDouble();
	int dRyForce = m_Controls.lineEdit_ForceRy->text().toDouble();
	int dRzForce = m_Controls.lineEdit_ForceRz->text().toDouble();
	int boxID = 0;
	int robotID = 0;
	int result = HRIF_SetForceControlGoal(boxID, robotID, dXForce, dYForce, dZForce, dRxForce, dRyForce, dRzForce);
	if (result) std::cout << result << std::endl;
	else {
		m_Controls.textBrowser_Log->append("goalforce:\nX:	" + QString::number(dXForce) + "		Y:	" + QString::number(dYForce) + "		Z:	" + QString::number(dZForce) + "  \nRX:	" +
			QString::number(dRxForce) + "		RY:	" + QString::number(dRyForce) + "		RZ:	" + QString::number(dRzForce));
	}
}

void DazuTest::setforceDataLimits() {
	// 设置正方向力最大阈值
	double dMax_X = 500; double dMax_Y = 500; double dMax_Z = 500;
	double dMax_Rx = 50; double dMax_Ry = 50; double dMax_Rz = 50;
	// 设置负方向力最大阈值
	double dMin_X = 300; double dMin_Y = 300; double dMin_Z = 300;
	double dMin_Rx = 30; double dMin_Ry = 30; double dMin_Rz = 30;
	// 设置力保护限制范围

	int boxID = 0;
	int robotID = 0;
	int result = HRIF_SetForceDataLimit(boxID, robotID, dMax_X, dMax_Y, dMax_Z, dMax_Rx, dMax_Ry, dMax_Rz, dMin_X, dMin_Y, dMin_Z, dMin_Rx, dMin_Ry, dMin_Rz);
	if (result) std::cout << result << std::endl;
	else {
		m_Controls.textBrowser_Log->append("ForceLimit:\nMAX:\nX:	" + QString::number(dMax_X) + "		Y:	" + QString::number(dMax_Y) + "		Z:	" + QString::number(dMax_Z) + "  \nRX:	" +
			QString::number(dMax_Rx) + "		RY:	" + QString::number(dMax_Ry) + "		RZ:	" + QString::number(dMax_Rz) + "\nMIN:\nX:	" +
			QString::number(dMin_X) + "		Y:	" + QString::number(dMin_Y) + "		Z:	" + QString::number(dMin_Z) + "  \nRX:	" +
			QString::number(dMin_Rx) + "		RY:	" + QString::number(dMin_Ry) + "		RZ:	" + QString::number(dMin_Rz));
	}
}

void DazuTest::setforcefreedrivemode() {
	int boxID = 0;
	int robotID = 0;
	int nState = 1;
	int result = HRIF_SetForceFreeDriveMode(boxID, robotID, nState);
	if (result) std::cout << result << std::endl;
	else {
		m_Controls.textBrowser_Log->append("set force free drive mode");
	}
}

void DazuTest::canforcefreedrivemode() {
	int boxID = 0;
	int robotID = 0;
	int nState = 0;
	int result = HRIF_SetForceFreeDriveMode(boxID, robotID, nState);
	if (result) std::cout << result << std::endl;
	else {
		m_Controls.textBrowser_Log->append("cancel force free drive mode");
	}
}

void DazuTest::readforce() {
	double dX = 0; double dY = 0; double dZ = 0;
	double dRx = 0; double dRy = 0; double dRz = 0;
	// 读取标定后力传感器数据
	int boxID = 0;
	int robotID = 0;
	int result = HRIF_ReadFTCabData(boxID, robotID, dX, dY, dZ, dRx, dRy, dRz);
	if (result) std::cout << result << std::endl;
	else {
		m_Controls.textBrowser_Log->append("the force sensor read\nX:	" + QString::number(dX) + "		Y:	" + QString::number(dY) + "		Z:	" + QString::number(dZ) + "  \nRX:	" +
			QString::number(dRx) + "		RY:	" + QString::number(dRy) + "		RZ:	" + QString::number(dRz));
	}
}

void DazuTest::setfreedrivefreedom() {
	int x = m_Controls.lineEdit_ForceFreeDriveX->text().toInt();
	int y = m_Controls.lineEdit_ForceFreeDriveY->text().toInt();
	int z = m_Controls.lineEdit_ForceFreeDriveZ->text().toInt();
	int rx = m_Controls.lineEdit_ForceFreeDriveRx->text().toInt();
	int ry = m_Controls.lineEdit_ForceFreeDriveRy->text().toInt();
	int rz = m_Controls.lineEdit_ForceFreeDriveRz->text().toInt();
	int boxID = 0;
	int robotID = 0;
	int result = HRIF_SetFreeDriveMotionFreedom(boxID, robotID, x, y, z, rx, ry, rz);
	if (result) std::cout << result << std::endl;
	else {
		m_Controls.textBrowser_Log->append("freedom:\nX:	" + QString::number(x) + "		Y:	" + QString::number(y) + "		Z:	" + QString::number(z) + "  \nRX:	" +
			QString::number(rx) + "		RY:	" + QString::number(ry) + "		RZ:	" + QString::number(rz));
	}
}

void DazuTest::setFTFreeFaztor() {
	double x = m_Controls.lineEdit_ForceFreeDriveX->text().toDouble();
	double y = m_Controls.lineEdit_ForceFreeDriveY->text().toDouble();
	int boxID = 0;
	int robotID = 0;
	int result = HRIF_SetFTFreeFactor(boxID, robotID, x, y);
	if (result) std::cout << result << std::endl;
	else {
		m_Controls.textBrowser_Log->append("freedom:\nlinear:	" + QString::number(x) + "		Angular:	" + QString::number(y));
	}
}

void DazuTest::setcompensateforce() {
	double x = m_Controls.lineEdit_ForceFreeDriveX->text().toDouble();
	double y = m_Controls.lineEdit_ForceFreeDriveY->text().toDouble();
	double z = m_Controls.lineEdit_ForceFreeDriveZ->text().toDouble();

	int boxID = 0;
	int robotID = 0;
	double force = m_Controls.lineEdit_ForceFreeDriveRx->text().toDouble();
	int result = HRIF_SetFreeDriveCompensateForce(boxID, robotID, force, x, y, z);

	if (result) std::cout << result << std::endl;
	else {
		m_Controls.textBrowser_Log->append("CompensateForce:\nforce:	" + QString::number(force) + "		Direction:	" + QString::number(x) + QString::number(y) + QString::number(z));
	}
}

//void DazuTest::setfreedriveforce() {
//	// 定义力阈值
//	double m_Controls.lineEdit_Oral_x_2->text().toDouble();
//	// 定义力矩阈值
//	double m_Controls.lineEdit_Oral_y_2->text().toDouble();
//	// 设置力控自由驱动启动阈值（力与力矩）
//	int nRet = HRIF_SetFTWrenchThresholds(0, 0, dForceThreshold, dTorqueThreshold);
//}

//void DazuTest::setfreefrivevel() {
//	// 定义自由驱动最大直线速度
//	double dMaxLinearVelocity;
//	// 定义自由驱动最大角速度
//	double dMaxAngularVelocity;
//	// 设置力控自由驱动最大直线速度及姿态角速度
//	hrif_setmax
//	int nRet = HRIF_SetMaxFreeDriveVel(0, 0, dMaxLinearVelocity, dMaxAngularVelocity);
//}

//void DazuTest::setmaxdistance() {
//	// 定义各自由度力控探寻最大距离
//	double Dis_X = 300; double Dis_Y = 300; double Dis_Z = 300;
//	double Dis_RX = 20; double Dis_RY = 20; double Dis_RZ = 20;
//	// 设置各自由度力控探寻最大距离
//	int nRet = HRIF_SetMaxSearchDistance(0, 0, Dis_X, Dis_Y, Dis_Z, Dis_RX, Dis_RY, Dis_RZ);
//}

//void DazuTest::setdeviationrange() {
//	double Pos_X = 100; double Pos_Y = 100; double Pos_Z = 100;
//	double Pos_RX = 20; double Pos_RY = 20; double Pos_RZ = 20;
//	double Neg_X = -100; double Neg_Y = -100; double Neg_Z = -100;
//	double Neg_RX = -20; double Neg_RY = -20; double Neg_RZ = -20;
//	// 设置恒力控稳定阶段边界
//	int nRet = HRIF_SetSteadyContactDeviationRange(0, 0, Pos_X, Pos_Y, Pos_Z, Pos_RX, Pos_RY, Pos_RZ, Neg_X, Neg_Y, Neg_Z, Neg_RX, Neg_RY, Neg_RZ);
//}

void DazuTest::settangentforcebound() {
	double dMax = m_Controls.lineEdit_ForceFreeDriveX->text().toDouble();
	double dMin = m_Controls.lineEdit_ForceFreeDriveY->text().toDouble();
	double dVel = m_Controls.lineEdit_ForceFreeDriveZ->text().toDouble();

	// 设置 X/Y 方向切向力最大值、最小值和上抬最大速度
	int result = HRIF_SetTangentForceBounds(0, 0, dMax, dMin, dVel);
	if (result) std::cout << result << std::endl;
	else {
		m_Controls.textBrowser_Log->append("TangentForce:\nMax:	" + QString::number(dMax) + "		Min:	" + QString::number(dMin) + "		Vel:	" + QString::number(dVel));
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DazuTest::TrackingPosition() {

	vtkMatrix4x4* vtkT_BaseToBaseRF = vtkMatrix4x4::New();
	vtkT_BaseToBaseRF->DeepCopy(T_BaseToBaseRF);

	//获取T_BaseRFToCamera
	auto vtkT_BaseRFToCamera = vtkMatrix4x4::New();
	vtkT_BaseRFToCamera->DeepCopy(T_CamToBaseRF);
	vtkT_BaseRFToCamera->Invert();

	//获取T_CameraToPatientRF
	auto vtkT_CameraToProbe = vtkMatrix4x4::New();
	vtkT_CameraToProbe->DeepCopy(T_CamToProbe);

	auto trans_basetoprobe = vtkTransform::New();
	Robot->UtilityFunction->MultiplyMatrices(trans_basetoprobe, vtkT_BaseToBaseRF, vtkT_BaseRFToCamera, vtkT_CameraToProbe);
	trans_basetoprobe->PostMultiply();
	trans_basetoprobe->Translate(0, 100, 0);
	trans_basetoprobe->Update();

	auto vtkBasetoProbe = vtkMatrix4x4::New();
	vtkBasetoProbe->DeepCopy(trans_basetoprobe->GetMatrix());
	Robot->UtilityFunction->Matrix4x4ToDouble4x4(T_BasetoTarget, vtkBasetoProbe);

	double dMaxLineVel = 10;
	double dMaxOriVel = 10;
	int nRet = HRIF_SetPoseTrackingMaxMotionLimit(0, 0, dMaxLineVel, dMaxOriVel);

	double dTime = 10;
	nRet = HRIF_SetPoseTrackingStopTimeOut(0, 0, dTime);

	double dPosPID1 = 3; double dPosPID2 = 0.1; double dPosPID3 = 3;
	double dOriPID1 = 3; double dOriPID2 = 0.1; double dOriPID3 = 3;
	nRet = HRIF_SetPoseTrackingPIDParams(0, 0, dPosPID1, dPosPID2, dPosPID3, dOriPID1, dOriPID2, dOriPID3);

	int nState = 1;
	nRet = HRIF_SetPoseTrackingTargetPos(0, 0, 0, 0, 0, 0, 0, 0);

	nRet = HRIF_SetPoseTrackingState(0, 0, nState);


	
	Eigen::Matrix3d Re;
	Re << vtkBasetoProbe->GetElement(0, 0), vtkBasetoProbe->GetElement(0, 1), vtkBasetoProbe->GetElement(0, 2),
		vtkBasetoProbe->GetElement(1, 0), vtkBasetoProbe->GetElement(1, 1), vtkBasetoProbe->GetElement(1, 2),
		vtkBasetoProbe->GetElement(2, 0), vtkBasetoProbe->GetElement(2, 1), vtkBasetoProbe->GetElement(2, 2);

	Eigen::Vector3d angle = Re.eulerAngles(2, 1, 0);
	std::array<double, 6> Target{ vtkBasetoProbe->GetElement(0, 3), vtkBasetoProbe->GetElement(1, 3), vtkBasetoProbe->GetElement(2, 3),
		vtkMath::DegreesFromRadians(angle[2]), vtkMath::DegreesFromRadians(angle[1]), vtkMath::DegreesFromRadians(angle[0]) };

	Robot->MoveJ(m_Controls.textBrowser_Log, Target);
	
	nRet = HRIF_SetUpdateTrackingPose(0, 0, 0, 0, 0, 0, 0, 0);

	if (TrackingTimer == nullptr)
	{
		TrackingTimer = new QTimer(this);
	}
	connect(TrackingTimer, &QTimer::timeout, this, &DazuTest::StartTrackingPose);
	TrackingTimer->start(50);
}

void DazuTest::StartTrackingPose() {
	vtkMatrix4x4* vtkT_BaseToBaseRF = vtkMatrix4x4::New();
	vtkT_BaseToBaseRF->DeepCopy(T_BaseToBaseRF);

	//获取T_BaseRFToCamera
	auto vtkT_BaseRFToCamera = vtkMatrix4x4::New();
	vtkT_BaseRFToCamera->DeepCopy(T_CamToBaseRF);
	vtkT_BaseRFToCamera->Invert();

	//获取T_CameraToPatientRF
	auto vtkT_CameraToProbe = vtkMatrix4x4::New();
	vtkT_CameraToProbe->DeepCopy(T_CamToProbe);

	auto trans_basetoprobe = vtkTransform::New();
	Robot->UtilityFunction->MultiplyMatrices(trans_basetoprobe, vtkT_BaseToBaseRF, vtkT_BaseRFToCamera, vtkT_CameraToProbe);
	trans_basetoprobe->PostMultiply();
	trans_basetoprobe->Translate(0, 100, 0);
	trans_basetoprobe->Update();

	auto vtkBasetoProbe = vtkMatrix4x4::New();
	vtkBasetoProbe->DeepCopy(trans_basetoprobe->GetMatrix());

	auto oldTargettobase = vtkMatrix4x4::New();
	oldTargettobase->DeepCopy(T_BasetoTarget);
	oldTargettobase->Invert();

	auto trans_oldtargettotarget = vtkTransform::New();
	Robot->UtilityFunction->MultiplyMatrices(trans_oldtargettotarget, oldTargettobase, vtkBasetoProbe);

	auto vtk_oldtargettotarget = vtkMatrix4x4::New();
	vtk_oldtargettotarget->DeepCopy(trans_oldtargettotarget->GetMatrix());

	Eigen::Vector3d angle = Robot->GetAulerAngle(vtk_oldtargettotarget);

	double rx, ry, rz;
	rx = vtkMath::DegreesFromRadians(angle(2));
	ry = vtkMath::DegreesFromRadians(angle(1));
	rz = vtkMath::DegreesFromRadians(angle(0));

	int nRet = HRIF_SetPoseTrackingTargetPos(0, 0, vtk_oldtargettotarget->GetElement(0, 3), vtk_oldtargettotarget->GetElement(1, 3),
		vtk_oldtargettotarget->GetElement(2, 3), rx, ry, rz);



	std::array<double, 6> TCP = Robot->ReadBasetoTCP(m_Controls.textBrowser_Log);

	auto tmpTrans = vtkTransform::New();
	Robot->TranslateAndRotateMatrix(tmpTrans, TCP);

	auto vtkT_BaseToTCP = vtkMatrix4x4::New();
	vtkT_BaseToTCP = tmpTrans->GetMatrix();

	auto trans_oldtargettonow = vtkTransform::New();
	Robot->UtilityFunction->MultiplyMatrices(trans_oldtargettonow, oldTargettobase, vtkT_BaseToTCP);

	auto vtk_oldtargettonow = vtkMatrix4x4::New();
	vtk_oldtargettonow->DeepCopy(trans_oldtargettonow->GetMatrix());

	Eigen::Vector3d Angle = Robot->GetAulerAngle(vtk_oldtargettonow);

	rx = vtkMath::DegreesFromRadians(Angle(2));
	ry = vtkMath::DegreesFromRadians(Angle(1));
	rz = vtkMath::DegreesFromRadians(Angle(0));

	nRet = HRIF_SetUpdateTrackingPose(0, 0, vtk_oldtargettonow->GetElement(0, 3), vtk_oldtargettonow->GetElement(1, 3), vtk_oldtargettonow->GetElement(2, 3),
		rx, ry, rz);
}