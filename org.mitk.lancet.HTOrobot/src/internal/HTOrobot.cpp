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
#include "HTOrobot.h"

// Qt
#include <QMessageBox>
#include <mutex>
// mitk image
#include <mitkImage.h>
#include "mitkPointSet.h"
#include "mitkNodePredicateAnd.h"
#include "mitkNodePredicateDataType.h"
#include "mitkNodePredicateNot.h"
#include "mitkNodePredicateOr.h"
#include "mitkNodePredicateProperty.h"
#include "mitkImageToSurfaceFilter.h"

//vtk
#include <vtkLandmarkTransform.h>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkMatrix3x3.h>
#include <vtkPointData.h>
#include <QtCore\qmath.h>
#include <vtkRenderWindow.h>
#include <qthread.h>
AimHandle aimHandle = NULL;
E_Interface EI;
T_MarkerInfo markerSt;
T_AimPosStatusInfo statusSt;
const double PI = 3.1415926;

using namespace Eigen;
using namespace std;

const std::string HTOrobot::VIEW_ID = "org.mitk.views.htorobot";

void HTOrobot::SetFocus()
{
  //m_Controls.buttonPerformImageProcessing->setFocus();
}

void HTOrobot::CreateQtPartControl(QWidget *parent)
{
  // create GUI widgets from the Qt Designer's .ui file
  m_Controls.setupUi(parent);
  InitSurfaceSelector(m_Controls.mitkNodeSelectWidget_surface_regis); // Select a icp surface
  InitPointSetSelector(m_Controls.mitkNodeSelectWidget_landmark_src); // Select a landmark pointset
  InitPointSetSelector(m_Controls.mitkNodeSelectWidget_featurePoint);
  InitPointSetSelector(m_Controls.mitkNodeSelectWidget_tibiaPoint);
  InitPointSetSelector(m_Controls.mitkNodeSelectWidget_imageTargetPlane);

  m_NodetreeModel = new QmitkDataStorageTreeModel(this->GetDataStorage());
  //connect(m_Controls.buttonPerformImageProcessing, &QPushButton::clicked, this, &HTOrobot::DoImageProcessing);
  //HTO PrePlan
  connect(m_Controls.pushButton_createCutPlane, &QPushButton::clicked, this, &HTOrobot::CreateCutPlane);
  connect(m_Controls.pushButton_cutTibia, &QPushButton::clicked, this, &HTOrobot::CutTibia);
  connect(m_Controls.pushButton_showLine, &QPushButton::clicked, this, &HTOrobot::ShowLine);
  connect(m_Controls.pushButton_caculateFemurCenter, &QPushButton::clicked, this, &HTOrobot::OnFemurCenterClicked);
  connect(m_Controls.pushButton_unShowLine, &QPushButton::clicked, this, &HTOrobot::UnShowLine);
  connect(m_Controls.pushButton_xp, &QPushButton::clicked, this, &HTOrobot::TranslatePlusX);
  connect(m_Controls.pushButton_yp, &QPushButton::clicked, this, &HTOrobot::TranslatePlusY);
  connect(m_Controls.pushButton_zp, &QPushButton::clicked, this, &HTOrobot::TranslatePlusZ);
  connect(m_Controls.pushButton_xm, &QPushButton::clicked, this, &HTOrobot::TranslateMinusX);
  connect(m_Controls.pushButton_ym, &QPushButton::clicked, this, &HTOrobot::TranslateMinusY);
  connect(m_Controls.pushButton_zm, &QPushButton::clicked, this, &HTOrobot::TranslateMinusZ);
  connect(m_Controls.pushButton_rxp, &QPushButton::clicked, this, &HTOrobot::RotatePlusX);
  connect(m_Controls.pushButton_ryp, &QPushButton::clicked, this, &HTOrobot::RotatePlusY);
  connect(m_Controls.pushButton_rzp, &QPushButton::clicked, this, &HTOrobot::RotatePlusZ);
  connect(m_Controls.pushButton_rxm, &QPushButton::clicked, this, &HTOrobot::RotateMinusX);
  connect(m_Controls.pushButton_rym, &QPushButton::clicked, this, &HTOrobot::RotateMinusY);
  connect(m_Controls.pushButton_rzm, &QPushButton::clicked, this, &HTOrobot::RotateMinusZ);
  connect(m_Controls.pushButton_lt, &QPushButton::clicked, this, &HTOrobot::RotateMinus);
  connect(m_Controls.pushButton_rt, &QPushButton::clicked, this, &HTOrobot::RotatePlus);
  connect(m_Controls.pushButton_caculateAngle, &QPushButton::clicked, this, &HTOrobot::CaculateStrechAngle);
  connect(m_Controls.pushButton_setGizmo, &QPushButton::clicked, this, &HTOrobot::on_pushButton_addGizmo_clicked);
  //Hans Robot
  connect(m_Controls.pushButton_connectHans_2, &QPushButton::clicked, this, &HTOrobot::connectHans);
  connect(m_Controls.pushButton_powerOn, &QPushButton::clicked, this, &HTOrobot::HansPowerOn);
  connect(m_Controls.pushButton_powerOff, &QPushButton::clicked, this, &HTOrobot::HansPowerOFF);
  connect(m_Controls.pushButton_openHandGuiding, &QPushButton::clicked, this, &HTOrobot::HandGuiding);
  connect(m_Controls.pushButton_closeHandGuiding, &QPushButton::clicked, this, &HTOrobot::closeHandGuiding);
  connect(m_Controls.pushButton_suddenStop, &QPushButton::clicked, this, &HTOrobot::suddenStop);
  connect(m_Controls.pushButton_coutMatrix, &QPushButton::clicked, this, &HTOrobot::PrintToolMatrix);
  //set tcp to flange
  connect(m_Controls.pushButton_setTcpInFlange, &QPushButton::clicked, this, &HTOrobot::SetTcpToFlange);//table1
  connect(m_Controls.pushButton_setTcpToflange, &QPushButton::clicked, this, &HTOrobot::SetTcpToFlange);//table2
  connect(m_Controls.pushButton_connectAim, &QPushButton::clicked, this, &HTOrobot::connectAimooe);
  connect(m_Controls.pushButton_updateData, &QPushButton::clicked, this, &HTOrobot::upDateData);

  //Robot Registration
  connect(m_Controls.pushButton_captureRobot, &QPushButton::clicked, this, &HTOrobot::captureRobot);
  connect(m_Controls.pushButton_replaceMatrix, &QPushButton::clicked, this, &HTOrobot::ReplaceRegistration);
  connect(m_Controls.pushButton_saveMatrix, &QPushButton::clicked, this, &HTOrobot::saveRegistration);
  connect(m_Controls.pushButton_reuseMatrix, &QPushButton::clicked, this, &HTOrobot::reuseRegistration);
  connect(m_Controls.pushButton_setinitial, &QPushButton::clicked, this, &HTOrobot::setInitializationPoint);
  connect(m_Controls.pushButton_gotoinitial, &QPushButton::clicked, this, &HTOrobot::gotoInitialization);
  connect(m_Controls.pushButton_setInitialPoint, &QPushButton::clicked, this, &HTOrobot::setInitializationPoint);
  connect(m_Controls.pushButton_GotoInitialPoint, &QPushButton::clicked, this, &HTOrobot::gotoInitialization);


  //Robot move
  connect(m_Controls.pushButton_xp_2, &QPushButton::clicked, this, &HTOrobot::xp);
  connect(m_Controls.pushButton_yp_2, &QPushButton::clicked, this, &HTOrobot::yp);
  connect(m_Controls.pushButton_zp_2, &QPushButton::clicked, this, &HTOrobot::zp);
  connect(m_Controls.pushButton_xm_2, &QPushButton::clicked, this, &HTOrobot::xm);
  connect(m_Controls.pushButton_ym_2, &QPushButton::clicked, this, &HTOrobot::ym);
  connect(m_Controls.pushButton_zm_2, &QPushButton::clicked, this, &HTOrobot::zm);
  connect(m_Controls.pushButton_rxp_2, &QPushButton::clicked, this, &HTOrobot::rxp);
  connect(m_Controls.pushButton_ryp_2, &QPushButton::clicked, this, &HTOrobot::ryp);
  connect(m_Controls.pushButton_rzp_2, &QPushButton::clicked, this, &HTOrobot::rzp);
  connect(m_Controls.pushButton_rxm_2, &QPushButton::clicked, this, &HTOrobot::rxm);
  connect(m_Controls.pushButton_rym_2, &QPushButton::clicked, this, &HTOrobot::rym);
  connect(m_Controls.pushButton_rzm_2, &QPushButton::clicked, this, &HTOrobot::rzm);



  //Osteotomy guide calibration
  //导板注册按钮，写入已标定好的固定值
  connect(m_Controls.pushButton_cailbration, &QPushButton::clicked, this, &HTOrobot::OstGuidCalibration);
  //前三个点采集
  connect(m_Controls.GetProbeEndPosOneBtn, &QPushButton::clicked, this, [=]() {GetProbeEndPosBtnClicked(1); });
  //重置采集点
  connect(m_Controls.ResetProbeEndPosOneBtn, &QPushButton::clicked, this, [=]() {ResetProbeEndPosBtnClicked(1); });
  //原点获取
  connect(m_Controls.GetGuiderOriginPosBtn, &QPushButton::clicked, this, &HTOrobot::GetGuiderOriginPosBtnClicked);
  //计算tcp的同时写入tcp至导板末端
  connect(m_Controls.CalculateGuiderTCPOneBtn, &QPushButton::clicked, this, &HTOrobot::CalculateGuideTCP);
  probeEndOneVector.resize(4);
  //根据图像Tbase2tool矩阵移动机械臂
  connect(m_Controls.pushButton_MTCP, &QPushButton::clicked, this, &HTOrobot::MoveRobotTCP);
  connect(m_Controls.pushButton_navigationGuide, &QPushButton::clicked, this, &HTOrobot::startNavigationGuidePosition);//锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
  connect(m_Controls.pushButton_goTOobjPosition, &QPushButton::clicked, this, &HTOrobot::guideToObjPosition);//锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷

  //tibiaRegistration
  connect(m_Controls.pushButton_ballImage, &QPushButton::clicked, this, &HTOrobot::getImageBallcenter);
  connect(m_Controls.pushButton_tibiaRegistration, &QPushButton::clicked, this, &HTOrobot::tibiaRegistration);
  connect(m_Controls.pushButton_saveRegistration, &QPushButton::clicked, this, &HTOrobot::saveTibiaRegistration);
  connect(m_Controls.pushButton_reuseMatrix_2, &QPushButton::clicked, this, &HTOrobot::reuseMatrix);
  //ICP
  connect(m_Controls.pushButton_collectLandmark, &QPushButton::clicked, this, &HTOrobot::collectLandmark);
  connect(m_Controls.pushButton_landmarkRegistration, &QPushButton::clicked, this, &HTOrobot::landmarkRegistration);
  connect(m_Controls.pushButton_collectICP, &QPushButton::clicked, this, &HTOrobot::collectICP);
  connect(m_Controls.pushButton_ICPRegistration, &QPushButton::clicked, this, &HTOrobot::ICPRegistration);
  connect(m_Controls.pushButton_ResetImageConfiguration, &QPushButton::clicked, this, &HTOrobot::ResetImageConfiguration);
  connect(m_Controls.pushButton_saveImageMatrix, &QPushButton::clicked, this, &HTOrobot::saveImageMatrix);
  connect(m_Controls.pushButton_reuseImageMatrix, &QPushButton::clicked, this, &HTOrobot::reuseImageMatrix);

  //saw calibrator and navigation
  connect(m_Controls.pushButton_collectPoint, &QPushButton::clicked, this, &HTOrobot::getPointOnSawRF);
  connect(m_Controls.pushButton_navigationSaw, &QPushButton::clicked, this, &HTOrobot::startSawPositionCapture);
  connect(m_Controls.pushButton_probeCalibrator, &QPushButton::clicked, this, &HTOrobot::OnCaculateProbeClicked);
  //cut Tibia
  connect(m_Controls.pushButton_sawCutTibia, &QPushButton::clicked, this, &HTOrobot::navigationCutTibia);
  //navigation StrechAngle

  connect(m_Controls.pushButton_pointOnPatientRF, &QPushButton::clicked, this, &HTOrobot::getPointOnPatientRF);
  connect(m_Controls.pushButton_startStrechNavigation, &QPushButton::clicked, this, &HTOrobot::startNavigationStrech);
  connect(m_Controls.pushButton_getPlanPosition, &QPushButton::clicked, this, &HTOrobot::judgeTibiaShowPlanPosition);//锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷

  //系统精度测试
  connect(m_Controls.pushButton_setPlanePrecisionTestTcp, &QPushButton::clicked, this, &HTOrobot::SetPlanePrecisionTestTcp);
  connect(m_Controls.pushButton_confirmImageTargetPlane, &QPushButton::clicked, this, &HTOrobot::InterpretImagePlane);
  connect(m_Controls.pushButton_goToFakePlane, &QPushButton::clicked, this, &HTOrobot::On_pushButton_goToFakePlane_clicked);
  connect(m_Controls.pushButton_gotoPlaneEdge, &QPushButton::clicked, this, &HTOrobot::OnAutoPositionStart);

  //力线验证
  connect(m_Controls.pushButton_captureMalleolusPoint, &QPushButton::clicked, this, &HTOrobot::captureMalleolusPoint);
  //检查点验证
  connect(m_Controls.pushButton_proximalStaple, &QPushButton::clicked, this, &HTOrobot::collectProximalStaplePoint);
  connect(m_Controls.pushButton_distalStaple, &QPushButton::clicked, this, &HTOrobot::collectDistalStaplePoint);
  connect(m_Controls.pushButton_sawStable, &QPushButton::clicked, this, &HTOrobot::collectSawStaplePoint);
  connect(m_Controls.pushButton_verifyPromix, &QPushButton::clicked, this, &HTOrobot::testProximalPoint);
  connect(m_Controls.pushButton_verifyDistal, &QPushButton::clicked, this, &HTOrobot::testDistalPoint);
  connect(m_Controls.pushButton_verifySaw, &QPushButton::clicked, this, &HTOrobot::testSawPoint);
  //手动运动tcp
  connect(m_Controls.pushButton_movetcpline, &QPushButton::clicked, this, &HTOrobot::movetcpline);



}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void HTOrobot::InitPointSetSelector(QmitkSingleNodeSelectionWidget* widget)
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
void HTOrobot::InitSurfaceSelector(QmitkSingleNodeSelectionWidget* widget)
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
void HTOrobot::DoImageProcessing()
{
  //QList<mitk::DataNode::Pointer> nodes = this->GetDataManagerSelection();
  //if (nodes.empty())
  //  return;

  //mitk::DataNode *node = nodes.front();

  //if (!node)
  //{
  //  // Nothing selected. Inform the user and return
  //  QMessageBox::information(nullptr, "Template", "Please load and select an image before starting image processing.");
  //  return;
  //}

  //// here we have a valid mitk::DataNode

  //// a node itself is not very useful, we need its data item (the image)
  //mitk::BaseData *data = node->GetData();
  //if (data)
  //{
  //  // test if this data item is an image or not (could also be a surface or something totally different)
  //  mitk::Image *image = dynamic_cast<mitk::Image *>(data);
  //  if (image)
  //  {
  //    std::stringstream message;
  //    std::string name;
  //    message << "Performing image processing for image ";
  //    if (node->GetName(name))
  //    {
  //      // a property called "name" was found for this DataNode
  //      message << "'" << name << "'";
  //    }
  //    message << ".";
  //    MITK_INFO << message.str();

  //    // actually do something here...
  //  }
  //}
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HTOrobot::connectHans()
{
	int nRet = HRIF_Connect(boxID, hostname, nPort);
	if (HRIF_IsConnected(0))
	{
		m_Controls.textBrowser_HTO->append("Robotic arm Hans connection successfully!");
	}
	else
	{
		m_Controls.textBrowser_HTO->append("disconnect!");
	}
}

void HTOrobot::HansPowerOn()
{
	if (!HRIF_GrpEnable(boxID, rbtID))
	{
		m_Controls.textBrowser_HTO->append("power on successfully");
	}
	else
	{
		m_Controls.textBrowser_HTO->append("failed");
	}
}


void HTOrobot::HansPowerOFF()
{
	if (HRIF_GrpDisable(boxID, rbtID) == 0)
	{
		m_Controls.textBrowser_HTO->append("power off true");
	}
	else
	{
		m_Controls.textBrowser_HTO->append("failed");
	}
}

void HTOrobot::HandGuiding()
{
	HRIF_GrpOpenFreeDriver(boxID, rbtID);
	m_Controls.textBrowser_HTO->append("Open handguiding");
}

void HTOrobot::closeHandGuiding()
{
	HRIF_GrpCloseFreeDriver(boxID, rbtID);
	m_Controls.textBrowser_HTO->append("Turn OFF handguiding");
}

void HTOrobot::SetTcpToFlange()
{
	m_Controls.textBrowser_HTO->append("TCP initialization");
	dTcp_Rx = 0; dTcp_Ry = 0; dTcp_Rz = 0; dTcp_X = 0; dTcp_Y = 0; dTcp_Z = 0;
	int nRet = HRIF_SetTCP(boxID, rbtID, dTcp_X, dTcp_Y, dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz);
	if (nRet == 0)
	{
		MITK_INFO << "TCP initialization successfully";
	}
	else
	{
		MITK_INFO << "failed";
	}
}

void HTOrobot::connectAimooe()
{
	T_AIMPOS_DATAPARA mPosDataPara;
	Aim_API_Initial(aimHandle);
	Aim_SetEthernetConnectIP(aimHandle, 192, 168, 31, 10);
	rlt = Aim_ConnectDevice(aimHandle, I_ETHERNET, mPosDataPara);


	if (rlt == AIMOOE_OK)
	{

		qDebug() << "connect success";
	}
	else {

		qDebug() << "connect failed";
	}

	QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", "");
	if (filename.isNull()) return;
	filename.append("/");
	qDebug() << "The selected folder address :" << filename;
	rlt = Aim_SetToolInfoFilePath(aimHandle, filename.toLatin1().data());

	if (rlt == AIMOOE_OK)
	{

		qDebug() << "set filenemae success";
	}
	else {

		qDebug() << "set filenemae failed";
	}

	int size = 0;
	Aim_GetCountOfToolInfo(aimHandle, size);

	if (size != 0)
	{
		t_ToolBaseInfo* toolarr = new t_ToolBaseInfo[size];

		rlt = Aim_GetAllToolFilesBaseInfo(aimHandle, toolarr);

		if (rlt == AIMOOE_OK)
		{
			for (int i = 0; i < size; i++)
			{
						char* ptool = toolarr[i].name;
						QString toolInfo = QString("Tool Name：") + QString::fromLocal8Bit(ptool);
						m_Controls.textBrowser_HTO->append(toolInfo);
			}
		}
		delete[] toolarr;
	}
	else {
		std::cout << "There are no tool identification files in the current directory:";
	}
	getToolInfor();
	std::cout << "End of connection";
	rlt = AIMOOE_OK;
}

void HTOrobot::upDateData()
{
	if (m_AimoeVisualizeTimer == nullptr)
	{
		m_AimoeVisualizeTimer = new QTimer(this);
	}
	connect(m_AimoeVisualizeTimer, SIGNAL(timeout()), this, SLOT(GetMatrix()));
	m_AimoeVisualizeTimer->start(100);
	qDebug() << "连接成功！";
}

void HTOrobot::setInitializationPoint()
{
	/*m_Controls.textBrowser_HTO->append("Set Initial Point");
	int initializationPoints = HRIF_ReadActPos(boxID, rbtID, dX, dY, dZ, dRx, dRy, dRz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, dTcp_X, dTcp_Y,
		dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz, dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz);
	if (initializationPoints == 0)
	{
		std::cout << "Initial Position:X=" << dX << "Initial Position:Y=" << dY << "Initial Position:Z=" << dZ << "Initial Position:Rx=" << dRx << "Initial Position:Ry=" << dRy << "Initial Position:Rz=" << dRz << std::endl;
	}
	else
	{
		std::cerr << "Failed to read initial Position.Error code:" << initializationPoints << std::endl;
	}*/
	int result = HRIF_ReadActPos(0, 0, g_init_X, g_init_Y, g_init_Z, g_init_Rx, g_init_Ry, g_init_Rz, dJ1_init, dJ2_init, dJ3_init, dJ4_init,
		dJ5_init, dJ6_init, dTcp_X, dTcp_Y, dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz, dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz);
	PrintResult(m_Controls.textBrowser_HTO, result, "SetInitialPoint");
}

void HTOrobot::gotoInitialization()
{
	//m_Controls.textBrowser_HTO->append("Goto Initial Points");
	int result = HRIF_MoveJ(0, 0, g_init_X, g_init_Y, g_init_Z, g_init_Rx, g_init_Ry, g_init_Rz, dJ1_init, dJ2_init, dJ3_init, dJ4_init, dJ5_init, dJ6_init, sTcpName, sUcsName,
		dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
	/*int result = HRIF_WayPoint(0, 0, nMoveType, g_init_X, g_init_Y, g_init_Z, g_init_Rx, g_init_Ry, g_init_Rz,
		dJ1_init, dJ2_init, dJ3_init, dJ4_init, dJ5_init, dJ6_init, sTcpName, sUcsName, dVelocity, dAcc, dRadius,
		nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);*/
	PrintResult(m_Controls.textBrowser_HTO, result, "Goto Initial Position111");
	
}

void HTOrobot::getToolInfor()
{
	rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, I_ETHERNET, markerSt, statusSt);
	if (rlt == AIMOOE_NOT_REFLASH)
	{
		std::cout << "camera get data failed";
	}

	std::vector<int> residualIndex;
	for (int i = 0; i < markerSt.MarkerNumber; i++)
	{
		residualIndex.push_back(i);
	}

	T_AimToolDataResult* mtoolsrlt = new T_AimToolDataResult;
	mtoolsrlt->next = NULL;
	mtoolsrlt->validflag = false;
	rlt = Aim_FindToolInfo(aimHandle, markerSt, mtoolsrlt, 0);
	T_AimToolDataResult* prlt = mtoolsrlt;

	if (rlt == AIMOOE_OK)
	{
		do
		{
			if (prlt->validflag)
			{
				float Pi = 3.141592;
				std::cout << "找到工具: " << prlt->toolname << "  平均误差" << prlt->MeanError << " RMS误差" << prlt->Rms;
				std::cout << "工具原点:" << prlt->OriginCoor[0] << "," << prlt->OriginCoor[1] << "," << prlt->OriginCoor[2];
				std::cout << "工具角度:" << prlt->rotationvector[0] * 180 / PI << "," << prlt->rotationvector[1] * 180 / PI << "," << prlt->rotationvector[2] * 180 / PI;
				std::cout << "标记点坐标:";
				for (int i = 0; i < prlt->toolptidx.size(); i++)
				{
					int idx = prlt->toolptidx[i];
					std::vector<int>::iterator iter = residualIndex.begin();
					iter = find(residualIndex.begin(), residualIndex.end(), idx);
					if (iter != residualIndex.end())
					{
						residualIndex.erase(iter);
					}
					if (idx < 0)
						cout << "0 0 0";
					else
						cout << markerSt.MarkerCoordinate[idx * 3 + 0] << " " << markerSt.MarkerCoordinate[idx * 3 + 1] << " " << markerSt.MarkerCoordinate[idx * 3 + 2] << " ";
				}
			}
			T_AimToolDataResult* pnext = prlt->next;
			delete prlt;
			prlt = pnext;
		} while (prlt != NULL);
		/*cout << endl;*/
		if (residualIndex.size() > 0)
		{
			/*		cout << "in all" << residualIndex.size() << "point：";
					for (int i = 0; i < residualIndex.size(); i++)
					{
						int j = residualIndex[i];
						cout << i << ":" << markerSt.MarkerCoordinate[j * 3 + 0] << "," << markerSt.MarkerCoordinate[j * 3 + 1] << "," << markerSt.MarkerCoordinate[j * 3 + 2];
					}*/
		}
	}
	else
	{
		delete prlt;
	}
	std::cout << "查找结束";
	rlt = AIMOOE_OK;
}


bool HTOrobot::OnCaculateProbeClicked()
{
	try {
		// --------------------init----------------------------
		AimHandle aimHandle = NULL;
		E_Interface EI = I_ETHERNET;
		T_AIMPOS_DATAPARA mPosDataPara;

		E_ReturnValue rlt = Aim_API_Initial(aimHandle);
		// 连接设备
		if (EI == I_ETHERNET) {
			Aim_SetEthernetConnectIP(aimHandle, 192, 168, 31, 10);
		}
		rlt = Aim_ConnectDevice(aimHandle, EI, mPosDataPara);
		if (rlt != AIMOOE_OK) {
			std::cout << "connect error." << std::endl;
			return false;
		}

#if GET_DATA_FROM_HARDWARE 
		Aim_SetAcquireData(aimHandle, EI, DT_NONE);
#else
		Aim_SetAcquireData(aimHandle, EI, DT_INFO);
#endif
		if (rlt != AIMOOE_OK) {
			std::cout << "set data error." << std::endl;
			return false;
		}
		// ------------------------------------------------
		char* path = "E:\\AimPosDemoV2.3.3\\AimTools\\";
		rlt = Aim_SetToolInfoFilePath(aimHandle, path);
		//E_ReturnValue rlt = Aim_SetToolInfoFilePath(aimHandle, toolFolderPath);
		if (rlt != AIMOOE_OK) {
			throw std::runtime_error("ERROR: Aim_SetToolInfoFilePath Failed.");
		}

		Aim_GetToolInfoFilePath(aimHandle);


		rlt = Aim_InitToolTipPivotWithToolId(aimHandle, "HTO_Probe");
		if (rlt != AIMOOE_OK) {
			m_Controls.textBrowser_HTO->append("ERROR: Aim_InitToolTipPivotWithToolId Failed");
			return false;
		}

		T_ToolTipPivotInfo toolTipInfo;
		toolTipInfo.isPivotFinished = false;
		int cntTimes = 0;

		// 循环获取标记点和状态信息，并进行工具针尖标定
		do {
			Sleep(50);
			cntTimes++;

			T_MarkerInfo markerSt;
			T_AimPosStatusInfo statusSt;

#if GET_DATA_FROM_HARDWARE 
			rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, EI, markerSt, statusSt);
#else
			rlt = Aim_GetMarkerInfo(aimHandle, EI, markerSt);
#endif    
			if (rlt != AIMOOE_OK) {
				continue;
			}
			// process tip normolize
			rlt = Aim_ProceedToolTipPivot(aimHandle, markerSt, toolTipInfo);
			// when do, pivotRate ++, then update error
			m_Controls.textBrowser_HTO->append("Process: " + QString::number((int)(toolTipInfo.pivotRate * 100)) + "%");

		} while (toolTipInfo.isPivotFinished == false && cntTimes < 1000);

		if (toolTipInfo.isPivotFinished && toolTipInfo.pivotMeanError < 1)
		{
			rlt = Aim_SaveToolTipCalibration(aimHandle);
			if (rlt == AIMOOE_OK) {
				m_Controls.textBrowser_HTO->append("Probe init down, error(mm): " + QString::number(toolTipInfo.pivotMeanError));
				m_Controls.LineEdit_probeError->setText(QString::number(toolTipInfo.pivotMeanError));
			}
			else {
				m_Controls.textBrowser_HTO->append("ERROR: Aim_SaveToolTipCalibration Failed.");
			}
		}
		else if (toolTipInfo.isPivotFinished && toolTipInfo.pivotMeanError > 1) {
			m_Controls.LineEdit_probeError->setText("Probe init failed, error > 1 mm");
		}

		return true;
	}
	catch (const std::exception & e) {
		m_Controls.LineEdit_probeError->setText("Error: " + QString::fromStdString(e.what()));
		return false;
	}
}
void HTOrobot::GetMatrix()//Print Matrix
{
	QString position_text;
	std::vector<std::string> toolidarr;
	toolidarr.push_back("HTO_RobotBaseRF");
	toolidarr.push_back("HTO_RobotEndRF");
	toolidarr.push_back("HTO_Probe");
	toolidarr.push_back("HTO_PatientRF");
	toolidarr.push_back("HTO_MetalBallRF");
	toolidarr.push_back("HTO_SawRF");

	rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, I_ETHERNET, markerSt, statusSt);
	//qDebug() << rlt << endl;

	
	T_AimToolDataResult* mtoolsrlt = new T_AimToolDataResult;
	mtoolsrlt->next = nullptr;
	mtoolsrlt->validflag = false;

	//rlt = Aim_FindToolInfo(aimHandle, markerSt, mtoolsrlt, 0);
	rlt = Aim_FindSpecificToolInfo(aimHandle, markerSt, toolidarr, mtoolsrlt, 3);
	//qDebug()<< rlt << endl;
	T_AimToolDataResult* prlt = mtoolsrlt;

	if (rlt == AIMOOE_OK)
	{
		do
		{
			////获取HTO_RobotBaseRF
			if (strcmp(prlt->toolname, "HTO_RobotBaseRF") == 0)
			{
				//qDebug() << "prlt->toolname:" << prlt->toolname << endl;
				if (prlt->validflag)
				{
					t_tran[0] = prlt->Tto[0];
					t_tran[1] = prlt->Tto[1];
					t_tran[2] = prlt->Tto[2];
					for (int i = 0; i < 3; i++)
					{
						for (int j = 0; j < 3; j++)
						{
							R_tran[i][j] = prlt->Rto[i][j];
						}
					}
					//拼接矩阵
					vtkNew<vtkMatrix4x4> m_T_temp1;
					CombineRotationTranslation(R_tran, t_tran, m_T_temp1);
					memcpy_s(T_CamToBaseRF, sizeof(double) * 16, m_T_temp1->GetData(), sizeof(double) * 16);

				}
				else
				{

				}
			}
			//获取HTO_Probe数据
			if (strcmp(prlt->toolname, "HTO_Probe") == 0)
			{
				//qDebug() << "prlt->toolname:"<< prlt->toolname << endl;
				if (prlt->validflag)
				{
					t_tran[0] = prlt->Tto[0];
					t_tran[1] = prlt->Tto[1];
					t_tran[2] = prlt->Tto[2];
					for (int i = 0; i < 3; i++)
					{
						for (int j = 0; j < 3; j++)
						{
							R_tran[i][j] = prlt->Rto[i][j];
							//qDebug() << "R_tran[i][j]:" << R_tran[i][j] << endl;
						}
					}
					//拼接矩阵
					vtkNew<vtkMatrix4x4> m_T_temp2;
					CombineRotationTranslation(R_tran, t_tran, m_T_temp2);
					memcpy_s(T_CamToProbe, sizeof(double) * 16, m_T_temp2->GetData(), sizeof(double) * 16);
					Cam_ProbeTop[0] = prlt->tooltip[0];
					Cam_ProbeTop[1] = prlt->tooltip[1];
					Cam_ProbeTop[2] = prlt->tooltip[2];

					ProbeTop[0] = prlt->tooltip[0];
					ProbeTop[1] = prlt->tooltip[1];
					ProbeTop[2] = prlt->tooltip[2];
				
					
				}
				else
				{

				}
			}
			//获取HTO_RobotEndRF数据
			if (strcmp(prlt->toolname, "HTO_RobotEndRF") == 0)
			{
				//qDebug() << "prlt->toolname:" << prlt->toolname << endl;
				if (prlt->validflag)
				{
					t_tran[0] = prlt->Tto[0];
					t_tran[1] = prlt->Tto[1];
					t_tran[2] = prlt->Tto[2];
					for (int i = 0; i < 3; i++)
					{
						for (int j = 0; j < 3; j++)
						{
							R_tran[i][j] = prlt->Rto[i][j];
						}
					}
					//拼接矩阵
					vtkNew<vtkMatrix4x4> m_T_temp3;
					CombineRotationTranslation(R_tran, t_tran, m_T_temp3);
					memcpy_s(T_CamToEndRF, sizeof(double) * 16, m_T_temp3->GetData(), sizeof(double) * 16);

				}
				else
				{

				}
			}
			//获取HTO_PatientRF数据
			if (strcmp(prlt->toolname, "HTO_PatientRF") == 0)
			{
				
				if (prlt->validflag)
				{
					t_tran[0] = prlt->Tto[0];
					t_tran[1] = prlt->Tto[1];
					t_tran[2] = prlt->Tto[2];
					for (int i = 0; i < 3; i++)
					{
						for (int j = 0; j < 3; j++)
						{
							R_tran[i][j] = prlt->Rto[i][j];
						}
					}
					//拼接矩阵
					vtkNew<vtkMatrix4x4> m_T_temp4;
					CombineRotationTranslation(R_tran, t_tran, m_T_temp4);
					memcpy_s(T_CamToPatientRF, sizeof(double) * 16, m_T_temp4->GetData(), sizeof(double) * 16);

				}
				else
				{

				}
			}
			//获取HTO_MetalBallRF数据
			if (strcmp(prlt->toolname, "HTO_MetalBallRF") == 0)
			{
				
				if (prlt->validflag)
				{
					t_tran[0] = prlt->Tto[0];
					t_tran[1] = prlt->Tto[1];
					t_tran[2] = prlt->Tto[2];
					for (int i = 0; i < 3; i++)
					{
						for (int j = 0; j < 3; j++)
						{
							R_tran[i][j] = prlt->Rto[i][j];
						}
					}
					//拼接矩阵
					vtkNew<vtkMatrix4x4> m_T_temp5;
					CombineRotationTranslation(R_tran, t_tran, m_T_temp5);
					memcpy_s(T_CamToMetalBallRF, sizeof(double) * 16, m_T_temp5->GetData(), sizeof(double) * 16);

				}
				else
				{

				}
			}
				//获取HTO_SawRF数据
			if (strcmp(prlt->toolname, "HTO_SawRF") == 0)
			{
				//qDebug() << "prlt->toolname:"<< prlt->toolname << endl;
				if (prlt->validflag)
				{
					t_tran[0] = prlt->Tto[0];
					t_tran[1] = prlt->Tto[1];
					t_tran[2] = prlt->Tto[2];
					for (int i = 0; i < 3; i++)
					{
						for (int j = 0; j < 3; j++)
						{
							R_tran[i][j] = prlt->Rto[i][j];
							//qDebug() << "R_tran[i][j]:" << R_tran[i][j] << endl;
						}
					}
					//拼接矩阵
					vtkNew<vtkMatrix4x4> m_T_temp6;
					CombineRotationTranslation(R_tran, t_tran, m_T_temp6);
					memcpy_s(T_CamToSaw, sizeof(double) * 16, m_T_temp6->GetData(), sizeof(double) * 16);

				}
				else
				{

				}
			}
		
			T_AimToolDataResult* pnext = prlt->next;
			delete prlt;
			prlt = pnext;
		} while (prlt != NULL);
	}
	else
	{
		delete prlt;
	}
}

void HTOrobot::getBaseToFlangeMatrix()
{

	//Read TCP get T_BaseToFlange
	double dX = 0; double dY = 0; double dZ = 0;
	double dRx = 0; double dRy = 0; double dRz = 0;
	int nRet = HRIF_ReadActTcpPos(boxID, rbtID, dX, dY, dZ, dRx, dRy, dRz);

	if (nRet == 0)
	{
		//std::cout << "get BaseToFlange Matrix" << std::endl;
		m_Controls.textBrowser_HTO->append("get BaseTotool Matrix");

		std::cout << "Current TCP Position and Orientation:" << std::endl;
		std::cout << "Position - X: " << dX << ", Y: " << dY << ", Z: " << dZ << std::endl;
		std::cout << "Orientation - Rx: " << dRx << ", Ry: " << dRy << ", Rz: " << dRz << std::endl;

		auto  tmpTrans = vtkTransform::New();
		tmpTrans->PostMultiply();
		tmpTrans->RotateX(dRx);
		tmpTrans->RotateY(dRy);
		tmpTrans->RotateZ(dRz);
		
		tmpTrans->Translate(dX, dY, dZ);
		tmpTrans->Update();
		//VTKT_BaseTotool
	   this->VTKT_BaseTotool_read = tmpTrans->GetMatrix();
		QVector<double> _vtkMatrix4x4;
		_vtkMatrix4x4 = { VTKT_BaseTotool_read->GetElement(0,0), VTKT_BaseTotool_read->GetElement(0, 1), VTKT_BaseTotool_read->GetElement(0, 2), VTKT_BaseTotool_read->GetElement(0,3),
						  VTKT_BaseTotool_read->GetElement(1, 0),VTKT_BaseTotool_read->GetElement(1, 1), VTKT_BaseTotool_read->GetElement(1, 2), VTKT_BaseTotool_read->GetElement(1,3),
						  VTKT_BaseTotool_read->GetElement(2, 0), VTKT_BaseTotool_read->GetElement(2, 1), VTKT_BaseTotool_read->GetElement(2, 2), VTKT_BaseTotool_read->GetElement(2,3),
						  VTKT_BaseTotool_read->GetElement(3, 0), VTKT_BaseTotool_read->GetElement(3, 1), VTKT_BaseTotool_read->GetElement(3, 2), VTKT_BaseTotool_read->GetElement(3,3)
		};
		std::cout << "VTKT_BaseTotool_read Matrix:" << std::endl << std::endl;
		for (int i = 0; i < 4; ++i)
		{
			for (int j = 0; j < 4; ++j)
			{
				std::cout << VTKT_BaseTotool_read->GetElement(i, j) << " ";
			}
			std::cout << std::endl;
		}
	}
	else {
		//qDebug() << "Failed to read TCP position and orientation.";
		m_Controls.textBrowser_HTO->append("Failed to read TCP position and orientation.");
	}

}

void HTOrobot::CombineRotationTranslation(float Rt[3][3], float Tto[3], vtkMatrix4x4* resultMatrix)
{
	// Set rotation part
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			resultMatrix->SetElement(i, j, Rt[i][j]);
		}
	}
	for (int i = 0; i < 3; ++i)
	{
		resultMatrix->SetElement(i, 3, Tto[i]);
	}
}

void HTOrobot::suddenStop()
{
	HRIF_GrpStop(boxID, rbtID);
	MITK_INFO << "HRIF_GrpStop(boxID, rbtID);" << HRIF_GrpStop(boxID, rbtID);;
}
//
//void HTOrobot::captureRobot()
//{
//	m_Controls.textBrowser_HTO->append("captureRobot");
//	if (hto_IndexOfRobotCapture < 5)
//	{
//		hto_IndexOfRobotCapture++;
//		m_Controls.lineEdit_HansCapture->setText(QString::number(hto_IndexOfRobotCapture));
//		std::cout << "hto_IndexOfRobotCapture:" << hto_IndexOfRobotCapture << std::endl;
//		HansCapturePose(true);
//	}
//	else if (hto_IndexOfRobotCapture < 10)
//	{
//		hto_IndexOfRobotCapture++;
//		m_Controls.lineEdit_HansCapture->setText(QString::number(hto_IndexOfRobotCapture));
//		std::cout << "hto_IndexOfRobotCapture:" << hto_IndexOfRobotCapture << std::endl;
//		HansCapturePose(false);
//	}
//	else
//	{
//		vtkNew<vtkMatrix4x4> robotEndToFlangeMatrix;
//		hto_RobotRegistration.GetTCPmatrix(robotEndToFlangeMatrix);
//
//		vtkMatrix4x4* matrix4x4 = vtkMatrix4x4::New();
//		hto_RobotRegistration.GetRegistraionMatrix(matrix4x4);
//
//		double x = robotEndToFlangeMatrix->GetElement(0, 3);
//		double y = robotEndToFlangeMatrix->GetElement(1, 3);
//		double z = robotEndToFlangeMatrix->GetElement(2, 3);
//
//		std::cout << "X: " << x << std::endl;
//		std::cout << "Y: " << y << std::endl;
//		std::cout << "Z: " << z << std::endl;
//
//		robotEndToFlangeMatrix->Invert();
//		m_Controls.textBrowser_HTO->append("Registration RMS:" + QString::number(hto_RobotRegistration.RMS()));
//		std::cout << "Registration RMS:" << hto_RobotRegistration.RMS() << std::endl;
//
//		vtkMatrix4x4* VtkT_BaseToBaseRF = vtkMatrix4x4::New();
//		hto_RobotRegistration.GetRegistraionMatrix(VtkT_BaseToBaseRF);
//		VtkT_BaseToBaseRF->Invert();
//		memcpy_s(T_BaseToBaseRF, sizeof(double) * 16, VtkT_BaseToBaseRF->GetData(), sizeof(double) * 16);
//
//		vtkMatrix4x4* vtkT_FlangeToEndRF = vtkMatrix4x4::New();
//		hto_RobotRegistration.GetTCPmatrix(vtkT_FlangeToEndRF);
//		memcpy_s(T_FlangeToEndRF, sizeof(double) * 16, vtkT_FlangeToEndRF->GetData(), sizeof(double) * 16);
//
//	}
//
//}
//
//void HTOrobot::HansCapturePose(bool translationOnly)
//{
//	double dX = 0; double dY = 0; double dZ = 0;
//	double dRx = 0; double dRy = 0; double dRz = 0;
//	int nRet = HRIF_ReadActTcpPos(boxID, rbtID, dX, dY, dZ, dRx, dRy, dRz);
//
//	auto tempTrans = vtkTransform::New();
//	tempTrans->PostMultiply();
//	tempTrans->RotateX(dRx);
//	tempTrans->RotateY(dRy);
//	tempTrans->RotateZ(dRz);
//	tempTrans->Translate(dX, dY, dZ);
//	tempTrans->Update();
//
//	vtkSmartPointer<vtkMatrix4x4> VTK_BaseToFlange = tempTrans->GetMatrix();//从机械臂系统中获取BaseToFlange
//	QVector<double>  z_vtkMatrix4x4;
//	z_vtkMatrix4x4 = { VTK_BaseToFlange->GetElement(0,0), VTK_BaseToFlange->GetElement(0, 1), VTK_BaseToFlange->GetElement(0, 2), VTK_BaseToFlange->GetElement(0,3),
//				  VTK_BaseToFlange->GetElement(1, 0),VTK_BaseToFlange->GetElement(1, 1), VTK_BaseToFlange->GetElement(1, 2), VTK_BaseToFlange->GetElement(1,3),
//				  VTK_BaseToFlange->GetElement(2, 0), VTK_BaseToFlange->GetElement(2, 1), VTK_BaseToFlange->GetElement(2, 2), VTK_BaseToFlange->GetElement(2,3),
//				  VTK_BaseToFlange->GetElement(3, 0), VTK_BaseToFlange->GetElement(3, 1), VTK_BaseToFlange->GetElement(3, 2), VTK_BaseToFlange->GetElement(3,3) };
//
//
//	auto VtkT_CameraToEndRF = vtkMatrix4x4::New();
//	VtkT_CameraToEndRF->DeepCopy(T_CamToEndRF);//从相机系统中获取CameraToEndRF
//
//	auto VtkT_BaseRFToCamera = vtkMatrix4x4::New();
//	VtkT_BaseRFToCamera->DeepCopy(T_CamToBaseRF);//从相机系统中获取BaseRFToCamera
//	VtkT_BaseRFToCamera->Invert();//将其进行转置,获得想要的CameraToBaseRF
//
//
//	vtkNew<vtkTransform> tmpTransform;
//	tmpTransform->PostMultiply();
//	tmpTransform->Identity();
//	tmpTransform->SetMatrix(VtkT_CameraToEndRF);
//	tmpTransform->Concatenate(VtkT_BaseRFToCamera);
//	tmpTransform->Update();
//	auto vtkBaseRFtoRobotEndRFMatrix = tmpTransform->GetMatrix();
//
//	hto_RobotRegistration.AddPoseWithVtkMatrix(VTK_BaseToFlange, vtkBaseRFtoRobotEndRFMatrix, translationOnly);
//}


void HTOrobot::captureRobot()
{
	m_Controls.textBrowser_HTO->append("captureRobot");
	if (hto_IndexOfRobotCapture < 5) //The first five translations, 
	{
		hto_IndexOfRobotCapture++;
		std::cout << "m_IndexOfRobotCapture: " << hto_IndexOfRobotCapture << std::endl;
		m_Controls.lineEdit_HansCapture->setText(QString::number(hto_IndexOfRobotCapture));
		CapturePose(true);


	}
	else if (hto_IndexOfRobotCapture < 10) //the last five rotations
	{


		hto_IndexOfRobotCapture++;
		m_Controls.lineEdit_HansCapture ->setText(QString::number(hto_IndexOfRobotCapture));
		std::cout << "m_IndexOfRobotCapture: " << hto_IndexOfRobotCapture << std::endl;

		CapturePose(false);
	}
	else
	{
		//MITK_INFO << "OnRobotCapture finish: " << m_IndexOfRobotCapture;
		vtkNew<vtkMatrix4x4> robotEndToFlangeMatrix;
		hto_RobotRegistration.GetTCPmatrix(robotEndToFlangeMatrix);

		vtkMatrix4x4* matrix4x4 = vtkMatrix4x4::New();
		hto_RobotRegistration.GetRegistraionMatrix(matrix4x4);


		double x = robotEndToFlangeMatrix->GetElement(0, 3);
		double y = robotEndToFlangeMatrix->GetElement(1, 3);
		double z = robotEndToFlangeMatrix->GetElement(2, 3);
		std::cout << "X: " << x << std::endl;
		std::cout << "Y: " << y << std::endl;
		std::cout << "Z: " << z << std::endl;

		robotEndToFlangeMatrix->Invert();

		m_Controls.textBrowser_HTO->append("Registration RMS: " + QString::number(hto_RobotRegistration.RMS()));
		std::cout << "Registration RMS: " << hto_RobotRegistration.RMS() << std::endl;


		vtkMatrix4x4* vtkT_BaseToBaseRF = vtkMatrix4x4::New();
		hto_RobotRegistration.GetRegistraionMatrix(vtkT_BaseToBaseRF);
		vtkT_BaseToBaseRF->Invert();
		memcpy_s(T_BaseToBaseRF, sizeof(double) * 16, vtkT_BaseToBaseRF->GetData(), sizeof(double) * 16);


		vtkMatrix4x4* vtkT_FlangeToEndRF = vtkMatrix4x4::New();
		hto_RobotRegistration.GetTCPmatrix(vtkT_FlangeToEndRF);
		memcpy_s(T_FlangeToEndRF, sizeof(double) * 16, vtkT_FlangeToEndRF->GetData(), sizeof(double) * 16);

		//打印T_BaseToBaseRF
		/*std::cout << "----------------------------------------" << std::endl;
		std::cout << "T_BaseRFToBase:" << std::endl;
		vtkMatrix4x4* vtkT_BaseRFToBase = vtkMatrix4x4::New();
		m_RobotRegistration.GetRegistraionMatrix(vtkT_BaseRFToBase);
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				std::cout << std::fixed << std::setprecision(6) << vtkT_BaseRFToBase->GetElement(i, j) << " ";
			}
			std::cout << std::endl;*/
		//}

	}

}

void HTOrobot::CapturePose(bool translationOnly)
{


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


	vtkSmartPointer<vtkMatrix4x4> VTKT_BaseToFlanger = tmpTrans->GetMatrix();
	QVector<double> _vtkMatrix4x4;
	_vtkMatrix4x4 = { VTKT_BaseToFlanger->GetElement(0,0), VTKT_BaseToFlanger->GetElement(0, 1), VTKT_BaseToFlanger->GetElement(0, 2), VTKT_BaseToFlanger->GetElement(0,3),
					  VTKT_BaseToFlanger->GetElement(1, 0),VTKT_BaseToFlanger->GetElement(1, 1), VTKT_BaseToFlanger->GetElement(1, 2), VTKT_BaseToFlanger->GetElement(1,3),
					  VTKT_BaseToFlanger->GetElement(2, 0), VTKT_BaseToFlanger->GetElement(2, 1), VTKT_BaseToFlanger->GetElement(2, 2), VTKT_BaseToFlanger->GetElement(2,3),
					  VTKT_BaseToFlanger->GetElement(3, 0), VTKT_BaseToFlanger->GetElement(3, 1), VTKT_BaseToFlanger->GetElement(3, 2), VTKT_BaseToFlanger->GetElement(3,3)
	};

	//std::cout << "VTKT_BaseToFlanger Matrix Contents:" << std::endl;
	//for (int i = 0; i < 4; i++) {
	//	for (int j = 0; j < 4; j++) {
	//		std::cout << _vtkMatrix4x4[i * 4 + j] << "\t";
	//	}
	//	std::cout << std::endl;
	//}

	auto VTKT_CameratoEndRF = vtkMatrix4x4::New();
	VTKT_CameratoEndRF->DeepCopy(T_CamToEndRF);


	auto VTKT_BaseRFToCamera = vtkMatrix4x4::New();
	VTKT_BaseRFToCamera->DeepCopy(T_CamToBaseRF);
	VTKT_BaseRFToCamera->Invert();


	vtkNew<vtkTransform> tmpTransform;
	tmpTransform->PostMultiply();
	tmpTransform->Identity();
	tmpTransform->SetMatrix(VTKT_CameratoEndRF);
	tmpTransform->Concatenate(VTKT_BaseRFToCamera);
	tmpTransform->Update();
	auto vtkBaseRFtoRoboEndRFMatrix = tmpTransform->GetMatrix();


	//vtkMatrix4x4* matrix = tmpTransform->GetMatrix();
	//std::cout << "vtkBaseRFtoRoboEndRFMatrix:" << std::endl;
	//for (int i = 0; i < 4; i++) {
	//	for (int j = 0; j < 4; j++) {
	//		std::cout << matrix->GetElement(i, j) << "\t";
	//	}
	//	std::cout << std::endl;
	//}

	//Robotic arm registration
	hto_RobotRegistration.AddPoseWithVtkMatrix(VTKT_BaseToFlanger, vtkBaseRFtoRoboEndRFMatrix, translationOnly);



}

void HTOrobot::ReplaceRegistration()
{
	m_Controls.textBrowser_HTO->append("replace Registration");
	hto_RobotRegistration.RemoveAllPose();
	hto_IndexOfRobotCapture = 0;
	m_Controls.lineEdit_HansCapture->setText(QString::number(0));
}

void HTOrobot::saveRegistration()
{
	std::ofstream robotMatrixFile(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\T_BaseToBaseRF.txt");
	for (int i = 0; i < 16; i++) {
		robotMatrixFile << T_BaseToBaseRF[i];
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
		robotMatrixFile1 << T_FlangeToEndRF[i];
		if (i != 15) {
			robotMatrixFile1 << ",";
		}
		else {
			robotMatrixFile1 << ";";
		}
	}
	robotMatrixFile1 << std::endl;
	robotMatrixFile1.close();
	m_Controls.textBrowser_HTO->append("saveArmMatrix");
}

void HTOrobot::reuseRegistration()
{
	std::ifstream inputFile(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\T_BaseToBaseRF.txt");
	if (inputFile.is_open()) {
		std::string line;
		if (std::getline(inputFile, line)) {
			std::stringstream ss(line);
			std::string token;
			int index = 0;
			while (std::getline(ss, token, ',')) {
				T_BaseToBaseRF[index] = std::stod(token);
				index++;
			}
		}
		inputFile.close();
	}
	else {
		m_Controls.textBrowser_HTO->append("无法打开文件:T_BaseToBaseRF.txt");
	}
	QString output;
	for (int i = 0; i < 16; i++) {
		output += "T_BaseToBaseRF[" + QString::number(i) + "]: " + QString::number(T_BaseToBaseRF[i]) + " ";
	}
	m_Controls.textBrowser_HTO->append(output);
	std::ifstream inputFile2(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\T_FlangeToEndRF.txt");
	if (inputFile2.is_open()) {
		std::string line2;
		if (std::getline(inputFile2, line2)) {
			std::stringstream ss2(line2);
			std::string token2;
			int index2 = 0;
			while (std::getline(ss2, token2, ',')) {
				T_FlangeToEndRF[index2] = std::stod(token2);
				index2++;
			}
		}
		inputFile2.close();
	}
	else {
		m_Controls.textBrowser_HTO->append("无法打开文件：T_FlangeToEndRF.txt");
	}
	QString output1;
	for (int i = 0; i < 16; i++) {
		output1 += "T_FlangeToEdnRF[" + QString::number(i) + "]: " + QString::number(T_FlangeToEndRF[i]) + " ";
	}
	m_Controls.textBrowser_HTO->append(output1);
}


void HTOrobot::xp()
{
	nAxisID = 0;
	nDirection = 1;
	nToolMotion = 1;
	m_Controls.textBrowser_HTO->append("xp");
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_2->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser_HTO->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser_HTO->append(QString::number(nMoveRelL));
	m_Controls.textBrowser_HTO->append("xp finish");
}

void HTOrobot::yp()
{
	nAxisID = 1;
	nDirection = 1;
	nToolMotion = 1;
	m_Controls.textBrowser_HTO->append("yp");
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_2->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser_HTO->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser_HTO->append("yp finish");
}

void HTOrobot::zp()
{
	nAxisID = 2;
	nDirection = 1;
	nToolMotion = 1;
	m_Controls.textBrowser_HTO->append("zp");
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_2->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser_HTO->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser_HTO->append("zp finish");
}

void HTOrobot::xm()
{
	nAxisID = 0;
	nDirection = 0;
	nToolMotion = 1;
	m_Controls.textBrowser_HTO->append("xm");
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_2->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser_HTO->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser_HTO->append("xm finish");
}

void HTOrobot::ym()
{
	nAxisID = 1;
	nDirection = 0;
	nToolMotion = 1;
	m_Controls.textBrowser_HTO->append("ym");
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_2->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser_HTO->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser_HTO->append("ym finish");
}

void HTOrobot::zm()//HRIF_MoveRelL
{
	nAxisID = 2;
	nDirection = 0;
	nToolMotion = 1;
	m_Controls.textBrowser_HTO->append("zm");
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_2->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser_HTO->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser_HTO->append("zm finish");
}

void HTOrobot::rxp()
{
	nAxisID = 3;
	nDirection = 1;
	nToolMotion = 1;
	m_Controls.textBrowser_HTO->append("rxp");
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_5->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser_HTO->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser_HTO->append("rxp finish");
}

void HTOrobot::ryp()
{
	nAxisID = 4;
	nDirection = 1;
	nToolMotion = 1;
	m_Controls.textBrowser_HTO->append("ryp");
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_5->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser_HTO->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser_HTO->append("ryp finish");
}

void HTOrobot::rzp()
{
	nAxisID = 5;
	nDirection = 1;
	nToolMotion = 1;
	m_Controls.textBrowser_HTO->append("rzp");
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_5->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser_HTO->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser_HTO->append("rzp finish");
}

void HTOrobot::rxm()
{
	nAxisID = 3;
	nDirection = 0;
	nToolMotion = 1;
	m_Controls.textBrowser_HTO->append("rxm");
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_5->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser_HTO->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser_HTO->append("rxm finish");
}

void HTOrobot::rym()
{
	nAxisID = 4;
	nDirection = 0;
	nToolMotion = 1;
	m_Controls.textBrowser_HTO->append("rym");
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_5->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser_HTO->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser_HTO->append("rym finish");
}

void HTOrobot::rzm()
{
	nAxisID = 5;
	nDirection = 0;
	nToolMotion = 1;
	m_Controls.textBrowser_HTO->append("rzm");
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_5->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser_HTO->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser_HTO->append("rzm finish");
}

void HTOrobot::CalculateGuideTCP()
{
	auto Tflange2camera = CalculateTFlange2Camera(T_FlangeToEndRF, T_CamToEndRF);
	std::vector<Eigen::Vector3d> posInGuide;
	for (int i = 0; i < 4; ++i)
	{
		Eigen::Vector3d point = CalculatePointInFlangePos(Tflange2camera, probeEndOneVector[i]);
		std::string str = "Point" + std::to_string(i);
		CoutTextBrowerArray("CalculateGuideTCP" + i, point.data(), 3);
		//PrintDataHelper::AppendTextBrowserArray(m_Controls.textBrowser_HTO,  point,str.c_str());

		posInGuide.push_back(point);
	}

	Eigen::Vector3d p1 = posInGuide[0];
	Eigen::Vector3d p2 = posInGuide[1];
	Eigen::Vector3d p3 = posInGuide[2];
	Eigen::Vector3d p4 = posInGuide[3];


	/*Eigen::Vector3d RX = Eigen::Vector3d(p1[0] - p3[0], p1[1] - p3[1], p1[2] - p3[2]);
	RX.normalize();

	auto projectPoint = CalculatePointProjectInLine(p2, p1, p3);

	Eigen::Vector3d RY = Eigen::Vector3d(projectPoint[0] - p2[0], projectPoint[1] - p2[1], projectPoint[2] - p2[2]);
	RY.normalize();
	Eigen::Vector3d RZ = RX.cross(RY);
	RZ.normalize();*/
	Eigen::Vector3d Rx = p2 - p3;
	Rx.normalize();
	Eigen::Vector3d tmp = p1 - p2;
	Eigen::Vector3d Ry = tmp.cross(Rx);
	Ry.normalize();
	Eigen::Vector3d Rz = Rx.cross(Ry);
	Rz.normalize();

	Eigen::Matrix3d Re;

	Re << Rx[0], Ry[0], Rz[0],
		Rx[1], Ry[1], Rz[1],
		Rx[2], Ry[2], Rz[2];

	Eigen::Vector3d eulerAngle = Re.eulerAngles(2, 1, 0);
	std::cout << "eulerAngle" << std::endl;
	//------------------------------------------------
	/*double tcp[6];*/
	tcp[0] = p4[0];
	tcp[1] = p4[1];
	tcp[2] = p4[2];
	tcp[3] = vtkMath::DegreesFromRadians(eulerAngle(2));
	tcp[4] = vtkMath::DegreesFromRadians(eulerAngle(1));
	tcp[5] = vtkMath::DegreesFromRadians(eulerAngle(0));

	m_Controls.textBrowser_HTO->append("tcp rx ry rz is:");
	m_Controls.textBrowser_HTO->append(QString::number(tcp[3]) + " /" + QString::number(tcp[4]) + " /" + QString::number(tcp[5]));
	m_Controls.textBrowser_HTO->append("tcp x y z is:");
	m_Controls.textBrowser_HTO->append(QString::number(tcp[0]) + " /" + QString::number(tcp[1]) + "/ " + QString::number(tcp[2]));
	std::cout << "HRIF_SetTCP" << std::endl;
	int nRet = HRIF_SetTCP(boxID, rbtID, tcp[0], tcp[1], tcp[2], tcp[3], tcp[4], tcp[5]);
	if (nRet != 0)
	{
		m_Controls.textBrowser_HTO->append("set tcp failed , Please find The Reason!");
	}

	Matrix4d transformMatrix = Matrix4d::Identity();



	transformMatrix.block<3, 3>(0, 0) = Re;

	transformMatrix(0, 3) = tcp[0];
	transformMatrix(1, 3) = tcp[1];
	transformMatrix(2, 3) = tcp[2];

	transformMatrix(3, 0) = 0;
	transformMatrix(3, 1) = 0;
	transformMatrix(3, 2) = 0;
	transformMatrix(3, 3) = 1;
	TcpMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	for (int row = 0; row < 4; ++row) {
		for (int col = 0; col < 4; ++col) {
			this->TcpMatrix->SetElement(row, col, transformMatrix(row, col));
		}
	}
}
void HTOrobot::StartDisplayTCPAxesActor()
{
	/*if (!m_RobotTCPAxesTimer)
	{
		m_RobotTCPAxesTimer = new QTimer();
		endMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	}
	if (!m_IsDisplayRobotTCP)
	{
		m_TCPAxesActor = vtkSmartPointer<vtkAxesActor>::New();
		m_TCPAxesActor->SetTotalLength(30, 30, 30);
		m_TCPAxesActor->SetAxisLabels(false);
		m_TCPAxesActor->Modified();
		QmitkRenderWindow* mitkRenderWindow = this->GetRenderWindowPart()->GetQmitkRenderWindow("3d");
		vtkRenderWindow* renderWindow = mitkRenderWindow->GetVtkRenderWindow();
		vtkSmartPointer<vtkRenderer> renderer;
		renderer = renderWindow->GetRenderers()->GetFirstRenderer();
		renderer->AddActor(m_TCPAxesActor);

		connect(m_RobotTCPAxesTimer, SIGNAL(timeout()), this, SLOT(UpdateDisplayRobotTCP()));
		m_RobotTCPAxesTimer->setInterval(150);
		m_IsDisplayRobotTCP = !m_IsDisplayRobotTCP;
	}
	else
	{
		disconnect(m_RobotTCPAxesTimer, SIGNAL(timeout()), this, SLOT(UpdateDisplayRobotTCP()));
		QmitkRenderWindow* mitkRenderWindow = this->GetRenderWindowPart()->GetQmitkRenderWindow("3d");

		vtkRenderWindow* renderWindow = mitkRenderWindow->GetVtkRenderWindow();

		vtkSmartPointer<vtkRenderer> renderer;

		renderer = renderWindow->GetRenderers()->GetFirstRenderer();
		renderer->RemoveActor(m_TCPAxesActor);
		m_IsDisplayRobotTCP = !m_IsDisplayRobotTCP;
	}*/
}

void HTOrobot::OstGuidCalibration()
{
	m_Controls.textBrowser_HTO->append("TCP Osteotomy guide calibration");

	//R面数值 
	dTcp_X = 104.522;
	dTcp_Y = 45.4198;
	dTcp_Z = 105.891;
	dTcp_Rx = 5.61485;
	dTcp_Ry = 0.125467;
	dTcp_Rz = 19.338;
	int nOstGuidCal = HRIF_SetTCP(boxID, rbtID, dTcp_X, dTcp_Y, dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz);
	if (nOstGuidCal == 0) {

		m_Controls.textBrowser_HTO->append("OstGuidCalibration  TCP Set succeed");
	}
	else {

		m_Controls.textBrowser_HTO->append("OstGuidCalibration TCP Set failed");
	}

	double Rx = dTcp_Rx * M_PI / 180.0;
	double Ry = dTcp_Ry * M_PI / 180.0;
	double Rz = dTcp_Rz * M_PI / 180.0;

	Matrix4d transformMatrix = Matrix4d::Identity();

	Matrix3d rotationMatrix;
	rotationMatrix(0, 0) = cos(Ry) * cos(Rz);
	rotationMatrix(0, 1) = -cos(Ry) * sin(Rz);
	rotationMatrix(0, 2) = sin(Ry);
	rotationMatrix(1, 0) = sin(Rx) * sin(Ry) * cos(Rz) + cos(Rx) * sin(Rz);
	rotationMatrix(1, 1) = -sin(Rx) * sin(Ry) * sin(Rz) + cos(Rx) * cos(Rz);
	rotationMatrix(1, 2) = -sin(Rx) * cos(Ry);
	rotationMatrix(2, 0) = -cos(Rx) * sin(Ry) * cos(Rz) + sin(Rx) * sin(Rz);
	rotationMatrix(2, 1) = cos(Rx) * sin(Ry) * sin(Rz) + sin(Rx) * cos(Rz);
	rotationMatrix(2, 2) = cos(Rx) * cos(Ry);


	transformMatrix.block<3, 3>(0, 0) = rotationMatrix;

	transformMatrix(0, 3) = dTcp_X;
	transformMatrix(1, 3) = dTcp_Y;
	transformMatrix(2, 3) = dTcp_Z;

	transformMatrix(3, 0) = 0;
	transformMatrix(3, 1) = 0;
	transformMatrix(3, 2) = 0;
	transformMatrix(3, 3) = 1;
	vtkSmartPointer<vtkMatrix4x4> TcpMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	for (int row = 0; row < 4; ++row) {
		for (int col = 0; col < 4; ++col) {
			TcpMatrix->SetElement(row, col, transformMatrix(row, col));
		}
	}
	std::cout << "Converted TCP Matrix:" << std::endl;
	for (int row = 0; row < 4; ++row) {
		for (int col = 0; col < 4; ++col) {
			std::cout << TcpMatrix->GetElement(row, col) << " ";
		}
		std::cout << std::endl;
	}
}

void HTOrobot::GetProbeEndPosBtnClicked(int type)
{

	std::cout << "111" << std::endl;
	// ReadSaveFileData();
	int& currentCount = (type == 1) ? ProbEndCountOne : ProbEndCountTwo;
	int maxCount = (type == 1) ? 3 : 2;
	QLineEdit* countLineEdit = (type == 1) ? m_Controls.ProbeEndOneCountLineEdit : m_Controls.ProbeEndTwoCountLineEdit;
	std::vector<Eigen::Vector3d>& probeEndVector = (type == 1) ? probeEndOneVector : probeEndTwoVector;



	if (currentCount == maxCount)
	{
		QMessageBox msgBox;
		msgBox.setText(QString::fromLocal8Bit("waring"));
		msgBox.setInformativeText(QString::fromLocal8Bit("The collection quantity has reached the upper limit, please reset"));
		msgBox.setStandardButtons(QMessageBox::Ok);
		msgBox.setDefaultButton(QMessageBox::Ok);
		msgBox.exec();
		return;
	}

	currentCount++;
	countLineEdit->setText(QString::number(currentCount));

	//auto newData = GetNewToolData();
	
	//UpdateCameraToToolMatrix(newData, "Spine_Probe", T_CamToProbe, nullptr);

	//Eigen::Vector3d v(newData->tooltip[0], newData->tooltip[1], newData->tooltip[2]);
	std::string str = "Probe End Pos " + std::to_string(currentCount) + ": ";
	AppendTextBrowerArray(str.c_str(), ProbeTop, 3);

	probeEndVector[currentCount - 1] = Eigen::Vector3d(ProbeTop);
}

void HTOrobot::ResetProbeEndPosBtnClicked(int type)
{
	if (type == 1)
	{
		for (int i = 0; i < probeEndOneVector.size(); ++i)
		{
			this->probeEndOneVector[i].setZero();
		}
		ProbEndCountOne = 0;
		m_Controls.ProbeEndOneCountLineEdit->setText(QString::number(ProbEndCountOne));
		return;
	}
	for (int i = 0; i < probeEndTwoVector.size(); ++i)
	{
		this->probeEndTwoVector[i].setZero();
	}
	ProbEndCountTwo = 0;
	m_Controls.ProbeEndTwoCountLineEdit->setText(QString::number(ProbEndCountTwo));
}
void HTOrobot::AppendTextBrowerArray(const char* text, std::vector<double> array)
{
	QString str;
	for (int i = 0; i < array.size(); ++i)
	{
		str = str + QString::number(array[i]) + " ";
	}
	str = QString(text) + " " + str;
	m_Controls.textBrowser_HTO->append(str);
}

void HTOrobot::AppendTextBrowerArray(const char* text, double* array, int size)
{
	QString str;
	for (int i = 0; i < size; ++i)
	{
		str = str + QString::number(array[i]) + " ";
	}
	str = QString(text) + " " + str;
	m_Controls.textBrowser_HTO->append(str);
}

void HTOrobot::CoutTextBrowerArray(const char* text, double* array, int size)
{
	QString str;
	for (int i = 0; i < size; ++i)
	{
		str = str + QString::number(array[i]) + " ";
	}
	str = QString(text) + " " + str;
	std::cout << str << std::endl;
}

void HTOrobot::AppendTextBrowerArray(const char* text, Eigen::Vector3d array)
{
	QString str;
	for (int i = 0; i < array.size(); ++i)
	{
		str = str + QString::number(array[i]) + " ";
	}
	str = QString(text) + " " + str;
	m_Controls.textBrowser_HTO->append(str);
}

void HTOrobot::GetGuiderOriginPosBtnClicked()
{
	//auto newData = GetNewToolData();
	///
	//UpdateCameraToToolMatrix(newData, "Spine_Probe", T_CamToProbe, nullptr);
	//Eigen::Vector3d v(newData->tooltip[0], newData->tooltip[1], newData->tooltip[2]);
	//AppendTextBrowerArray("Origin Pos: ", v);
	//probeEndOneVector[3] = v;
	probeEndOneVector[3] = Eigen::Vector3d(ProbeTop);
	AppendTextBrowerArray("Origin is: ", probeEndOneVector[3]);
}

void HTOrobot::PrintToolMatrix()
{
	callCount++;
	std::cout << "Function has been called " << callCount << " times." << std::endl;

	//打印T_CamToEndRF
	std::cout << "----------------------------------------" << std::endl;
	std::cout << "T_CamToEndRF:" << std::endl;
	for (int i = 0; i < 4; ++i)
	{
		std::string row;
		for (int j = 0; j < 4; ++j)
		{
			row += std::to_string(T_CamToEndRF[i * 4 + j]) + " ";
		}
		std::cout << row << std::endl;
	}

	//打印T_CamToBaseRF
	std::cout << "----------------------------------------" << std::endl;
	std::cout << "T_CamToBaseRF:" << std::endl;
	for (int i = 0; i < 4; ++i)
	{
		std::string row;
		for (int j = 0; j < 4; ++j)
		{
			row += std::to_string(T_CamToBaseRF[i * 4 + j]) + " ";
		}
		std::cout << row << std::endl;
	}


	//打印T_CamToProbe
	std::cout << "----------------------------------------" << std::endl;
	std::cout << "T_CamToProbe:" << std::endl;
	for (int i = 0; i < 4; ++i)
	{
		std::string row;
		for (int j = 0; j < 4; ++j)
		{
			row += std::to_string(T_CamToProbe[i * 4 + j]) + " ";
		}
		std::cout << row << std::endl;
	}
	std::cout << "----------------------------------------" << std::endl;
	//打印数据
	std::cout << "ProbeTop: " << ProbeTop[0] << "/" << ProbeTop[1] << "/" << ProbeTop[2] << std::endl;


	//打印T_CamToPatientRF
	std::cout << "----------------------------------------" << std::endl;
	std::cout << "T_CamToPatientRF:" << std::endl;
	for (int i = 0; i < 4; ++i)
	{
		std::string row;
		for (int j = 0; j < 4; ++j)
		{
			row += std::to_string(T_CamToPatientRF[i * 4 + j]) + " ";
		}
		std::cout << row << std::endl;
	}
	//打印T_CamToMetalBallRF
	std::cout << "----------------------------------------" << std::endl;
	std::cout << "T_CamToMetalBallRF:" << std::endl;
	for (int i = 0; i < 4; ++i)
	{
		std::string row;
		for (int j = 0; j < 4; ++j)
		{
			row += std::to_string(T_CamToMetalBallRF[i * 4 + j]) + " ";
		}
		std::cout << row << std::endl;
	}
	//打印T_CamToSaw
	std::cout << "----------------------------------------" << std::endl;
	std::cout << "T_CamToSaw:" << std::endl;
	for (int i = 0; i < 4; ++i)
	{
		std::string row;
		for (int j = 0; j < 4; ++j)
		{
			row += std::to_string(T_CamToSaw[i * 4 + j]) + " ";
		}
		std::cout << row << std::endl;
	}
	//打印T_basetobaseRF
	std::cout << "----------------------------------------" << std::endl;
	std::cout << "T_basetobaseRF:" << std::endl;
	for (int i = 0; i < 4; ++i)
	{
		std::string row;
		for (int j = 0; j < 4; ++j)
		{
			row += std::to_string(T_BaseToBaseRF[i * 4 + j]) + " ";
		}
		std::cout << row << std::endl;
	}


	//打印机械臂的相关矩阵
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
	std::cout << "----------------------------------------" << std::endl;
	std::cout << "T_BaseToFlanger:" << std::endl;
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			//std::cout << VTKT_BaseToFlanger->GetElement(i, j) << " ";
			std::cout << std::fixed << std::setprecision(6) << VTKT_BaseToFlanger->GetElement(i, j) << " ";
		}
		std::cout << std::endl;
	}

	//打印T_FlangeToEndRF
	std::cout << "----------------------------------------" << std::endl;


}
vtkSmartPointer<vtkMatrix4x4>HTOrobot::GetArray2vtkMatrix(double* array16)
{
	vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			matrix->SetElement(i, j, array16[i * 4 + j]);
		}
	}
	return matrix;
}
vtkSmartPointer<vtkMatrix4x4> HTOrobot::CalculateTFlange2Camera(double* TF2ENDRF, double* TCamera2EndRF)
{
	auto matrixF2ENDRF = GetArray2vtkMatrix(TF2ENDRF);
	PrintDataHelper::AppendTextBrowserMatrix(m_Controls.textBrowser_HTO, "matrixF2ENDRF: ", matrixF2ENDRF);

	vtkSmartPointer<vtkMatrix4x4> matrixCamera2EndRF = GetArray2vtkMatrix(TCamera2EndRF);
	PrintDataHelper::AppendTextBrowserMatrix(m_Controls.textBrowser_HTO, "matrixCamera2EndRF: ", matrixCamera2EndRF);

	vtkSmartPointer<vtkMatrix4x4> matrixEndRF2Camera = vtkSmartPointer<vtkMatrix4x4>::New();
	matrixEndRF2Camera->DeepCopy(matrixCamera2EndRF);
	matrixEndRF2Camera->Invert();
	PrintDataHelper::AppendTextBrowserMatrix(m_Controls.textBrowser_HTO, "matrixEndRF2Camera: ", matrixEndRF2Camera);
	vtkSmartPointer<vtkMatrix4x4> result = vtkSmartPointer<vtkMatrix4x4>::New();

	vtkMatrix4x4::Multiply4x4(matrixF2ENDRF, matrixEndRF2Camera, result);
	//result = T_flange2EndRf  *  T_EndR2Camera   

	PrintDataHelper::AppendTextBrowserMatrix(m_Controls.textBrowser_HTO, "result: ", result);	//T_flange2Camera
	PrintDataHelper::CoutMatrix("CalculateTFlange2Camera: ", result);
	return result;	//T_flange2Camera
}
Eigen::Vector3d HTOrobot::CalculatePointInFlangePos(vtkMatrix4x4* matrixFlange2Camera, Eigen::Vector3d posInCamera)
{
	vtkSmartPointer<vtkMatrix4x4> Flange2Camera = vtkSmartPointer<vtkMatrix4x4>::New();
	Flange2Camera->DeepCopy(matrixFlange2Camera);
	PrintDataHelper::AppendTextBrowserMatrix(m_Controls.textBrowser_HTO, "Flange2Camera: ", Flange2Camera);


	double point[4] = { posInCamera[0], posInCamera[1], posInCamera[2], 1 };
	double m[4];
	Flange2Camera->MultiplyPoint(point, m);
	PrintDataHelper::AppendTextBrowserArray(m_Controls.textBrowser_HTO, "Pos in Flange is", 3, m);

	return Eigen::Vector3d(m[0], m[1], m[2]);
}
Eigen::Vector3d HTOrobot::CalculatePointProjectInLine(Eigen::Vector3d P, Eigen::Vector3d A, Eigen::Vector3d B)
{
	Eigen::Vector3d AP = P - A;
	Eigen::Vector3d AB = B - A;

	Eigen::Vector3d AB_normalized = AB.normalized();

	double projection_length = AP.dot(AB_normalized);
	Eigen::Vector3d projection = projection_length * AB_normalized;


	//Eigen::Vector3d perpendicular_end = A + projection;
	return projection;
}

void HTOrobot::MoveRobotTCP()
{
	//vtkSmartPointer<vtkMatrix4x4> T_BaseToTool = vtkSmartPointer<vtkMatrix4x4>::New();
	Eigen::Vector3d translation;
	translation << T_BaseToTool->GetElement(0, 3), T_BaseToTool->GetElement(1, 3), T_BaseToTool->GetElement(2, 3);
	Eigen::Matrix3d rotation;
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			rotation(i, j) = T_BaseToTool->GetElement(i, j);
	Eigen::Vector3d eulerAngles = rotation.eulerAngles(2, 1, 0);
	
	m_Controls.textBrowser_HTO->append("moveRobotTCP");
	double dX = 0; double dY = 0; double dZ = 0;
	double dRx = 0; double dRy = 0; double dRz = 0;
	dX= translation[0];
	dY = translation[1];
	dZ = translation[2];
	dRx = eulerAngles[2] * radius2degree;
	dRy = eulerAngles[1] * radius2degree;
	dRz = eulerAngles[0] * radius2degree;

	int nIsUseJoint = 0;
	int nIsSeek = 0;
	int nIOBit = 0; 
	int nIOState = 0; 


	int ret = HRIF_MoveJ(0, 0, dX, dY, dZ, dRx, dRy, dRz, dJ1_init, dJ2_init, dJ3_init, dJ4_init, dJ5_init, dJ6_init, sTcpName, sUcsName,
		dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
	//int ret = HRIF_MoveL(0, 0, dX, dY, dZ, dRx, dRy, dRz,
	//	dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName, sUcsName, dVelocity, dAcc, dRadius,
	//	nIsSeek, nIOBit, nIOState, strCmdID);
	if (ret == 0)
	{

		std::cout << "succesed" << std::endl;
	}
	else
	{

		std::cerr << "Failed to read initial position. Error code: " << ret << std::endl;
	}
}

void HTOrobot::PrintMatrix(std::string matrixName, double* matrix)
{
	m_Controls.textBrowser_HTO->append("---------------------------------------------------");
	m_Controls.textBrowser_HTO->append(QString::fromStdString(matrixName + ":"));
	/*std::cout << matrixName + ": " << std::endl;*/
	for (int i = 0; i < 4; ++i)
	{
		std::string row;
		for (int j = 0; j < 4; ++j)
		{
			row += std::to_string(matrix[i * 4 + j]) + " ";
		}
		m_Controls.textBrowser_HTO->append(QString::fromStdString(row) + "\n");
	}
	m_Controls.textBrowser_HTO->append("---------------------------------------------------");
}

void HTOrobot::Print_Matrix(const std::string& title, vtkMatrix4x4* matrix)
{
	std::cout << title << std::endl;
	std::cout << "+-----------------------------------------------+" << std::endl;
	std::cout << "|       Matrix: " << title << "              |" << std::endl;
	std::cout << "+-----------+-----------+----------+------------+" << std::endl;
	for (int i = 0; i < 4; i++)
	{
		std::cout << "|  ";
		for (int j = 0; j < 4; j++)
		{
			std::cout << std::fixed << std::setprecision(6) << matrix->GetElement(i, j) << " ";
		}
		std::cout << "|" << std::endl;
	}
	std::cout << "+-----------+-----------+----------+------------+" << std::endl;
}

void HTOrobot::PrintTCP_qt(QTextBrowser* browser, double x, double y, double z, double rx, double ry, double rz)
{
	browser->append("---------------------------------------------------");
	browser->append("dX_tcp=" + QString::number(x));
	browser->append("dY_tcp=" + QString::number(y));
	browser->append("dZ_tcp=" + QString::number(z));
	browser->append("dRx_tcp=" + QString::number(rx));
	browser->append("dRy_tcp=" + QString::number(ry));
	browser->append("dRz_tcp=" + QString::number(rz));
	browser->append("---------------------------------------------------");
}
//建立验证手指tcp坐标，具体建立方式请见飞书文档
bool  HTOrobot::SetPlanePrecisionTestTcp()
{
	//验证配准结果
	vtkMatrix4x4* vtkT_FlangeToEndRF = vtkMatrix4x4::New();
	vtkT_FlangeToEndRF->DeepCopy(T_FlangeToEndRF); 
	//加入打印信息方便调试
	PrintMatrix("vtkT_FlangeToEndRF", vtkT_FlangeToEndRF->GetData());
	Print_Matrix("vtkT_FlangeToEndRF", vtkT_FlangeToEndRF);

	Eigen::Vector3d p1;
	p1[0] = m_Controls.lineEdit_plane_p2_x->text().toDouble();
	p1[1] = m_Controls.lineEdit_plane_p2_y->text().toDouble();
	p1[2] = m_Controls.lineEdit_plane_p2_z->text().toDouble();

	Eigen::Vector3d p2;
	p2[0] = m_Controls.lineEdit_plane_p3_x->text().toDouble();
	p2[1] = m_Controls.lineEdit_plane_p3_y->text().toDouble();
	p2[2] = m_Controls.lineEdit_plane_p3_z->text().toDouble();

	Eigen::Vector3d p3;
	p3[0] = m_Controls.lineEdit_plane_p4_x->text().toDouble();
	p3[1] = m_Controls.lineEdit_plane_p4_y->text().toDouble();
	p3[2] = m_Controls.lineEdit_plane_p4_z->text().toDouble();

	Eigen::Vector3d x_tcp;
	x_tcp = p3 - p1;
	x_tcp.normalize();

	Eigen::Vector3d y_tmp;
	y_tmp = p2 - p1;
	y_tmp.normalize();

	Eigen::Vector3d z_tcp;
	z_tcp = x_tcp.cross(y_tmp);
	z_tcp.normalize();

	Eigen::Vector3d y_tcp;
	y_tcp = z_tcp.cross(x_tcp);
	y_tcp.normalize();

	Eigen::Matrix3d Re;

	Re << x_tcp[0], y_tcp[0], z_tcp[0],
		x_tcp[1], y_tcp[1], z_tcp[1],
		x_tcp[2], y_tcp[2], z_tcp[2];

	Eigen::Vector3d eulerAngle = Re.eulerAngles(2, 1, 0);
	//------------------------------------------------
	double tcp[6];
	tcp[0] = p1[0]; // tx
	tcp[1] = p1[1]; // ty
	tcp[2] = p1[2]; // tz
	tcp[3] = eulerAngle(2) * radius2degree; //-0.81;// -0.813428203; // rx
	tcp[4] = eulerAngle(1) * radius2degree; // ry
	tcp[5] = eulerAngle(0) * radius2degree; // rz
	PrintTCP_qt(m_Controls.textBrowser_HTO, tcp[0], tcp[1], tcp[2], tcp[3], tcp[4], tcp[5]);

	int nRet = HRIF_SetTCP(0, 0, tcp[0], tcp[1], tcp[2], tcp[3], tcp[4], tcp[5]);
	//set tcp to robot

	if (nRet == 0)
	{
		std::cout << "line  TCP Set succeed" << std::endl;
	}
	else
	{
		std::cout << "line TCP Set failed" << std::endl;
	}

	return true;
}
//前往目标平面边界
bool HTOrobot::InterpretImagePlane()
{
	//获取图像坐标系立柱工装上的点
	auto targetPlanePoints = dynamic_cast<mitk::PointSet*>(m_Controls.mitkNodeSelectWidget_imageTargetPlane->GetSelectedNode()->GetData());
	auto targetPoint_0 = targetPlanePoints->GetPoint(0); // TCP frame origin should move to this point
	auto targetPoint_1 = targetPlanePoints->GetPoint(1);
	auto targetPoint_2 = targetPlanePoints->GetPoint(2);
	//打印点位
	std::cout << "targetPoint_0: (" << targetPoint_0[0] << ", " << targetPoint_0[1] << ", " << targetPoint_0[2] << ")" << std::endl;
	std::cout << "targetPoint_1: (" << targetPoint_1[0] << ", " << targetPoint_1[1] << ", " << targetPoint_1[2] << ")" << std::endl;
	std::cout << "targetPoint_1: (" << targetPoint_2[0] << ", " << targetPoint_2[1] << ", " << targetPoint_2[2] << ")" << std::endl;
	double targetPointUnderBase_0[3]{ 0 };
	//采样部分，取20个数值取平均，这部分有待调试具体sleep部分会出错，在for中确保能拿到20个数值不同的点
	vtkMatrix4x4* samples[20];
	vtkMatrix4x4* vtkT_BaseToBaseRF = vtkMatrix4x4::New();
	auto vtkT_BaseRFToCamera = vtkMatrix4x4::New();
	auto vtkT_CameraToPatientRF = vtkMatrix4x4::New();
	auto vtkT_PatientRFToImage = vtkMatrix4x4::New();
	auto vtkT_BaseToImage = vtkMatrix4x4::New();
	for (size_t i = 0; i < 20; i++)
	{
		QThread::sleep(0.1);
		GetMatrix();
		//获取机械臂配准矩阵T_BaseToBaseRF
		vtkT_BaseToBaseRF->DeepCopy(T_BaseToBaseRF);

		//获取T_BaseRFToCamera
		vtkT_BaseRFToCamera->DeepCopy(T_CamToBaseRF);
		vtkT_BaseRFToCamera->Invert();

		//获取T_CameraToPatientRF

		vtkT_CameraToPatientRF->DeepCopy(T_CamToPatientRF);
		//获取T_PatientRFToImage
		vtkT_PatientRFToImage->DeepCopy(T_PatientRFtoImage);
		//计算T_BaseToImage
		vtkNew<vtkTransform> Transform;
		Transform->Identity();
		Transform->PostMultiply();
		Transform->SetMatrix(vtkT_PatientRFToImage);
		Transform->Concatenate(vtkT_CameraToPatientRF);
		Transform->Concatenate(vtkT_BaseRFToCamera);
		Transform->Concatenate(vtkT_BaseToBaseRF);
		Transform->Update();
		auto vtkT_BaseToImage = Transform->GetMatrix();
		//赋值
		samples[i] = vtkT_BaseToImage;
	}
	vtkMatrix4x4* average;
	//计算平均值，参考surgicalsimulated
	computeAverageTransform(samples, average);
	//得到平均值矩阵
	vtkT_BaseToImage = average;


	//get surface to target plane matrix建立图像目标点坐标系
	Eigen::Vector3d x_surfaceToPlane;
	x_surfaceToPlane << targetPoint_2[0] - targetPoint_0[0],
		targetPoint_2[1] - targetPoint_0[1],
		targetPoint_2[2] - targetPoint_0[2];
	x_surfaceToPlane.normalize();

	Eigen::Vector3d tmp_y;
	tmp_y << targetPoint_0[0] - targetPoint_1[0],
		targetPoint_0[1] - targetPoint_1[1],
		targetPoint_0[2] - targetPoint_1[2];
	tmp_y.normalize();

	Eigen::Vector3d z_surfaceToPlane = x_surfaceToPlane.cross(tmp_y);
	z_surfaceToPlane.normalize();

	Eigen::Vector3d y_surfaceToPlane = z_surfaceToPlane.cross(x_surfaceToPlane);
	y_surfaceToPlane.normalize();

	double array_surfaceToPlane[16]
	{
		x_surfaceToPlane[0], y_surfaceToPlane[0], z_surfaceToPlane[0], targetPoint_0[0],
		x_surfaceToPlane[1], y_surfaceToPlane[1], z_surfaceToPlane[1], targetPoint_0[1],
		x_surfaceToPlane[2], y_surfaceToPlane[2], z_surfaceToPlane[2], targetPoint_0[2],
		0,0,0,1
	};
	vtkNew<vtkMatrix4x4> vtkimageSurfaceToPlane;
	vtkimageSurfaceToPlane->DeepCopy(array_surfaceToPlane);

	// get base to target plane matrix
	vtkNew<vtkTransform> Trans;
	Trans->Identity();
	Trans->PostMultiply();
	Trans->SetMatrix(vtkimageSurfaceToPlane);
	Trans->Concatenate(vtkT_BaseToImage);
	Trans->Update();
	vtkBaseToTargetPlaneTransform = Trans->GetMatrix();

	return true;
}
//前往初始位置，对一个轴做偏移，也就是到达起点
void HTOrobot::On_pushButton_goToFakePlane_clicked()
{
	vtkMatrix4x4* t = vtkBaseToTargetPlaneTransform;

	auto tmpVtkTrans = vtkTransform::New();
	//存疑 z轴偏还是x轴偏
	auto offsetMatrix = vtkMatrix4x4::New();
	offsetMatrix->Identity();
	offsetMatrix->SetElement(1, 3, 50);

	tmpVtkTrans->PostMultiply();
	tmpVtkTrans->Identity();
	tmpVtkTrans->SetMatrix(offsetMatrix);
	tmpVtkTrans->Concatenate(t);
	tmpVtkTrans->Update();

	auto resultMatrix = tmpVtkTrans->GetMatrix();


	Eigen::Matrix3d Re;

	Re << resultMatrix->GetElement(0, 0), resultMatrix->GetElement(0, 1), resultMatrix->GetElement(0, 2),
		resultMatrix->GetElement(1, 0), resultMatrix->GetElement(1, 1), resultMatrix->GetElement(1, 2),
		resultMatrix->GetElement(2, 0), resultMatrix->GetElement(2, 1), resultMatrix->GetElement(2, 2);

	Eigen::Vector3d eulerAngle = Re.eulerAngles(2, 1, 0);
	double x = resultMatrix->GetElement(0, 3);
	double y = resultMatrix->GetElement(1, 3);
	double z = resultMatrix->GetElement(2, 3);
	double rx =  eulerAngle[2]*radius2degree ;
	double ry =  eulerAngle[1] * radius2degree;
	double rz =  eulerAngle[0] * radius2degree;

	int nTargetPoint = HRIF_MoveL(0, 0, dX, dY, dZ, dRx, dRy, dRz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName, sUcsName,
		dVelocity, dAcc, dRadius, nIsSeek, nIOBit, nIOState, strCmdID);//机械臂移动

	if (nTargetPoint == 0)
	{
		m_Controls.textBrowser_HTO->append("MOVE OK");
	}
	else
	{
		m_Controls.textBrowser_HTO->append("MOVE FAILED");
	}
}
//前往目标位置
void HTOrobot::OnAutoPositionStart()
{


	Eigen::Matrix3d Re;

	Re << vtkBaseToTargetPlaneTransform->GetElement(0, 0), vtkBaseToTargetPlaneTransform->GetElement(0, 1), vtkBaseToTargetPlaneTransform->GetElement(0, 2),
		vtkBaseToTargetPlaneTransform->GetElement(1, 0), vtkBaseToTargetPlaneTransform->GetElement(1, 1), vtkBaseToTargetPlaneTransform->GetElement(1, 2),
		vtkBaseToTargetPlaneTransform->GetElement(2, 0), vtkBaseToTargetPlaneTransform->GetElement(2, 1), vtkBaseToTargetPlaneTransform->GetElement(2, 2);

	Eigen::Vector3d eulerAngle = Re.eulerAngles(2, 1, 0);
	double x = vtkBaseToTargetPlaneTransform->GetElement(0, 3);
	double y = vtkBaseToTargetPlaneTransform->GetElement(1, 3);
	double z = vtkBaseToTargetPlaneTransform->GetElement(2, 3);
	double rx =  eulerAngle[2] * radius2degree;
	double ry =  eulerAngle[1] * radius2degree;
	double rz =  eulerAngle[0] * radius2degree;
	m_Controls.textBrowser_HTO->append("-------------------------------------------------------------------------------------------");
	m_Controls.textBrowser_HTO->append("TCP_move");
	m_Controls.textBrowser_HTO->append("dx=" + QString::number(x));
	m_Controls.textBrowser_HTO->append("dy=" + QString::number(y));
	m_Controls.textBrowser_HTO->append("dz=" + QString::number(z));
	m_Controls.textBrowser_HTO->append("dRx=" + QString::number(rx));
	m_Controls.textBrowser_HTO->append("dRy=" + QString::number(ry));
	m_Controls.textBrowser_HTO->append("dRz=" + QString::number(rz));
	("-------------------------------------------------------------------------------------------");

	dX = x;
	dY = y;
	dZ = z;
	dRx = rx;
	dRy = ry;
	dRz = rz;

	int nTargetPoint = HRIF_MoveL(0, 0, dX, dY, dZ, dRx, dRy, dRz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName, sUcsName,
		dVelocity, dAcc, dRadius, nIsSeek, nIOBit, nIOState, strCmdID);//机械臂移动

	if (nTargetPoint == 0)
	{
		m_Controls.textBrowser_HTO->append("MOVE OK");
	}
	else
	{
		m_Controls.textBrowser_HTO->append("MOVE FAILED");
	}

}

Eigen::Matrix4d HTOrobot::vtkMatrix4x4ToEigen(const vtkMatrix4x4* vtkMatrix)
{
	Eigen::Matrix4d eigenMatrix;
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			eigenMatrix(i, j) = vtkMatrix->GetElement(i, j);
		}
	}
	return eigenMatrix;
}
vtkMatrix4x4* HTOrobot::eigenToVtkMatrix4x4(const Eigen::Matrix4d& eigenMatrix)
{
	vtkMatrix4x4* vtkMatrix = vtkMatrix4x4::New();
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			vtkMatrix->SetElement(i, j, eigenMatrix(i, j));
		}
	}
	return vtkMatrix;
}
void HTOrobot::computeAverageTransform(vtkMatrix4x4* samples[], vtkMatrix4x4* average)
{
	double tmp_x[3]{ 0,0,0 };
	double tmp_y[3]{ 0,0,0 };
	double tmp_translation[3]{ 0,0,0 };
	for (int i{ 0 }; i < 20; i++)
	{


		auto tmpMatrix = samples[i];

		std::cout << "Averaging NavigationData and print the 1st element:" << tmpMatrix->GetElement(0, 0);

		tmp_x[0] += tmpMatrix->GetElement(0, 0);
		tmp_x[1] += tmpMatrix->GetElement(1, 0);
		tmp_x[2] += tmpMatrix->GetElement(2, 0);

		tmp_y[0] += tmpMatrix->GetElement(0, 1);
		tmp_y[1] += tmpMatrix->GetElement(1, 1);
		tmp_y[2] += tmpMatrix->GetElement(2, 1);

		tmp_translation[0] += tmpMatrix->GetElement(0, 3);
		tmp_translation[1] += tmpMatrix->GetElement(1, 3);
		tmp_translation[2] += tmpMatrix->GetElement(2, 3);

	}
	Eigen::Vector3d x;
	x[0] = tmp_x[0];
	x[1] = tmp_x[1];
	x[2] = tmp_x[2];
	x.normalize();

	Eigen::Vector3d h;
	h[0] = tmp_y[0];
	h[1] = tmp_y[1];
	h[2] = tmp_y[2];
	h.normalize();

	Eigen::Vector3d z;
	z = x.cross(h);
	z.normalize();

	Eigen::Vector3d y;
	y = z.cross(x);
	y.normalize();

	tmp_translation[0] = tmp_translation[0] / 20;
	tmp_translation[1] = tmp_translation[1] / 20;
	tmp_translation[2] = tmp_translation[2] / 20;
	average->Identity();
	average->SetElement(0, 0, x[0]);
	average->SetElement(0, 1, y[0]);
	average->SetElement(0, 2, z[0]);
	average->SetElement(1, 0, x[1]);
	average->SetElement(1, 1, y[1]);
	average->SetElement(1, 2, z[1]);
	average->SetElement(2, 0, x[2]);
	average->SetElement(2, 1, y[2]);
	average->SetElement(2, 2, z[2]);

	average->SetElement(0, 3, tmp_translation[0]);
	average->SetElement(1, 3, tmp_translation[1]);
	average->SetElement(2, 3, tmp_translation[2]);
}
void HTOrobot::textEditProcess(double num[6], QLineEdit* LineEdit)
{
	QString text = LineEdit->text();
	//去除空白字符
	QStringList values = text.split(',');

	if (values.size() == 6)
	{
		bool ok;
		for (int i = 0; i < 6; ++i)
		{
			num[i] = values[i].trimmed().toDouble(&ok);
			if (!ok)
			{
				qDebug() << "Conversion error at index" << i;
				return;
			}
		}

		// 打印数组内容以验证
		for (int i = 0; i < 6; ++i)
		{
			qDebug() << "[" << i << "] =" << num[i];
		}
	}
	else
	{
		qDebug() << "Input does not contain 6 values";
	}
}
//---------------------------------------------------------------------------------------------------------------
/**
*@brief 打印机械臂函数是否执行成功，不成功返回错误码和错误类型
*@note  用法：PrintResult(m_Controls.textBrowser, n, "你想写的话");
*/
//---------------------------------------------------------------------------------------------------------------
void HTOrobot::PrintResult(QTextBrowser* browser, int nRet, const char* message)
{
	if (nRet == 0)
	{
		browser->append(QString(message) + " Succeed");
		std::cout << message << " Succeed" << std::endl;
		std::cout << "---------------------------------------------------" << std::endl;
		browser->append("---------------------------------------------------");
	}
	else
	{
		browser->append(QString(message) + " Failed");
		std::cout << message << " Failed" << std::endl;

		std::cout << "ErrorCode: " << nRet << std::endl;
		string tmpstr;
		int zyx = HRIF_GetErrorCodeStr(0, nRet, tmpstr);
		std::cout << nRet << std::endl;
		/*browser->append("ErrorCode：" + QString::fromStdString(tmpstr));*/
		browser->append("ErrorCode: " + QString::number(nRet));
		//针对一些常见错误做的一些解释
		if (nRet == 39500)
		{
			std::cout << "Please check the connection of the robot arm" << std::endl;
			browser->append("Please check the connection of the robot arm");

		}
		if (nRet == 40025)
		{
			browser->append("The robot is not in a state of readiness" + QString::number(nRet));
		}

		if (nRet >= 40000 && nRet <= 40500)
		{
			std::cout << "CDS executed command with error: " << nRet << std::endl;
			/*browser->append("CDS executed command with error: " + QString::number(nRet));*/
			browser->append("CDS执行命令时发生错误：" + QString::number(nRet));
		}
		else if (nRet >= 10000 && nRet <= 10015)
		{
			std::cout << "Robot servo drive reported fault code: " << nRet << std::endl;
			/*browser->append("Robot servo drive reported fault code: " + QString::number(nRet));*/
			browser->append("机器人伺服驱动报告故障代码：" + QString::number(nRet));
		}
		else if (nRet >= 10016 && nRet <= 11000)
		{
			std::cout << "Robot collaboration algorithm detected fault: " << nRet << std::endl;
			/*browser->append("Robot collaboration algorithm detected fault: " + QString::number(nRet));*/
			browser->append("检测到机器人协作算法故障：" + QString::number(nRet));
		}
		else if (nRet >= 15000 && nRet <= 16000)
		{
			std::cout << "Robot control module detected fault: " << nRet << std::endl;
			/*browser->append("Robot control module detected fault: " + QString::number(nRet));*/
			/*browser->append("检测到机器人控制模块故障：" + QString::number(nRet));*/
		}
		else if (nRet >= 30001 && nRet <= 30016)
		{
			std::cout << "Modbus module error during command execution: " << nRet << std::endl;
			/*browser->append("Modbus module error during command execution: " + QString::number(nRet));*/
			/*browser->append("Modbus模块在执行命令时发生错误：" + QString::number(nRet));*/
		}
		else if (nRet >= 20000 && nRet <= 39999)
		{
			std::cout << "CPS executed command with error: " << nRet << std::endl;
			/*browser->append("CPS executed command with error: " + QString::number(nRet));*/
			/*browser->append("CPS执行命令时发生错误：" + QString::number(nRet));*/
		}

		std::cout << "---------------------------------------------------" << std::endl;
		browser->append("---------------------------------------------------");
	}
}
void HTOrobot::textEditProcess(int num[6], QLineEdit* LineEdit)
{
	QString text = LineEdit->text();
	//去除空白字符
	QStringList values = text.split(',');

	if (values.size() == 6)
	{
		bool ok;
		for (int i = 0; i < 6; ++i)
		{
			num[i] = values[i].trimmed().toInt(&ok);
			if (!ok)
			{
				qDebug() << "Conversion error at index" << i;
				return;
			}
		}

		// 打印数组内容以验证
		for (int i = 0; i < 6; ++i)
		{
			qDebug() << "degreesOfFreedom[" << i << "] =" << num[i];
		}
	}
	else
	{
		qDebug() << "Input does not contain 6 values";
	}
}
//movetcp 通过输入xyz rx ry rz的方式
/*
1：e05 pro支持3种运动模式控制机械臂移动，经测试1 、3 运动速度较慢，2 movel直线运动同样速度下运动最快，厂家推荐使用第一种
2：如果控制法兰可将sTcpName 替换为默认法兰 "TCP" 及可运动

*/
void HTOrobot::movetcpline()
{
	double position[6] = { 0 };
	//读取当前参考关节坐标
	double now_joint[6] = { 0 };
	int nRet = HRIF_ReadActJointPos(boxID, rbtID, now_joint[0], now_joint[1], now_joint[2], now_joint[3], now_joint[4], now_joint[5]);

	textEditProcess(position, m_Controls.lineEdit_movetcpline);

	int type  = m_Controls.lineEdit_movetpye->text().toInt();
	if (type == 1)
	{
		 nRet = HRIF_WayPoint(0, 0, 1, position[0], position[1], position[2], position[3], position[4], position[5], now_joint[0], now_joint[1], now_joint[2], now_joint[3], now_joint[4], now_joint[5], "TCP_1", "Base", dVelocity, dAcc, dRadius, 0, 0, 0, 0, "ID0");
	}
	else if (type == 2)
	{
		nRet = HRIF_MoveJ(0, 0, position[0], position[1], position[2], position[3], position[4], position[5], now_joint[0], now_joint[1], now_joint[2], now_joint[3], now_joint[4], now_joint[5], sTcpName, sUcsName,
			dVelocity, dAcc, dRadius, 0, nIsSeek, nIOBit, nIOState, strCmdID);
	}
	else
	{
		nRet = HRIF_MoveL(0, 0, position[0], position[1], position[2], position[3], position[4], position[5], now_joint[0], now_joint[1], now_joint[2], now_joint[3], now_joint[4], now_joint[5], sTcpName, sUcsName,
			dVelocity, dAcc, dRadius, nIsSeek, nIOBit, nIOState, strCmdID);//机械臂移动
	}
	
	
	


	PrintResult(m_Controls.textBrowser_HTO, nRet, "move robot tcp");
}
//tcp 为 x y z（mm） rx ry rz（degree）。
void HTOrobot::tcp2homoguous(double tcp[], vtkSmartPointer<vtkMatrix4x4> &VTK_BaseToFlange)
{
	double dRx = tcp[3];
	double dRy = tcp[4];
	double dRz = tcp[5];
	double dX = tcp[0];
	double dY = tcp[1];
	double dZ = tcp[2];

	auto tempTrans = vtkTransform::New();
		tempTrans->PostMultiply();
		tempTrans->RotateX(dRx);
		tempTrans->RotateY(dRy);
		tempTrans->RotateZ(dRz);
		tempTrans->Translate(dX, dY, dZ);
		tempTrans->Update();
	
	VTK_BaseToFlange = tempTrans->GetMatrix();
}
