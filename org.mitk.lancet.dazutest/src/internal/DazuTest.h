/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#ifndef DazuTest_h
#define DazuTest_h

#include <berryISelectionListener.h>

#include <QmitkAbstractView.h>

#include "ui_DazuTestControls.h"

#include "AimPositionDef.h"
#include "AimPositionAPI.h"

#include "mitkPointSet.h"
#include "mitkTrackingDeviceSource.h"
#include <QmitkPointListModel.h>
#include <mitkPointSetDataInteractor.h>
#include <mitkImageToItk.h>
#include <mitkITKImageImport.h>
#include <mitkDataNode.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <QFuture>
#include <QtConcurrent>
#include <QMessageBox>
#include <QKeyEvent>
#include "ToolFunction.h"
#include "DaZu_Robot.h"

//#include "PrintDataHelper.h"


#include "robotRegistration.h"
/**
  \brief DazuTest

  \warning  This class is not yet documented. Use "git blame" and ask the author to provide basic documentation.

  \sa QmitkAbstractView
  \ingroup ${plugin_target}_internal
*/
class DazuTest : public QmitkAbstractView
{
	// this is needed for all Qt objects that should have a Qt meta-object
	// (everything that derives from QObject and wants to have signal/slots)
	Q_OBJECT

public slots:
	void ConnectAim();
	void AimAcquisition();
	void updateCameraData();
	void UpdateCameraToToolMatrix(T_AimToolDataResult* ToolData, const char* Name, double* aCamera2Tool, QLabel* label);
	void DeepProtection();
	void StartTrackingPose();

public:
	static const std::string VIEW_ID;
	DaZuRobot Robot1;
	DaZuRobot* Robot = &Robot1;
	
	AimHandle aimhandle = NULL;
	//
	QTimer* m_AimoeVisualizeTimer{ nullptr };
	QTimer* DeepProtectionTimer{ nullptr };
	QTimer* TrackingTimer{ nullptr };
	T_MarkerInfo markerSt;
	T_AimPosStatusInfo statusSt;
	RobotRegistration m_RobotRegistration;
	int m_IndexOfRobotCapture = 0;
	//QTimer* m_CheckIfMoveDone{ nullptr };

	
	void JustForTest();
	void FPS_test();
	void RobotDelayTest();
	void DealwithData();
	std::array<QDateTime, 2> GetDelayTime(std::array<double, 6> Joint);
	void DealwithDelayData();

	void ConfigUCS();
	void SetTCPbyName();
	void ConfigTCP();
	void ReadBasetoFlange();

	void ReadActPose();
	void gotoinitpos();

	void ReadActTCPByName();
	void ReadcmdTCPPose();
	void ReadCurTCP();

	void xp();
	void yp();
	void zp();
	void rxp();
	void ryp();
	void rzp();
	void xm();
	void ym();
	void zm();
	void rxm();
	void rym();
	void rzm();

	void MoveJ();
	void MoveL();
	void MoveJJoint();

	void setforcecontrol();
	void cancelforcecontrol();
	void settoolmove();
	void setforcezero();
	void setvelocities();
	void setfreedom();
	void setConstantforce();
	void setObstacleavoidance();
	void setmassparam();
	void setdampparam();
	void setstiffparam();
	void setgoalforce();
	void setforceDataLimits();

	void setforcefreedrivemode();
	void canforcefreedrivemode();
	void readforce();
	void setfreedrivefreedom();
	void setFTFreeFaztor();
	void setcompensateforce();
	//void setfreedriveforce();
	//void setfreefrivevel();
	//void setmaxdistance();
	//void setdeviationrange();
	void settangentforcebound();



	//---------------------------------------------------------------------------------------------------------------
	/**
	 * 口腔
	 */
	 //---------------------------------------------------------------------------------------------------------------
	
	void InitPointSetSelector(QmitkSingleNodeSelectionWidget* widget);
	void InitSurfaceSelector(QmitkSingleNodeSelectionWidget* widget);
	

	
	void powerOff();
	void powerOn();
	void connectArm();
	void Printmatrix();
	void Reset();


	void OpenFreeDrag();
	void TurnOffFreeDrag();
	void suddenstop();
	void SetToolMotion();
	void SetBaseMotion();

	

	void setTCPToFlange();
	void CaptureRobotPose(bool translationOnly);
	void CaptureRobot();
	void CaptureRobotFinash();
	void SaveRobotMatrix();
	void ReuseRobotMatrix();

	void DeleteRobotMatrixCache();
	void AutoCaptureRobot();
	void automove();
	//void CheckIfMoveDone();
	void DemarcateTCP();
	void SaveTCP();
	void ReuseTCP();
	void ApplyTCP();

	void ImageBoneLandmarkCapture();
	void ImageBoneLandmark();
	void ImageBoneICPCapture();
	void ImageBoneICP();
	void MixLandmarkandICPImageBone();
	void UpdateImageBone();

	void CleanRegistrationCacheImageBone();
	void SaveRegistrationMatrixImageBone();
	void ReuseRegistrationMatrixImageBone();


	void OnAutoPositionPosture();
	void OnAutoPositionLocate();
	void OnAutoPositionAll();
	void StartForceControl();

	void ColumnToolLandmarkCapture();
	void ColumnToolLandmark();
	void ColumnToolICPCapture();
	void ColumnToolICP();
	void MixLandmarkandICPColumnTool();
	void UpdateColumnTool();

	void CleanRegistrationCacheColumnTool();
	void SaveRegistrationMatrixColumnTool();
	void ReuseRegistrationMatrixColumnTool();

	void SetColumnToolTCP();

	void TrackingPosition();
	

	DazuTest();

	// 带参数的构造函数，允许自定义日志路径
	DazuTest(const std::string& logPath);

	~DazuTest() override;

	std::vector <std::string> ToolName = { "Oral_RobotBaseRF", "Oral_RobotEndRF", "Oral_PatientRF", "Oral_Probe", "Oral_TCPRF"};
	//vector<double[16]> ToolMatrix = {};

	std::string sUcsName = "Base";
	std::string sTcpName = "TCP_Oral";

	std::array<double, 6> Joint_init{ 0, 0, 0, 0, 0, 0 };

	double ProbeTop[4] = { 0.0, 0.0, 0.0, 0.0 };
	//double T_FlangeToTCP[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	double T_CamToBaseRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	double T_CamToEndRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	double T_CamToPatientRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	double T_CamToProbe[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	double T_CamToCalibratorRF[16] { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	double T_BaseToBaseRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	double T_FlangeToEndRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	double T_FlangeToTCP[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };


	std::array<double, 6> T_TargetPoint{ 354.264,-6.902,486.621,-174.133,40.062,-175.509 };

	double T_BasetoTarget[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };


protected:
	virtual void CreateQtPartControl(QWidget* parent) override;

	virtual void SetFocus() override;

	/// \brief called by QmitkFunctionality when DataManager's selection has changed
	virtual void OnSelectionChanged(berry::IWorkbenchPart::Pointer source,
		const QList<mitk::DataNode::Pointer>& nodes) override;

	/// \brief Called when the user clicks the GUI button

	Ui::DazuTestControls m_Controls;

	E_ReturnValue RLT;

private:
	std::ofstream logFile;
	TeeStreambuf* teeStreambuf;
	std::streambuf* oldCoutStreambuf;
	void init(const std::string& logPath);

};

#endif // DazuTest_h
