/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


#ifndef DentalRobot_h
#define DentalRobot_h

#include <berryISelectionListener.h>

#include <QmitkAbstractView.h>
#include <vtkPolyData.h>

#include "lancetApplySurfaceRegistratioinStaticImageFilter.h"
#include "lancetNavigationObjectVisualizationFilter.h"
#include "mitkPointSet.h"
#include "mitkTrackingDeviceSource.h"
#include "ui_DentalRobotControls.h"

#include "AimPositionDef.h"
#include "AimPositionAPI.h"

#include "ProbeInteractor.h"

#include "robotRegistration.h"

#include <berryISelectionListener.h>
#include <QmitkAbstractView.h>
#include "AimPositionDef.h"
#include "AimPositionAPI.h"
#include <HR_Pro.h>
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

#include "PrintDataHelper.h"
#include "LancetHansRobot.h"
#include "AimCamera.h"
#include "mitkNodePredicateAnd.h"
#include "mitkNodePredicateDataType.h"
#include "mitkNodePredicateNot.h"
#include "mitkNodePredicateOr.h"
#include "mitkNodePredicateProperty.h"


/**
  \brief DentalRobot

  \warning  This class is not yet documented. Use "git blame" and ask the author to provide basic documentation.

  \sa QmitkAbstractView
  \ingroup ${plugin_target}_internal
*/
class DentalRobot : public QmitkAbstractView
{
  // this is needed for all Qt objects that should have a Qt meta-object
  // (everything that derives from QObject and wants to have signal/slots)
  Q_OBJECT

public:
  static const std::string VIEW_ID;

  AimHandle aimHandle = NULL;

  //初始化机械配准的类
  RobotRegistration::Pointer m_RobotRegistration = RobotRegistration::New();

  mitk::DataNode* m_IcpSourceSurface{ nullptr };
  void InitPointSetSelector(QmitkSingleNodeSelectionWidget* widget);

  ///////////////////////////////////机械臂控制变量///////////////////////////////////

	// 空间坐标点
  double dX = 0; double dY = 0; double dZ = 0;
  double dRx = 0; double dRy = 0; double dRz = 0;
  // 关节坐标点
  double dJ1 = 0; double dJ2 = 0; double dJ3 = 0;
  double dJ4 = 0; double dJ5 = 0; double dJ6 = 0;
  // 工具坐标点
  double dTcp_X = 0; double dTcp_Y = 0; double dTcp_Z = 0;
  double dTcp_Rx = 0; double dTcp_Ry = 0; double dTcp_Rz = 0;
  // 用户坐标点
  double dUcs_X = 0; double dUcs_Y = 0; double dUcs_Z = 0;
  double dUcs_Rx = 0; double dUcs_Ry = 0; double dUcs_Rz = 0;
  // 初始的函数
  double g_init_X = 0; double g_init_Y = 0; double g_init_Z = 0;
  double g_init_Rx = 0; double g_init_Ry = 0; double g_init_Rz = 0;
  double dJ1_init = 0; double dJ2_init = 0; double dJ3_init = 0;
  double dJ4_init = 0; double dJ5_init = 0; double dJ6_init = 0;
  //记录点1
  double g_X = 0; double g_Y = 0; double g_Z = 0;
  double g_Rx = 0; double g_Ry = 0; double g_Rz = 0;
  double dJ1_1 = 0; double dJ2_1 = 0; double dJ3_1 = 0;
  double dJ4_1 = 0; double dJ5_1 = 0; double dJ6_1 = 0;

  //记录点A
  double g_X_A = 0; double g_Y_A = 0; double g_Z_A = 0;
  double g_Rx_A = 0; double g_Ry_A = 0; double g_Rz_A = 0;
  double dJ1_A = 0; double dJ2_A = 0; double dJ3_A = 0;
  double dJ4_A = 0; double dJ5_A = 0; double dJ6_A = 0;

  //记录点B
  double g_X_B = 0; double g_Y_B = 0; double g_Z_B = 0;
  double g_Rx_B = 0; double g_Ry_B = 0; double g_Rz_B = 0;
  double dJ1_B = 0; double dJ2_B = 0; double dJ3_B = 0;
  double dJ4_B = 0; double dJ5_B = 0; double dJ6_B = 0;

  //记录点C
  double g_X_C = 0; double g_Y_C = 0; double g_Z_C = 0;
  double g_Rx_C = 0; double g_Ry_C = 0; double g_Rz_C = 0;
  double dJ1_C = 0; double dJ2_C = 0; double dJ3_C = 0;
  double dJ4_C = 0; double dJ5_C = 0; double dJ6_C = 0;

  //记录点D
  double g_X_D = 0; double g_Y_D = 0; double g_Z_D = 0;
  double g_Rx_D = 0; double g_Ry_D = 0; double g_Rz_D = 0;
  double dJ1_D = 0; double dJ2_D = 0; double dJ3_D = 0;
  double dJ4_D = 0; double dJ5_D = 0; double dJ6_D = 0;

  //记录点E
  double g_X_E = 0; double g_Y_E = 0; double g_Z_E = 0;
  double g_Rx_E = 0; double g_Ry_E = 0; double g_Rz_E = 0;
  double dJ1_E = 0; double dJ2_E = 0; double dJ3_E = 0;
  double dJ4_E = 0; double dJ5_E = 0; double dJ6_E = 0;

  //记录点F
  double g_X_F = 0; double g_Y_F = 0; double g_Z_F = 0;
  double g_Rx_F = 0; double g_Ry_F = 0; double g_Rz_F = 0;
  double dJ1_F = 0; double dJ2_F = 0; double dJ3_F = 0;
  double dJ4_F = 0; double dJ5_F = 0; double dJ6_F = 0;

  //记录点G
  double g_X_G = 0; double g_Y_G = 0; double g_Z_G = 0;
  double g_Rx_G = 0; double g_Ry_G = 0; double g_Rz_G = 0;
  double dJ1_G = 0; double dJ2_G = 0; double dJ3_G = 0;
  double dJ4_G = 0; double dJ5_G = 0; double dJ6_G = 0;

  //记录点H
  double g_X_H = 0; double g_Y_H = 0; double g_Z_H = 0;
  double g_Rx_H = 0; double g_Ry_H = 0; double g_Rz_H = 0;
  double dJ1_H = 0; double dJ2_H = 0; double dJ3_H = 0;
  double dJ4_H = 0; double dJ5_H = 0; double dJ6_H = 0;

  //记录点J
  double g_X_J = 0; double g_Y_J = 0; double g_Z_J = 0;
  double g_Rx_J = 0; double g_Ry_J = 0; double g_Rz_J = 0;
  double dJ1_J = 0; double dJ2_J = 0; double dJ3_J = 0;
  double dJ4_J = 0; double dJ5_J = 0; double dJ6_J = 0;

  int nMoveType = 0;
  string sTcpName = "TCP_Dental";
  string sUcsName = "Base";
  double dVelocity = 10;
  double dAcc = 12;
  double dRadius = 50;
  int nIsUseJoint = 1;
  int nIsSeek = 0;
  int nIOBit = 0;
  int nIOState = 0;
  string strCmdID = "0";
  std::vector<double> dCoord = { dX, dY, dZ, dRx, dRy, dRz };

  ///////////////////////////////////导航数据模块///////////////////////////////////
  //机械臂配准数据
  double m_T_BaseToBaseRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  double m_T_FlangeToEdnRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  double m_T_FlangeToTCP[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  double m_T_calibratorRFToTCP[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  double m_T_calibratorRFToInputTCP[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  double m_T_EndRFToCalibratorRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  double m_T_EndRFToInputTCP[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  double m_T_ImageToTCP[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  double m_T_EndRFToTCP[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  vtkSmartPointer<vtkPoints> vtkProbeTip_onEndRF = vtkSmartPointer<vtkPoints>::New();

public slots:
	void get_ToolInfor();
	void CollectProbeData();

	//About print
	void PrintArray16ToMatrix(const std::string& title, double* Array);

	//Activate robotic arm and camera
	void connectArm();
	void powerOn();
	void powerOff();
	void connectCamera();
	void openCameraQtTimer();
	void updateCameraData_Dental();
	void PrintResult(QTextBrowser* browser, int nRet, const char* message);

	//Robot Registration
	void setTCPToFlange();
	void setInitialPoint();
	void goToInitial();
	void captureRobot();
	void replaceRegistration();
	void saveArmMatrix();
	void reuseArmMatrix();
	void JiontAngleMotion();
	void AutoMoveJ();
	void CapturePose(bool translationOnly);
	//Robot move
	void xp();
	void xm();
	void yp();
	void ym();
	void zp();
	void zm();
	void rxp();
	void rxm();
	void ryp();
	void rym();
	void rzp();
	void rzm();

	//Image Registration
	void calibrateTCP();
	void PrintTCP_qt(QTextBrowser* browser, double x, double y, double z, double rx, double ry, double rz);
	void saveTCPCalibrateMatrix();
	void reuseTCPCalibrateMatrix();
	void collectDitch();
	void resetImageRegistration();
	void imageRegistration();
	void saveImageregisResult();
	void reuseImageregisResult();
	void startNavigation();
	//void UpdateDrillVisual();
	void ConvertVTKPointsToMITKPointSet(vtkSmartPointer<vtkPoints> vtkPoints, mitk::PointSet::Pointer mitkPointSet);

	//navigation
	void OnAutoPositionStart();
	void PrintMatrix(std::string matrixName, double* matrix);


protected:
  virtual void CreateQtPartControl(QWidget *parent) override;

  virtual void SetFocus() override;

  Ui::DentalRobotControls m_Controls;
  unsigned int m_IndexOfRobotCapture{ 0 };
  unsigned int m_IndexOfRobotCurrents{ 0 };
  unsigned int m_IndexOfLandmark{ 0 };
  unsigned int m_IndexOfICP{ 0 };
  unsigned int move_id{ 0 };
  unsigned int move_id_test{ 0 };
  unsigned int AutoMoveJ_id{ 0 };



	// Slots

  void on_pushButton_testMoveImplant_clicked();
  bool ObtainImplantExitEntryPts(mitk::Surface::Pointer implantSurface, mitk::PointSet::Pointer exitEntryPts);
  bool CutPolyDataWithPlane(vtkSmartPointer<vtkPolyData> dataToCut,
	  vtkSmartPointer<vtkPolyData> largerSubPart,
	  vtkSmartPointer<vtkPolyData> smallerSubPart,
	  double planeOrigin[3], double planeNormal[3]);

  void on_pushButton_followAbutment_clicked();
  void on_pushButton_followCrown_clicked();
  void on_pushButton_implantTipExtract_clicked();
  void on_pushButton_implantToCrown_clicked();
  void on_pushButton_abutmentToImplant_clicked();

  void on_pushButton_planeAdjust_clicked(); // Use Gizmo

  void on_pushButton_splineAndPanorama_clicked();

  void on_pushButton_AutoGenerate_clicked();

  void on_pushButton_viewPano_clicked();

  void on_pushButton_viewCursiveMPR_clicked();

  void valueChanged_horizontalSlider();

  void on_pushButton_CBCTreconstruct_clicked();

  void on_pushButton_iosCBCT_clicked();

  void on_pushButton_setCrown_clicked();

  void on_pushButton_setImplant_clicked();

  void on_pushButton_steelballExtract_clicked();

  void on_comboBox_plate_changed(int index);

  void on_pushButton_connectVega_clicked();

  void on_pushButton_updateData_clicked();

  void on_pushButton_disaplayProbe_clicked();

  void on_pushButton_outputMatrix_clicked();

  void on_pushButton_imageRegis_clicked();

  void on_pushButton_calibrateDrill_clicked();

  void on_pushButton_saveCalibrateMatrix_clicked();

  void on_pushButton_reuseImageMatrix_clicked();

  void on_pushButton_genSplineAndAppend_clicked();

  void on_pushButton_GenSeeds_clicked();

  void on_pushButton_collectDitch_clicked();

  void on_pushButton_imageRegisNew_clicked();

  void on_pushButton_resetImageRegis_clicked();

  void on_pushButton_implantFocus_clicked();


	// U: up
	// D: down
	// R: right
	// L: left
	// clock: clockwise rotation
	// counter: counterclockwise rotation 
  void on_pushButton_U_ax_clicked();
  void on_pushButton_D_ax_clicked();
  void on_pushButton_L_ax_clicked();
  void on_pushButton_R_ax_clicked();
  void on_pushButton_counter_ax_clicked();
  void on_pushButton_clock_ax_clicked();

  void on_pushButton_U_sag_clicked();
  void on_pushButton_D_sag_clicked();
  void on_pushButton_L_sag_clicked();
  void on_pushButton_R_sag_clicked();
  void on_pushButton_counter_sag_clicked();
  void on_pushButton_clock_sag_clicked();

  void on_pushButton_U_cor_clicked();
  void on_pushButton_D_cor_clicked();
  void on_pushButton_L_cor_clicked();
  void on_pushButton_R_cor_clicked();
  void on_pushButton_counter_cor_clicked();
  void on_pushButton_clock_cor_clicked();
  
 /* T_AimToolDataResult* DentalRobot::GetNewToolData();
  void UpdateCameraToToolMatrix(T_AimToolDataResult* ToolData, const char* Name, double* aCamera2Tool, QLabel* label);*/


  // Project the implant head and tail points onto the "Panorama"
  void ProjectImplantOntoPanorama();

  double CalImplantToAlveolarNerve();

  mitk::NavigationData::Pointer GetNavigationDataInRef(mitk::NavigationData::Pointer nd,
	  mitk::NavigationData::Pointer nd_ref);

  void TurnOffAllNodesVisibility();

  void ResetView();

  void ClipTeeth();

  vtkSmartPointer<vtkPolyData> SweepLine_2Sides(vtkPolyData* line, double direction[3],
	  double distance, unsigned int cols);

  vtkSmartPointer<vtkPolyData> ExpandSpline(vtkPolyData* line, int divisionNum,
	  double stepSize);



  void UpdateAllBallFingerPrint(mitk::PointSet::Pointer stdSteelballCenters);
  double GetPointDistance(const mitk::Point3D p0, const mitk::Point3D p1);
  bool GetCoarseSteelballCenters(double steelballVoxel);
  void IterativeScreenCoarseSteelballCenters(int requiredNeighborNum, int stdNeighborNum, std::vector<int>& foundIDs);
  void ScreenCoarseSteelballCenters(int requiredNeighborNum, int stdNeighborNum, std::vector<int>& foundIDs);
  void RemoveRedundantCenters();
  void RearrangeSteelballs(int stdNeighborNum, std::vector<int>& foundIDs);


  void OnVegaVisualizeTimer();
  void ShowToolStatus_Vega();

  QString MatrixToString(vtkMatrix4x4* matrix);
  
  //void get_ToolInfor();
  //void GetNewToolData();
  T_AimToolDataResult* GetNewToolData();
  void UpdateCameraToToolMatrix(T_AimToolDataResult* ToolData, const char* Name, double* aCamera2Tool, QLabel* label);



	// Variables

  mitk::NavigationToolStorage::Pointer m_VegaToolStorage;
  mitk::TrackingDeviceSource::Pointer m_VegaSource;
  lancet::NavigationObjectVisualizationFilter::Pointer m_VegaVisualizer;
  QTimer* m_VegaVisualizeTimer{ nullptr };
  QTimer* m_AimoeVisualizeTimer{ nullptr };
  std::vector<mitk::NavigationData::Pointer> m_VegaNavigationData;
  //Aimooe
  lancet::NavigationObjectVisualizationFilter::Pointer m_AimooeVisualizer;
  mitk::NavigationToolStorage::Pointer m_AimooeToolStorage;
  mitk::TrackingDeviceSource::Pointer m_AimooeSource;

  //ProbeInteractor PIController;
  //int ProbeMarkerCnt = 0;


  // Old style Image registration
  lancet::NavigationObject::Pointer m_NavigatedImage;
  vtkNew<vtkMatrix4x4> m_ImageRegistrationMatrix; // image(surface) to ObjectRf matrix
  lancet::ApplySurfaceRegistratioinStaticImageFilter::Pointer m_SurfaceRegistrationStaticImageFilter;


  // ************* SteelBall extraction and dental splint movement Modified by Yuhan *******************
  std::vector<double> allBallFingerPrint;
  std::vector<double> stdCenters;
  int realballnumber{0};
  int edgenumber{0};

  mitk::Surface::Pointer m_splinterSurface = mitk::Surface::New();
  mitk::PointSet::Pointer m_steelBalls_cmm = mitk::PointSet::New();


  // Rewrite the image registration and navigation part without using the MITK IGT pipeline

  mitk::PointSet::Pointer m_probeDitchPset_cmm = mitk::PointSet::New();// probe ditch points from CMM
  mitk::PointSet::Pointer m_probeDitchPset_image = mitk::PointSet::New(); // probe ditch points under image frame
  //mitk::PointSet::Pointer m_probeDitchPset_rf; // probe ditch points under patientRF

  //double CalculateDistance(const mitk::Point3D& point1, const mitk::Point3D& point2);
  mitk::Point3D CalculateMassCenter(const mitk::PointSet::Pointer& pointSet);
  //bool ComparePointsByDistance(const mitk::Point3D& massCenter, const mitk::Point3D& point1, const mitk::Point3D& point2);
  void SortPointSetByDistance(mitk::PointSet::Pointer inputPointSet, mitk::PointSet::Pointer outputPointSet);
  void GenerateCombinations(int m, int n, int index, std::vector<int>& currentCombination, std::vector<std::vector<int>>& result);
  std::vector<std::vector<int>> GenerateAllCombinations(int m, int n);


  void on_pushButton_startNavi_clicked();
  void UpdateDrillVisual();

  void on_pushButton_startNaviImplant_clicked();
  void UpdateImplantAndCarrierVisual();

  // Navigation data from the camera
  double m_T_cameraToGrindingRod[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  bool m_Stat_cameraToGrindingRod{ true };
  double m_T_cameraToHandpieceRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  bool m_Stat_cameraToHandpieceRF{false};
  double m_T_cameraToPatientRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  bool m_Stat_cameraToPatientRF{false};
  double m_T_cameraToCalibratorRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  bool m_Stat_cameraToCalibratorRF{ false };
  double m_T_cameraToProbe[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  bool m_Stat_cameraToProbe{ true };
  double identityMatrix[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  

  // Image registration matrix and tool calibration matrix
  bool m_Stat_calibratorRFtoDrill{ false };
  double m_T_calibratorRFtoDrill[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 }; // from hardware design, here the drill is actually the probe
 
  bool m_Stat_GrindingRodtoDrill{ false };
  double m_T_GrindingRodtoDrill[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 }; // Perform handpiece calibration to acquire,, here the drill is actually the probe
  double m_T_GrindingRodtoInputDrill[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 }; // consider the actual length of the selected drill

  bool m_Stat_handpieceRFtoDrill{ false };
  double m_T_handpieceRFtoDrill[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 }; // Perform handpiece calibration to acquire,, here the drill is actually the probe
  double m_T_handpieceRFtoInputDrill[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 }; // consider the actual length of the selected drill

  double m_T_handpieceRFtoInputCarrier[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 }; // consider the actual length of the selected carrier
  double m_T_handpieceRFtoInputImplant[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 }; // consider the actual length of the selected implant

  double m_T_imageToInputDrill[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 }; // consider the actual length of the selected drill

  bool m_Stat_patientRFtoImage{ false };
  double m_T_patientRFtoImage[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 }; // Perform image registration to acquire  

  //用采集数据的时候转VTK矩阵使用
  float R_tran[3][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };
  float t_tran[3] = { 0.0f, 0.0f, 0.0f };
  //相机到机械基座Mark
  float R_CamToBaseRF[3][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };
  float t_CamToBaseRF[3] = { 0.0f, 0.0f, 0.0f };
  double m_T_CamToBaseRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  //相机到机械臂末端Mark
  float R_CamToEndRF[3][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };
  float t_CamToEndRF[3] = { 0.0f, 0.0f, 0.0f };
  double m_T_CamToEndRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  //相机到病人Mark
  float R_CamToPatientRF[3][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };
  float t_CamToPatientRF[3] = { 0.0f, 0.0f, 0.0f };
  double m_T_CamToPatientRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  //从病人mark到影像空间image
  double m_T_PatientRFtoImage[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  double m_T_ImageToImage_icp[16]{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
  //相机到探针的Mark
  float R_CamToProbe[3][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };
  float t_CamToProbe[3] = { 0.0f, 0.0f, 0.0f };
  double m_T_CamToProbe[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  double ProbeTop[4] = { 0.0f, 0.0f, 0.0f, 1.0f };//在相机坐标系下的探针尖端
  double ProbeTop_EndRF[4] = { 0.0f, 0.0f, 0.0f, 1.0f };//在EndRF坐标系下的探针尖端
  double nd_tip_FpatientRF[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
  double nd_tip_FImage_icp[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
  double ProofPointInit[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
  double ProofPoint[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
  double m_T_imageToProbe[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  //相机到TCP
  float R_CamToTCPRF[3][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };
  float t_CamToTCPRF[3] = { 0.0f, 0.0f, 0.0f };
  double m_T_CamToCalibratorRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  double finaltcp[6];
	//相机到金属球Mark
  float R_CamToMetalBallRF[3][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };
  float t_CamToMetalBallRF[3] = { 0.0f, 0.0f, 0.0f };
  double m_T_CamToMetalBallRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  //金属球mark到影像空间image
  double m_T_CameraToImage[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  double m_T_ImageToImage_icp_SPI[16]{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
  double m_T_ImageToMetalRF[16]{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
  double m_T_PatientRFtoImage_SPI[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  //随动功能
  double m_T_CameraToProbe1[16]{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
  double m_T_CameraToProbe2[16]{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
  double m_T_Probe1ToProbe2[16]{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
  double m_T_BaseToFlange1[16]{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
  double m_T_CamToBase[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  std::vector<double> dTargetPoint = { 0,0,0,0,0,0 };

  //机械臂执行
  double m_T_BaseToTarget[16]{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

  // 图像到TCP
  double m_T_ImageToInputTCP[16]{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

  //采集的点集
  vtkSmartPointer<vtkPoints> vtkProbeTip_onObjRf = vtkSmartPointer<vtkPoints>::New();//landmark的点集
  vtkSmartPointer<vtkPoints> vtkProbeTip_onObjRf_icp = vtkSmartPointer<vtkPoints>::New();//icp的点集

  vtkSmartPointer<vtkPoints> vtkProbeTip_onObjRf_SPI = vtkSmartPointer<vtkPoints>::New();//landmark的点集(脊柱模型)
  vtkSmartPointer<vtkPoints> vtkProbeTip_onObjRf_icp_SPI = vtkSmartPointer<vtkPoints>::New();//icp的点集(脊柱模型)

  vtkSmartPointer<vtkPoints> vtkProbeTip_Oral = vtkSmartPointer<vtkPoints>::New();//landmark的点集（口腔）
  mitk::PointSet::Pointer m_probeDitchPset_rf; // 探针在PatientRF坐标系下的点集合vtkProbeTip_Oral一样格式不一样
  vtkSmartPointer<vtkPoints> vtkProbeTip_Oral_icp = vtkSmartPointer<vtkPoints>::New();//icp的点集（口腔）

  bool ComparePointsByDistance(const mitk::Point3D& massCenter, const mitk::Point3D& point1, const mitk::Point3D& point2);
  double CalculateDistance(const mitk::Point3D& point1, const mitk::Point3D& point2);
  int callCount = 0; // 添加一个静态变量来记录函数调用次数

  void CombineRotationTranslation(float Rto[3][3], float Tto[3], vtkMatrix4x4* resultMatrix);
  

  // Error deviation and navigation mode switch
  int m_NaviMode{ 0 }; // 0 is drilling mode; 1 is implantation mode
  double m_AngleError{ 0 };
  double m_DrillTipTotalError{ 0 };
  double m_DrillTipVertiError{ 0 };
  double m_DrillTipHoriError{ 0 };

  double m_EntryTotalError{ 0 };
  double m_EntryVertiError{ 0 };
  double m_EntryHoriError{ 0 };

  double m_ApexTotalError{ 0 };
  double m_ApexVertiError{ 0 };
  double m_ApexHoriError{ 0 };

  void UpdateDeviation();

  E_ReturnValue rlt;
  QTimer* m_AimooeVisualizeTimer{ nullptr };
  LancetHansRobot m_HansRobot;
  LancetHansRobot* m_LancetHansRobot = &m_HansRobot;
};

#endif // DentalRobot_h
