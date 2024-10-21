/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


#ifndef HTOrobot_h
#define HTOrobot_h

#include <berryISelectionListener.h>

//Hans
#include <HR_Pro.h>
//Aimooe
#include "AimPositionAPI.h"
#include "AimPositionDef.h"
//Robot Registration
#include "robotRegistration.h"
//QT
#include <qfiledialog.h>
#include <qtimer.h>
#include <QmitkAbstractView.h>
#include <mutex>
#include <QMessageBox>
//vtk
#include <vtkMath.h>
#include "PrintDataHelper.h"
#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkMatrix3x3.h>
#include "ui_HTOrobotControls.h"
#include <vtkPolyData.h>
#include <vtkRenderWindow.h>
//#include <QtCore\qmath.h>
#include <Eigen/Dense>
//mitk
#include "mitkPointSet.h"
#include <vtkAxesActor.h>
#include "QmitkRenderWindow.h"
#include "QmitkDataStorageTreeModel.h"
#include "mitkImageToSurfaceFilter.h"
#include "surfaceregistraion.h"

/**
  \brief HTOrobot

  \warning  This class is not yet documented. Use "git blame" and ask the author to provide basic documentation.

  \sa QmitkAbstractView
  \ingroup ${plugin_target}_internal
*/
class HTOrobot : public QmitkAbstractView
{
  // this is needed for all Qt objects that should have a Qt meta-object
  // (everything that derives from QObject and wants to have signal/slots)
  Q_OBJECT
public slots:
	void upDateData();
	void getToolInfor();
	void GetMatrix();
	void startSawPositionCapture();
	void captureSawPosition();
	void startNavigationGuidePosition();
	bool judgeTibiaShowPlanPosition();
	void navigationGuidePosition();

	void captureMalleolusPoint();
	void testProximalPoint();
	void testDistalPoint();
	void testSawPoint();
	void collectProximalStaplePoint();
	void collectDistalStaplePoint();
	void collectSawStaplePoint();
	void reuseMatrix();

	//HTOPrePlan
	bool CutTibia(); // cut tibia image and surface
	void ShowLine();
	void UnShowLine();
	void CaculateStrechAngle();
	void on_pushButton_addGizmo_clicked();
	bool OnFemurCenterClicked();
	void navigationCutTibia();
	void startNavigationStrech();
	void strechAngleNavigation();
	bool OnCaculateProbeClicked();
	void MoveRobotTCP();
	void modifyPointCoordinate();

	//zzx
	bool SetPlanePrecisionTestTcp();
	bool InterpretImagePlane();
	void On_pushButton_goToFakePlane_clicked();
	void OnAutoPositionStart();
	//ICP
	void collectLandmark();
	void collectICP_SPI();
	void ICPRegistration();
	void collectICP();
	void landmarkRegistration();
	void ResetImageConfiguration();
	void saveImageMatrix();
	void reuseImageMatrix();
	
public:
  static const std::string VIEW_ID;

  //Hans initialization
  void connectHans();
  void HansPowerOn();
  void HansPowerOFF();

  void HandGuiding();
  void closeHandGuiding();
  void SetTcpToFlange();

  void suddenStop();
  //Aimooe  initialization
  void connectAimooe();

  void getBaseToFlangeMatrix();
  //robot Registration
  void setInitializationPoint();
  void gotoInitialization();
  void CombineRotationTranslation(float Rt[3][3], float Tto[3], vtkMatrix4x4* resultMatrix);

  void captureRobot();
  void CapturePose(bool translationOnly);
  void ReplaceRegistration();
  void reuseRegistration();
  void saveRegistration();
  void PrintToolMatrix();


  //Robot Move
  void xp();
  void yp();
  void zp();
  void xm();
  void ym();
  void zm();
  void rxp();
  void ryp();
  void rzp();
  void rxm();
  void rym();
  void rzm();

  void OstGuidCalibration();


  vtkSmartPointer<vtkMatrix4x4> CalculateTFlange2Camera(double* TF2ENDRF, double* TCamera2EndRF);
  Eigen::Vector3d CalculatePointInFlangePos(vtkMatrix4x4* matrixFlange2Camera, Eigen::Vector3d posInCamera);
  Eigen::Vector3d CalculatePointProjectInLine(Eigen::Vector3d P, Eigen::Vector3d A, Eigen::Vector3d B);
  vtkSmartPointer<vtkMatrix4x4> GetArray2vtkMatrix(double* array16);

  void GetProbeEndPosBtnClicked(int type);
  void ResetProbeEndPosBtnClicked(int type);
  void GetGuiderOriginPosBtnClicked();
  void CalculateGuideTCP();
  std::vector<Eigen::Vector3d> probeEndOneVector;
  std::vector<Eigen::Vector3d> probeEndTwoVector;

  void StartDisplayTCPAxesActor();
  void UpdateDisplayRobotTCP();
  void PrintTCP(std::string tcpName, double x, double y, double z, double rx, double ry, double rz);
  Eigen::Matrix3d EulerAnglesToRotationMatrix(double alpha, double beta, double gamma);
  vtkSmartPointer<vtkMatrix4x4> GetMatrixByRotationAndTranslation(Eigen::Matrix3d rotation, Eigen::Vector3d translation);
  vtkSmartPointer<vtkMatrix4x4> endMatrix;
  QTimer* m_RobotTCPAxesTimer = nullptr;
  QTimer* m_timer_navigationGuidePosition{ nullptr };
  bool m_IsDisplayRobotTCP = false;
  int ProbEndCountOne = 0;
  int ProbEndCountTwo = 0;
  vtkSmartPointer<vtkAxesActor> m_TCPAxesActor;

  void PrintArray16ToMatrix(const std::string& title, double* Array);
  void AppendTextBrowerArray(const char* text, std::vector<double> array);
  void AppendTextBrowerArray(const char* text, double* array, int size);
  void CoutTextBrowerArray(const char* text, double* array, int size);
  void AppendTextBrowerArray(const char* text, Eigen::Vector3d array);



  //ImageRegistration
  void getImageBallcenter();
  void tibiaRegistration();
  void saveTibiaRegistration();
  void getPointOnSawRF();
  void sawStlRegistration();
  void getSawError();
  double GetRegisrationRMS(mitk::PointSet* points, mitk::Surface* surface, vtkMatrix4x4* matrix);
  vtkSmartPointer<vtkPoints> TransformVTKPoints(vtkPoints* in, vtkMatrix4x4* m);
  //strech Angle Navigation
  void navigationSawCut();
  void showPlanCutPlanePosition();
  void showPlanCutPlanePositionL();

  void getPointOnPatientRF();
  void distalTibia();
  void bindNormalToTibiaRF();
  void getStrechT();
  void getStrechT_realtime();
  void updatePowerLine();
 
  void updateProportation_ReallTime(mitk::DataNode::Pointer existingForceLineNode);
  //Post-operative verification
  void PostOperativeVerification();
  void verifyPoint(vtkSmartPointer<vtkMatrix4x4> T_CamToRF, mitk::Point3D& outputPoint);

protected:
  virtual void CreateQtPartControl(QWidget *parent) override;
  vtkSmartPointer<vtkMatrix4x4> ComputeTransformMartix(mitk::PointSet::Pointer points_set1,
	  mitk::PointSet::Pointer points_set2);
  mitk::DataNode::Pointer saw_DataNode; // Member variable to store the DataNode
  mitk::DataNode::Pointer sawNode1 ;
  mitk::DataNode::Pointer m_DataNode;
  mitk::DataNode::Pointer ankle_DataNode;
  mitk::DataNode::Pointer distalTibiaNode;
  virtual void SetFocus() override;
  void InitPointSetSelector(QmitkSingleNodeSelectionWidget* widget);
  void InitSurfaceSelector(QmitkSingleNodeSelectionWidget* widget);
  /// \brief called by QmitkFunctionality when DataManager's selection has changed
  virtual void OnSelectionChanged(berry::IWorkbenchPart::Pointer source,
                                  const QList<mitk::DataNode::Pointer> &nodes) override;

  /// \brief Called when the user clicks the GUI button
  void DoImageProcessing();
  void ReadPositionSawOnMetalBallRF_realtime();
  void getStrechAngleAndHeight();
 


  void CreateCoordinateSystemMatrix(const Eigen::Vector3d& xAxis, const Eigen::Vector3d& yAxis, const Eigen::Vector3d& zAxis, const Eigen::Vector3d& origin, vtkMatrix4x4* resultMatrix);
  void applyTransformMatrix(vtkMatrix4x4* transMatrix);
  void guideToObjPosition();
  Ui::HTOrobotControls m_Controls;



  // ----HTOPrePlan
  //Cut tibia into two parts with a plane 
  double distance1;//截骨面入口距内侧平台
  double distance2;//截骨面末端距外侧平台
  double distance3;//保留外侧合页
  double depth;//截骨面入骨深度
  double Line_length;//力线的长度
  double angleInDegrees;//撑开角度
  int judgModel_flag = 1;//0：左腿，1:右腿
  bool CreateOneCutPlane();
  bool CreateCutPlane(); // create one or two cut planes

  bool CutPolyDataWithPlane(vtkSmartPointer<vtkPolyData> dataToCut,
	  vtkSmartPointer<vtkPolyData> largerSubPart,
	  vtkSmartPointer<vtkPolyData> smallerSubPart,
	  double planeOrigin[3], double planeNormal[3]);
  std::pair<Eigen::Vector3d, double> FitSphere(const std::vector<Eigen::Vector3d>& points);
  mitk::PointSet::Pointer ballCenters_ballRF = mitk::PointSet::New();
  bool CutTibiaWithOnePlane(); // cut tibia surface with one plane
  bool CutTibiaWithTwoPlanes(); // cut tibia surface with two planes
  bool CutTibiaSurface(); // cut tibia surface with one or two planes

  bool CutTibiaImage(); // cut tibia image with one or two planes

 

  bool GetPlaneProperty(vtkSmartPointer<vtkPolyData> plane, double normal[3], double center[3]);

  // Register femur, proximal tibia and distal tibia
  bool RegisterFemur();
  bool RegisterPoximalTibia();
  bool RegisterDistalTibia();


  // Toolset 1 (Intuitive)
  void Translate(double direction[3], double length, mitk::BaseData* data);
  void Rotate(double center[3], double direction[3], double counterclockwiseDegree, mitk::BaseData* data);
  void TranslatePlusX();
  void TranslatePlusY();
  void TranslatePlusZ();
  void TranslateMinusX();
  void TranslateMinusY();
  void TranslateMinusZ();
  void RotatePlusX();
  void RotatePlusY();
  void RotatePlusZ();
  void RotateMinusX();
  void RotateMinusY();
  void RotateMinusZ();

  //void TranslatePlus();
  //void TranslateMinus();
  void RotatePlus();
  void RotateMinus();
  void PercentageofInterpoint();
  // Test cutting
  mitk::DataNode::Pointer m_growingCutterNode{ nullptr };
  void TestCut();
  void TestCut2();
  void updateProportation();
  // Variables
  bool GetIntersectionLine();
  void GetDistancefromTibia();
  bool calculateIntersection(double p1[3], double p2[3], double p3[3], double p4[3], double intersection[3]);
  QmitkDataStorageTreeModel* m_NodetreeModel{ nullptr };
  mitk::BaseData* m_baseDataToMove{ nullptr };
  mitk::DataNode* m_currentSelectedNode{ nullptr };
  mitk::PointSet::Pointer mitkPointSet1 = mitk::PointSet::New();
  mitk::PointSet::Pointer planeAndTibiaIntersectionPointSet;//存放规划面与胫骨的交点
 /* mitk::PointSet::Pointer mitkPointSet2 = mitk::PointSet::New();*/
  mitk::Point3D minPoint;//截骨线末端中点
  mitk::Point3D maxPoint;//截骨线入口点

  void TraverseIntersectionLines(vtkSmartPointer<vtkPolyData> intersectionLine);
  void getPointOnDaoban();
  void CaculateStrechHeigh();

public:
	//初始化机械臂配准的类
	RobotRegistration hto_RobotRegistration;
	mitk::DataNode* m_IcpSourceSurface{ nullptr };



	//Hans Declare variables
	unsigned int boxID = 0;
	unsigned int rbtID = 0;
	unsigned short nPort = 10003;
	char* hostname = "192.168.0.10";
	//默认使用关节运动
	int nIsUseJoint = 1;
	int nIsSeek = 0;
	int nIOBit = 0;
	int nIOState = 0;
	string strCmdID = "0";
	int nMoveType = 0;
	string sTcpName = "TCP_1";
	string sUcsName = "Base";
	double dVelocity = 20;
	double dAcc = 50;
	double dRadius = 50;
	int nToolMotion = 0;
	int nAxisID = 0;
	int nDirection = 0;

	/*
	由于大族机械臂api所需定义变量众多，可以用数组的方式例如 vectory<double> position;
	*/
	// 定义空间位置变量
	double dX = 0; double dY = 0; double dZ = 0;
	double dRx = 0; double dRy = 0; double dRz = 0;
	// 定义关节位置变量
	double dJ1 = 0; double dJ2 = 0; double dJ3 = 0;
	double dJ4 = 0; double dJ5 = 0; double dJ6 = 0;
	// 定义工具坐标变量
	double dTcp_X = 0; double dTcp_Y = 0; double dTcp_Z = 0;
	double dTcp_Rx = 0; double dTcp_Ry = 0; double dTcp_Rz = 0;
	// 定义用户坐标变量
	double dUcs_X = 0; double dUcs_Y = 0; double dUcs_Z = 0;
	double dUcs_Rx = 0; double dUcs_Ry = 0; double dUcs_Rz = 0;
	// 初始的函数
	double g_init_X = 0; double g_init_Y = 0; double g_init_Z = 0;
	double g_init_Rx = 0; double g_init_Ry = 0; double g_init_Rz = 0;
	double dJ1_init = 0; double dJ2_init = 0; double dJ3_init = 0;
	double dJ4_init = 0; double dJ5_init = 0; double dJ6_init = 0;


	int callCount = 0; // 添加一个静态变量来记录函数调用次数
protected: 
	E_ReturnValue rlt;
	QTimer* m_AimoeVisualizeTimer{ nullptr };


	unsigned int hto_IndexOfRobotCapture{ 0 };

	double tcp[6]{0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

	//机械臂配准数据
	double T_BaseToBaseRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	double T_FlangeToEndRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	//用采集数据的时候转VTK矩阵使用
	float R_tran[3][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };
	float t_tran[3] = { 0.0f, 0.0f, 0.0f };

	//相机到机械台车Marker
	float R_CamToBaseRF[3][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };
	float t_CamToBaseRF[3] = { 0.0f, 0.0f, 0.0f };
	double T_CamToBaseRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	
	//相机到病人胫骨柄Mark
	double T_CamToPatientRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 }; 
	//相机到机械臂末端Mark
	double T_CamToEndRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	//相机到探针
	double T_CamToProbe[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	//相机到摆锯末端
	double T_CamToSaw[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	//相机到病人胫骨近端Marker
	double T_CamToMetalBallRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	//相机到探针Marker
	double ProbeTop[4] = { 0.0f, 0.0f, 0.0f, 1.0f };//在探针坐标系下的探针尖端
	double Cam_ProbeTop[4] = { 0.0f, 0.0f, 0.0f, 1.0f };//在相机坐标系下的探针尖端
	//从病人mark到探针
	double T_PatientRFToProbe[16]{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
	//从病人mark到影像空间image
	double T_PatientRFtoImage[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	double T_ImageToImage_icp[16]{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
	double nd_tip_FpatientRF[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
	double nd_tip_FImage_icp[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
	double ProofPointInit[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
	double ProofPoint[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
	double T_imageToProbe[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	//double T_SawRFtoImage[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	//ICP
	unsigned int m_IndexOfLandmark{ 0 };
	unsigned int m_IndexOfICP{ 0 };

	

	//胫骨配准矩阵
	vtkSmartPointer<vtkMatrix4x4> T_imageFrameToSteelballRF;//金属球配准的矩阵
	vtkSmartPointer<vtkMatrix4x4> T_PatientRFToimageFrame;//胫骨柄到影像空间参考阵列的转换矩阵
	vtkSmartPointer<vtkMatrix4x4> T_strechAngleNavigation;//撑开过程中的转换矩阵
	vtkSmartPointer<vtkMatrix4x4> T_MetalRFtoPatientRF;//金属球阵列到胫骨柄阵列的转换矩阵
	vtkSmartPointer<vtkMatrix4x4> T_MetalRFtoPatientRF_realTime;//金属球阵列到胫骨柄阵列的转换矩阵
	vtkSmartPointer<vtkMatrix4x4> m_vtkregistrationMatrix_Saw;//锯片配准的矩阵
	vtkSmartPointer<vtkMatrix4x4> TcpMatrix;//tcp矩阵
	vtkSmartPointer<vtkMatrix4x4> T_BaseToTool;//台车基座到导板tcp；
	vtkSmartPointer<vtkMatrix4x4> T_toolToPlanPlane;//图像空间导板stl到规划位；
	vtkSmartPointer<vtkMatrix4x4> VTKT_BaseTotool_read;
	//采集的点集
	vtkSmartPointer<vtkPoints> vtkProbeTip_onObjRf = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> shieldProbeTip_onObjRf = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> pointOnSawRF_onImage = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> pointOnSawRF_onMetalRF = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> vtkProbeTip_onObjRf_icp = vtkSmartPointer<vtkPoints>::New();//icp的点集
	vtkSmartPointer<vtkPoints> vtkProbeTip_onObjRf_icp_SPI = vtkSmartPointer<vtkPoints>::New();//icp的点集(脊柱模型)


	mitk::PointSet::Pointer m_PointSet_saw;
	mitk::PointSet::Pointer m_PointSet_ankle;
	mitk::Point3D proximalStaplePoint;//胫骨近端验证点
	mitk::Point3D distalStaplePoint;//胫骨远端验证点
	mitk::Point3D SawTestPoint;//胫骨远端验证点
	mitk::DataNode::Pointer daobanNode1;
	//锯片上特征点在摆锯末端阵列下的坐标
	double PointOnSawRF[3];
	double PointOnMetalRF[3];//锯片上特征点在metalRF下的坐标；
	int clickCount = 0; // 初始化计数器为0
	double ankleCenterOnPatientRF[4] = { 0.0, 0.0, 0.0, 0.0 };//踝关节中心点在胫骨柄参考阵列下的坐标
	double ankleCenterOnPatientRFtoImagePoint[4] = { 0.0, 0.0, 0.0, 0.0 };
	QTimer* m_timer_saw{ nullptr };
	QTimer* m_timer_strech{ nullptr };
	double normal[3];//，锯片截骨时最后状态下锯片平面法向量

	double normalOnPatientRF[4]= { 0.0, 0.0, 0.0, 0.0 };
	
	double normalOnPatientRFtoImage[4]= { 0.0, 0.0, 0.0, 0.0 };

	//zzx
	/*vtkSmartPointer<vtkMatrix4x4> T_BaseToTool;*/
	double T_X = 0; double T_Y = 0; double T_Z = 0;
	double T_Rx = 0; double T_Ry = 0; double T_Rz = 0;
	double dJ1_1 = 0; double dJ2_1 = 0; double dJ3_1 = 0;
	double dJ4_1 = 0; double dJ5_1 = 0; double dJ6_1 = 0;
	//用来打印矩阵的
	void PrintMatrix(std::string matrixName, double* matrix);
	void Print_Matrix(const std::string& title, vtkMatrix4x4* matrix);
	//常量定义
	const double pi = 3.14159265358979323846;
	void PrintTCP_qt(QTextBrowser* browser, double x, double y, double z, double rx, double ry, double rz);
	//系统精度测试
	vtkMatrix4x4* vtkBaseToTargetPlaneTransform;
	void computeAverageTransform(vtkMatrix4x4* samples[], vtkMatrix4x4* average);
	Eigen::Matrix4d vtkMatrix4x4ToEigen(const vtkMatrix4x4* vtkMatrix);
	vtkMatrix4x4* eigenToVtkMatrix4x4(const Eigen::Matrix4d& eigenMatrix);
	const double radius2degree = 57.2957805;

public:
	void textEditProcess(double num[6], QLineEdit* LineEdit);
	void PrintResult(QTextBrowser* browser, int nRet, const char* message);
	void textEditProcess(int num[6], QLineEdit* LineEdit);
public slots:
	void movetcpline();
	void tcp2homoguous(double tcp[], vtkSmartPointer<vtkMatrix4x4>& VTK_BaseToFlange);
};

#endif // HTOrobot_h
