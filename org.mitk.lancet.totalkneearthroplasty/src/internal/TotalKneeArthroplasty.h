/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


#ifndef TotalKneeArthroplasty_h
#define TotalKneeArthroplasty_h

#include <berryISelectionListener.h>

#include <QmitkAbstractView.h>

#include "ui_TotalKneeArthroplastyControls.h"

#include "kukaRobotDevice.h"
#include "mitkVirtualTrackingDevice.h"
#include "mitkVirtualTrackingTool.h"
#include "lancetNavigationObjectVisualizationFilter.h"
#include "lancetApplyDeviceRegistratioinFilter.h"
#include "lancetApplySurfaceRegistratioinFilter.h"
#include "lancetApplySurfaceRegistratioinStaticImageFilter.h"
#include "lancetPathPoint.h"
#include "mitkTrackingDeviceSource.h"
#include "robotRegistration.h"
#include <vtkActor.h>
#include <vtkAxesActor.h>

//Aimooe
#include "AimPositionAPI.h"
#include "AimPositionDef.h"

#include <mitkRenderWindow.h>
#include <mitkPlaneGeometry.h>

//class Registration; // Forward Declaration
//extern Registration imageRegistration; // declaration a global variable

/**
  \brief TotalKneeArthroplasty

  \warning  This class is not yet documented. Use "git blame" and ask the author to provide basic documentation.

  \sa QmitkAbstractView
  \ingroup ${plugin_target}_internal
*/
class TotalKneeArthroplasty : public QmitkAbstractView
{
  // this is needed for all Qt objects that should have a Qt meta-object
  // (everything that derives from QObject and wants to have signal/slots)
  Q_OBJECT

public:
  static const std::string VIEW_ID;


public slots:
	// Connection
	void UseVega(); // connect with "NDI vega" Tracking Device 
	void UseRuitongPro(); // connect with "RuiTong Pro" Tracking Device
	void UseAimooe(); // connect with "Aimooe" Tracking Device

	void OnVisualizeTimer(); // NDI update
	void UpdateToolStatusWidget(); // update/refresh tool status in widget
	void ShowToolStatus(); // show the position of tools.
	void UpdateProbePosition(); //  update the position of Probe tip each frame (transform the data from Camera coordinate to Image coordinate)
	void UpdateBoneGuidePosition(); //  update the position of TibiaBoneGuide each frame (transform the data from Camera coordinate to Image coordinate)
	void ProbeMouse(); // use probe like mouse.

	void CollectLandmarkProbeByTool(); // Collect landmark by checking the status the probe3_click.

	int getTargetRFIdx(); // return the selected tool/targetRF idx(ObjectRF/BoneRF/PatientRF, TibiaBoneGuideRF, DistalFemurBoneGuide, 4in1BoneGuide)  
	std::string getTargetRFName();

	// Aimooe 
	void UpdateAimooeData();
	void UpdateAimooeToolStatus();

	// 4-in-1 combox box
	void onComboBoxIndexChanged(int index);

	// precision planning plane selection
	void onComboBoxIndexChanged_precision(int index);
protected:
  virtual void CreateQtPartControl(QWidget *parent) override;

  virtual void SetFocus() override;

  /// \brief called by QmitkFunctionality when DataManager's selection has changed
  virtual void OnSelectionChanged(berry::IWorkbenchPart::Pointer source,
                                  const QList<mitk::DataNode::Pointer> &nodes) override;

  /// \brief Called when the user clicks the GUI button

  Ui::TotalKneeArthroplastyControls m_Controls;


  /**********Variables**********/


  mitk::TrackingDeviceSource::Pointer m_CameraSource; // Tracking Device Source, for NDI
  QTimer* m_VisualizeTimer{ nullptr }; // timer
  lancet::NavigationObjectVisualizationFilter::Pointer m_Visualizer;
  mitk::NavigationToolStorage::Pointer m_ToolStorage; // tool storage
  std::vector<mitk::NavigationData::Pointer> m_NavigationData; // navigation data

  QButtonGroup* radioButtonGroup{nullptr}; // radio button selection (Image, Tibia, Distal Femur, 4-in-1)
  int radioButtonID = 1; // 1: Tibia Image, 2: Femur Image 3: Tibia, 4: Distal Femur, 5: 4-in-1, 6: Plane Validator, 7: Precision Block, 8: Precision BG
  int fourInOneComboIdx = 0; // 0. AnteriorCutPlane, 1. AnteriorChamferCutPlane, 2. PosteriorCutPlane, 3. PosteriorChamferCutPlane
  std::string precPlanPlaneName = "ABF"; // ABF, ACE, BCD, ... totally 9 combinaitons

  // count the consecutive frames when the condition is met.
  int m_ConsecFrame = 0; // count the consecutive frames
  int m_ConsecFrameThreshold = 10; // threshold
  bool m_clickPermit = true;

  // data/landmarks collection
  mitk::PointSet::Pointer m_RfToProbeLandmarkPointSet{ nullptr }; // record all the landmarks under Target/Object/Patient_RF coordinate via Probe
  mitk::PointSet::Pointer m_RfToProbeICPLandmarkPointSet{ nullptr }; // for ICP, record all the landmarks under Object/Patient_RF coordinate

  // transform matrix (patientRF to Image)
  vtkSmartPointer<vtkMatrix4x4> m_T_patientRFtoImage = vtkSmartPointer<vtkMatrix4x4>::New();// { nullptr };
  vtkSmartPointer<vtkMatrix4x4> m_T_tibiaBoneGuideRFtoTBGImage = vtkSmartPointer<vtkMatrix4x4>::New(); // { nullptr };


  vtkSmartPointer<vtkAxesActor> AxesActor = { nullptr }; // for visualization, show direction.


  /* Aimooe Connection*/
  AimHandle aimHandle = { nullptr };
  E_Interface EI;
  T_AIMPOS_DATAPARA mPosDataPara;
  T_MarkerInfo markerSt;
  T_ManufactureInfo manufactureInfo;
  T_AimPosStatusInfo statusSt;
  E_ReturnValue rlt;
  /* Aimooe Connection*/


  /**********Variables**********/

  /**********Functions**********/

  // Selector Initialization
  void InitSurfaceSelector(QmitkSingleNodeSelectionWidget* widget);
  void InitPointSetSelector(QmitkSingleNodeSelectionWidget* widget);

  // connection with btn
  bool CollectProbeTipUnderTargetRF(mitk::Point3D& TargetRFToProbeTip); // collect Probe tip under TargetRF, for "CollectLandmarkProbe()" and "CollectIcpProbe()"
  void CollectLandmarkProbe(); // collect Landmarks
  void CollectIcpProbe(); // collect ICP (Iterative Closest Point) 

  void onRadioButtonToggled(QAbstractButton* button, bool checked); // radio button selection function
  void onCollectPermissionCheckButtonClicked(); // for notification
  void onCollectPermissionCheckButtonClicked_icp(); // for icp notificaiton, permission status
  void onValidatePermissionCheckButtonClicked(); // for point-to-point validation.

  void ResetLandmarkCollection(); // clear all collected landmarks.
  void ResetIcpCollection(); // clear all collected ICPs.
  void DeleteLastLandmark(); // delete last/previous landmark 
  void DeleteLastIcp(); // delete last/previous landmark
  void PrintAllLandmarks(); // print all collected landmarks
  void PrintAllIcps(); // print all collected ICP points

  void ApplySurfaceRegistration(); // landmark registration
  void ApplyICPRegistration(); // ICP registration
  vtkSmartPointer<vtkMatrix4x4> LandmarkRegistration(mitk::PointSet::Pointer srcPointset, mitk::PointSet::Pointer tarPointset, mitk::Surface::Pointer surface);
  void ErrorEvaluation();

  void GeneratePlaneByPointset(); // generate Bone Guide cutting plane (stl file) by pointset
  void CalculatePlanePoints(const mitk::Point3D& center, const mitk::Vector3D& normal, double height, double width, double point1[3], double point2[3], double point3[3]); // Calculate corner points in Plane by normals and centriod

  void onGuideNavigationCheckButtonClicked(); // enable "UpdateBoneGuidePosition" function, disable "UpdateProbePosition" function
  void ComputeTwoSurfaceError(mitk::Surface::Pointer targetPlane, mitk::Surface::Pointer cuttingPlane, vtkSmartPointer<vtkMatrix4x4> cuttingPlaneTransform); // compare bone guide cutting plane and the planning plane

  void resetView(); 

  void FourInOneAdjustCrosshair(); // adjust cross hair when selecting corresponding 4-in-1 planning plane
  void FocusOnTargetPlane(double point[3]); // set camera focus on target plane. make the view more closer.

  // Debug functions
  void PrintVtkMatrix(vtkSmartPointer<vtkMatrix4x4> matrix, QString message);
  void PrintTransformMatrix();
  
  void saveTransformMatrix();
  void reloadTransformMatrix();

  // Tab 4 precision test
  void printNaviTransformMatrix();
  void saveNaviTransformMatrix();
  void reloadNaviTransformMatrix();

  void generateLaserPointsNPlanes();
  mitk::Point3D getMitkPoint3D(double x, double y, double z); // helper function, xyz to mitk::point2d

  void generatePointSet(mitk::PointSet::Pointer pointSet, std::string pointSetName);
  void generatePlane(mitk::PointSet::Pointer pointSet, std::string pointSetName, std::string colorName = "green");
  void generateRectanglePoints(double centroid[3], double normal[3], double& width, double& height, double p1[3], double p2[3], double p3[3]);

  void calibrateWithPointID();

  void onPrecCuttingPlaneCheckButtonClicked();

  //*********Helper Function****************
  //mitk::NavigationData::Pointer GetNavigationDataInRef(mitk::NavigationData::Pointer nd,
	 // mitk::NavigationData::Pointer nd_ref); // convert the nd data to nd_ref Coordinate.

  void ComputeCenterAndNormalFromSurface(mitk::Surface::Pointer surface, double center[3], double normal[3]);
  vtkSmartPointer<vtkMatrix4x4> SurfaceToMatrix(mitk::Surface::Pointer surface);
  //void MatrixToEulerAngles(vtkSmartPointer<vtkMatrix4x4> matrix, double eulerAngles[3]);
  void CalculateEulerAnglesDifference(const double eulerAngles1[3], const double eulerAngles2[3], double difference[3]);
  void MatrixToAxialSagittalCoronal(vtkSmartPointer<vtkMatrix4x4> matrix, double axial[3], double sagittal[3], double coronal[3]);
  double CalculateAngleBetweenVectors(const double u[3], const double v[3]);
  double CalculatePointToPlaneDistance(vtkSmartPointer<vtkMatrix4x4> matrixPoint, vtkSmartPointer<vtkMatrix4x4> matrixSurface);
  double CalculatePointToPlaneDistance(double point[3], double center[3], double normal[3]);

  void CalculateVector(mitk::Surface::Pointer targetPlaneSurface, vtkSmartPointer<vtkMatrix4x4> cuttingPlaneMatrix, double normal_dir[3]);
  // visualizaion function 
  void VisualizeNormalAndCenter(vtkSmartPointer<vtkMatrix4x4> matrix);
  //void UpdateVisualizeNormalAndCenter(vtkSmartPointer<vtkMatrix4x4> matrix);

  vtkSmartPointer<vtkAxesActor> GenerateAxesActor(double axesLength, vtkSmartPointer<vtkMatrix4x4> matrix, double normal_dir[3]);

  void UpdateAxesActor(double axesLength, vtkSmartPointer<vtkAxesActor> actor, vtkSmartPointer<vtkMatrix4x4> matrix, double normal_dir[3]);
  void UpdateAxesActor(double axesLength, vtkSmartPointer<vtkAxesActor> actor, double center[3], double normal_dir[3]);
  void AddActor(mitk::IRenderWindowPart* renderWindowPart3D, vtkSmartPointer<vtkAxesActor> actor);
  vtkSmartPointer<vtkActor> AddArrowActor();

  // visualize dis, pitch, roll
  void VisualizePlaneAngle(mitk::Surface::Pointer ref, mitk::Surface::Pointer surface, double& angle, double dis = 0);

  /**********Functions**********/
};

#endif // TotalKneeArthroplasty_h
