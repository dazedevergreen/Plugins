#ifndef TKAN_H
#define TKAN_H

// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qmitk
//#include "TotalKneeArthroplasty.h"

// Qt
#include <QMessageBox>
#include <QRadioButton>
#include <QButtonGroup>

// mitk image
#include <mitkImage.h>
#include <mitkAffineTransform3D.h>
#include <mitkMatrixConvert.h>
#include <mitkPoint.h>
#include <mitkSurface.h>
#include <QmitkRenderWindow.h>
#include <mitkIRenderWindowPart.h>

// vtk
#include <vtkSmartPointer.h>
#include <vtkPlaneSource.h>
#include <vtkSTLWriter.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkCenterOfMass.h>
#include <vtkPolyDataNormals.h>
#include <vtkCellData.h>
#include <vtkImplicitPolyDataDistance.h>
#include <vtkArrowSource.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkTransform.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>

//igt
#include <lancetVegaTrackingDevice.h>
#include <lancetApplyDeviceRegistratioinFilter.h>
#include <mitkNavigationDataToPointSetFilter.h>
#include <vtkQuaternion.h>

#include "lancetTrackingDeviceSourceConfigurator.h"
#include "mitkNavigationToolStorageDeserializer.h"
#include <QtWidgets\qfiledialog.h>

#include "mitkIGTIOException.h"
#include "mitkNavigationToolStorageSerializer.h"
#include "QmitkIGTCommonHelper.h"
#include "lancetTreeCoords.h"

class TKAN {
public:
	TKAN();

	/****** Registration ******/

	/// Landmark Registration
	/// <summary>
	/// Tibia Image Registration.
	/// </summary>
	/// <param name="srcPointset">The loaded Source Pointset</param>
	/// <param name="tarPointset">The collected Pointset by Probe</param>
	/// <param name="surface">Target Object Surface(stl file), ex: bone image, bone guide image</param>
	void TibiaImageRegistration(mitk::PointSet::Pointer srcPointset, mitk::PointSet::Pointer tarPointset, mitk::Surface::Pointer surface); // Bone Image
	void FemurImageRegistration(mitk::PointSet::Pointer srcPointset, mitk::PointSet::Pointer tarPointset, mitk::Surface::Pointer surface); // Bone Image
	void TibiaBoneGuideRegistration(mitk::PointSet::Pointer srcPointset, mitk::PointSet::Pointer tarPointset, mitk::Surface::Pointer surface); // Tibia Bone Guide
	void FemurBoneGuideRegistration(mitk::PointSet::Pointer srcPointset, mitk::PointSet::Pointer tarPointset, mitk::Surface::Pointer surface); // Femur Bone Guide
	void FourInOneBoneGuideRegistration(mitk::PointSet::Pointer srcPointset, mitk::PointSet::Pointer tarPointset, mitk::Surface::Pointer surface); // 4-in-1 Bone Guide
	void PlaneValidatorRegistration(mitk::PointSet::Pointer srcPointset, mitk::PointSet::Pointer tarPointset, mitk::Surface::Pointer surface); // Plane Validator
	void PrecisionBlockRegistration(mitk::PointSet::Pointer srcPointset, mitk::PointSet::Pointer tarPointset, mitk::Surface::Pointer surface); // precision block 
	void PrecisionBGRegistration(mitk::PointSet::Pointer srcPointset, mitk::PointSet::Pointer tarPointset, mitk::Surface::Pointer surface); // precision Test Bone Guide

	// for precision, laser block calibration.
	void PBLaserRegistration(mitk::PointSet::Pointer srcPointset, mitk::PointSet::Pointer tarPointset, mitk::Surface::Pointer surface);

	// ICP (Iterative Closest Point) Registration, for refine the Transform Matrix "T_RFToImage"
	void TibiaImageICP(mitk::PointSet::Pointer dstPointset, mitk::Surface::Pointer surface); // Bone Image
	void FemurImageICP(mitk::PointSet::Pointer dstPointset, mitk::Surface::Pointer surface); // Bone Image
	void TibiaBoneGuideICP(mitk::PointSet::Pointer dstPointset, mitk::Surface::Pointer surface); // Tibia Bone Guide
	void FemurBoneGuideICP(mitk::PointSet::Pointer dstPointset, mitk::Surface::Pointer surface); // Femur Bone Guide
	void FourInOneBoneGuideICP(mitk::PointSet::Pointer dstPointset, mitk::Surface::Pointer surface); // 4-in-1 Bone Guide
	void PlaneValidatorICP(mitk::PointSet::Pointer dstPointset, mitk::Surface::Pointer surface); // Plane Validator
	void PrecisionBlockICP(mitk::PointSet::Pointer dstPointset, mitk::Surface::Pointer surface); // Precision Block
	void PrecisionBoneGuideICP(mitk::PointSet::Pointer dstPointset, mitk::Surface::Pointer surface); // Precision Bone Guide

	/****** Navigation ******/

	// Bone Guide Navigation in Target Bone.
	vtkSmartPointer<vtkMatrix4x4> TibiaBoneGuideNavigation(const double nd_CameraToBoneGuideRF[16], const double nd_CameraToBoneRF[16]);
	vtkSmartPointer<vtkMatrix4x4> FemurBoneGuideNavigation(const double nd_CameraToBoneGuideRF[16], const double nd_CameraToBoneRF[16]);
	vtkSmartPointer<vtkMatrix4x4> FourInOneBoneGuideNavigation(const double nd_CameraToBoneGuideRF[16], const double nd_CameraToBoneRF[16]);
	vtkSmartPointer<vtkMatrix4x4> PrecisionBoneGuideNavigation(const double nd_CameraToBoneGuideRF[16], const double nd_CameraToBoneRF[16]);

	/// <summary>
	/// Probe Navigation in Target Image.
	/// </summary>
	/// <param name="nd_CameraToProbe"> the navigation data of Probe RF (marker) </param>
	/// <param name="nd_CameraToBoneRF"> the navigation data of Tibia Bone RF (marker) </param>
	/// <returns> the position of Probe in Tibia Bone Image </returns>
	vtkSmartPointer<vtkMatrix4x4> ProbeNavigationInTibiaBone(const double nd_CameraToProbe[16], const double nd_CameraToTargetRF[16]);
	vtkSmartPointer<vtkMatrix4x4> ProbeNavigationInFemurBone(const double nd_CameraToProbe[16], const double nd_CameraToTargetRF[16]);
	vtkSmartPointer<vtkMatrix4x4> ProbeNavigationInTibiaBG(const double nd_CameraToProbe[16], const double nd_CameraToTargetRF[16]);
	vtkSmartPointer<vtkMatrix4x4> ProbeNavigationInFemurBG(const double nd_CameraToProbe[16], const double nd_CameraToTargetRF[16]);
	vtkSmartPointer<vtkMatrix4x4> ProbeNavigationInFourInOneBG(const double nd_CameraToProbe[16], const double nd_CameraToTargetRF[16]);
	vtkSmartPointer<vtkMatrix4x4> ProbeNavigationInPlaneValidator(const double nd_CameraToProbe[16], const double nd_CameraToTargetRF[16]);
	vtkSmartPointer<vtkMatrix4x4> ProbeNavigationInPrecisionBlock(const double nd_CameraToProbe[16], const double nd_CameraToTargetRF[16]);
	vtkSmartPointer<vtkMatrix4x4> ProbeNavigationInPrecisionBG(const double nd_CameraToProbe[16], const double nd_CameraToTargetRF[16]);

	// Navigate Plane Validator in two Bone Image (Tibia OR Femur)
	vtkSmartPointer<vtkMatrix4x4> PVNavigationInTibiaBone(const double nd_CameraToPV[16], const double nd_CameraToBoneRF[16]);
	vtkSmartPointer<vtkMatrix4x4> PVNavigationInFemurBone(const double nd_CameraToPV[16], const double nd_CameraToBoneRF[16]);


	/***** Bone Guides/Plane Validator Validation, Error Computation(p2p) ******/

	/// <summary>
	/// Bone Guide/Plane Validator Validation
	/// </summary>
	/// <param name="nd_CameraToProbe">nd_CameraToProbe, Probe tip position</param>
	/// <param name="nd_CameraToBoneGuideRF">nd_CameraToBoneGuideRF, Target Bone Guide RF/maker position </param>
	void TibiaBGValidation(const double nd_CameraToProbe[16], const double nd_CameraToBoneGuideRF[16]);
	void FemurBGValidation(const double nd_CameraToProbe[16], const double nd_CameraToBoneGuideRF[16]);
	void FourInOneBGValidation(const double nd_CameraToProbe[16], const double nd_CameraToBoneGuideRF[16]);
	void PlaneValidatorValidation(const double nd_CameraToProbe[16], const double nd_CameraToBoneGuideRF[16]);
	void PrecisionBlockValidation(const double nd_CameraToProbe[16], const double nd_CameraToBoneGuideRF[16]);
	void PrecisionBGValidation(const double nd_CameraToProbe[16], const double nd_CameraToBoneGuideRF[16]);


	/*** Surface Error Calculation, For Navigation/Guiding; ***/
	// Compute ERROR(distance, euler angle) of 
	// the Cutting Plane of the Target Bone Guide(Tibia,  Frmur, 4-in-1)
	// and the Planning Plane of the Target Bone (Tibia, Femur)

	/// <summary>
	/// Computes the error between Bone Guide Cutting Plane and Bone Planning Plane.
	/// </summary>
	/// <param name="planningPlaneSurface">The Target Bone planning plane surface. (const) </param>
	/// <param name="cuttingPlaneSurface">The cutting plane surface. (const) </param>
	/// <param name="T_BGImageToBoneImage"> the transformed matrix after do Bone Guide Navigation </param>
	/// <returns>tuple, ***** {distance, surface_error, pitch, roll} *****</returns>
	//std::tuple<double, double, double> ComputeNavigationError(mitk::Surface::Pointer planningPlaneSurface, mitk::Surface::Pointer cuttingPlaneSurface, vtkSmartPointer<vtkMatrix4x4> T_BGImageToBoneImage); // BG: Bone Guide;
	std::tuple<double, double, double, double> ComputeTibiaBGSurfaceError(mitk::Surface::Pointer planningPlaneSurface, mitk::Surface::Pointer cuttingPlaneSurface, vtkSmartPointer<vtkMatrix4x4> T_BGImageToBoneImage); // BG: Bone Guide;
	std::tuple<double, double, double, double> ComputeFemurBGSurfaceError(mitk::Surface::Pointer planningPlaneSurface, mitk::Surface::Pointer cuttingPlaneSurface, vtkSmartPointer<vtkMatrix4x4> T_BGImageToBoneImage);
	std::tuple<double, double, double, double> ComputeFourInOneBGSurfaceError(mitk::Surface::Pointer planningPlaneSurface, mitk::Surface::Pointer cuttingPlaneSurface, vtkSmartPointer<vtkMatrix4x4> T_BGImageToBoneImage);
	std::tuple<double, double> ComputePlaneValidatorSurfaceError(mitk::Surface::Pointer planningPlaneSurface, mitk::Surface::Pointer cuttingPlaneSurface, vtkSmartPointer<vtkMatrix4x4> T_BGImageToBoneImage);
	std::tuple<double, double, double, double, double> ComputePrecisionBGSurfaceError(mitk::Surface::Pointer planningPlaneSurface, mitk::Surface::Pointer cuttingPlaneSurface, vtkSmartPointer<vtkMatrix4x4> T_BGImageToBoneImage);
	
	// for computing the point-to-surface distance errror;
	std::tuple<double, double, double> ComputePointToSurfaceDisError(std::vector<std::array<double, 3>>& planningPlanePoints, std::vector<std::array<double, 3>>& cuttingPlanePoints, vtkSmartPointer<vtkMatrix4x4> T_BGImageToBoneImage);
	std::tuple<double, double, double> ComputePointToSurfaceDisError(mitk::PointSet::Pointer planningPlanePointset, mitk::PointSet::Pointer cuttingPlanePointset, vtkSmartPointer<vtkMatrix4x4> T_BGImageToBoneImage);
	/****** Set/Get Functions,  set Variables, for Test/Debug ******/
	
	// Set Transform Matrix.
	void SetTibiaRFtoImageTrans(vtkSmartPointer<vtkMatrix4x4> matrix); // set T_TibiaRFtoImage
	void SetFemurRFtoImageTrans(vtkSmartPointer<vtkMatrix4x4> matrix); // set T_FemurRFtoImage
	void SetTBGRFtoTBGImageTrans(vtkSmartPointer<vtkMatrix4x4> matrix); // set T_TibiaBoneGuideRFtoTBGImage
	void SetFBGRFtoFBGImageTrans(vtkSmartPointer<vtkMatrix4x4> matrix); // set T_FemurBoneGuideRFtoFBGImage
	void SetFIOBGRFtoFIOBGImageTrans(vtkSmartPointer<vtkMatrix4x4> matrix); // set T_FourInOneBoneGuideRFtoFIOBGImage
	void SetPVRFtoPVImageTrans(vtkSmartPointer<vtkMatrix4x4> matrix); // set T_PlaneValidatorRFtoPVImage
	void SetPBRFtoPBImageTrans(vtkSmartPointer<vtkMatrix4x4> matrix); // set T_PrecisionBlockRFtoPBImage
	void SetPBGRFtoPBGImageTrans(vtkSmartPointer<vtkMatrix4x4> matrix); // set T_PrecisionBoneGuideRFtoPBGImage

	void SetPBGtoTargetImageTrans(vtkSmartPointer<vtkMatrix4x4> matrix); // set T_PBGtoTargetImage, navi trans Mtx
	void SetPBGNaviMtxInTargetPos(vtkSmartPointer<vtkMatrix4x4> matrix); // set T_PBGNaviMtxInTargetPos, the PBG navigation transform matrix when cutting plane in the target plane/pos 

	void SetPBLasertoPBImageTrans(vtkSmartPointer<vtkMatrix4x4> matrix); // set T_PBLasertoPBImage

	// Set/Init the Bone Guides/Plane Validator Surface Source POINTSET
	void SetTibiaBGSrcPointset(mitk::PointSet::Pointer srcPointset); // set the Tibia Bone Guide Surface Source Pointset for Calibratrion/Registration
	void SetFemurBGSrcPointset(mitk::PointSet::Pointer srcPointset);
	void SetFourInOneBGSrcPointset(mitk::PointSet::Pointer srcPointset);
	void SetPlaneValidatorSrcPointset(mitk::PointSet::Pointer srcPointset);
	void SetPrecisionBlockSrcPointset(mitk::PointSet::Pointer srcPointset);
	void SetPrecisionBGSrcPointset(mitk::PointSet::Pointer srcPointset);

	// Get the Transform Matrix
	vtkSmartPointer<vtkMatrix4x4> GetTibiaRFtoImageTrans(); // T_TibiaRFtoImage
	vtkSmartPointer<vtkMatrix4x4> GetFemurRFtoImageTrans(); // T_FemurRFtoImage
	vtkSmartPointer<vtkMatrix4x4> GetTBGRFtoTBGImageTrans(); // TBG: TibiaBoneGuide, T_TibiaBoneGuideRFtoTBGImage
	vtkSmartPointer<vtkMatrix4x4> GetFBGRFtoFBGImageTrans(); // FBG: FemurBoneGuide, T_FemurBoneGuideRFtoFBGImage
	vtkSmartPointer<vtkMatrix4x4> GetFIOBGRFtoFIOBGImageTrans(); // FIO: Four In One, T_FourInOneBoneGuideRFtoFIOBGImage
	vtkSmartPointer<vtkMatrix4x4> GetPVRFtoPVImageTrans(); // PV: Plane Validator, T_PlaneValidatorRFtoPVImage
	vtkSmartPointer<vtkMatrix4x4> GetPBRFtoPBImageTrans(); //  PB: Precision Block, T_PrecisionBlockRFtoPBImage
	vtkSmartPointer<vtkMatrix4x4> GetPBGRFtoPBGImageTrans(); // PBG: Precision Bone Guide, T_PrecisionBoneGuideRFtoPBGImage

	vtkSmartPointer<vtkMatrix4x4> GetPBGtoTargetImageTrans(); // get T_PBGtoTargetImage
	vtkSmartPointer<vtkMatrix4x4> GetPBGNaviMtxInTargetPos(); // get T_PBGNaviMtxInTargetPos, the PBG navigation transform matrix when cutting plane in the target plane/pos 

	vtkSmartPointer<vtkMatrix4x4> GetPBLasertoPBImageTrans(); // T_PBLasertoPBImage


	// Get the Distance Error of Bone Guides/Plane Validator Validation.
	double GetTibiaBGError();
	double GetFemurBGError();
	double GetFourInOneBGError();
	double GetPlaneValidatorError();
	double GetPrecisionBlockError();
	double GetPrecisionBGError();
	
private:
	/********* Variables *********/

	// Transform matrix (2 Bone images(Tibia, Femur), 3 Bone Guides(Tibia, Distal Femur, 4-in-one), 1 Plane Validator)
	vtkSmartPointer<vtkMatrix4x4> T_TibiaRFtoImage ; // tibia image transform mtx
	vtkSmartPointer<vtkMatrix4x4> T_FemurRFtoImage; // femur image transform mtx
	vtkSmartPointer<vtkMatrix4x4> T_TibiaBoneGuideRFtoTBGImage ; // tibia Bone Guide RF to TBG Image
	vtkSmartPointer<vtkMatrix4x4> T_FemurBoneGuideRFtoFBGImage; // Femur Bone Guide RF to FBG Image
	vtkSmartPointer<vtkMatrix4x4> T_FourInOneBoneGuideRFtoFIOBGImage; // 4-in-1 Bone Fuide RF to 4-in-1 BG image.
	vtkSmartPointer<vtkMatrix4x4> T_PlaneValidatorRFtoPVImage; // Plane validator RF to Plane Validator image.
	vtkSmartPointer<vtkMatrix4x4> T_PrecisionBlockRFtoPBImage; // Precision Block RF to Precision Block image.
	vtkSmartPointer<vtkMatrix4x4> T_PrecisionBoneGuideRFtoPBGImage; // Precision Bone Guide RF to Precision Bone Guide image.

	vtkSmartPointer<vtkMatrix4x4> T_PBLasertoPBImage; // PB Laser pointset/plane to corresponding PB Image Pointset/plane.

	// Surface Source(image point) Registration/Calibration Pointset
	mitk::PointSet::Pointer m_TibiaBGSrcPointset{ nullptr }; // Tibia Bone Guide Source Node Pointset
	mitk::PointSet::Pointer m_FemurBGSrcPointset{ nullptr }; // Distal Bone Guide Source Node Pointset
	mitk::PointSet::Pointer m_FourInOneBGSrcPointset{ nullptr }; // 4-in-1 Bone Guide Source Node Pointset
	mitk::PointSet::Pointer m_PlaneValidatorSrcPointset{ nullptr }; // Plane Validator Source Node Pointset
	mitk::PointSet::Pointer m_PrecisionBlockSrcPointset{ nullptr }; // precision bone guide
	mitk::PointSet::Pointer m_PrecisionBGSrcPointset{ nullptr }; // precision bone guide

	// Bone Guides/Plane Validator Distance Error/Validation
	double TibiaBGErr = -1.0;
	double FemurBGErr = -1.0;
	double FourInOneBGErr = -1.0;
	double PlaneValidatorErr = -1.0;
	double PrecisionBlockErr = -1.0;
	double PrecisionBGErr = -1.0;

	vtkSmartPointer<vtkMatrix4x4> T_PBGtoTargetImage; // precision bone guide navigation tranform matrix;
	vtkSmartPointer<vtkMatrix4x4> T_PBGNaviMtxInTargetPos; // the PBG navigation transform matrix when cutting plane in the target plane/pos 

	/********* Functions *********/

	/// <summary>
	/// Do Landmark Registraiton for BoneImage/BoneGuide
	/// </summary>
	/// <param name="srcPointset">The loaded Source Pointset</param>
	/// <param name="tarPointset">The collected Pointset by Probe</param>
	/// <param name="surface">Target Object Surface(stl file), ex: bone image, bone guide image</param>
	/// <returns> Transform Matrix </returns>
	vtkSmartPointer<vtkMatrix4x4> LandmarkRegistration(mitk::PointSet::Pointer srcPointset, mitk::PointSet::Pointer tarPointset, mitk::Surface::Pointer surface);
		
	/// <summary>
	/// ICP (Iterative Closest Point) Registration
	/// </summary>
	/// <param name="T_RFToImage"> The Transform Matrix from "RF" to "Target Image" </param>
	/// <param name="dstPointset"> Collected by Probe (ICP pointset) </param>
	/// <param name="surface"> Target Image </param>
	/// <returns> the Transform Matrix that do "Landmark" and "ICP" registration </returns>
	vtkSmartPointer<vtkMatrix4x4> ICPRegistration(vtkSmartPointer<vtkMatrix4x4> T_RFToImage, mitk::PointSet::Pointer dstPointset, mitk::Surface::Pointer surface);

	/// <summary>
	/// Bone Guide Navigation
	/// </summary>
	/// <param name="T_BoneGuideRFtoBGImage">the Transform matrix of BoneGuideRF_To_BoneGuideImage</param>
	/// <param name="T_BoneRFtoImage">the Transform matrix of BoneRF_To_BoneImage</param>
	/// <param name="CuttingPlane">The Bone Guide Cutting Plane Surface</param>
	/// <param name="nd_CameraToBoneGuideRF">Navigation data of Camera_To_BoneGuideRF</param>
	/// <param name="nd_CameraToBoneRF">Navigation data of Camera_To_Target_BoneRF</param>
	/// <returns> The Positon of Bone Guide Cutting Plane in Bone Image</returns>
	vtkSmartPointer<vtkMatrix4x4> BoneGuideNavigation(vtkSmartPointer<vtkMatrix4x4> T_BoneGuideRFtoBGImage, vtkSmartPointer<vtkMatrix4x4> T_BoneRFtoImage,
													const double nd_CameraToBoneGuideRF[16], const double nd_CameraToBoneRF[16]);
	
	/// <summary>
	/// Probe navigation.
	/// </summary>
	/// <param name="T_RFToImage">the transform that Bone RF/marker to Bone Image</param>
	/// <param name="nd_CameraToProbe">nd_CameraToProbe, Probe tip position under Camera coordinate</param>
	/// <param name="nd_CameraToTargetRF">nd_CameraToTargetRF, Target RF/maker position</param>
	/// <returns>Probe transform matrix</returns>
	vtkSmartPointer<vtkMatrix4x4> ProbeNavigation(vtkSmartPointer<vtkMatrix4x4> T_RFToImage, const double nd_CameraToProbe[16], const double nd_CameraToTargetRF[16]);

	/// <summary>
	/// Points to point validation.
	/// </summary>
	/// <param name="T_BoneGuideRFtoBGImage">The t bone guide r fto bg image.</param>
	/// <param name="srcPointset">The source pointset.</param>
	/// <param name="nd_CameraToProbe">The nd camera to probe.</param>
	/// <param name="nd_CameraToTargetRF">The nd camera to target rf.</param>
	/// <returns> Distance error between image point (labeled point) and transformed point (predicted/calculated point) </returns>
	double PointToPointValidation(vtkSmartPointer<vtkMatrix4x4> T_BoneGuideRFtoBGImage, mitk::PointSet::Pointer srcPointset, const double nd_CameraToProbe[16], const double nd_CameraToTargetRF[16]);
	double PointToPointValidation(vtkSmartPointer<vtkMatrix4x4> T_BoneGuideRFtoBGImage, mitk::PointSet::Pointer srcPointset, mitk::PointSet::Pointer tarPointset);
	mitk::PointSet::Pointer GetAllPointsExceptLast(mitk::PointSet::Pointer originalPointSet); // create/copy a new pointset except last point.

	std::tuple<double, double, double, double, double> SurfaceToSurfaceError(mitk::Surface::Pointer planningPlaneSurface, mitk::Surface::Pointer cuttingPlaneSurface, vtkSmartPointer<vtkMatrix4x4> T_BGImageToBoneImage);
	//void QuaternionToEulerAngles(const vtkQuaternion<double>& quaternion, double& roll, double& pitch, double& yaw);
	double SignedAngleBetweenVectors( double v1[3],  double v2[3],  double normal[3]);
	double ComputePointToPlaneDistance(double point[3], double surfacePoint[3], double surfaceNormal[3]); // get point-to-plane distance
};


#endif 