// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qmitk

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
#include <vtkQuaternion.h>

//igt
#include <lancetVegaTrackingDevice.h>
#include <kukaRobotDevice.h>
#include <lancetApplyDeviceRegistratioinFilter.h>
#include <mitkNavigationDataToPointSetFilter.h>
#include <lancetPathPoint.h>

#include "lancetTrackingDeviceSourceConfigurator.h"
#include "mitkNavigationToolStorageDeserializer.h"
#include <QtWidgets\qfiledialog.h>

#include "mitkIGTIOException.h"
#include "mitkNavigationToolStorageSerializer.h"
#include "QmitkIGTCommonHelper.h"
#include "lancetTreeCoords.h"

#include "TKAN.h"
#include "Helper.h"

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

TKAN::TKAN() 
{
	// initalize the Transform Matrix
	T_TibiaRFtoImage = vtkSmartPointer<vtkMatrix4x4>::New();
	T_FemurRFtoImage = vtkSmartPointer<vtkMatrix4x4>::New();

	T_TibiaBoneGuideRFtoTBGImage = vtkSmartPointer<vtkMatrix4x4>::New();
	T_FemurBoneGuideRFtoFBGImage = vtkSmartPointer<vtkMatrix4x4>::New();
	T_FourInOneBoneGuideRFtoFIOBGImage = vtkSmartPointer<vtkMatrix4x4>::New();

	T_PlaneValidatorRFtoPVImage = vtkSmartPointer<vtkMatrix4x4>::New();

	T_PrecisionBlockRFtoPBImage = vtkSmartPointer<vtkMatrix4x4>::New();
	T_PrecisionBoneGuideRFtoPBGImage = vtkSmartPointer<vtkMatrix4x4>::New();

	T_PBGtoTargetImage = vtkSmartPointer<vtkMatrix4x4>::New(); // navigation transform matrix

	T_PBGNaviMtxInTargetPos = vtkSmartPointer<vtkMatrix4x4>::New(); // the PBG navigation transform matrix when cutting plane in the target plane/pos 

	T_PBLasertoPBImage = vtkSmartPointer<vtkMatrix4x4>::New(); // PB Laser pointset/plane to corresponding PB Image Pointset/plane. 
}

vtkSmartPointer<vtkMatrix4x4> TKAN::LandmarkRegistration(mitk::PointSet::Pointer srcPointset, mitk::PointSet::Pointer tarPointset, mitk::Surface::Pointer surface)
{
	if (srcPointset == nullptr || tarPointset == nullptr) // surface == nullptr || 
	{
		MITK_INFO << "'srcPointset' OR 'tarPointset' Not Found!";
		return vtkSmartPointer<vtkMatrix4x4>::New();
	}

	//// Do Landmark Registration, cal the transform from srcLandmark(probeTipUnderRF/ObjectRFToProbe) to dstLandmark(image)
	auto landmarkRegistrator = mitk::SurfaceRegistration::New();
	landmarkRegistrator->SetLandmarksSrc(tarPointset);
	landmarkRegistrator->SetLandmarksTarget(srcPointset);
	landmarkRegistrator->ComputeLandMarkResult(); // get the matrix from RF to image

	vtkSmartPointer<vtkMatrix4x4> landmarkResult = landmarkRegistrator->GetResult();

	return landmarkResult;
}

/*
Transform matrix:

T_TibiaRFtoImage; // tibia image transform mtx
T_FemurRFtoImage; // femur image transform mtx
T_TibiaBoneGuideRFtoTBGImage; // tibia Bone Guide RF to TBG Image
T_FemurBoneGuideRFtoFBGImage; // Femur Bone Guide RF to FBG Image
T_FourInOneBoneGuideRFtoFIOBGImage; // 4-in-1 Bone Fuide RF to 4-in-1 BG image.
T_PlaneValidatorRFtoPVImage; // Plane validator RF to Plane Validator image.

*/

void TKAN::TibiaImageRegistration(mitk::PointSet::Pointer srcPointset, mitk::PointSet::Pointer tarPointset, mitk::Surface::Pointer surface)
{
	T_TibiaRFtoImage = LandmarkRegistration(srcPointset, tarPointset, surface);
}

void TKAN::FemurImageRegistration(mitk::PointSet::Pointer srcPointset, mitk::PointSet::Pointer tarPointset, mitk::Surface::Pointer surface)
{
	T_FemurRFtoImage = LandmarkRegistration(srcPointset, tarPointset, surface);
}

void TKAN::TibiaBoneGuideRegistration(mitk::PointSet::Pointer srcPointset, mitk::PointSet::Pointer tarPointset, mitk::Surface::Pointer surface)
{
	m_TibiaBGSrcPointset = srcPointset;
	// split the pointset(src & tar), remove the last point, use the rest of points to do landmark registration. 
	auto newSrcPointset = GetAllPointsExceptLast(srcPointset); // pointset except last point
	auto newTarPointset = GetAllPointsExceptLast(tarPointset);
	T_TibiaBoneGuideRFtoTBGImage = LandmarkRegistration(newSrcPointset, newTarPointset, surface); // for registration
	TibiaBGErr = PointToPointValidation(T_TibiaBoneGuideRFtoTBGImage, srcPointset, tarPointset); // for validation after landmark Registration
}

void TKAN::FemurBoneGuideRegistration(mitk::PointSet::Pointer srcPointset, mitk::PointSet::Pointer tarPointset, mitk::Surface::Pointer surface)
{
	m_FemurBGSrcPointset = srcPointset;

	// split the pointset(src & tar), remove the last point, use the rest of points to do landmark registration. 
	auto newSrcPointset = GetAllPointsExceptLast(srcPointset); // pointset except last point
	auto newTarPointset = GetAllPointsExceptLast(tarPointset);
	T_FemurBoneGuideRFtoFBGImage = LandmarkRegistration(newSrcPointset, newTarPointset, surface); // for registration
	FemurBGErr = PointToPointValidation(T_FemurBoneGuideRFtoFBGImage, srcPointset, tarPointset); // for validation after landmark Registration
}

void TKAN::FourInOneBoneGuideRegistration(mitk::PointSet::Pointer srcPointset, mitk::PointSet::Pointer tarPointset, mitk::Surface::Pointer surface)
{
	m_FourInOneBGSrcPointset = srcPointset;

	// split the pointset(src & tar), remove the last point, use the rest of points to do landmark registration. 
	auto newSrcPointset = GetAllPointsExceptLast(srcPointset); // pointset except last point
	auto newTarPointset = GetAllPointsExceptLast(tarPointset);
	T_FourInOneBoneGuideRFtoFIOBGImage = LandmarkRegistration(newSrcPointset, newTarPointset, surface); // for registration
	FourInOneBGErr = PointToPointValidation(T_FourInOneBoneGuideRFtoFIOBGImage, srcPointset, tarPointset); // for validation after landmark Registration
}

void TKAN::PlaneValidatorRegistration(mitk::PointSet::Pointer srcPointset, mitk::PointSet::Pointer tarPointset, mitk::Surface::Pointer surface)
{
	m_PlaneValidatorSrcPointset = srcPointset;

	// split the pointset(src & tar), remove the last point, use the rest of points to do landmark registration. 
	auto newSrcPointset = GetAllPointsExceptLast(srcPointset); // pointset except last point
	auto newTarPointset = GetAllPointsExceptLast(tarPointset);
	T_PlaneValidatorRFtoPVImage = LandmarkRegistration(newSrcPointset, newTarPointset, surface); // for registration
	PlaneValidatorErr = PointToPointValidation(T_PlaneValidatorRFtoPVImage, srcPointset, tarPointset); // for validation after landmark Registration
}

void TKAN::PrecisionBlockRegistration(mitk::PointSet::Pointer srcPointset, mitk::PointSet::Pointer tarPointset, mitk::Surface::Pointer surface) 
{
	T_PrecisionBlockRFtoPBImage = LandmarkRegistration(srcPointset, tarPointset, surface);
}

void TKAN::PrecisionBGRegistration(mitk::PointSet::Pointer srcPointset, mitk::PointSet::Pointer tarPointset, mitk::Surface::Pointer surface) 
{
	m_PrecisionBGSrcPointset = srcPointset;

	// split the pointset(src & tar), remove the last point, use the rest of points to do landmark registration. 
	auto newSrcPointset = GetAllPointsExceptLast(srcPointset); // pointset except last point
	auto newTarPointset = GetAllPointsExceptLast(tarPointset);
	T_PrecisionBoneGuideRFtoPBGImage = LandmarkRegistration(newSrcPointset, newTarPointset, surface); // for registration
	PrecisionBGErr = PointToPointValidation(T_PrecisionBoneGuideRFtoPBGImage, srcPointset, tarPointset); // for validation after landmark Registration
}

void TKAN::PBLaserRegistration(mitk::PointSet::Pointer srcPointset, mitk::PointSet::Pointer tarPointset, mitk::Surface::Pointer surface)
{
	T_PBLasertoPBImage = LandmarkRegistration(srcPointset, tarPointset, surface);
}

vtkSmartPointer<vtkMatrix4x4> TKAN::ICPRegistration(vtkSmartPointer<vtkMatrix4x4> T_RFToImage, mitk::PointSet::Pointer dstPointset, mitk::Surface::Pointer surface)
{
	if (surface == nullptr || dstPointset == nullptr)
	{
		MITK_INFO << "'surface' OR 'dstPointset' Not Found!";
		return T_RFToImage;
	}

	// process the dstPointset, transform the pointset with 'T_RFToImage' matrix
	mitk::PointSet::Pointer newDstPointSet = mitk::PointSet::New();
	for (auto it = dstPointset->Begin(); it != dstPointset->End(); ++it)
	{
		mitk::Point3D point = it->Value();
		vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
		matrix->Identity(); // Start with the identity matrix
		matrix->SetElement(0, 3, point[0]);
		matrix->SetElement(1, 3, point[1]);
		matrix->SetElement(2, 3, point[2]);

		vtkSmartPointer<vtkTransform> tmpVtkTransform = vtkSmartPointer<vtkTransform>::New();
		tmpVtkTransform->PostMultiply();
		tmpVtkTransform->Identity();
		tmpVtkTransform->SetMatrix(matrix);
		tmpVtkTransform->Concatenate(T_RFToImage); // coordinate: tibiaBoneGuideRF -> camera 

		mitk::Point3D newPoint;
		newPoint[0] = tmpVtkTransform->GetMatrix()->GetElement(0, 3);
		newPoint[1] = tmpVtkTransform->GetMatrix()->GetElement(1, 3);
		newPoint[2] = tmpVtkTransform->GetMatrix()->GetElement(2, 3);

		newDstPointSet->InsertPoint(newPoint);
	}

	// do ICP (Surface to Pointset)
	auto icpRegistrator = mitk::SurfaceRegistration::New();

	icpRegistrator->SetSurfaceSrc(surface);
	icpRegistrator->SetIcpPoints(newDstPointSet);
	icpRegistrator->ComputeIcpResult();

	/// T_landmark * inv(T_ICP) -> refined tranform matrix "T_RFToImage"
	vtkSmartPointer<vtkMatrix4x4> T_inv_ICP = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(DoubleToVtkMatrix(icpRegistrator->GetResult()->GetData()), T_inv_ICP);

	vtkSmartPointer<vtkTransform> tmpVtkTransform = vtkSmartPointer<vtkTransform>::New();
	tmpVtkTransform->PostMultiply();
	tmpVtkTransform->Identity();
	tmpVtkTransform->SetMatrix(T_RFToImage);
	tmpVtkTransform->Concatenate(T_inv_ICP); 

	return tmpVtkTransform->GetMatrix(); // refined T_RFToImage matrix
}

void TKAN::TibiaImageICP(mitk::PointSet::Pointer dstPointset, mitk::Surface::Pointer surface)
{
	T_TibiaRFtoImage = ICPRegistration(T_TibiaRFtoImage, dstPointset, surface);
}

void TKAN::FemurImageICP(mitk::PointSet::Pointer dstPointset, mitk::Surface::Pointer surface)
{
	T_FemurRFtoImage = ICPRegistration(T_FemurRFtoImage, dstPointset, surface);
}

void TKAN::TibiaBoneGuideICP(mitk::PointSet::Pointer dstPointset, mitk::Surface::Pointer surface)
{
	T_TibiaBoneGuideRFtoTBGImage = ICPRegistration(T_TibiaBoneGuideRFtoTBGImage, dstPointset, surface);
}

void TKAN::FemurBoneGuideICP(mitk::PointSet::Pointer dstPointset, mitk::Surface::Pointer surface)
{
	T_FemurBoneGuideRFtoFBGImage = ICPRegistration(T_FemurBoneGuideRFtoFBGImage, dstPointset, surface);
}

void TKAN::FourInOneBoneGuideICP(mitk::PointSet::Pointer dstPointset, mitk::Surface::Pointer surface)
{
	T_FourInOneBoneGuideRFtoFIOBGImage = ICPRegistration(T_FourInOneBoneGuideRFtoFIOBGImage, dstPointset, surface);
}

void TKAN::PlaneValidatorICP(mitk::PointSet::Pointer dstPointset, mitk::Surface::Pointer surface)
{
	T_PlaneValidatorRFtoPVImage = ICPRegistration(T_PlaneValidatorRFtoPVImage, dstPointset, surface);
}

void TKAN::PrecisionBlockICP(mitk::PointSet::Pointer dstPointset, mitk::Surface::Pointer surface)
{
	T_PrecisionBlockRFtoPBImage = ICPRegistration(T_PrecisionBlockRFtoPBImage, dstPointset, surface);
}

void TKAN::PrecisionBoneGuideICP(mitk::PointSet::Pointer dstPointset, mitk::Surface::Pointer surface)
{
	T_PrecisionBoneGuideRFtoPBGImage = ICPRegistration(T_PrecisionBoneGuideRFtoPBGImage, dstPointset, surface);
}

vtkSmartPointer<vtkMatrix4x4> TKAN::BoneGuideNavigation(vtkSmartPointer<vtkMatrix4x4> T_BoneGuideRFtoBGImage, vtkSmartPointer<vtkMatrix4x4> T_BoneRFtoImage,
								 const double nd_CameraToBoneGuideRF[16], const double nd_CameraToBoneRF[16])
{
	if (T_BoneGuideRFtoBGImage->IsIdentity() || T_BoneRFtoImage->IsIdentity() || !nd_CameraToBoneGuideRF || !nd_CameraToBoneRF) {
		//MITK_INFO << "Check the Bone Guide Navigation Variables.";
		return vtkSmartPointer<vtkMatrix4x4>::New();
	}

	// OverView: convert tibiaBoneGuideRFToCuttingPlane -> CameraToCuttingPlane -> PatientRFToCuttingPlane -> ImageToCuttingPlane

	// 1. convert surface's coordinate: tibiaBoneGuideImage -> tibiaBoneGuideRF, using invert(m_T_tibiaBoneGuideRFtoTBGImage), cal by bone registration/calibration
	vtkSmartPointer<vtkMatrix4x4> T_BGImageToBoneGuideRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(T_BoneGuideRFtoBGImage, T_BGImageToBoneGuideRF);

	// 2. convert surface's coordinate: tibiaBoneGuideRF -> Camera
	//mitk::NavigationData::Pointer nd_CameraToBoneGuideRf = m_CameraSource->GetOutput(m_ToolStorage->GetToolIndexByName("TibiaBoneGuideRF")); // target Bone Guide RF
	auto T_BoneGuideRFToCamera = DoubleToVtkMatrix(nd_CameraToBoneGuideRF);

	// 3. convert surface's coordinate: Camera -> patient/Object RF, known: CameraToCuttingPlane,  
	//mitk::NavigationData::Pointer nd_CameraToObjectRf = m_CameraSource->GetOutput(m_ToolStorage->GetToolIndexByName("ObjectRf"));
	auto T_CameraToPatientRF = DoubleToVtkMatrix(nd_CameraToBoneRF);
	vtkSmartPointer<vtkMatrix4x4> T_PatientRFToCamera = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(T_CameraToPatientRF, T_PatientRFToCamera);

	// 4. convert surface's coordinate: patient/Object RF -> Image, using m_T_patientRFtoImage (cal by image registration)
	// T_BoneRFtoImage

	// do vtk transform 
	vtkSmartPointer<vtkTransform> tmpVtkTransform = vtkTransform::New();
	tmpVtkTransform->PostMultiply();
	tmpVtkTransform->Identity();
	tmpVtkTransform->SetMatrix(T_BGImageToBoneGuideRF); // coordinate: tibiaBoneGuideImage->tibiaBoneGuideRF
	tmpVtkTransform->Concatenate(T_BoneGuideRFToCamera); // coordinate: tibiaBoneGuideRF -> camera 
	tmpVtkTransform->Concatenate(T_PatientRFToCamera); // coordinate: camera -> patient/Object Rf 
	tmpVtkTransform->Concatenate(T_BoneRFtoImage); // coordinate: patient/Object Rf -> Image

	return tmpVtkTransform->GetMatrix();
}

vtkSmartPointer<vtkMatrix4x4> TKAN::TibiaBoneGuideNavigation(const double nd_CameraToBoneGuideRF[16], const double nd_CameraToBoneRF[16])
{
	return BoneGuideNavigation(T_TibiaBoneGuideRFtoTBGImage, T_TibiaRFtoImage, nd_CameraToBoneGuideRF, nd_CameraToBoneRF);
}

vtkSmartPointer<vtkMatrix4x4> TKAN::FemurBoneGuideNavigation(const double nd_CameraToBoneGuideRF[16], const double nd_CameraToBoneRF[16])
{
	return BoneGuideNavigation(T_FemurBoneGuideRFtoFBGImage, T_FemurRFtoImage, nd_CameraToBoneGuideRF, nd_CameraToBoneRF); // 
}

vtkSmartPointer<vtkMatrix4x4> TKAN::FourInOneBoneGuideNavigation(const double nd_CameraToBoneGuideRF[16], const double nd_CameraToBoneRF[16])
{
	return BoneGuideNavigation(T_FourInOneBoneGuideRFtoFIOBGImage, T_FemurRFtoImage, nd_CameraToBoneGuideRF, nd_CameraToBoneRF);
}

vtkSmartPointer<vtkMatrix4x4> TKAN::PrecisionBoneGuideNavigation(const double nd_CameraToBoneGuideRF[16], const double nd_CameraToBoneRF[16])
{
	T_PBGtoTargetImage = BoneGuideNavigation(T_PrecisionBoneGuideRFtoPBGImage, T_PrecisionBlockRFtoPBImage, nd_CameraToBoneGuideRF, nd_CameraToBoneRF);

	return T_PBGtoTargetImage;
}

vtkSmartPointer<vtkMatrix4x4> TKAN::ProbeNavigation(vtkSmartPointer<vtkMatrix4x4> T_RFToImage, const double nd_CameraToProbe[16], const double nd_CameraToTargetRF[16])
{
	//mitk::NavigationData::Pointer nd_RFToProbe = GetNavigationDataInRef(nd_CameraToProbe, nd_CameraToTargetRF);
	auto vtk_CameraToProbe = DoubleToVtkMatrix(nd_CameraToProbe);
	auto vtk_CameraToTargetRF = DoubleToVtkMatrix(nd_CameraToTargetRF);
	vtkSmartPointer<vtkMatrix4x4> T_TargetRFToCamera = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(vtk_CameraToTargetRF, T_TargetRFToCamera);

	vtkSmartPointer<vtkTransform> tmpVtkTransform = vtkTransform::New();
	tmpVtkTransform->PostMultiply();
	tmpVtkTransform->Identity();
	tmpVtkTransform->SetMatrix(vtk_CameraToProbe);
	tmpVtkTransform->Concatenate(T_TargetRFToCamera); // coordinate: camera -> TargetRF
	tmpVtkTransform->Concatenate(T_RFToImage); // coordinate: TargetRF -> target Image

	return tmpVtkTransform->GetMatrix();
}

vtkSmartPointer<vtkMatrix4x4> TKAN::ProbeNavigationInTibiaBone(const double nd_CameraToProbe[16], const double nd_CameraToTargetRF[16])
{
	return ProbeNavigation(T_TibiaRFtoImage, nd_CameraToProbe, nd_CameraToTargetRF);
}

vtkSmartPointer<vtkMatrix4x4> TKAN::ProbeNavigationInFemurBone(const double nd_CameraToProbe[16], const double nd_CameraToTargetRF[16])
{
	return ProbeNavigation(T_FemurRFtoImage, nd_CameraToProbe, nd_CameraToTargetRF);
}

vtkSmartPointer<vtkMatrix4x4> TKAN::ProbeNavigationInTibiaBG(const double nd_CameraToProbe[16], const double nd_CameraToTargetRF[16])
{
	return ProbeNavigation(T_TibiaBoneGuideRFtoTBGImage, nd_CameraToProbe, nd_CameraToTargetRF);
}

vtkSmartPointer<vtkMatrix4x4> TKAN::ProbeNavigationInFemurBG(const double nd_CameraToProbe[16], const double nd_CameraToTargetRF[16])
{
	return ProbeNavigation(T_FemurBoneGuideRFtoFBGImage, nd_CameraToProbe, nd_CameraToTargetRF);
}

vtkSmartPointer<vtkMatrix4x4> TKAN::ProbeNavigationInFourInOneBG(const double nd_CameraToProbe[16], const double nd_CameraToTargetRF[16])
{
	return ProbeNavigation(T_FourInOneBoneGuideRFtoFIOBGImage, nd_CameraToProbe, nd_CameraToTargetRF);
}

vtkSmartPointer<vtkMatrix4x4> TKAN::ProbeNavigationInPlaneValidator(const double nd_CameraToProbe[16], const double nd_CameraToTargetRF[16])
{
	return ProbeNavigation(T_PlaneValidatorRFtoPVImage, nd_CameraToProbe, nd_CameraToTargetRF);
}

vtkSmartPointer<vtkMatrix4x4> TKAN::ProbeNavigationInPrecisionBlock(const double nd_CameraToProbe[16], const double nd_CameraToTargetRF[16])
{
	return ProbeNavigation(T_PrecisionBlockRFtoPBImage, nd_CameraToProbe, nd_CameraToTargetRF);
}

vtkSmartPointer<vtkMatrix4x4> TKAN::ProbeNavigationInPrecisionBG(const double nd_CameraToProbe[16], const double nd_CameraToTargetRF[16])
{
	return ProbeNavigation(T_PrecisionBoneGuideRFtoPBGImage, nd_CameraToProbe, nd_CameraToTargetRF);
}

vtkSmartPointer<vtkMatrix4x4> TKAN::PVNavigationInTibiaBone(const double nd_CameraToPV[16], const double nd_CameraToBoneRF[16])
{
	return BoneGuideNavigation(T_PlaneValidatorRFtoPVImage, T_TibiaRFtoImage, nd_CameraToPV, nd_CameraToBoneRF);
}

vtkSmartPointer<vtkMatrix4x4> TKAN::PVNavigationInFemurBone(const double nd_CameraToPV[16], const double nd_CameraToBoneRF[16])
{
	return BoneGuideNavigation(T_PlaneValidatorRFtoPVImage, T_FemurRFtoImage, nd_CameraToPV, nd_CameraToBoneRF);
}

double TKAN::PointToPointValidation(vtkSmartPointer<vtkMatrix4x4> T_BoneGuideRFtoBGImage, mitk::PointSet::Pointer srcPointset, mitk::PointSet::Pointer tarPointset)
{
	// this function will be called After DO BGs/Plane Validator Landmark Registation.

	if (!srcPointset || !tarPointset) {
		MITK_INFO << "Empty srcPointset OR tarPointset!";
		return -1.0;
	}
	
	int numPoints = tarPointset->GetSize();
	mitk::Point3D TargetRFtoProbe = tarPointset->GetPoint(numPoints - 1);
	vtkSmartPointer<vtkMatrix4x4> vtkTargetRFToProbe = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkTargetRFToProbe->SetElement(0, 3, TargetRFtoProbe[0]);
	vtkTargetRFToProbe->SetElement(1, 3, TargetRFtoProbe[1]);
	vtkTargetRFToProbe->SetElement(2, 3, TargetRFtoProbe[2]);

	// transform "vtkTargetRFToProbe" under TargetBGImage Coordinate
	vtkSmartPointer<vtkTransform> tmpVtkTransform = vtkTransform::New();
	tmpVtkTransform->PostMultiply();
	tmpVtkTransform->Identity();
	tmpVtkTransform->SetMatrix(vtkTargetRFToProbe);
	tmpVtkTransform->Concatenate(T_BoneGuideRFtoBGImage); // coordinate: TargetRF -> target Image

	// convert to double[3], compute the EuclideanDistance
	mitk::Point3D srcPoint = srcPointset->GetPoint(numPoints - 1);
	double imageCheckPoint[3]{ srcPoint[0], srcPoint[1], srcPoint[2] };

	return EuclideanDistance(imageCheckPoint, tmpVtkTransform->GetPosition());
}

double TKAN::PointToPointValidation( vtkSmartPointer<vtkMatrix4x4> T_BoneGuideRFtoBGImage, mitk::PointSet::Pointer srcPointset, const double nd_CameraToProbe[16], const double nd_CameraToTargetRF[16])
{
	// We will select the last_point(default) of "srcPointset" as the Validation Point
	// Compare with the Probe Tip Coordinate (Transform it under <BGImage> Coordinate by "T_BoneGuideRFtoBGImage" and "nd_CameraToTargetRF").
	// Compute the error (mm)

	if (!srcPointset) {
		MITK_INFO << "Empty srcPointset!";
		return -1.0;
	}

	// transform the ProbeTip under TargetRF Coordinate.
	auto vtkCameraToProbe = DoubleToVtkMatrix(nd_CameraToProbe);
	auto vtkCameraToTargetRF = DoubleToVtkMatrix(nd_CameraToTargetRF);
	auto vtkTargetRFToProbe = getVtkMatrix4x4InRef(vtkCameraToProbe, vtkCameraToTargetRF);

	// transform "vtkTargetRFToProbe" under TargetBGImage Coordinate
	vtkSmartPointer<vtkTransform> tmpVtkTransform = vtkTransform::New();
	tmpVtkTransform->PostMultiply();
	tmpVtkTransform->Identity();
	tmpVtkTransform->SetMatrix(vtkTargetRFToProbe);
	tmpVtkTransform->Concatenate(T_BoneGuideRFtoBGImage); // coordinate: TargetRF -> target Image
	
	// convert to double[3], compute the Euclidean Distance
	mitk::Point3D srcPoint = srcPointset->GetPoint(srcPointset->GetSize() - 1);
	double imageCheckPoint[3]{ srcPoint[0], srcPoint[1], srcPoint[2] };
	return EuclideanDistance(imageCheckPoint, tmpVtkTransform->GetPosition());

	/*
	double probeTipInImage[3] 
	{
		tmpVtkTransform->GetMatrix()->GetElement(0,3),
		tmpVtkTransform->GetMatrix()->GetElement(1,3),
		tmpVtkTransform->GetMatrix()->GetElement(2,3),
	};
	
	mitk::Point3D srcPoint = srcPointset->GetPoint(srcPointset->GetSize() - 1);
	double imageCheckPoint[3] { srcPoint[0], srcPoint[1], srcPoint[2]};

	double distance = sqrt(pow(probeTipInImage[0] - imageCheckPoint[0], 2) +
		 	pow(probeTipInImage[1] - imageCheckPoint[1], 2) +
		 	pow(probeTipInImage[2] - imageCheckPoint[2], 2)); 

	//MITK_INFO << "imageCheckPoint " << imageCheckPoint[0] << " " << imageCheckPoint[1] << " " << imageCheckPoint[2];
	//MITK_INFO << "probeTipInImage " << probeTipInImage[0] << " " << probeTipInImage[1] << " " << probeTipInImage[2];

	return distance;*/
}

void TKAN::TibiaBGValidation(const double nd_CameraToProbe[16], const double nd_CameraToBoneGuideRF[16])
{
	TibiaBGErr = PointToPointValidation( T_TibiaBoneGuideRFtoTBGImage, m_TibiaBGSrcPointset, nd_CameraToProbe, nd_CameraToBoneGuideRF);
}

void TKAN::FemurBGValidation(const double nd_CameraToProbe[16], const double nd_CameraToBoneGuideRF[16])
{
	FemurBGErr = PointToPointValidation(T_FemurBoneGuideRFtoFBGImage, m_FemurBGSrcPointset, nd_CameraToProbe, nd_CameraToBoneGuideRF);
}

void TKAN::FourInOneBGValidation(const double nd_CameraToProbe[16], const double nd_CameraToBoneGuideRF[16])
{
	FourInOneBGErr = PointToPointValidation(T_FourInOneBoneGuideRFtoFIOBGImage, m_FourInOneBGSrcPointset, nd_CameraToProbe, nd_CameraToBoneGuideRF);
}

void TKAN::PlaneValidatorValidation(const double nd_CameraToProbe[16], const double nd_CameraToBoneGuideRF[16])
{
	PlaneValidatorErr = PointToPointValidation(T_PlaneValidatorRFtoPVImage, m_PlaneValidatorSrcPointset, nd_CameraToProbe, nd_CameraToBoneGuideRF);
}

void TKAN::PrecisionBlockValidation(const double nd_CameraToProbe[16], const double nd_CameraToBoneGuideRF[16])
{
	PrecisionBlockErr = PointToPointValidation(T_PrecisionBlockRFtoPBImage, m_PrecisionBlockSrcPointset, nd_CameraToProbe, nd_CameraToBoneGuideRF);
}

void TKAN::PrecisionBGValidation(const double nd_CameraToProbe[16], const double nd_CameraToBoneGuideRF[16])
{
	PrecisionBGErr = PointToPointValidation(T_PrecisionBoneGuideRFtoPBGImage, m_PrecisionBGSrcPointset, nd_CameraToProbe, nd_CameraToBoneGuideRF);
}

mitk::PointSet::Pointer TKAN::GetAllPointsExceptLast(mitk::PointSet::Pointer originalPointSet) 
{
	mitk::PointSet::Pointer newPointSet = mitk::PointSet::New();

	int numPoints = originalPointSet->GetSize();

	for ( int i = 0; i < numPoints - 1; ++i)
	{
		mitk::Point3D point = originalPointSet->GetPoint(i);
		newPointSet->InsertPoint(point);
	}

	return newPointSet;
}

double TKAN::SignedAngleBetweenVectors(double v1[3], double v2[3], double refNormal[3]) {
	vtkMath::Normalize(v1);
	vtkMath::Normalize(v2);
	vtkMath::Normalize(refNormal);

	double dotProduct = vtkMath::Dot(v1, v2);
	dotProduct = std::clamp(dotProduct, -1.0, 1.0); // acos: [-1.0, 1.0]

	double crossProduct[3];
	vtkMath::Cross(v1, v2, crossProduct); // get v1&v2 normal direction

	//double angle = acos(dotProduct);
	double angle = atan2(vtkMath::Norm(crossProduct), dotProduct); // sin / cos
	if (angle > vtkMath::RadiansFromDegrees(90.0)) { // make angle < 90 degree
		angle = vtkMath::RadiansFromDegrees(180.0) - angle;
	}
	
	double sign = vtkMath::Dot(crossProduct, refNormal) >= 0 ? 1.0 : -1.0; // angle between two vectors.

	return vtkMath::DegreesFromRadians(angle) * sign;
}

//// test quaternion to Euler angles.
//void TKAN::QuaternionToEulerAngles(const vtkQuaternion<double>& quaternion, double& roll, double& pitch, double& yaw) {
//	double q[4] = { quaternion.GetW(), quaternion.GetX(), quaternion.GetY(), quaternion.GetZ() };
//
//	// Compute the roll (x-axis rotation)
//	double sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
//	double cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
//	roll = atan2(sinr_cosp, cosr_cosp);
//
//	// Compute the pitch (y-axis rotation)
//	double sinp = 2 * (q[0] * q[2] - q[3] * q[1]);
//	if (abs(sinp) >= 1)
//		pitch = copysign(vtkMath::Pi() / 2, sinp); // use 90 degrees if out of range
//	else
//		pitch = asin(sinp);
//
//	// Compute the yaw (z-axis rotation)
//	double siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
//	double cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
//	yaw = atan2(siny_cosp, cosy_cosp);
//}

std::tuple<double, double, double, double, double> TKAN::SurfaceToSurfaceError(mitk::Surface::Pointer planningPlaneSurface, mitk::Surface::Pointer cuttingPlaneSurface, vtkSmartPointer<vtkMatrix4x4> T_BGImageToBoneImage)
{
	if (!planningPlaneSurface || !cuttingPlaneSurface) {
		return std::make_tuple(-1.0, -1.0, -1.0, -1.0, -1.0);
	}

	/* get distance of two surface (point to surface) */

	// get the Normal and Centroid of Planning Plane 
	double targetCenter[3], targetNormal[3];
	ComputeCenterAndNormalFromSurface(planningPlaneSurface, targetCenter, targetNormal); 
	vtkMath::Normalize(targetNormal);

	// get the Normal and Centroid of Cutting Plane 
	double tmp_cuttingCenter[3], cuttingCenter[3], tmp_cuttingNormal[3], cuttingNormal[3];
	ComputeCenterAndNormalFromSurface(cuttingPlaneSurface, tmp_cuttingCenter, tmp_cuttingNormal);

	// convert center and normal with vtkMatrix4x4, Coordinate transform: Bone Guide Image -> Bone Image
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->SetMatrix(T_BGImageToBoneImage);
	transform->TransformPoint(tmp_cuttingCenter, cuttingCenter);
	transform->TransformNormal(tmp_cuttingNormal, cuttingNormal);
	vtkMath::Normalize(cuttingNormal);

	double dis = ComputePointToPlaneDistance(targetCenter, cuttingCenter, cuttingNormal); // the distance from planning centroid to cutting plane.
	dis *= -1.0; // for visualization, When the cutting plane is away from the target plane, the distance is positive; conversely, it is negative.

	//// get Quaternion from two Vectors
	//double axis[3];
	//vtkMath::Cross(targetNormal, cuttingNormal, axis);

	//double cosTheta = vtkMath::Dot(targetNormal, cuttingNormal); 
	//double sinTheta = vtkMath::Norm(axis);

	//vtkQuaternion<double> quaternion;
	//quaternion.SetW(cosTheta + 1);  
	//quaternion.SetX(axis[0]);
	//quaternion.SetY(axis[1]);
	//quaternion.SetZ(axis[2]);
	//quaternion.Normalize();

	//double yaw, roll, pitch;
	//Eigen::Quaterniond q(quaternion.GetW(), quaternion.GetX(), quaternion.GetY(), quaternion.GetZ());
	//Eigen::Vector3d euler = q.normalized().toRotationMatrix().eulerAngles(2, 1, 0); // ZYX order


	// get Quaternion from two Vectors



	/* get pitch and roll difference. (Normals difference) */


	/* Previous Method: cal the euler angle from Normals, get the difference(pitch, roll) between them. */
	//double targetPlaneEulerAngles[3], cuttingPlaneEulerAngles[3];
	//vtkSmartPointer<vtkMatrix4x4> cuttingPlaneMtx = vtkSmartPointer<vtkMatrix4x4>::New();
	//cuttingPlaneMtx->Identity();
	//cuttingPlaneMtx->SetElement(0, 2, cuttingNormal[0]);
	//cuttingPlaneMtx->SetElement(1, 2, cuttingNormal[1]);
	//cuttingPlaneMtx->SetElement(2, 2, cuttingNormal[2]);

	///*mitk::BaseGeometry::Pointer geometry = cuttingPlaneSurface->GetGeometry();
	//vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	//geometry->GetVtkTransform()->GetMatrix(matrix);

	//vtkSmartPointer<vtkTransform> tmpVtkTransform = vtkTransform::New();
	//tmpVtkTransform->PostMultiply();
	//tmpVtkTransform->Identity();
	//tmpVtkTransform->SetMatrix(matrix);
	//tmpVtkTransform->Concatenate(T_BGImageToBoneImage);*/

	//vtkTransform::GetOrientation(cuttingPlaneEulerAngles, cuttingPlaneMtx);


	//vtkSmartPointer<vtkMatrix4x4> targetPlaneMtx = vtkSmartPointer<vtkMatrix4x4>::New();
	//targetPlaneMtx->Identity();
	//targetPlaneMtx->SetElement(0, 2, targetNormal[0]);
	//targetPlaneMtx->SetElement(1, 2, targetNormal[1]);
	//targetPlaneMtx->SetElement(2, 2, targetNormal[2]);
	//vtkTransform::GetOrientation(targetPlaneEulerAngles, targetPlaneMtx);

	//double pitch = cuttingPlaneEulerAngles[0] - targetPlaneEulerAngles[0];
	//double roll = cuttingPlaneEulerAngles[1] - targetPlaneEulerAngles[1];
	//double yaw = cuttingPlaneEulerAngles[2] - targetPlaneEulerAngles[2];

	/* compuate the difference angle between two Normals */
	double tmp_normal[3] = { 0.0, 0.0, 1.0 };
	double diff_angle = SignedAngleBetweenVectors(cuttingNormal, targetNormal, tmp_normal); // total diff angle

	//return std::make_tuple(dis, abs(diff_angle), vtkMath::DegreesFromRadians(euler[0]), vtkMath::DegreesFromRadians(euler[1]), vtkMath::DegreesFromRadians(euler[2]));


	/* compare two Normal, get angles in xy, xz, yz plane (mapping the normal to each plane)*/ 
	double v1_xy[3] = { cuttingNormal[0], cuttingNormal[1], 0.0 };
	double v2_xy[3] = { targetNormal[0], targetNormal[1], 0.0 };
	double xy_normal[3] = { 0.0, 0.0, 1.0};
	// XY plane, yaw
	double angleXY = SignedAngleBetweenVectors(v1_xy, v2_xy, xy_normal); // vtkMath::DegreesFromRadians(vtkMath::AngleBetweenVectors(v1_xy, v2_xy)); 

	// YZ plane, pitch
	double v1_yz[3] = { 0.0, cuttingNormal[1], cuttingNormal[2] };
	double v2_yz[3] = { 0.0, targetNormal[1], targetNormal[2] };
	double yz_normal[3] = { 1.0, 0.0, 0.0 };
	double angleYZ = SignedAngleBetweenVectors(v1_yz, v2_yz, yz_normal); // vtkMath::DegreesFromRadians(vtkMath::AngleBetweenVectors(v1_yz, v2_yz));

	// XZ plane, roll
	double v1_xz[3] = { cuttingNormal[0], 0.0, cuttingNormal[2] };
	double v2_xz[3] = { targetNormal[0], 0.0, targetNormal[2] };
	double xz_normal[3] = { 0.0, 1.0, 0.0 };
	double angleXZ = SignedAngleBetweenVectors(v1_xz, v2_xz, xz_normal);  // vtkMath::DegreesFromRadians(vtkMath::AngleBetweenVectors(v1_xz, v2_xz));

	
	return std::make_tuple(dis, abs(diff_angle), angleXY, angleYZ, angleXZ);
}


//std::tuple<double, double, double> TKAN::ComputeNavigationError(mitk::Surface::Pointer planningPlaneSurface, mitk::Surface::Pointer cuttingPlaneSurface, vtkSmartPointer<vtkMatrix4x4> T_BGImageToBoneImage)
//{
//	auto [dis, angleXY, angleYZ, angleXZ] = SurfaceToSurfaceError(planningPlaneSurface, cuttingPlaneSurface, T_BGImageToBoneImage);
//	return std::make_tuple(dis, angleYZ, angleXZ); 
//}

std::tuple<double, double, double, double> TKAN::ComputeTibiaBGSurfaceError(mitk::Surface::Pointer planningPlaneSurface, mitk::Surface::Pointer cuttingPlaneSurface, vtkSmartPointer<vtkMatrix4x4> T_BGImageToBoneImage)
{
	double dis, angleErr, angleXY, angleYZ, angleXZ;
	std::tie(dis, angleErr, angleXY, angleYZ, angleXZ) = SurfaceToSurfaceError(planningPlaneSurface, cuttingPlaneSurface, T_BGImageToBoneImage);
	return std::make_tuple(dis, angleErr, angleYZ, angleXZ);
}

std::tuple<double, double, double, double> TKAN::ComputeFemurBGSurfaceError(mitk::Surface::Pointer planningPlaneSurface, mitk::Surface::Pointer cuttingPlaneSurface, vtkSmartPointer<vtkMatrix4x4> T_BGImageToBoneImage)
{
	double dis, angleErr, angleXY, angleYZ, angleXZ;
	std::tie(dis, angleErr, angleXY, angleYZ, angleXZ) = SurfaceToSurfaceError(planningPlaneSurface, cuttingPlaneSurface, T_BGImageToBoneImage);
	return std::make_tuple(dis, angleErr, angleYZ, angleXZ);
}

std::tuple<double, double, double, double> TKAN::ComputeFourInOneBGSurfaceError(mitk::Surface::Pointer planningPlaneSurface, mitk::Surface::Pointer cuttingPlaneSurface, vtkSmartPointer<vtkMatrix4x4> T_BGImageToBoneImage)
{
	double dis, angleErr, angleXY, angleYZ, angleXZ;
	std::tie(dis, angleErr, angleXY, angleYZ, angleXZ) = SurfaceToSurfaceError(planningPlaneSurface, cuttingPlaneSurface, T_BGImageToBoneImage);
	return std::make_tuple(dis, angleErr, angleYZ, angleXY); // **NOTE** because the 4-in-1 Bone Guide's position when doing Navigation. (use Yaw(XY) to replace the Roll(XZ));
}

std::tuple<double, double> TKAN::ComputePlaneValidatorSurfaceError(mitk::Surface::Pointer planningPlaneSurface, mitk::Surface::Pointer cuttingPlaneSurface, vtkSmartPointer<vtkMatrix4x4> T_BGImageToBoneImage)
{
	double dis, angleErr, angleXY, angleYZ, angleXZ;
	std::tie(dis, angleErr, angleXY, angleYZ, angleXZ) = SurfaceToSurfaceError(planningPlaneSurface, cuttingPlaneSurface, T_BGImageToBoneImage);
	return std::make_tuple(dis, angleErr); 
}

std::tuple<double, double, double, double, double> TKAN::ComputePrecisionBGSurfaceError(mitk::Surface::Pointer planningPlaneSurface, mitk::Surface::Pointer cuttingPlaneSurface, vtkSmartPointer<vtkMatrix4x4> T_BGImageToBoneImage)
{
	double dis, angleErr, angleXY, angleYZ, angleXZ;
	std::tie(dis, angleErr, angleXY, angleYZ, angleXZ) = SurfaceToSurfaceError(planningPlaneSurface, cuttingPlaneSurface, T_BGImageToBoneImage);
	return std::make_tuple(dis, angleErr, angleYZ, angleXY, angleXZ); // like 4-in-1, roll -> yaw.
}

std::tuple<double, double, double> TKAN::ComputePointToSurfaceDisError(std::vector<std::array<double, 3>>& planningPlanePoints, std::vector<std::array<double, 3>>& cuttingPlanePoints, vtkSmartPointer<vtkMatrix4x4> T_BGImageToBoneImage) 
{
	double planningNormal[3];
	ComputeNormalFromThreePoints(planningPlanePoints[0].data(), planningPlanePoints[1].data(), planningPlanePoints[2].data(), planningNormal);
	
	double tmp_cuttingNormal[3], cuttingNormal[3], cuttingPoint[3];
	ComputeNormalFromThreePoints(cuttingPlanePoints[0].data(), cuttingPlanePoints[1].data(), cuttingPlanePoints[2].data(), tmp_cuttingNormal);

	// convert  tmp_cutting_normal with vtkMatrix4x4, Coordinate transform: Bone Guide Image -> Bone Image
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->SetMatrix(T_BGImageToBoneImage);
	transform->TransformPoint(cuttingPlanePoints[0].data(), cuttingPoint);
	transform->TransformNormal(tmp_cuttingNormal, cuttingNormal);
	vtkMath::Normalize(cuttingNormal);

	double dis1 = ComputePointToPlaneDistance(planningPlanePoints[0].data(), cuttingPoint, cuttingNormal); // the distance from planning plane to cutting plane.
	double dis2 = ComputePointToPlaneDistance(planningPlanePoints[1].data(), cuttingPoint, cuttingNormal);
	double dis3 = ComputePointToPlaneDistance(planningPlanePoints[2].data(), cuttingPoint, cuttingNormal);

	return std::make_tuple(dis1, dis2, dis3);
}

std::tuple<double, double, double> TKAN::ComputePointToSurfaceDisError(mitk::PointSet::Pointer planningPlanePointset, mitk::PointSet::Pointer cuttingPlanePointset, vtkSmartPointer<vtkMatrix4x4> T_BGImageToBoneImage)
{
	std::vector<std::array<double, 3>> planningPlanePoints, cuttingPlanePoints;
	if (planningPlanePointset.IsNotNull())
	{
		unsigned int numPoints = planningPlanePointset->GetSize();
		for (unsigned int i = 0; i < numPoints; ++i)
		{
			mitk::Point3D point = planningPlanePointset->GetPoint(i);
			std::array<double, 3> pointArray = { point[0], point[1], point[2] };
			planningPlanePoints.push_back(pointArray);
		}
	}

	if (cuttingPlanePointset.IsNotNull())
	{
		unsigned int numPoints = cuttingPlanePointset->GetSize();
		for (unsigned int i = 0; i < numPoints; ++i)
		{
			mitk::Point3D point = cuttingPlanePointset->GetPoint(i);
			std::array<double, 3> pointArray = { point[0], point[1], point[2] };
			cuttingPlanePoints.push_back(pointArray);
		}
	}

	return ComputePointToSurfaceDisError(planningPlanePoints, cuttingPlanePoints, T_BGImageToBoneImage);
}

double TKAN::ComputePointToPlaneDistance(double point[3], double surfacePoint[3], double surfaceNormal[3])
{
	/*
	input: surface center (3D point)
	input: surface point(any point in surface) and normal (3D point and Normal)
	output: Point to Plane Distance
	*/

	double normalMagnitude = vtkMath::Norm(surfaceNormal);

	double pointToPlaneVector[3];
	pointToPlaneVector[0] = point[0] - surfacePoint[0];
	pointToPlaneVector[1] = point[1] - surfacePoint[1];
	pointToPlaneVector[2] = point[2] - surfacePoint[2];

	double distance = vtkMath::Dot(pointToPlaneVector, surfaceNormal) / normalMagnitude;

	return distance;
}

/**************  Set/Get FUNCTIONS  **************/

void TKAN::SetTibiaRFtoImageTrans(vtkSmartPointer<vtkMatrix4x4> matrix)
{
	T_TibiaRFtoImage->DeepCopy(matrix);
}

void TKAN::SetFemurRFtoImageTrans(vtkSmartPointer<vtkMatrix4x4> matrix)
{
	T_FemurRFtoImage->DeepCopy(matrix);
}

void TKAN::SetTBGRFtoTBGImageTrans(vtkSmartPointer<vtkMatrix4x4> matrix) // TBG: Tibia Bone Guide
{
	T_TibiaBoneGuideRFtoTBGImage->DeepCopy(matrix);
}

void TKAN::SetFBGRFtoFBGImageTrans(vtkSmartPointer<vtkMatrix4x4> matrix)  // FBG: FemurBoneGuide
{
	T_FemurBoneGuideRFtoFBGImage->DeepCopy(matrix);
}

void TKAN::SetFIOBGRFtoFIOBGImageTrans(vtkSmartPointer<vtkMatrix4x4> matrix) // FIOBG: Four In One Bone Guide
{
	T_FourInOneBoneGuideRFtoFIOBGImage->DeepCopy(matrix);
}
 
void TKAN::SetPVRFtoPVImageTrans(vtkSmartPointer<vtkMatrix4x4> matrix) // PV: Plane Validator
{
	T_PlaneValidatorRFtoPVImage->DeepCopy(matrix);
}

void TKAN::SetPBRFtoPBImageTrans(vtkSmartPointer<vtkMatrix4x4> matrix) // PB: Precision Block
{
	T_PrecisionBlockRFtoPBImage->DeepCopy(matrix);
}

void TKAN::SetPBGRFtoPBGImageTrans(vtkSmartPointer<vtkMatrix4x4> matrix) // PBG: Precision Bone Guide
{
	T_PrecisionBoneGuideRFtoPBGImage->DeepCopy(matrix);
}

// navigation transform matrix
void TKAN::SetPBGtoTargetImageTrans(vtkSmartPointer<vtkMatrix4x4> matrix) // PBG: Precision Bone Guide
{
	T_PBGtoTargetImage = matrix;
}

void TKAN::SetPBGNaviMtxInTargetPos(vtkSmartPointer<vtkMatrix4x4> matrix)
{
	T_PBGNaviMtxInTargetPos = matrix;
}

void TKAN::SetPBLasertoPBImageTrans(vtkSmartPointer<vtkMatrix4x4> matrix)
{
	T_PBLasertoPBImage = matrix;
}

void TKAN::SetTibiaBGSrcPointset(mitk::PointSet::Pointer srcPointset)
{
	m_TibiaBGSrcPointset = srcPointset;
}

void TKAN::SetFemurBGSrcPointset(mitk::PointSet::Pointer srcPointset)
{
	m_FemurBGSrcPointset = srcPointset;
}

void TKAN::SetFourInOneBGSrcPointset(mitk::PointSet::Pointer srcPointset)
{
	m_FourInOneBGSrcPointset = srcPointset;
}

void TKAN::SetPlaneValidatorSrcPointset(mitk::PointSet::Pointer srcPointset)
{
	m_PlaneValidatorSrcPointset = srcPointset;
}

void TKAN::SetPrecisionBlockSrcPointset(mitk::PointSet::Pointer srcPointset)
{
	m_PrecisionBlockSrcPointset = srcPointset;
}

void TKAN::SetPrecisionBGSrcPointset(mitk::PointSet::Pointer srcPointset)
{
	m_PrecisionBGSrcPointset = srcPointset;
}

vtkSmartPointer<vtkMatrix4x4> TKAN::GetTibiaRFtoImageTrans()
{
	return T_TibiaRFtoImage;
}

vtkSmartPointer<vtkMatrix4x4> TKAN::GetFemurRFtoImageTrans() {
	return T_FemurRFtoImage;
}

vtkSmartPointer<vtkMatrix4x4> TKAN::GetTBGRFtoTBGImageTrans() // TBG: Tibia Bone Guide
{
	return T_TibiaBoneGuideRFtoTBGImage;
}

vtkSmartPointer<vtkMatrix4x4> TKAN::GetFBGRFtoFBGImageTrans() // FBG: FemurBoneGuide
{
	return T_FemurBoneGuideRFtoFBGImage;
}

vtkSmartPointer<vtkMatrix4x4> TKAN::GetFIOBGRFtoFIOBGImageTrans() // FIO: Four In One
{
	return  T_FourInOneBoneGuideRFtoFIOBGImage;
}
vtkSmartPointer<vtkMatrix4x4> TKAN::GetPVRFtoPVImageTrans() // PV: Plane Validator
{
	return  T_PlaneValidatorRFtoPVImage;
}

vtkSmartPointer<vtkMatrix4x4> TKAN::GetPBRFtoPBImageTrans() // PB: Precision Block
{
	return T_PrecisionBlockRFtoPBImage;
}

vtkSmartPointer<vtkMatrix4x4> TKAN::GetPBGRFtoPBGImageTrans() // PBG: Precision Bone Guide
{
	return T_PrecisionBoneGuideRFtoPBGImage;
}

// navigation transform matrix
vtkSmartPointer<vtkMatrix4x4> TKAN::GetPBGtoTargetImageTrans() // PBG: Precision Bone Guide
{
	return T_PBGtoTargetImage;
}

// navigation transform matrix when cutting plane in the target plane/pos 
vtkSmartPointer<vtkMatrix4x4> TKAN::GetPBGNaviMtxInTargetPos() // PBG: Precision Bone Guide
{
	return T_PBGNaviMtxInTargetPos;
}

vtkSmartPointer<vtkMatrix4x4> TKAN::GetPBLasertoPBImageTrans()
{
	return T_PBLasertoPBImage;
}


double TKAN::GetTibiaBGError() 
{
	return TibiaBGErr;
}

double TKAN::GetFemurBGError()
{
	return FemurBGErr;
}

double TKAN::GetFourInOneBGError()
{
	return FourInOneBGErr;
}

double TKAN::GetPlaneValidatorError()
{
	return PlaneValidatorErr;
}

double TKAN::GetPrecisionBlockError()
{
	return PrecisionBlockErr;
}

double TKAN::GetPrecisionBGError() 
{
	return PrecisionBGErr;
}