// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qmitk
#include "TotalKneeArthroplasty.h"
//#include "Registration.h"


// Qt
#include <QMessageBox>
#include <QButtonGroup>

// mitk image
#include <mitkImage.h>
#include "surfaceregistraion.h"

// igt
#include "mitkNodePredicateAnd.h"
#include "mitkNodePredicateDataType.h"
#include "mitkNodePredicateNot.h"
#include "mitkNodePredicateOr.h"
#include "mitkNodePredicateProperty.h"
#include "QmitkDataStorageTreeModel.h"
#include <QmitkSingleNodeSelectionWidget.h>
#include <vtkImplicitPolyDataDistance.h>

#include "Globals.h"
#include "TKAN.h"
#include "Helper.h"

void TotalKneeArthroplasty::InitSurfaceSelector(QmitkSingleNodeSelectionWidget* widget)
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

void TotalKneeArthroplasty::InitPointSetSelector(QmitkSingleNodeSelectionWidget* widget)
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

bool TotalKneeArthroplasty::CollectProbeTipUnderTargetRF(mitk::Point3D& TargetRFToProbeTip)
{
	std::string targetRF = getTargetRFName(); // selected by radio btn
	if (!toolStatusMap[targetRF] || !toolStatusMap["Probe"]) {  // check the targetRF/probe data status, DO NOT COLLECT when objectRF/BoneGuideRF->isDataValid()==false
		m_Controls.textBrowser->append("targetRF OR Probe Not Found!");
		return false;
	}

	// collect data, first: convert Probe Tip data under TargetRF coordinate
	// ProbeTip[3], TargetRF[16] -> TargetRFToProbeTip
	vtkSmartPointer<vtkMatrix4x4> vtkCameraToTargetRF = vtkSmartPointer<vtkMatrix4x4>::New();
	if (radioButtonID == 1) { // tibia RF
		vtkCameraToTargetRF = DoubleToVtkMatrix(nd_CameraToTibiaRF);
	}
	else if (radioButtonID == 2) { // femur RF
		vtkCameraToTargetRF = DoubleToVtkMatrix(nd_CameraToFemurRF);
	}
	else if (radioButtonID == 3 || radioButtonID == 4 || radioButtonID == 5 || radioButtonID == 8) { // Bone Guides
		vtkCameraToTargetRF = DoubleToVtkMatrix(nd_CameraToBoneGuideRF); // nd_CameraToTibiaBoneGuideRF
	}
	else if (radioButtonID == 6) { // plane validator
		vtkCameraToTargetRF = DoubleToVtkMatrix(nd_CameraToPlaneValidator);
	}
	else if (radioButtonID == 7) { // precision block
		vtkCameraToTargetRF = DoubleToVtkMatrix(nd_CameraToPrecisionBlockRF);
	}

	// coordinate transform, CameraToProbeTip -> TargetRFToProbeTip
	auto vtkTargetRFToProbeTip = getVtkMatrix4x4InRef(DoubleToVtkMatrix(nd_CameraToProbe), vtkCameraToTargetRF);

	TargetRFToProbeTip[0] = vtkTargetRFToProbeTip->GetElement(0, 3);
	TargetRFToProbeTip[1] = vtkTargetRFToProbeTip->GetElement(1, 3);
	TargetRFToProbeTip[2] = vtkTargetRFToProbeTip->GetElement(2, 3);

	return true;
}


// for collecting Landmark via a Probe.
void TotalKneeArthroplasty::CollectLandmarkProbe()
{
	if (!m_Controls.mitkNodeSelectWidget_landmark_src->GetSelectedNode()) {
		m_Controls.textBrowser->append("There is no 'Landmark src'!");
		return ;
	}

	// check the number of the collected landmark
	auto landmarkSrcNode = m_Controls.mitkNodeSelectWidget_landmark_src->GetSelectedNode();
	int requiredDataNum = dynamic_cast<mitk::PointSet*>(landmarkSrcNode->GetData())->GetSize();
	if (m_RfToProbeLandmarkPointSet && m_RfToProbeLandmarkPointSet->GetSize() == requiredDataNum) {
		m_Controls.textBrowser->append("--- Enough landmarks have been collected ----");
		return;
	}

	/*std::string targetRF = getTargetRFName(); // selected by radio btn
	if (!toolStatusMap[targetRF] || !toolStatusMap["Probe"]) {  // check the targetRF/probe data status, DO NOT COLLECT when objectRF/BoneGuideRF->isDataValid()==false
		m_Controls.textBrowser->append("targetRF OR Probe Not Found!");
		return;
	}

	// collect data, first: convert Probe Tip data under TargetRF coordinate
	// ProbeTip[3], TargetRF[16] -> TargetRFToProbeTip
	vtkSmartPointer<vtkMatrix4x4> vtkCameraToTargetRF = vtkSmartPointer<vtkMatrix4x4>::New();
	if (radioButtonID == 1) { // tibia RF
		vtkCameraToTargetRF = DoubleToVtkMatrix(nd_CameraToTibiaRF);
	}
	else if (radioButtonID == 2) { // femur RF
		vtkCameraToTargetRF = DoubleToVtkMatrix(nd_CameraToFemurRF);
	}
	else if (radioButtonID == 3 || radioButtonID == 4 || radioButtonID == 5) { // Bone Guides
		vtkCameraToTargetRF = DoubleToVtkMatrix(nd_CameraToBoneGuideRF); // nd_CameraToTibiaBoneGuideRF
	}
	else if (radioButtonID == 6) { // plane validator
		vtkCameraToTargetRF = DoubleToVtkMatrix(nd_CameraToPlaneValidator);
	}

	// coordinate transform, CameraToProbeTip -> TargetRFToProbeTip
	auto vtkTargetRFToProbeTip = getVtkMatrix4x4InRef(DoubleToVtkMatrix(nd_CameraToProbe), vtkCameraToTargetRF);

	mitk::Point3D TargetRFToProbeTip;
	TargetRFToProbeTip[0] = vtkTargetRFToProbeTip->GetElement(0, 3);
	TargetRFToProbeTip[1] = vtkTargetRFToProbeTip->GetElement(1, 3);
	TargetRFToProbeTip[2] = vtkTargetRFToProbeTip->GetElement(2, 3);*/

	mitk::Point3D TargetRFToProbeTip; 
	if (!CollectProbeTipUnderTargetRF(TargetRFToProbeTip)) return; // // can't find targetRF or Probe.

	//pointSet_probeLandmark->InsertPoint(probeTipPointUnderRf);
	if (!m_RfToProbeLandmarkPointSet) m_RfToProbeLandmarkPointSet = mitk::PointSet::New(); // init
	m_RfToProbeLandmarkPointSet->InsertPoint(TargetRFToProbeTip); // insert probe tip coordinate into PointSet.

	m_Controls.textBrowser->append("Added landmark " + QString::number(m_RfToProbeLandmarkPointSet->GetSize()) + ": " + QString::number(TargetRFToProbeTip[0]) +
		"/ " + QString::number(TargetRFToProbeTip[1]) + "/ " + QString::number(TargetRFToProbeTip[2]) );


	/// ******* original code for NDI Camera, need to MODIFY ************////
	/*
	if (!m_Controls.mitkNodeSelectWidget_landmark_src->GetSelectedNode()) {
		m_Controls.textBrowser->append("There is no 'Landmark src'!");
		return;
	}

	// check the number of the collected landmark
	auto landmarkSrcNode = m_Controls.mitkNodeSelectWidget_landmark_src->GetSelectedNode();
	int requiredDataNum = dynamic_cast<mitk::PointSet*>(landmarkSrcNode->GetData())->GetSize();
	if (m_RfToProbeLandmarkPointSet && m_RfToProbeLandmarkPointSet->GetSize() == requiredDataNum) {
		m_Controls.textBrowser->append("--- Enough landmarks have been collected ----");
		return;
	}

	// get navigation data in ndi/camera coords
	auto probeIndex = m_ToolStorage->GetToolIndexByName("Probe"); // probe
	auto targetRfIndex = getTargetRFIdx(); // auto objectRfIndex = m_ToolStorage->GetToolIndexByName("TibiaRF"); // bone
	if (probeIndex == -1 || targetRfIndex == -1)
	{
		m_Controls.textBrowser->append("There is no 'Probe' or 'ObjectRf' in the toolStorage!");
		return ;
	}
	
	mitk::NavigationData::Pointer nd_ndiToProbe = m_CameraSource->GetOutput(probeIndex); // get data of Probe under "ndi/camera" coordinate
	//nd_ndiToProbe->IsDataValid(); // check the Probe status. (be detected(1) or not(0))
	mitk::NavigationData::Pointer nd_ndiToTargetRf = m_CameraSource->GetOutput(targetRfIndex);  // get data of Target/Object/Patient RF
	mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToTargetRf); // ObjRF_Ndi * Ndi_Probe -> ObjRFPorbe

	mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition(); // "probe tip" under target/Object/Patient coordinate.

	//pointSet_probeLandmark->InsertPoint(probeTipPointUnderRf);
	if (!m_RfToProbeLandmarkPointSet) m_RfToProbeLandmarkPointSet = mitk::PointSet::New(); // init
	m_RfToProbeLandmarkPointSet->InsertPoint(probeTipPointUnderRf); // insert probe tip coordinate into PointSet.
	//MITK_INFO << m_probeLandmarkPointSet->GetSize();
	
	m_Controls.textBrowser->append("Added landmark " + QString::number(m_RfToProbeLandmarkPointSet->GetSize()) + ": " + QString::number(probeTipPointUnderRf[0]) +
		"/ " + QString::number(probeTipPointUnderRf[1]) + "/ " + QString::number(probeTipPointUnderRf[2]));

	*/
}

void TotalKneeArthroplasty::CollectIcpProbe()
{
	mitk::Point3D TargetRFToProbeTip;
	if (!CollectProbeTipUnderTargetRF(TargetRFToProbeTip)) return; // can't find targetRF or Probe.

	if (!m_RfToProbeICPLandmarkPointSet) m_RfToProbeICPLandmarkPointSet = mitk::PointSet::New(); // init
	m_RfToProbeICPLandmarkPointSet->InsertPoint(TargetRFToProbeTip); // insert probe tip coordinate into PointSet.

	m_Controls.textBrowser->append("Added ICP " + QString::number(m_RfToProbeICPLandmarkPointSet->GetSize()) + ": " + QString::number(TargetRFToProbeTip[0]) +
		"/ " + QString::number(TargetRFToProbeTip[1]) + "/ " + QString::number(TargetRFToProbeTip[2]));

}

void TotalKneeArthroplasty::ResetLandmarkCollection()
{
	if (m_RfToProbeLandmarkPointSet) {
		m_RfToProbeLandmarkPointSet->Clear();
	}
	
	m_Controls.textBrowser->clear();
}

void TotalKneeArthroplasty::ResetIcpCollection()
{
	if (m_RfToProbeICPLandmarkPointSet) {
		m_RfToProbeICPLandmarkPointSet->Clear();
	}

	m_Controls.textBrowser->clear();
}

void TotalKneeArthroplasty::DeleteLastLandmark()
{
	if (m_RfToProbeLandmarkPointSet && m_RfToProbeLandmarkPointSet->GetSize() > 0) {
		m_RfToProbeLandmarkPointSet->RemovePointAtEnd(); // remove the last point.

		// iterate the points in m_RfToProbeLandmarkPointSet, update the textBrowser.
		m_Controls.textBrowser->clear();
		for (auto it = m_RfToProbeLandmarkPointSet->Begin(); it != m_RfToProbeLandmarkPointSet->End(); ++it) {
			int idx = it->Index() + 1; // start at 1
			auto point = it->Value();
			m_Controls.textBrowser->append("Added landmark " + QString::number(idx) + ": " + QString::number(point[0]) +
				"/ " + QString::number(point[1]) + "/ " + QString::number(point[2]));
		}

		//// remove the last row of text in textBrowser;
		//QString content = m_Controls.textBrowser->toPlainText();
		//m_Controls.textBrowser->clear();
		//QStringList lines = content.split('\n');
		//int targetLine = m_RfToProbeLandmarkPointSet->GetSize();
		//if (targetLine >= 0 && targetLine < lines.size()) { // check
		//	lines.removeAt(targetLine);
		//	m_Controls.textBrowser->setPlainText(lines.join('\n'));
		//}
	}
}

void TotalKneeArthroplasty::DeleteLastIcp()
{
	if (m_RfToProbeICPLandmarkPointSet && m_RfToProbeICPLandmarkPointSet->GetSize() > 0) {
		m_RfToProbeICPLandmarkPointSet->RemovePointAtEnd(); // remove the last point.

		// iterate the points in m_RfToProbeLandmarkPointSet, update the textBrowser.
		m_Controls.textBrowser->clear();
		for (auto it = m_RfToProbeICPLandmarkPointSet->Begin(); it != m_RfToProbeICPLandmarkPointSet->End(); ++it) {
			int idx = it->Index() + 1; // start at 1
			auto point = it->Value();
			m_Controls.textBrowser->append("Added ICP " + QString::number(idx) + ": " + QString::number(point[0]) +
				"/ " + QString::number(point[1]) + "/ " + QString::number(point[2]));
		}
	}
}

void TotalKneeArthroplasty::PrintAllLandmarks() 
{
	if (m_RfToProbeLandmarkPointSet && m_RfToProbeLandmarkPointSet->GetSize() > 0) {
		// iterate the points in m_RfToProbeLandmarkPointSet, update the textBrowser.
		m_Controls.textBrowser->clear();
		for (auto it = m_RfToProbeLandmarkPointSet->Begin(); it != m_RfToProbeLandmarkPointSet->End(); ++it) {
			int idx = it->Index() + 1; // start at 1
			auto point = it->Value();
			m_Controls.textBrowser->append("Added landmark " + QString::number(idx) + ": " + QString::number(point[0]) +
				"/ " + QString::number(point[1]) + "/ " + QString::number(point[2]));
		}
	}
}

void TotalKneeArthroplasty::PrintAllIcps() 
{
	if (m_RfToProbeICPLandmarkPointSet && m_RfToProbeICPLandmarkPointSet->GetSize() > 0) {
		// iterate the points in m_RfToProbeICPLandmarkPointSet, update the textBrowser.
		m_Controls.textBrowser->clear();
		for (auto it = m_RfToProbeICPLandmarkPointSet->Begin(); it != m_RfToProbeICPLandmarkPointSet->End(); ++it) {
			int idx = it->Index() + 1; // start at 1, just for vis.
			auto point = it->Value();
			m_Controls.textBrowser->append("Added ICP " + QString::number(idx) + ": " + QString::number(point[0]) +
				"/ " + QString::number(point[1]) + "/ " + QString::number(point[2]));
		}
	}
}
//Registration imageRegistration;


void TotalKneeArthroplasty::ApplySurfaceRegistration()
{
	auto surfaceNode = m_Controls.mitkNodeSelectWidget_surface_regis->GetSelectedNode();
	auto landmarkSrcNode = m_Controls.mitkNodeSelectWidget_landmark_src->GetSelectedNode();

	if (surfaceNode == nullptr || landmarkSrcNode == nullptr)
	{
		m_Controls.textBrowser->append("Source surface or source landmarks is not ready!");
		return;
	}

	if (radioButtonID == 1) { // Tibia RF
		Tkan.TibiaImageRegistration(dynamic_cast<mitk::PointSet*>(landmarkSrcNode->GetData()), m_RfToProbeLandmarkPointSet, dynamic_cast<mitk::Surface*>(surfaceNode->GetData()));
	}
	else if (radioButtonID == 2) { // Femur RF
		Tkan.FemurImageRegistration(dynamic_cast<mitk::PointSet*>(landmarkSrcNode->GetData()), m_RfToProbeLandmarkPointSet, dynamic_cast<mitk::Surface*>(surfaceNode->GetData()));
	}
	else if (radioButtonID == 3) { // TibiaBoneGuideRF
		Tkan.TibiaBoneGuideRegistration(dynamic_cast<mitk::PointSet*>(landmarkSrcNode->GetData()), m_RfToProbeLandmarkPointSet, dynamic_cast<mitk::Surface*>(surfaceNode->GetData()));
		m_Controls.textBrowser->append("Tibia Bone Guide Error After do reg: " + QString::number(Tkan.GetTibiaBGError()));
	}
	else if (radioButtonID == 4) { // DistalFemurBoneGuide
		Tkan.FemurBoneGuideRegistration(dynamic_cast<mitk::PointSet*>(landmarkSrcNode->GetData()), m_RfToProbeLandmarkPointSet, dynamic_cast<mitk::Surface*>(surfaceNode->GetData()));
		m_Controls.textBrowser->append("Femur Bone Guide Error After do reg: " + QString::number(Tkan.GetFemurBGError()));
	}
	else if (radioButtonID == 5) { // 4in1BoneGuide
		Tkan.FourInOneBoneGuideRegistration(dynamic_cast<mitk::PointSet*>(landmarkSrcNode->GetData()), m_RfToProbeLandmarkPointSet, dynamic_cast<mitk::Surface*>(surfaceNode->GetData()));
		m_Controls.textBrowser->append("4-in-1 Bone Guide Error After do reg: " + QString::number(Tkan.GetFourInOneBGError()));
	}

	else if (radioButtonID == 6) { // Plane Validator
		Tkan.PlaneValidatorRegistration(dynamic_cast<mitk::PointSet*>(landmarkSrcNode->GetData()), m_RfToProbeLandmarkPointSet, dynamic_cast<mitk::Surface*>(surfaceNode->GetData()));
		m_Controls.textBrowser->append("Plane Validator Error After do reg: " + QString::number(Tkan.GetPlaneValidatorError()));
	}

	else if (radioButtonID == 7) { // precision block image
		Tkan.PrecisionBlockRegistration(dynamic_cast<mitk::PointSet*>(landmarkSrcNode->GetData()), m_RfToProbeLandmarkPointSet, dynamic_cast<mitk::Surface*>(surfaceNode->GetData()));
		//m_Controls.textBrowser->append("Plane Validator Error After do reg: " + QString::number(Tkan.GetPrecisionBlockError()));
	}
	else if (radioButtonID == 8) { // precision Test Bone Guide
		Tkan.PrecisionBGRegistration(dynamic_cast<mitk::PointSet*>(landmarkSrcNode->GetData()), m_RfToProbeLandmarkPointSet, dynamic_cast<mitk::Surface*>(surfaceNode->GetData()));
		m_Controls.textBrowser->append("Plane Validator Error After do reg: " + QString::number(Tkan.GetPrecisionBGError()));
	}
	

	////// Do Landmark Registration, cal the transform from srcLandmark(probeTipUnderRF/ObjectRFToProbe) to dstLandmark(image)
	//auto landmarkRegistrator = mitk::SurfaceRegistration::New();
	//landmarkRegistrator->SetLandmarksSrc(m_RfToProbeLandmarkPointSet);
	//landmarkRegistrator->SetLandmarksTarget(dynamic_cast<mitk::PointSet*>(landmarkSrcNode->GetData()));
	//landmarkRegistrator->ComputeLandMarkResult(); // get the matrix from RF to image

	//auto landmarkResult = landmarkRegistrator->GetResult();

	//if (radioButtonID == 1) { // ObjectRF/BoneRF/PatientRF 
	//	m_T_patientRFtoImage = landmarkResult;
	//}
	//else if (radioButtonID == 2) { // TibiaBoneGuideRF
	//	m_T_tibiaBoneGuideRFtoTBGImage = landmarkResult;
	//}
	//else if (radioButtonID == 3) { // DistalFemurBoneGuide
	//	// TODO
	//}
	//else if (radioButtonID == 4) { // 4in1BoneGuide
	//	// TODO
	//}
}

void TotalKneeArthroplasty::ApplyICPRegistration()
{
	auto surfaceNode = m_Controls.mitkNodeSelectWidget_surface_regis->GetSelectedNode();

	if (surfaceNode == nullptr || m_RfToProbeICPLandmarkPointSet->GetSize() == 0)
	{
		m_Controls.textBrowser->append("Source surface or ICP Points is not ready!");
		return;
	}

	if (radioButtonID == 1) { // Tibia RF
		Tkan.TibiaImageICP(m_RfToProbeICPLandmarkPointSet, dynamic_cast<mitk::Surface*>(surfaceNode->GetData()));
	}
	else if (radioButtonID == 2) { // Femur RF
		Tkan.FemurImageICP(m_RfToProbeICPLandmarkPointSet, dynamic_cast<mitk::Surface*>(surfaceNode->GetData()));
	}
	else if (radioButtonID == 3) { // TibiaBoneGuideRF
		Tkan.TibiaBoneGuideICP(m_RfToProbeICPLandmarkPointSet, dynamic_cast<mitk::Surface*>(surfaceNode->GetData()));
	}
	else if (radioButtonID == 4) { // DistalFemurBoneGuide
		Tkan.FemurBoneGuideICP(m_RfToProbeICPLandmarkPointSet, dynamic_cast<mitk::Surface*>(surfaceNode->GetData()));
	}
	else if (radioButtonID == 5) { // 4in1BoneGuide
		Tkan.FourInOneBoneGuideICP(m_RfToProbeICPLandmarkPointSet, dynamic_cast<mitk::Surface*>(surfaceNode->GetData()));
	}

	else if (radioButtonID == 6) { // Plane Validator
		Tkan.PlaneValidatorICP(m_RfToProbeICPLandmarkPointSet, dynamic_cast<mitk::Surface*>(surfaceNode->GetData()));
	}

	else if (radioButtonID == 7) { // precision block image
		Tkan.PrecisionBlockICP(m_RfToProbeICPLandmarkPointSet, dynamic_cast<mitk::Surface*>(surfaceNode->GetData()));
	}
	else if (radioButtonID == 8) { // precision Test Bone Guide
		Tkan.PrecisionBoneGuideICP(m_RfToProbeICPLandmarkPointSet, dynamic_cast<mitk::Surface*>(surfaceNode->GetData()));
	}

}


///// abandon, currently do registration by TKAN's function.
vtkSmartPointer<vtkMatrix4x4> TotalKneeArthroplasty::LandmarkRegistration(mitk::PointSet::Pointer srcPointset, mitk::PointSet::Pointer tarPointset, mitk::Surface::Pointer surface)
{
	if (surface == nullptr || srcPointset == nullptr)
	{
		//m_Controls.textBrowser->append("Source surface or source landmarks is not ready!");
		return nullptr;
	}

	//// Do Landmark Registration, cal the transform from srcLandmark(probeTipUnderRF/ObjectRFToProbe) to dstLandmark(image)
	auto landmarkRegistrator = mitk::SurfaceRegistration::New();
	landmarkRegistrator->SetLandmarksSrc(tarPointset);
	landmarkRegistrator->SetLandmarksTarget(srcPointset);
	landmarkRegistrator->ComputeLandMarkResult(); // get the matrix from RF to image

	auto landmarkResult = landmarkRegistrator->GetResult();
	
	return landmarkResult;
}

// for Bone Guide/Plane Validator Calibration Error Calculation.
void TotalKneeArthroplasty::ErrorEvaluation()
{
	auto landmarkSrcNode = m_Controls.mitkNodeSelectWidget_landmark_src->GetSelectedNode();
	//if (radioButtonID == 1) { // ObjectRF/BoneRF/PatientRF 
	//	// (Bone Validation) do not need validate in this CLASS
	//}
	
	if (radioButtonID == 3) { // TibiaBoneGuideRF
		Tkan.SetTibiaBGSrcPointset(dynamic_cast<mitk::PointSet*>(landmarkSrcNode->GetData())); // in case of empty pointset.
		Tkan.TibiaBGValidation(nd_CameraToProbe, nd_CameraToBoneGuideRF);
		m_Controls.textBrowser->append("Tibia Bone Guide Error: " + QString::number(Tkan.GetTibiaBGError()));
	}
	else if (radioButtonID == 4) { // FemurBoneGuide
		Tkan.SetFemurBGSrcPointset(dynamic_cast<mitk::PointSet*>(landmarkSrcNode->GetData()));
		Tkan.FemurBGValidation(nd_CameraToProbe, nd_CameraToBoneGuideRF);
		m_Controls.textBrowser->append("Femur Bone Guide Error: " + QString::number(Tkan.GetFemurBGError()));
	}
	else if (radioButtonID == 5) { // 4in1BoneGuide
		Tkan.SetFourInOneBGSrcPointset(dynamic_cast<mitk::PointSet*>(landmarkSrcNode->GetData()));
		Tkan.FourInOneBGValidation(nd_CameraToProbe, nd_CameraToBoneGuideRF);
		m_Controls.textBrowser->append("4-in-1 Bone Guide Error: " + QString::number(Tkan.GetFourInOneBGError()));
	}
	else if (radioButtonID == 6) { // Plane Validator
		Tkan.SetPlaneValidatorSrcPointset(dynamic_cast<mitk::PointSet*>(landmarkSrcNode->GetData()));
		Tkan.PlaneValidatorValidation(nd_CameraToProbe, nd_CameraToPlaneValidator);
		m_Controls.textBrowser->append("Plane Validator Error: " + QString::number(Tkan.GetPlaneValidatorError()));
	}
	else if (radioButtonID == 7) { // precision block image
		Tkan.SetPrecisionBlockSrcPointset(dynamic_cast<mitk::PointSet*>(landmarkSrcNode->GetData()));
		Tkan.PrecisionBlockValidation(nd_CameraToProbe, nd_CameraToPrecisionBlockRF);
		m_Controls.textBrowser->append("Precision Block Error: " + QString::number(Tkan.GetPrecisionBlockError()));
	}
	else if (radioButtonID == 8) { // precision Test Bone Guide
		Tkan.SetPrecisionBGSrcPointset(dynamic_cast<mitk::PointSet*>(landmarkSrcNode->GetData()));
		Tkan.PrecisionBGValidation(nd_CameraToProbe, nd_CameraToBoneGuideRF);
		m_Controls.textBrowser->append("Precision Bone Guide Error: " + QString::number(Tkan.GetPrecisionBGError()));
	}
	else {
		m_Controls.textBrowser->append("Tibia and Femur do not have Error Evaluation");
	}

}
