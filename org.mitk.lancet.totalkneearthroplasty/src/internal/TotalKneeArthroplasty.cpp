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
#include "TotalKneeArthroplasty.h"


// Qt
#include <QMessageBox>
#include <QRadioButton>
#include <QButtonGroup>
#include <QComboBox>
#include <QWidget>
#include <QVBoxLayout>

// mitk image
#include <mitkImage.h>
#include <mitkAffineTransform3D.h>
#include <mitkMatrixConvert.h>
#include <mitkPoint.h>
#include <mitkSurface.h>
#include <QmitkRenderWindow.h>
#include <mitkIRenderWindowPart.h>
#include <mitkRenderWindow.h>
#include <mitkPlaneGeometry.h>
#include <mitkRenderingManager.h>
#include <mitkDataNode.h>
#include <mitkBaseRenderer.h>
//#include <mitkCrosshairManager.h>
#include <mitkSliceNavigationController.h>
#include <QmitkStdMultiWidget.h>

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
#include <windows.h>
#include <iostream>
#include <vtkMath.h>
#include <vtkCamera.h>

//igt
#include <lancetVegaTrackingDevice.h>
#include <kukaRobotDevice.h>
#include <lancetApplyDeviceRegistratioinFilter.h>
#include <mitkNavigationDataToPointSetFilter.h>
#include <lancetPathPoint.h>
#include <vtkQuaternion.h>

#include "lancetTrackingDeviceSourceConfigurator.h"
#include "mitkNavigationToolStorageDeserializer.h"
#include <QtWidgets\qfiledialog.h>

#include "mitkIGTIOException.h"
#include "mitkNavigationToolStorageSerializer.h"
#include "QmitkIGTCommonHelper.h"
#include "lancetTreeCoords.h"

//#include <ep\include\vtk-9.1\vtkCenterOfMass.h>
//#include <ep\include\vtk-9.1\vtkPolyDataNormals.h>

const std::string TotalKneeArthroplasty::VIEW_ID = "org.mitk.views.totalkneearthroplasty";

// custom header
#include "TKAN.h"
#include "Globals.h"
#include "Helper.h"
#include "ProbeInteractor.h"

//Aimooe
#include "AimPositionAPI.h"
#include "AimPositionDef.h"

#include <fstream>
#include <iostream>
#include <string>
#include <cstdlib>
#include <filesystem>


void TotalKneeArthroplasty::SetFocus()
{

}

void TotalKneeArthroplasty::CreateQtPartControl(QWidget *parent)
{
	// create GUI widgets from the Qt Designer's .ui file
	m_Controls.setupUi(parent);

	// init
	InitSurfaceSelector(m_Controls.mitkNodeSelectWidget_surface_regis); // bone stl surface 
	InitPointSetSelector(m_Controls.mitkNodeSelectWidget_landmark_src); // bone landmarks src

	InitPointSetSelector(m_Controls.mitkNodeSelectWidget_CuttingPlanePoints); // Bone Guide Cutting Plane Pointset.(default: 3 points)
  
	//InitSurfaceSelector(m_Controls.mitkNodeSelectWidget_surface_regis_TibiaBoneGuide); // Tibia bone guide(cutting block) surface
	//InitPointSetSelector(m_Controls.mitkNodeSelectWidget_landmark_src_TibiaBoneGuide); // Tibia bone guide(cutting block) landmarks src

	// connect with camera btn
	connect(m_Controls.pushButton_connectVega, &QPushButton::clicked, this, &TotalKneeArthroplasty::UseVega); // connect with NDI vega Tracking Device.
	connect(m_Controls.pushButton_connectRuitongPro, &QPushButton::clicked, this, &TotalKneeArthroplasty::UseRuitongPro);
	connect(m_Controls.pushButton_connectAimooe, &QPushButton::clicked, this, &TotalKneeArthroplasty::UseAimooe);

  
	// Tab 2: Registration / Calibration

	// connct with raido btn
	radioButtonGroup = new QButtonGroup(this);
	radioButtonGroup->addButton(m_Controls.radioButton_tibiaImg, 1);
	radioButtonGroup->addButton(m_Controls.radioButton_femurImg, 2);
	radioButtonGroup->addButton(m_Controls.radioButton_tibiaBG, 3);
	radioButtonGroup->addButton(m_Controls.radioButton_distalFemurBG, 4);
	radioButtonGroup->addButton(m_Controls.radioButton_4in1BG, 5);
	radioButtonGroup->addButton(m_Controls.radioButton_planeValidator, 6);
	radioButtonGroup->addButton(m_Controls.radioButton_precisionBlock, 7);
	radioButtonGroup->addButton(m_Controls.radioButton_precisionBG, 8);
	connect(radioButtonGroup, QOverload<QAbstractButton*, bool>::of(&QButtonGroup::buttonToggled), this, &TotalKneeArthroplasty::onRadioButtonToggled);
	
	// connect with 4-in-1 combox box 
	connect(m_Controls.comboBox_4in1, SIGNAL(currentIndexChanged(int)), this, SLOT(onComboBoxIndexChanged(int)));

	// connect with Precision Planning Plane combo box
	connect(m_Controls.comboBox_precisionPlanningPlane, SIGNAL(currentIndexChanged(int)), this, SLOT(onComboBoxIndexChanged_precision(int)));
  
	// connnect with CheckBox btn.
	connect(m_Controls.checkBox_collectPermission, &QPushButton::clicked, this, &TotalKneeArthroplasty::onCollectPermissionCheckButtonClicked);
	connect(m_Controls.checkBox_collectPermission_icp, &QPushButton::clicked, this, &TotalKneeArthroplasty::onCollectPermissionCheckButtonClicked_icp);
	connect(m_Controls.checkBox_validatePermission, &QPushButton::clicked, this, &TotalKneeArthroplasty::onValidatePermissionCheckButtonClicked);

	// connect with btn
	connect(m_Controls.pushButton_collectLandmark, &QPushButton::clicked, this, &TotalKneeArthroplasty::CollectLandmarkProbe); // collect landmarks
	connect(m_Controls.pushButton_resetLandmarkCollection, &QPushButton::clicked, this, &TotalKneeArthroplasty::ResetLandmarkCollection); // clear all collected landmarks
	connect(m_Controls.pushButton_deleteLastLandmark, &QPushButton::clicked, this, &TotalKneeArthroplasty::DeleteLastLandmark); // delete last/previous landmark 
	connect(m_Controls.pushButton_getLandmarks, &QPushButton::clicked, this, &TotalKneeArthroplasty::PrintAllLandmarks); // print all landmarks.

	connect(m_Controls.pushButton_collectIcp, &QPushButton::clicked, this, &TotalKneeArthroplasty::CollectIcpProbe); // collect ICP (Iterative Closest Point) points
	connect(m_Controls.pushButton_resetICPCollection, &QPushButton::clicked, this, &TotalKneeArthroplasty::ResetIcpCollection); // clear all collected ICP pts
	connect(m_Controls.pushButton_deleteLastICP, &QPushButton::clicked, this, &TotalKneeArthroplasty::DeleteLastIcp); // delete last/previous ICP pts
	connect(m_Controls.pushButton_getIcps, &QPushButton::clicked, this, &TotalKneeArthroplasty::PrintAllIcps); // print all ICPs

	connect(m_Controls.pushButton_applyRegistration, &QPushButton::clicked, this, &TotalKneeArthroplasty::ApplySurfaceRegistration); // landmark registration
	connect(m_Controls.pushButton_applyRegistration_icp, &QPushButton::clicked, this, &TotalKneeArthroplasty::ApplyICPRegistration); // ICP registration
	connect(m_Controls.pushButton_p2pValidation, &QPushButton::clicked, this, &TotalKneeArthroplasty::ErrorEvaluation); // Point to Point Error Computation (distance).
  
	connect(m_Controls.pushButton_printTransMtx, &QPushButton::clicked, this, &TotalKneeArthroplasty::PrintTransformMatrix);
	connect(m_Controls.pushButton_saveTransMtx, &QPushButton::clicked, this, &TotalKneeArthroplasty::saveTransformMatrix); // save tranform matrix in txt file
	connect(m_Controls.pushButton_reloadTransMtx, &QPushButton::clicked, this, &TotalKneeArthroplasty::reloadTransformMatrix); // reload tranform matrix in txt file

	// Tab 1: Plane Generation
	connect(m_Controls.pushButton_generatePlaneByPointset, &QPushButton::clicked, this, &TotalKneeArthroplasty::GeneratePlaneByPointset);

	// Tab 3: Bone Guide Navigation 
	connect(m_Controls.checkBox_BoneGuideNavigationPermission, &QPushButton::clicked, this, &TotalKneeArthroplasty::onGuideNavigationCheckButtonClicked); // enable "UpdateBoneGuidePosition" function, disable "UpdateProbePosition" function
  
	// Tab 4: Precision Test, for entering the result of Laser and Bond the PBG plane with Bone Guide;
	connect(m_Controls.pushButton_printNaviTransMtx, &QPushButton::clicked, this, &TotalKneeArthroplasty::printNaviTransformMatrix); // print BG Navigation tranform matrix 
	connect(m_Controls.pushButton_saveNaviTransMtx, &QPushButton::clicked, this, &TotalKneeArthroplasty::saveNaviTransformMatrix); // save BG Navigation Tranform Matrix in txt file
	connect(m_Controls.pushButton_reloadNaviTransMtx, &QPushButton::clicked, this, &TotalKneeArthroplasty::reloadNaviTransformMatrix); // reload BG Navigation tranform matrix in txt file
  
	connect(m_Controls.pushButton_genLaserPointNPlane, &QPushButton::clicked, this, &TotalKneeArthroplasty::generateLaserPointsNPlanes); // generate the 6 points, and their planes.
  
	connect(m_Controls.pushButton_caliWithPointID, &QPushButton::clicked, this, &TotalKneeArthroplasty::calibrateWithPointID); // do calibration with laser planning plane (Pointset) and corresponding PointID(A-F). cal the cali trans mtx
  
	connect(m_Controls.checkBox_precCuttingPlaneSwitch, &QPushButton::clicked, this, &TotalKneeArthroplasty::onPrecCuttingPlaneCheckButtonClicked);
  
	/*m_T_patientRFtoImage = DoubleToVtkMatrix(m_T_patientRFtoImage_fake);
	m_T_tibiaBoneGuideRFtoTBGImage = DoubleToVtkMatrix(m_T_tibiaBoneGuideRFtoTBGImage_fake);*/

	Tkan.SetTibiaRFtoImageTrans(DoubleToVtkMatrix(m_T_tibiaRFtoImage_fake)); // set tibiaRF reg trans mtx
	Tkan.SetFemurRFtoImageTrans(DoubleToVtkMatrix(m_T_FemurRFtoImage_fake)); // set femurRF reg trans mtx
  
	Tkan.SetTBGRFtoTBGImageTrans(DoubleToVtkMatrix(m_T_TBGRFtoTBGImage_fake)); // set tibia Bone Guide
	Tkan.SetFIOBGRFtoFIOBGImageTrans(DoubleToVtkMatrix(m_T_FIOBGRFtoFIOBGImage_fake)); // set 4-in-1 cali trans mtx
	//Tkan.SetFBGRFtoFBGImageTrans(DoubleToVtkMatrix(m_T_FBGRFtoFBGImage_fake)); // set femur cali trans mtx
	Tkan.SetPVRFtoPVImageTrans(DoubleToVtkMatrix(m_T_PVRFtoPVImage_fake)); // set plane validator cali trans mtx

	Tkan.SetPBRFtoPBImageTrans(DoubleToVtkMatrix(m_T_PBRFtoPBImage_fake)); // set precision block RF trans mtx;
	Tkan.SetPBGRFtoPBGImageTrans(DoubleToVtkMatrix(m_T_PBGRFtoPBGImage_fake)); // set precision Bone Guide RF Trans mtx


}

void TotalKneeArthroplasty::resetView() 
{
	auto iRenderWindowPart = GetRenderWindowPart();
	//QmitkRenderWindow* renderWindow = iRenderWindowPart->GetQmitkRenderWindow("axial");
	//renderWindow->ResetView();
	
	for (auto& [k, name] : planeDataName) { // unsee all planning plane
		GetDataStorage()->GetNamedNode(name)->SetVisibility(false);
	}
	for (auto& [k, name] : boneGuideName) { // unsee all Bone Guide
		GetDataStorage()->GetNamedNode(name)->SetVisibility(false);
	}

	GetDataStorage()->GetNamedNode("Probe")->SetVisibility(false); 

	for (auto renderWindow : iRenderWindowPart->GetQmitkRenderWindows()) {
		renderWindow->ResetView();
	}
}

void TotalKneeArthroplasty::onComboBoxIndexChanged(int index)
{
	/// 0: AnteriorCutPlane
	/// 1: AnteriorChamferCutPlane 
	/// 2: PosteriorCutPlane 
	/// 3: PosteriorChamferCutPlane
	QComboBox* comboBox = qobject_cast<QComboBox*>(sender());
	if (comboBox)
	{
		fourInOneComboIdx = comboBox->currentIndex();
		QString selectedText = comboBox->currentText();
		m_Controls.textBrowser->append(QString::number(fourInOneComboIdx) + ": " +selectedText);

		 FourInOneAdjustCrosshair();
	}
}

void TotalKneeArthroplasty::onComboBoxIndexChanged_precision(int index) {
	/// 0: ABF, 1: ACE, 2: BCD, 3, ..., 8
	QComboBox* comboBox = qobject_cast<QComboBox*>(sender());
	if (comboBox)
	{
		auto idx = comboBox->currentIndex();
		QString selectedText = comboBox->currentText();
		precPlanPlaneName = selectedText.toStdString();
		m_Controls.textBrowser->append("Current Precision Planning Plane: " + QString::number(idx) + "-" + selectedText);

		// change the planningPlane
		planningPlane = GetDataStorage()->GetNamedNode(planeDataName[precPlanPlaneName]);
		for (auto& [k, name] : planeDataName) { // unsee all planning plane
			GetDataStorage()->GetNamedNode(name)->SetVisibility(false);
		}
		planningPlane->SetVisibility(true);
		mitk::RenderingManager::GetInstance()->RequestUpdateAll();

		// adjust the pointset in "precPlanningPointSet"
		mitk::PointSet::Pointer originalPointSet = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("CT steelball centers")->GetData());
		std::unordered_set<int> alphaSet;
		for (auto alpha : precPlanPlaneName) {
			alphaSet.insert(alpha - 'A');
		}
		//precPlanningPoints.clear();
		precPlanningPointSet->Clear();
		for (int i = 0; i < originalPointSet->GetSize(); ++i)
		{
			if (alphaSet.count(i)) {
				//m_Controls.textBrowser->append(QString::number(i));
				//const double* pointArray = originalPointSet->GetPoint(i).GetDataPointer();
				//precPlanningPoints.push_back({ pointArray[0], pointArray[1], pointArray[2] });
				//m_Controls.textBrowser->append(QString::number(pointArray[0]) + " " + QString::number(pointArray[1]) + " " + QString::number(pointArray[2]));
				precPlanningPointSet->InsertPoint(originalPointSet->GetPoint(i));
			}
		}
		for (int i = 0; i < precPlanningPointSet->GetSize(); i++) {
			const double* pointArray = precPlanningPointSet->GetPoint(i).GetDataPointer();
			m_Controls.textBrowser->append(QString::number(pointArray[0]) + " " + QString::number(pointArray[1]) + " " + QString::number(pointArray[2]));
		}
	}
}

void TotalKneeArthroplasty::FourInOneAdjustCrosshair()
{
	radioButtonGroup->button(5)->setChecked(true); // set checked the 4-in-1 radio btn

	//GetDataStorage()->GetNamedNode(surfaceDataName["FemurSurface"])->SetVisibility(true);
	//GetDataStorage()->GetNamedNode(surfaceDataName["TibiaSurface"])->SetVisibility(false);

	resetView();

	boneGuide = GetDataStorage()->GetNamedNode(boneGuideName["FourInOneBoneGuide"]);
	if (fourInOneComboIdx == 0) {
		planningPlane = GetDataStorage()->GetNamedNode(planeDataName["AnteriorFemurPlanningPlane"]);
		cuttingPlane = GetDataStorage()->GetNamedNode(planeDataName["AnteriorFemurCuttingPlane"]);
	}
	else if (fourInOneComboIdx == 1) {
		planningPlane = GetDataStorage()->GetNamedNode(planeDataName["AnteriorChamferFemurPlanningPlane"]);
		cuttingPlane = GetDataStorage()->GetNamedNode(planeDataName["AnteriorChamferFemurCuttingPlane"]);
	}
	else if (fourInOneComboIdx == 2) {
		planningPlane = GetDataStorage()->GetNamedNode(planeDataName["PosteriorFemurPlanningPlane"]);
		cuttingPlane = GetDataStorage()->GetNamedNode(planeDataName["PosteriorFemurCuttingPlane"]);
	}
	else if (fourInOneComboIdx == 3) {
		planningPlane = GetDataStorage()->GetNamedNode(planeDataName["PosteriorChamferFemurPlanningPlane"]);
		cuttingPlane = GetDataStorage()->GetNamedNode(planeDataName["PosteriorChamferFemurCuttingPlane"]);
	}

	double center[3], normal[3];
	ComputeCenterAndNormalFromSurface(dynamic_cast<mitk::Surface*>(planningPlane->GetData()), center, normal);
	AdjustCrosshair(mitk::RenderingManager::GetInstance(), center, normal, "Coronal"); // set planning plane normal to "Coronal" view nomal

	// focus on target planning plane 
	FocusOnTargetPlane(center);

	boneGuide->SetVisibility(true);
	planningPlane->SetVisibility(true);
	cuttingPlane->SetVisibility(true);
}

void TotalKneeArthroplasty::FocusOnTargetPlane(double point[3]) {
	mitk::BaseRenderer* renderer = mitk::BaseRenderer::GetInstance(mitk::BaseRenderer::GetRenderWindowByName("stdmulti.widget3"));
	vtkRenderer* vtkRenderer = renderer->GetVtkRenderer();
	vtkCamera* camera = vtkRenderer->GetActiveCamera();

	camera->SetFocalPoint(point[0], point[1], point[2]);
	camera->Zoom(3.5);

	vtkRenderer->ResetCameraClippingRange();
}

void TotalKneeArthroplasty::onRadioButtonToggled(QAbstractButton* button, bool checked)
{
	//MITK_INFO << radioButtonGroup->checkedButton();
	//MITK_INFO << radioButtonGroup->checkedId();

	if (checked)
	{
		// clear all landmark & ICP collection because change the targetRF
		ResetLandmarkCollection();
		ResetIcpCollection();

		radioButtonID = radioButtonGroup->id(button); // update global variable
		QString message = QString("Button ID %1 is checked: %2").arg(radioButtonID).arg(button->text());
		//QMessageBox::information(this, "Button Toggled", message);
		MITK_INFO << message;

		/// view toggle
		double center[3], normal[3];
		if (radioButtonID == 1 || radioButtonID == 2 || radioButtonID == 6) { // bone image / plane Validator
			resetView();
		}
		else if (radioButtonID == 3) { // tibia Bone Guide
			//GetDataStorage()->GetNamedNode(surfaceDataName["FemurSurface"])->SetVisibility(false);
			//GetDataStorage()->GetNamedNode(surfaceDataName["TibiaSurface"])->SetVisibility(true);

			resetView();

			boneGuide = GetDataStorage()->GetNamedNode(boneGuideName["TibiaBoneGuide"]);
			planningPlane = GetDataStorage()->GetNamedNode(planeDataName["TibiaPlanningPlane"]);
			cuttingPlane = GetDataStorage()->GetNamedNode(planeDataName["TibiaBoneGuideCuttingPlane"]);

			ComputeCenterAndNormalFromSurface(dynamic_cast<mitk::Surface*>(planningPlane->GetData()), center, normal);
			AdjustCrosshair(mitk::RenderingManager::GetInstance(), center, normal);

			// focus on target planning plane
			FocusOnTargetPlane(center);

			boneGuide->SetVisibility(true);
			planningPlane->SetVisibility(true);
			cuttingPlane->SetVisibility(true);
		}
		else if (radioButtonID == 4) { // Femur Bone Guide
			//GetDataStorage()->GetNamedNode(surfaceDataName["FemurSurface"])->SetVisibility(true);
			//GetDataStorage()->GetNamedNode(surfaceDataName["TibiaSurface"])->SetVisibility(false);

			resetView();

			boneGuide = GetDataStorage()->GetNamedNode(boneGuideName["FemurBoneGuide"]);
			planningPlane = GetDataStorage()->GetNamedNode(planeDataName["DistalFemurPlanningPlane"]);
			cuttingPlane = GetDataStorage()->GetNamedNode(planeDataName["FemurBoneGuideCuttingPlane"]);

			ComputeCenterAndNormalFromSurface(dynamic_cast<mitk::Surface*>(planningPlane->GetData()), center, normal);
			AdjustCrosshair(mitk::RenderingManager::GetInstance(), center, normal);

			// focus on target planning plane 
			FocusOnTargetPlane(center);

			boneGuide->SetVisibility(true);
			planningPlane->SetVisibility(true);
			cuttingPlane->SetVisibility(true);
		}
		else if (radioButtonID == 5) { // 4-in-1 Bone Guide, 4 planning plane u can select.
			FourInOneAdjustCrosshair();
		}

		else if (radioButtonID == 8) { // precision bone guide
			resetView();

			//  set corresponding "boneGuide", "cuttingPlane", default "planningPlane".
			// "planningPlane" can be adjusted in " bone guide navigaiton" tab
			boneGuide = GetDataStorage()->GetNamedNode(boneGuideName["PrecisionBoneGuide"]); // precision BG
			cuttingPlane = GetDataStorage()->GetNamedNode(planeDataName["PrecisionBGCuttingPlane"]); // precision BG cutting plane
			planningPlane = GetDataStorage()->GetNamedNode(planeDataName[precPlanPlaneName]); // ABF, ACE, BCD, ....
			
			// precision cutting pointset initilization
			precCuttingPointSet = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("PrecBoneGuideCuttingPlanePointset")->GetData());
			// precision planning pointset initilization
			m_Controls.textBrowser->append("Current Precision Planning Plane: " + QString::fromStdString(precPlanPlaneName));
			mitk::PointSet::Pointer originalPointSet = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("CT steelball centers")->GetData());
			precPlanningPointSet->Clear();
			std::unordered_set<int> alphaSet;
			for (auto alpha : precPlanPlaneName) { // default: ABF
				alphaSet.insert(alpha - 'A');
			}
			for (int i = 0; i < originalPointSet->GetSize(); ++i) {
				if (alphaSet.count(i)) precPlanningPointSet->InsertPoint(originalPointSet->GetPoint(i));
			}
			for (int i = 0; i < precPlanningPointSet->GetSize(); i++) { // debug
				const double* pointArray = precPlanningPointSet->GetPoint(i).GetDataPointer();
				m_Controls.textBrowser->append(QString::number(pointArray[0]) + " " + QString::number(pointArray[1]) + " " + QString::number(pointArray[2]));
			}

			boneGuide->SetVisibility(true);
			planningPlane->SetVisibility(true);
			cuttingPlane->SetVisibility(true);
		}
		mitk::RenderingManager::GetInstance()->RequestUpdateAll();


		//// 3 checkboxes initialization
		// reset/init checkbox status, disable the checkBox
		if (m_Controls.checkBox_collectPermission->isChecked()) {
			m_Controls.checkBox_collectPermission->setChecked(false);
			m_Controls.textBrowser->append("Landmark Collection Permission Denied");
		}

		// reset/init checkbox status, disable the checkBox
		if (m_Controls.checkBox_collectPermission_icp->isChecked()) {
			m_Controls.checkBox_collectPermission_icp->setChecked(false);
			m_Controls.textBrowser->append("ICP Collection Permission Denied");
		}

		// reset/init checkbox status, disable the checkBox
		if (m_Controls.checkBox_validatePermission->isChecked()) {
			m_Controls.checkBox_validatePermission->setChecked(false);
			m_Controls.textBrowser->append("Validation Permission Denied");
		}
		
	}

	/*QString message;
	switch (id)
	{
	case 1:
		message = "Option 1 selected";
		break;
	case 2:
		message = "Option 2 selected";
		break;
	case 3:
		message = "Option 3 selected";
		break;
	case 4:
		message = "Option 4 selected";
		break;
	default:
		message = "Unknown option selected";
		break;
	}

	MITK_INFO << message;*/
}

void TotalKneeArthroplasty::onCollectPermissionCheckButtonClicked() {
	if (m_Controls.checkBox_collectPermission->isChecked()) {
		m_Controls.checkBox_collectPermission_icp->setChecked(false);
		m_Controls.checkBox_validatePermission->setChecked(false);
		m_Controls.textBrowser->append(radioButtonGroup->checkedButton()->text() + " Registration/Calibration - Collection Permission Allowed");
	}
	else {
		m_Controls.textBrowser->append(radioButtonGroup->checkedButton()->text() + " Registration/Calibration - Collection Permission Denied");
	}
}

void TotalKneeArthroplasty::onCollectPermissionCheckButtonClicked_icp() {
	if (m_Controls.checkBox_collectPermission_icp->isChecked()) {
		m_Controls.checkBox_collectPermission->setChecked(false);
		m_Controls.checkBox_validatePermission->setChecked(false);
		m_Controls.textBrowser->append(radioButtonGroup->checkedButton()->text() + " Registration/Calibration - ICP Collection Permission Allowed");
	}
	else {
		m_Controls.textBrowser->append(radioButtonGroup->checkedButton()->text() + " Registration/Calibration - ICP Collection Permission Denied");
	}
}

void TotalKneeArthroplasty::onValidatePermissionCheckButtonClicked() 
{
	if (m_Controls.checkBox_validatePermission->isChecked()) {
		m_Controls.checkBox_collectPermission->setChecked(false);
		m_Controls.checkBox_collectPermission_icp->setChecked(false);
		m_Controls.textBrowser->append(radioButtonGroup->checkedButton()->text() + " Validation Point Collection Permission Allowed");
	}
	else {
		m_Controls.textBrowser->append(radioButtonGroup->checkedButton()->text() + " Validation Point Collection Permission Denied");
	}
}

void TotalKneeArthroplasty::UseVega() {
	//read in filename
	QString filename = QFileDialog::getOpenFileName(nullptr, tr("Open Tool Storage"), "/",
		tr("Tool Storage Files (*.IGTToolStorage)"));
	if (filename.isNull()) return;

	//read tool storage from disk
	std::string errorMessage = "";
	mitk::NavigationToolStorageDeserializer::Pointer myDeserializer = mitk::NavigationToolStorageDeserializer::New(
		GetDataStorage());
	m_ToolStorage = myDeserializer->Deserialize(filename.toStdString());
	m_ToolStorage->SetName(filename.toStdString());

  //Here we want to use the Kuka robot as a tracking device. Therefore we instantiate a object of the class
  //KukaRobotDevice and make some settings which are necessary for a proper connection to the device.
	MITK_INFO << "Vega tracking";
	//QMessageBox::warning(nullptr, "Warning", "You have to set the parameters for the NDITracking device inside the code (QmitkIGTTutorialView::OnStartIGT()) before you can use it.");
	lancet::NDIVegaTrackingDevice::Pointer vegaTrackingDevice = lancet::NDIVegaTrackingDevice::New(); //instantiate

	//Create Navigation Data Source with the factory class, and the visualize filter.
	lancet::TrackingDeviceSourceConfiguratorLancet::Pointer SourceFactory =
		lancet::TrackingDeviceSourceConfiguratorLancet::New(m_ToolStorage, vegaTrackingDevice);

	m_CameraSource = SourceFactory->CreateTrackingDeviceSource(m_Visualizer);
	m_CameraSource->SetToolMetaDataCollection(m_ToolStorage);
	m_CameraSource->Connect();
	m_CameraSource->StartTracking();

	//update visualize filter by timer
	if (m_VisualizeTimer == nullptr) // init the Timer
	{
		m_VisualizeTimer = new QTimer(this); //create a new timer
	}
	connect(m_VisualizeTimer, SIGNAL(timeout()), this, SLOT(OnVisualizeTimer())); // connect the timer to the method OnTimer()
	connect(m_VisualizeTimer, SIGNAL(timeout()), this, SLOT(UpdateToolStatusWidget()));  // connect the timer to the method OnTimer()

	connect(m_VisualizeTimer, SIGNAL(timeout()), this, SLOT(CollectLandmarkProbeByTool())); // collect landmark by tool/trigger.

	connect(m_VisualizeTimer, SIGNAL(timeout()), this, SLOT(UpdateProbePosition())); // change the probe_tip position(in image) after registration, navigate the probe tip in target(patient bone/bone guide) image.
	connect(m_VisualizeTimer, SIGNAL(timeout()), this, SLOT(UpdateBoneGuidePosition())); // navigate the bone guide in patient bone image

	ShowToolStatus();


	m_VisualizeTimer->start(100); //Every 100ms the method OnTimer() is called. -> 10fps

	auto geo = this->GetDataStorage()->ComputeBoundingGeometry3D(this->GetDataStorage()->GetAll());
	mitk::RenderingManager::GetInstance()->InitializeViews(geo);
}

void TotalKneeArthroplasty::UseRuitongPro() {

}

void TotalKneeArthroplasty::UseAimooe() {

	Aim_API_Initial(aimHandle);
	Aim_SetEthernetConnectIP(aimHandle, 192, 168, 31, 10);
	rlt = Aim_ConnectDevice(aimHandle, I_ETHERNET, mPosDataPara);

	QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", "");
	if (filename.isNull()) return;
	filename.append("/");
	rlt = Aim_SetToolInfoFilePath(aimHandle, filename.toLatin1().data());  // set tools file path

	int size = 0;
	Aim_GetCountOfToolInfo(aimHandle, size); // get the size of tools

	if (size != 0)
	{
		t_ToolBaseInfo* toolarr = new t_ToolBaseInfo[size];
		rlt = Aim_GetAllToolFilesBaseInfo(aimHandle, toolarr); // get all tools info
		if (rlt == AIMOOE_OK)
		{
			for (int i = 0; i < size; i++)
			{
				char* ptool = toolarr[i].name;
				QString toolInfo = QString(ptool);
				//m_Controls.textBrowser->append(toolInfo);
				/*        QString toolName = QString::fromLocal8Bit(toolarr[i].name);
						QString toolInfo = QString("Tool_Name£º") + QString::fromLocal8Bit(ptool);
						m_Controls.textBrowser->append(toolInfo);*/
			}
		}
		delete[] toolarr;
	}


	//update visualize filter by timer
	if (m_VisualizeTimer == nullptr) // init the Timer
	{
		m_VisualizeTimer = new QTimer(this); //create a new timer
	}
	connect(m_VisualizeTimer, SIGNAL(timeout()), this, SLOT(UpdateAimooeData())); // get data from Aimooe Camera
	connect(m_VisualizeTimer, SIGNAL(timeout()), this, SLOT(UpdateAimooeToolStatus())); // update the tool status in widget / visualization 

	connect(m_VisualizeTimer, SIGNAL(timeout()), this, SLOT(CollectLandmarkProbeByTool())); // collect landmark by tool/trigger.
	connect(m_VisualizeTimer, SIGNAL(timeout()), this, SLOT(UpdateProbePosition())); // change the probe_tip position(in image) after registration, navigate the probe tip in target(patient bone/bone guide) image.
	connect(m_VisualizeTimer, SIGNAL(timeout()), this, SLOT(UpdateBoneGuidePosition())); // navigate the bone guide in patient bone image

	//connect(m_VisualizeTimer, SIGNAL(timeout()), this, SLOT(ProbeMouse())); // use probe like mouse. // TODO
		
	m_VisualizeTimer->start(100); //Every 100ms the method OnTimer() is called. -> 10fps

	//auto geo = this->GetDataStorage()->ComputeBoundingGeometry3D(this->GetDataStorage()->GetAll());
	//mitk::RenderingManager::GetInstance()->InitializeViews(geo);
}

void TotalKneeArthroplasty::OnVisualizeTimer()
{
	//Here we call the Update() method from the Visualization Filter. Internally the filter checks if
	//new NavigationData is available. If we have a new NavigationData the cone position and orientation
	//will be adapted.
	if (m_Visualizer.IsNotNull())
	{
		m_Visualizer->Update();
		// auto geo = this->GetDataStorage()->ComputeBoundingGeometry3D(this->GetDataStorage()->GetAll());
		// mitk::RenderingManager::GetInstance()->InitializeViews(geo);
		this->RequestRenderWindowUpdate();
	}
}

void TotalKneeArthroplasty::UpdateToolStatusWidget()
{
	m_Controls.m_StatusWidgetToolToShow->Refresh();

}

void TotalKneeArthroplasty::UpdateAimooeData()
{
	rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, I_ETHERNET, markerSt, statusSt); // get marker info (coordinate) and status
	
	/* std::vector<std::string> toolids = { "Probe",
											"Probe3_trigger",
											"TibiaBoneGuideRF",
											"TibiaRF",
											"BoneGuideRF",
											"FemurRF",
											"PlaneValidator", 
											"PrecisionBlock"}; */
	for (auto toolName : toolids) {
		if (toolName == "Probe3_trigger") continue; // use normal Probe. so pass the "Probe3_trigger", currently "Probe3_trigger" is not a tool. 

		T_AimToolDataResultSingle* RltListSingle = new T_AimToolDataResultSingle;
		rlt = Aim_FindSingleToolInfo(aimHandle, markerSt, toolName.c_str(), *RltListSingle, 3); //3

		if (RltListSingle->validflag) {
			toolStatusMap[toolName] = true; // status

			// data format conversion
			float t_tran[3] = { 0.0f, 0.0f, 0.0f }, R_tran[3][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };;
			t_tran[0] = RltListSingle->Tto[0];
			t_tran[1] = RltListSingle->Tto[1];
			t_tran[2] = RltListSingle->Tto[2];
			for (int i = 0; i < 3; ++i) {
				for (int j = 0; j < 3; ++j) {
					R_tran[i][j] = RltListSingle->Rto[i][j];
				}
			}
			vtkNew<vtkMatrix4x4> m_T_temp;
			CombineRotationTranslation(R_tran, t_tran, m_T_temp);

			// Assign navigation data to Global Variables
			if (toolName == "Probe") {
				memcpy_s(nd_CameraToProbe, sizeof(double) * 16, m_T_temp->GetData(), sizeof(double) * 16);
				nd_CameraToProbeTip[0] = RltListSingle->tooltip[0], nd_CameraToProbeTip[1] = RltListSingle->tooltip[1], nd_CameraToProbeTip[2] = RltListSingle->tooltip[2];
				
				/// set the position to Probe Tip.
				nd_CameraToProbe[3] = RltListSingle->tooltip[0];
				nd_CameraToProbe[7] = RltListSingle->tooltip[1];
				nd_CameraToProbe[11] = RltListSingle->tooltip[2];

				/// check Probe marker nums
				//QString s = "";
				//for (int i = 0; i < RltListSingle->toolPtNum; i++) { // marker nums in tool
				//	s += QString::number(RltListSingle->toolPtId[i]) + " "; // see 4 markers: (0, 1, 2, 3), see 3 markers: (0, 1, 2, -1)
				//}
				//m_Controls.textBrowser->append(s);
				
				//int cnt = 0;
				//for (int i = 0; i < RltListSingle->toolPtNum; i++) { // marker nums in tool
				//	if (RltListSingle->toolPtId[i] != -1) cnt++;
				//}
				//m_Controls.textBrowser->append(QString::number(cnt));

				// check the target marker if is -1. (The marker closest to the probe tip) /// check valid marker cnt is 3 or 4.
				if (RltListSingle->toolPtId[3] == -1) {
					toolStatusMap["Probe3_trigger"] = false;
					probeMarkerCnt = 3;
				}
				else {
					toolStatusMap["Probe3_trigger"] = true;
					probeMarkerCnt = 4;
				}
			}
			else if (toolName == "Probe3_trigger") {
				memcpy_s(nd_CameraToProbeTrigger, sizeof(double) * 16, m_T_temp->GetData(), sizeof(double) * 16);
			}
			/*else if (toolName == "TibiaBoneGuideRF") { // abandon, because all Bone Guides use same RF/Markers.
				memcpy_s(nd_CameraToTibiaBoneGuideRF, sizeof(double) * 16, m_T_temp->GetData(), sizeof(double) * 16);
			}*/
			else if (toolName == "TibiaRF") {
				memcpy_s(nd_CameraToTibiaRF, sizeof(double) * 16, m_T_temp->GetData(), sizeof(double) * 16);
			}
			else if (toolName == "FemurRF") {
				memcpy_s(nd_CameraToFemurRF, sizeof(double) * 16, m_T_temp->GetData(), sizeof(double) * 16);
			}

			else if (toolName == "BoneGuideRF") { // 3 bone guide use same RF/maker
				memcpy_s(nd_CameraToBoneGuideRF, sizeof(double) * 16, m_T_temp->GetData(), sizeof(double) * 16);
			}
			
			else if (toolName == "PlaneValidator") {
				memcpy_s(nd_CameraToPlaneValidator, sizeof(double) * 16, m_T_temp->GetData(), sizeof(double) * 16);
			}
			else if (toolName == "PrecisionBlock") {
				memcpy_s(nd_CameraToPrecisionBlockRF, sizeof(double) * 16, m_T_temp->GetData(), sizeof(double) * 16);
			}

			/*else if (toolName == "LancetCalibrator") {
				memcpy_s(nd_CameraToLancetCalibratorRF, sizeof(double) * 16, m_T_temp->GetData(), sizeof(double) * 16);
			}*/
		}
		else {
			toolStatusMap[toolName] = false; // set tool status
		}
		delete RltListSingle;
	}
}

void TotalKneeArthroplasty::UpdateAimooeToolStatus()
{
	QString htmlText = "<html><body>";

	double x, y, z;
	for (auto toolName : toolids) {

		// update position in label widget
		if (toolName == "Probe") {
			x = nd_CameraToProbeTip[0], y = nd_CameraToProbeTip[1], z = nd_CameraToProbeTip[2];

			x = nd_CameraToProbe[3], y = nd_CameraToProbe[7], z = nd_CameraToProbe[11];
		}
		else if (toolName == "Probe3_trigger") {
			//x = nd_CameraToProbeTrigger[3], y = nd_CameraToProbeTrigger[7], z = nd_CameraToProbeTrigger[11];
			x = -1, y = -1, z = -1;
		}
		/*else if (toolName == "TibiaBoneGuideRF") { // the old version of the bone guide RF, abandon.
			x = nd_CameraToTibiaBoneGuideRF[3], y = nd_CameraToTibiaBoneGuideRF[7], z = nd_CameraToTibiaBoneGuideRF[11];
		}*/
		else if (toolName == "TibiaRF") {
			x = nd_CameraToTibiaRF[3], y = nd_CameraToTibiaRF[7], z = nd_CameraToTibiaRF[11];
		}
		else if (toolName == "BoneGuideRF") {
			x = nd_CameraToBoneGuideRF[3], y = nd_CameraToBoneGuideRF[7], z = nd_CameraToBoneGuideRF[11];
		}
		else if (toolName == "FemurRF") {
			x = nd_CameraToFemurRF[3], y = nd_CameraToFemurRF[7], z = nd_CameraToFemurRF[11];
		}
		else if (toolName == "PlaneValidator") {
			x = nd_CameraToPlaneValidator[3], y = nd_CameraToPlaneValidator[7], z = nd_CameraToPlaneValidator[11];
		}
		else if (toolName == "PrecisionBlock") {
			x = nd_CameraToPrecisionBlockRF[3], y = nd_CameraToPrecisionBlockRF[7], z = nd_CameraToPrecisionBlockRF[11];
		}
		/*else if (toolName == "LancetCalibrator") {
			x = nd_CameraToLancetCalibratorRF[3], y = nd_CameraToLancetCalibratorRF[7], z = nd_CameraToLancetCalibratorRF[11];
		}*/
		else {
			continue;
		}

		// update status -> background color
		QString color = toolStatusMap[toolName] ? "green" : "red";

		htmlText += QString("<div style=\"background-color: %1;\">%2: (%3, %4, %5)</div>")
			.arg(color)
			.arg(toolName.c_str())
			.arg(x)
			.arg(y)
			.arg(z);
	}

	htmlText += "</body></html>";

	m_Controls.m_AimooeToolStatusToShow->setText(htmlText);
	m_Controls.m_AimooeToolStatusToShow->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
}

void TotalKneeArthroplasty::ProbeMouse()
{
	///// Probe Control Mouse by Pitch & Roll
	// Probe Mouse, cal the euler angles of Probe. 
	double ProbeEulerAngles[3];
	vtkTransform::GetOrientation(ProbeEulerAngles, DoubleToVtkMatrix(nd_CameraToProbe));
	/*m_Controls.lineEdit_PlanningPlane_x->setText(QString::number(ProbeEulerAngles[0])); 
	m_Controls.lineEdit_PlanningPlane_y->setText(QString::number(ProbeEulerAngles[1]));
	m_Controls.lineEdit_PlanningPlane_z->setText(QString::number(ProbeEulerAngles[2]));*/

	//// Probe mouse method without smoothing
	//POINT newPosition = CalculateMousePosition(ProbeEulerAngles[0], ProbeEulerAngles[1], screenWidth, screenHeight);
	//// update Mouse position
	//MoveMouseToPosition(newPosition);

	// Probe mouse method with smoothing 
	/*POINT newPosition = CalculateMousePosition(ProbeEulerAngles[0], ProbeEulerAngles[1], screenWidth, screenHeight);
	POINT currentPosition;
	GetCursorPos(&currentPosition);
	SmoothMoveMouseToPosition(currentPosition, newPosition);*/
}


// Collect the Landmark of Probe_tip, by checking the status the probe3_click(trigger).
void TotalKneeArthroplasty::CollectLandmarkProbeByTool()
{
	if (!m_Controls.checkBox_collectPermission->isChecked() && !m_Controls.checkBox_collectPermission_icp->isChecked() && !m_Controls.checkBox_validatePermission->isChecked())
	{
		m_Controls.label_collectionStatus->setStyleSheet(QString("background-color: %1;").arg("red")); // notification
		return;
	}

	// check the tool status to collect the probe. triggered by Probe3_trigger
	std::string targetRF = getTargetRFName(); // selected by radio btn
	if (!toolStatusMap[targetRF] || !toolStatusMap["Probe"])  // check the targetRF/probe data status, DO NOT COLLECT when objectRF/BoneGuideRF->isDataValid()==false
	{
		m_Controls.label_collectionStatus->setStyleSheet(QString("background-color: %1;").arg("red"));
		return;
	}

	// vis, collection status notification, 
	if (!PIController.GetCollectPermit()) m_Controls.label_collectionStatus->setStyleSheet(QString("background-color: %1;").arg("red"));
	else m_Controls.label_collectionStatus->setStyleSheet(QString("background-color: %1;").arg("green"));


	// using "ProbeInteractor" class to interact with probe.
	if (PIController.CanCollectData(probeMarkerCnt)) {
		if (m_Controls.checkBox_collectPermission->isChecked()) CollectLandmarkProbe(); // do landmark Collection
		else if (m_Controls.checkBox_collectPermission_icp->isChecked()) CollectIcpProbe(); // do ICP Collection
		else if (m_Controls.checkBox_validatePermission->isChecked()) ErrorEvaluation();
	}


	/* // Old Probe Interaction code.
	if (!toolStatusMap["Probe3_trigger"] && toolStatusMap["Probe"] && m_clickPermit) // satisify the condition
	{
		// Count the consecutive frames.
		m_ConsecFrame++;
	}
	else if (toolStatusMap["Probe3_trigger"] && toolStatusMap["Probe"]) // if see both, then reset/init the status
	{
		// init 
		m_clickPermit = true; // active
		m_ConsecFrame = 0;
	}

	// !probe3_click && probe3 && consec_frame > threshold && click_permit
	if (!toolStatusMap["Probe3_trigger"] && toolStatusMap["Probe"] && m_ConsecFrame >= m_ConsecFrameThreshold && m_clickPermit)
	{
		//MITK_INFO << "click";
		if (m_Controls.checkBox_collectPermission->isChecked()) CollectLandmarkProbe(); // do landmark Collection
		else if (m_Controls.checkBox_collectPermission_icp->isChecked()) CollectIcpProbe(); // do ICP Collection

		m_clickPermit = false; // frozen, change/modify click permit
	}*/




	/// ******* original code for NDI Camera, need to MODIFY ************////
	/*
	if (!m_CameraSource || !m_Controls.checkBox_collectPermission->isChecked()) 
	{
		m_Controls.label_collectionStatus->setStyleSheet(QString("background-color: %1;").arg("red")); // notification
		return;
	}

	auto probeIndex = m_ToolStorage->GetToolIndexByName("Probe"); // get index of Probe
	auto probeTriggerIndex = m_ToolStorage->GetToolIndexByName("Probe3_trigger"); // get index of Probe3_trigger
	auto targetRfIndex = getTargetRFIdx(); // m_ToolStorage->GetToolIndexByName("TibiaRF"); // boneRF

	if (probeIndex == -1 || probeTriggerIndex == -1 || targetRfIndex == -1) // can't find the probe or objectRF
	{
		m_Controls.textBrowser->append("There is no 'Probe' or 'ObjectRF' in the toolStorage!");
		m_Controls.label_collectionStatus->setStyleSheet(QString("background-color: %1;").arg("red"));
		return;
	}

	// check the status of Probe3 and Probe3_click(for trigger). 
	mitk::NavigationData::Pointer nd_ndiToProbe = m_CameraSource->GetOutput(probeIndex); // get Probe data 
	mitk::NavigationData::Pointer nd_ndiToProbeTrigger = m_CameraSource->GetOutput(probeTriggerIndex); 
	mitk::NavigationData::Pointer nd_ndiToTargetRf = m_CameraSource->GetOutput(targetRfIndex);
	//MITK_INFO << nd_ndiToProbe->GetPosition();
	//MITK_INFO << "Probe Status: " << nd_ndiToProbe->IsDataValid(); // 1 true, 0 false
	//MITK_INFO << m_ConsecFrame << " " << m_ConsecFrameThreshold;

	if (!nd_ndiToTargetRf->IsDataValid() || !nd_ndiToProbe->IsDataValid())  // check the targetRF/probe data status, DO NOT COLLECT when objectRF/BoneGuideRF->isDataValid()==false
	{
		//m_Controls.textBrowser->append("Check the 'TargetRF' status!");
		m_Controls.label_collectionStatus->setStyleSheet(QString("background-color: %1;").arg("red"));
		return;
	}

	// vis, collection status notification
	if (!m_clickPermit) m_Controls.label_collectionStatus->setStyleSheet(QString("background-color: %1;").arg("red"));
	else m_Controls.label_collectionStatus->setStyleSheet(QString("background-color: %1;").arg("green"));

	if (!nd_ndiToProbeTrigger->IsDataValid() && nd_ndiToProbe->IsDataValid() && m_clickPermit) // satisify the condition
	{
		// Count the consecutive frames.
		m_ConsecFrame++;
	}
	else if (nd_ndiToProbeTrigger->IsDataValid() && nd_ndiToProbe->IsDataValid()) // see both to reset/init the status
	{
		// init 
		m_clickPermit = true; // active
		m_ConsecFrame = 0;
	}

	// !probe3_click && probe3 && consec_frame > threshold && click_permit
	if (!nd_ndiToProbeTrigger->IsDataValid() && nd_ndiToProbe->IsDataValid() && m_ConsecFrame >= m_ConsecFrameThreshold && m_clickPermit)
	{
		//MITK_INFO << "click";
		CollectLandmarkProbe(); // do collect

		m_clickPermit = false; // frozen, change/modify click permit

	}*/
}


// return the selected RF(ObjectRF/BoneRF/PatientRF, ...) by radio button
std::string TotalKneeArthroplasty::getTargetRFName()
{
	if (radioButtonID == 1) { // TibiaRF
		return "TibiaRF";
	}
	else if (radioButtonID == 2) { // FemurRF
		return "FemurRF";
	}

	// all Bone Guides use SAME RF/Marker
	else if (radioButtonID == 3) { // TibiaBoneGuideRF
		return "BoneGuideRF";
	}
	else if (radioButtonID == 4) { // DistalFemurBoneGuide
		return "BoneGuideRF"; 
	}
	else if (radioButtonID == 5) { // 4in1BoneGuide
		return "BoneGuideRF"; 
	}

	else if (radioButtonID == 6) { // plane validator
		return "PlaneValidator";
	}

	else if (radioButtonID == 7) { // precision block RF
		return "PrecisionBlock";
	}
	else if (radioButtonID == 8) { // precision bone guide
		return "BoneGuideRF";
	}
	
	return "";
}

// return the selected tool(ObjectRF/BoneRF/PatientRF, TibiaBoneGuideRF, DistalFemurBoneGuide, 4in1BoneGuide) idx 
int TotalKneeArthroplasty::getTargetRFIdx() { // abandon
	return -1;
	/// original code for NDI, need to MODIFY
	/* if (radioButtonID == 1) { // ObjectRF/BoneRF/PatientRF 
		return m_ToolStorage->GetToolIndexByName("TibiaRF");
	}
	else if (radioButtonID == 2) { // TibiaBoneGuideRF
		return m_ToolStorage->GetToolIndexByName("TibiaBoneGuideRF");
	}
	else if (radioButtonID == 3) { // DistalFemurBoneGuide
		return -1;
	} 
	else if (radioButtonID == 4) { // 4in1BoneGuide
		return -1; 
	} 

	return -1; */
}

// For image/bone guide registration/calibration. navigate the probe tip in the bone/boneguide image.
void TotalKneeArthroplasty::UpdateProbePosition()
{
	////// test Lancet Calibrator
	//auto vtkTargetRFToProbeTip = getVtkMatrix4x4InRef(DoubleToVtkMatrix(nd_CameraToProbe), DoubleToVtkMatrix(nd_CameraToLancetCalibratorRF));
	//double TargetRFToProbeTip[3];
	//TargetRFToProbeTip[0] = vtkTargetRFToProbeTip->GetElement(0, 3);
	//TargetRFToProbeTip[1] = vtkTargetRFToProbeTip->GetElement(1, 3);
	//TargetRFToProbeTip[2] = vtkTargetRFToProbeTip->GetElement(2, 3);

	//m_Controls.textBrowser->append(QString::number(TargetRFToProbeTip[0]) + " " + QString::number(TargetRFToProbeTip[1]) + " " + QString::number(TargetRFToProbeTip[2]) + " ");
	////// 

	if (m_Controls.checkBox_BoneGuideNavigationPermission->isChecked()) return; // doing "bone guide navigaiton", disable this function.

	/*m_T_patientRFtoImage = Tkan.GetTibiaRFtoImageTrans();
	m_T_tibiaBoneGuideRFtoTBGImage = Tkan.GetTBGRFtoTBGImageTrans();

	if (m_T_patientRFtoImage->IsIdentity() && m_T_tibiaBoneGuideRFtoTBGImage->IsIdentity()) return;*/

	//PrintVtkMatrix(m_T_patientRFtoImage, "m_T_patientRFtoImage");
	//PrintVtkMatrix(m_T_tibiaBoneGuideRFtoTBGImage, "m_T_tibiaBoneGuideRFtoTBGImage");

	vtkSmartPointer<vtkMatrix4x4> resultMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	if (!Tkan.GetTibiaRFtoImageTrans()->IsIdentity() && radioButtonID == 1) { // tibia
		resultMatrix->DeepCopy(Tkan.ProbeNavigationInTibiaBone(nd_CameraToProbe, nd_CameraToTibiaRF));
	}
	else if (!Tkan.GetFemurRFtoImageTrans()->IsIdentity() && radioButtonID == 2) { // femur
		resultMatrix->DeepCopy(Tkan.ProbeNavigationInFemurBone(nd_CameraToProbe, nd_CameraToFemurRF)); 
	}
	else if (!Tkan.GetTBGRFtoTBGImageTrans()->IsIdentity() && radioButtonID == 3) { // tibia BG
		resultMatrix->DeepCopy(Tkan.ProbeNavigationInTibiaBG(nd_CameraToProbe, nd_CameraToBoneGuideRF));
	}
	else if (!Tkan.GetFBGRFtoFBGImageTrans()->IsIdentity() && radioButtonID == 4) { // femur BG
		resultMatrix->DeepCopy(Tkan.ProbeNavigationInFemurBG(nd_CameraToProbe, nd_CameraToBoneGuideRF));
	}
	else if (!Tkan.GetFIOBGRFtoFIOBGImageTrans()->IsIdentity() && radioButtonID == 5) { // 4-in-1 BG
		resultMatrix->DeepCopy(Tkan.ProbeNavigationInFourInOneBG(nd_CameraToProbe, nd_CameraToBoneGuideRF));
	}
	else if (!Tkan.GetPVRFtoPVImageTrans()->IsIdentity() && radioButtonID == 6) { // plane validator
		resultMatrix->DeepCopy(Tkan.ProbeNavigationInPlaneValidator(nd_CameraToProbe, nd_CameraToPlaneValidator));
	} 
	else if (radioButtonID == 7) { // precision block 
		resultMatrix->DeepCopy(Tkan.ProbeNavigationInPrecisionBlock(nd_CameraToProbe, nd_CameraToPrecisionBlockRF));
	} 
	else if (radioButtonID == 8) { // precision bone guide
		resultMatrix->DeepCopy(Tkan.ProbeNavigationInPrecisionBG(nd_CameraToProbe, nd_CameraToBoneGuideRF));
	}
	else {
		return;
	}

	GetDataStorage()->GetNamedNode("Probe")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(resultMatrix); // move probe
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();

	/// ******* original code for NDI Camera, need to MODIFY ************////
	/*
	if (m_T_patientRFtoImage->IsIdentity() && m_T_tibiaBoneGuideRFtoTBGImage->IsIdentity()) return;

	PrintVtkMatrix(m_T_patientRFtoImage, "m_T_patientRFtoImage");
	PrintVtkMatrix(m_T_tibiaBoneGuideRFtoTBGImage, "m_T_tibiaBoneGuideRFtoTBGImage");

	auto probeIndex = m_ToolStorage->GetToolIndexByName("Probe");
	auto targetRfIndex = getTargetRFIdx();  //  m_ToolStorage->GetToolIndexByName("TibiaRF");
	if (probeIndex == -1 || targetRfIndex == -1) return;

	mitk::NavigationData::Pointer nd_ndiToProbe = m_CameraSource->GetOutput(probeIndex); // get data of Probe
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_CameraSource->GetOutput(targetRfIndex);  // get data of Object/Patient RF
	//mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf); // ObjRF_Ndi * Ndi_Probe -> ObjRFPorbe

	////MITK_INFO << nd_rfToProbe->GetPosition(); // the positon of Probe_tip under ObjectRF coordinate.
	//auto nd_rfToProbeVTK = getVtkMatrix4x4(nd_rfToProbe); // convert nd data to vtkMatrix4x4

	//vtkSmartPointer<vtkMatrix4x4> resultMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	//if(m_T_patientRFtoImage && radioButtonID == 1) vtkMatrix4x4::Multiply4x4(m_T_patientRFtoImage, nd_rfToProbeVTK, resultMatrix);
	//else if(m_T_tibiaBoneGuideRFtoTBGImage && radioButtonID == 2) vtkMatrix4x4::Multiply4x4(m_T_tibiaBoneGuideRFtoTBGImage, nd_rfToProbeVTK, resultMatrix);

	vtkSmartPointer<vtkMatrix4x4> resultMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	if (m_T_patientRFtoImage && radioButtonID == 1) {
		resultMatrix->DeepCopy(Tkan.ProbeNavigationInTibiaBone(nd_ndiToProbe, nd_ndiToObjectRf));
	}
	else if (m_T_tibiaBoneGuideRFtoTBGImage && radioButtonID == 2) {
		resultMatrix->DeepCopy(Tkan.ProbeNavigationInTibiaBG(nd_ndiToProbe, nd_ndiToObjectRf));
	}

	GetDataStorage()->GetNamedNode("Probe")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(resultMatrix); // move probe
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();

	*/
}

void TotalKneeArthroplasty::VisualizePlaneAngle(mitk::Surface::Pointer ref, mitk::Surface::Pointer surface, double& angle, double dis) {
	/*mitk::Point3D center = surface->GetGeometry()->GetCenter();

	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	
	transform->Translate(center[0], center[1], center[2]);
	transform->RotateY(angle);
	transform->Translate(-center[0], -center[1], -center[2] );

	dis = std::clamp(dis, -50.0, 50.0);
	transform->Translate(0.0, 0.0, dis);
	
	vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	transform->GetMatrix(matrix);*/


	// apply rotation 
	mitk::Point3D initialCenter = surface->GetGeometry()->GetCenter();
	vtkSmartPointer<vtkTransform> rotateTransform = vtkSmartPointer<vtkTransform>::New();
	rotateTransform->PostMultiply();
	rotateTransform->Identity();
	rotateTransform->Translate(-initialCenter[0], -initialCenter[1], -initialCenter[2]);
	rotateTransform->RotateY(angle);
	rotateTransform->Translate(initialCenter[0], initialCenter[1], initialCenter[2]);

	vtkSmartPointer<vtkTransformPolyDataFilter> rotateFilter =
		vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	rotateFilter->SetInputData(ref->GetVtkPolyData());
	rotateFilter->SetTransform(rotateTransform);
	rotateFilter->Update();

	// cal the center after rotation
	vtkSmartPointer<vtkCenterOfMass> centerFilter = vtkSmartPointer<vtkCenterOfMass>::New();
	centerFilter->SetInputData(rotateFilter->GetOutput());
	centerFilter->SetUseScalarsAsWeights(false);
	centerFilter->Update();

	double rotatedCenter[3];
	centerFilter->GetCenter(rotatedCenter);

	// move dis in Z axis, only change Z axis
	dis = std::clamp(dis, -50.0, 50.0);
	vtkSmartPointer<vtkTransform> translateTransform = vtkSmartPointer<vtkTransform>::New();
	translateTransform->PostMultiply();
	translateTransform->Identity();
	translateTransform->Translate(initialCenter[0] - rotatedCenter[0], // the translation in x axis, we need move back after rotation
		initialCenter[1] - rotatedCenter[1], // the translation in y axis 
		dis);

	// combination
	vtkSmartPointer<vtkTransform> combinedTransform = vtkSmartPointer<vtkTransform>::New();
	combinedTransform->PostMultiply();
	combinedTransform->Identity();
	combinedTransform->Concatenate(rotateTransform);
	combinedTransform->Concatenate(translateTransform);

	// apply 
	vtkSmartPointer<vtkTransformPolyDataFilter> finalFilter =
		vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	finalFilter->SetInputData(ref->GetVtkPolyData());
	finalFilter->SetTransform(combinedTransform);
	finalFilter->Update();

	surface->SetVtkPolyData(finalFilter->GetOutput());
	//surface->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(matrix);
	
}

void TotalKneeArthroplasty::UpdateBoneGuidePosition()
{
	if (!(m_Controls.checkBox_BoneGuideNavigationPermission->isChecked())) return;

	/*m_T_patientRFtoImage = Tkan.GetTibiaRFtoImageTrans();
	m_T_tibiaBoneGuideRFtoTBGImage = Tkan.GetTBGRFtoTBGImageTrans();*/

	/*PrintVtkMatrix(m_T_patientRFtoImage, "m_T_patientRFtoImage");
	PrintVtkMatrix(m_T_tibiaBoneGuideRFtoTBGImage, "m_T_tibiaBoneGuideRFtoTBGImage");*/


	/* // replace following code by radio&combo button.
	
	
	mitk::DataNode::Pointer BoneGuideCuttingPlane, BoneGuideSurface;
	mitk::DataNode::Pointer PlanningPlane;

	if (radioButtonID == 3 ) { // tibia BG navigaiton //// && !Tkan.GetTBGRFtoTBGImageTrans()->IsIdentity() && !Tkan.GetTibiaRFtoImageTrans()->IsIdentity()
		BoneGuideCuttingPlane = GetDataStorage()->GetNamedNode(planeDataName["TibiaBoneGuideCuttingPlane"]);
		BoneGuideSurface = GetDataStorage()->GetNamedNode("TibiaBoneGuideSurface"); 

		//BoneGuideCuttingPlane = GetDataStorage()->GetNamedNode("planeDataName["TibiaBoneGuideCuttingPlane"]"); // just for EMC test
		//BoneGuideSurface = GetDataStorage()->GetNamedNode("FourInOneBoneGuideSurface");// just for EMC test

		PlanningPlane = GetDataStorage()->GetNamedNode(planeDataName["TibiaPlanningPlane"]);

	}
	else if (radioButtonID == 4 ) { // femur BG navigaiton // && !Tkan.GetFBGRFtoFBGImageTrans()->IsIdentity() && !Tkan.GetFemurRFtoImageTrans()->IsIdentity()
		BoneGuideCuttingPlane = GetDataStorage()->GetNamedNode(planeDataName["FemurBoneGuideCuttingPlane"]);
		BoneGuideSurface = GetDataStorage()->GetNamedNode("FemurBoneGuideSurface");
		PlanningPlane = GetDataStorage()->GetNamedNode(planeDataName["DistalFemurPlanningPlane"]);
	}
	else if (radioButtonID == 5 ) { // 4-in-1 BG navigaiton /// && !Tkan.GetFIOBGRFtoFIOBGImageTrans()->IsIdentity() && !Tkan.GetFemurRFtoImageTrans()->IsIdentity()
		BoneGuideCuttingPlane = GetDataStorage()->GetNamedNode(planeDataName["AnteriorFemurCuttingPlane"]); /
		BoneGuideSurface = GetDataStorage()->GetNamedNode("FourInOneBoneGuideSurface");
		PlanningPlane = GetDataStorage()->GetNamedNode(planeDataName["AnteriorFemurPlanningPlane"]);
	}
	else if (radioButtonID == 6 && !Tkan.GetPVRFtoPVImageTrans()->IsIdentity()) { // plane validator navigaiton
		// two situation: "tibia" OR "Femur" planning plane Validation. so need to select the target Bone Image.
		//  navigate it like "Probe", see below code
		return;
	}
	else {
		return;
	}
	*/
	if (!cuttingPlane || !planningPlane)
	{
		m_Controls.textBrowser->append("There is no 'BoneGuideCuttingPlane' or 'PlanningPlane' in the toolStorage!");
		return;
	}

	//  Bone Guide Navigation
	vtkSmartPointer<vtkMatrix4x4>  BoneGuideTransform = vtkSmartPointer<vtkMatrix4x4>::New();
	if (radioButtonID == 3) BoneGuideTransform = Tkan.TibiaBoneGuideNavigation(nd_CameraToBoneGuideRF, nd_CameraToTibiaRF);
	else if (radioButtonID == 4) BoneGuideTransform = Tkan.FemurBoneGuideNavigation(nd_CameraToBoneGuideRF, nd_CameraToFemurRF);
	else if (radioButtonID == 5) BoneGuideTransform = Tkan.FourInOneBoneGuideNavigation(nd_CameraToBoneGuideRF, nd_CameraToFemurRF);
	else if (radioButtonID == 8) {
		BoneGuideTransform = Tkan.PrecisionBoneGuideNavigation(nd_CameraToBoneGuideRF, nd_CameraToPrecisionBlockRF);

		if (m_Controls.checkBox_precCuttingPlaneSwitch->isChecked()) { // Compare with Laser PBG Cutting plane/pointset.
			// change the "BoneGuideTransform" to {"Tkan.GetPBLasertoPBImageTrans()" x "invert(Tkan.GetPBGNaviMtxInTargetPos())" x "BoneGuideTransform" }
			// let LASER PBG Cutting Plane Bind with the PBG Image.
			vtkSmartPointer<vtkMatrix4x4> T_inv_NaviMtxINTarPos = vtkSmartPointer<vtkMatrix4x4>::New();
			vtkMatrix4x4::Invert(Tkan.GetPBGNaviMtxInTargetPos(), T_inv_NaviMtxINTarPos); // get invert of GetPBGNaviMtxInTargetPos

			vtkSmartPointer<vtkTransform> tmpVtkTransform = vtkTransform::New();
			tmpVtkTransform->PostMultiply();
			tmpVtkTransform->Identity();
			tmpVtkTransform->SetMatrix(Tkan.GetPBLasertoPBImageTrans()); //coordinate: Laser coord -> target/PrecisionBlock image
			tmpVtkTransform->Concatenate(T_inv_NaviMtxINTarPos); // coordinate: target/PB img -> Bone guide image.
			tmpVtkTransform->Concatenate(BoneGuideTransform); // coordinate: TargetRF -> target Image

			BoneGuideTransform->DeepCopy(tmpVtkTransform->GetMatrix());
		}
	}

	boneGuide->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(BoneGuideTransform);
	cuttingPlane->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(BoneGuideTransform);
	
	// Probe/Plane Validator Navigation
	vtkSmartPointer<vtkMatrix4x4>  ProbeTransform = vtkSmartPointer<vtkMatrix4x4>::New(), PVTransform = vtkSmartPointer<vtkMatrix4x4>::New();
	if (radioButtonID == 3) { // tibia 
		ProbeTransform = Tkan.ProbeNavigationInTibiaBone(nd_CameraToProbe, nd_CameraToTibiaRF); // probe navi
		PVTransform = Tkan.PVNavigationInTibiaBone(nd_CameraToPlaneValidator, nd_CameraToTibiaRF); // plane validator navi
	}
	else if (radioButtonID == 4 || radioButtonID == 5) { // femur
		ProbeTransform = Tkan.ProbeNavigationInFemurBone(nd_CameraToProbe, nd_CameraToFemurRF); // probe navi
		PVTransform = Tkan.PVNavigationInFemurBone(nd_CameraToPlaneValidator, nd_CameraToFemurRF); // plane validator navi
	}
	else if (radioButtonID == 8) { // precision block
		ProbeTransform = Tkan.ProbeNavigationInPrecisionBlock(nd_CameraToProbe, nd_CameraToPrecisionBlockRF); // probe navi
	}

	GetDataStorage()->GetNamedNode("Probe")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(ProbeTransform); 
	GetDataStorage()->GetNamedNode("PlaneValidatorSurface")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(PVTransform); 
	GetDataStorage()->GetNamedNode("PlaneValidatorPlane")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(PVTransform);
	
	// rendering update 
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();

	/****  compute the surface error (dis, angle_error, pitch, roll) ****/ 

	// bone guide surface error
	auto PlanningPlaneSurface = dynamic_cast<mitk::Surface*>(planningPlane->GetData());
	auto BoneGuideCuttingPlaneSurface = dynamic_cast<mitk::Surface*>(cuttingPlane->GetData());
	// plane validator surface error
	auto PlaneValidatorPlaneSurface = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("PlaneValidatorPlane")->GetData());
	double dis = -1.0, angleErr = -1.0, pitch = -1.0, roll = -1.0, dis_pv = -1.0, angleErr_pv = -1.0, pitch_pv = -1.0, roll_pv = -1.0;
	double dis1 = -1.0, dis2 = -1.0, dis3 = -1.0, yaw = -1.0; // for precision test
	if (radioButtonID == 3) {
		std::tie(dis, angleErr, pitch, roll) = Tkan.ComputeTibiaBGSurfaceError(PlanningPlaneSurface, BoneGuideCuttingPlaneSurface, BoneGuideTransform);
	}
	else if (radioButtonID == 4) {
		std::tie(dis, angleErr, pitch, roll) = Tkan.ComputeFemurBGSurfaceError(PlanningPlaneSurface, BoneGuideCuttingPlaneSurface, BoneGuideTransform);
	}
	else if (radioButtonID == 5) {
		std::tie(dis, angleErr, pitch, roll) = Tkan.ComputeFourInOneBGSurfaceError(PlanningPlaneSurface, BoneGuideCuttingPlaneSurface, BoneGuideTransform);
	}
	else if (radioButtonID == 8) { // precision test, 
		// compute point-to-surface distance error, compute surface-to-surface angle error.
		
		std::tie(dis, angleErr, pitch, roll, yaw) = Tkan.ComputePrecisionBGSurfaceError(PlanningPlaneSurface, BoneGuideCuttingPlaneSurface, BoneGuideTransform);
		std::tie(dis1, dis2, dis3) = Tkan.ComputePointToSurfaceDisError(precPlanningPointSet, precCuttingPointSet, BoneGuideTransform); 

		m_Controls.lineEdit_Dis_p2s_1->setText(QString::number(dis1));
		m_Controls.lineEdit_Dis_p2s_2->setText(QString::number(dis2));
		m_Controls.lineEdit_Dis_p2s_3->setText(QString::number(dis3));
		m_Controls.lineEdit_Angle_s2s->setText(QString::number(angleErr));

		m_Controls.lineEdit_AxialAngle_Diff->setText(QString::number(pitch));
		m_Controls.lineEdit_SagittalAngle_Diff->setText(QString::number(roll));
		m_Controls.lineEdit_CoronalAngle_Diff->setText(QString::number(yaw));
	}
	
	m_Controls.lineEdit_distanceErr->setText(QString::number(dis));
	m_Controls.lineEdit_pitchErr->setText(QString::number(pitch));
	m_Controls.lineEdit_rollErr->setText(QString::number(roll));
	m_Controls.lineEdit_surfaceError->setText(QString::number(angleErr));

	// plane validator surface error
	//std::tie(dis_pv, angleErr_pv) = Tkan.ComputePlaneValidatorSurfaceError(PlanningPlaneSurface, PlaneValidatorPlaneSurface, PVTransform);
	std::tie(dis_pv, angleErr_pv, pitch_pv, roll_pv) = Tkan.ComputeTibiaBGSurfaceError(PlanningPlaneSurface, PlaneValidatorPlaneSurface, PVTransform);
	m_Controls.lineEdit_distanceErr_PV->setText(QString::number(dis_pv));
	m_Controls.lineEdit_surfaceError_PV->setText(QString::number(angleErr_pv));
	m_Controls.lineEdit_pitchErr_PV->setText(QString::number(pitch_pv));
	m_Controls.lineEdit_rollErr_PV->setText(QString::number(roll_pv));

	/* Visualization for dis, pitch, roll*/

	/*VisualizePlaneAngle(dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("TargetPlaneDisRoll")->GetData()), dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("CuttingPlaneDisRoll")->GetData()), roll, dis);
	VisualizePlaneAngle(dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("TargetPlanePitch")->GetData()), dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("CuttingPlanePitch")->GetData()), pitch);*/


	//double center[3], normal[3];
	//ComputeCenterAndNormalFromSurface(dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("CuttingPlaneDisRoll")->GetData()), center, normal);
	//if (!AxesActor) {
	//	AxesActor = vtkSmartPointer<vtkAxesActor>::New();
	//	UpdateAxesActor(50, AxesActor, center, normal);
	//	AddActor(this->GetRenderWindowPart(), AxesActor);
	//}
	//else {
	//	UpdateAxesActor(50, AxesActor, center, normal);
	//	//UpdateAxesActor(50.0, AxesActor, TibiaBoneGuideTransform, normal_dir);
	//}

	


	/* 
	/// old/testing surfaces error computation method below.

	// cal the error(normal angle, point-to-surface distance) between planning plane and bone guide cutting plane. 
	//auto TibiaPlanningPlaneSurface = dynamic_cast<mitk::Surface*>(TibiaPlanningPlane->GetData());
	//vtkSmartPointer<vtkPolyData> TibiaPlanningPlanePolyData = TibiaPlanningPlaneSurface->GetVtkPolyData();
	ComputeTwoSurfaceError(TibiaPlanningPlaneSurface, TibiaBoneGuideCuttingPlaneSurface, TibiaBoneGuideTransform);

	//// visualization, Create an Arrow: guide direction for Bone Guide

	double normal_dir[3];
	CalculateVector(TibiaPlanningPlaneSurface, TibiaBoneGuideTransform, normal_dir);

	// test
	/// target plane
	double targetCenter[3];
	double targetNormal[3];
	ComputeCenterAndNormalFromSurface(TibiaPlanningPlaneSurface, targetCenter, targetNormal);

	/// cutting plane
	double tmp_cuttingCenter[3], cuttingCenter[3];
	double tmp_cuttingNormal[3], cuttingNormal[3];
	ComputeCenterAndNormalFromSurface(TibiaBoneGuideCuttingPlaneSurface, tmp_cuttingCenter, tmp_cuttingNormal);

	// convert center and normal with vtkMatrix4x4
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->SetMatrix(TibiaBoneGuideTransform);
	transform->TransformPoint(tmp_cuttingCenter, cuttingCenter);
	transform->TransformNormal(tmp_cuttingNormal, cuttingNormal);
	vtkMath::Normalize(cuttingNormal);
	for (int i = 0; i < 3; i++) {
		cuttingNormal[i] *= -1.0;
	}

	/// set the cutting plane center as a point, set planning plane center and normal
	double pointToPlaneDis = CalculatePointToPlaneDistance(cuttingCenter, targetCenter, targetNormal);
	m_Controls.lineEdit_AxialAngle_Diff->setText(QString::number(pointToPlaneDis));

	// set planning center as a point, set cutting plane center and normal
	double pointToPlaneDis_1 = CalculatePointToPlaneDistance(targetCenter, cuttingCenter, cuttingNormal);
	m_Controls.lineEdit_SagittalAngle_Diff->setText(QString::number(pointToPlaneDis_1));

	if (!AxesActor) {
		AxesActor = vtkSmartPointer<vtkAxesActor>::New();
		//AxesActor = GenerateAxesActor(75.0, tmpVtkTransform->GetMatrix(), normal_dir);
		UpdateAxesActor(50, AxesActor, cuttingCenter, cuttingNormal);
		//UpdateAxesActor(50.0, AxesActor, TibiaBoneGuideTransform, normal_dir);
		AddActor(this->GetRenderWindowPart(), AxesActor);
	}
	else {
		UpdateAxesActor(50, AxesActor, cuttingCenter, cuttingNormal);
		//UpdateAxesActor(50.0, AxesActor, TibiaBoneGuideTransform, normal_dir);
	}
	*/
	
	/*
	/// old Bond Guide Navigation method below.
	// OverView: convert tibiaBoneGuideImageToCuttingPlane -> tibiaBoneGuideRFToCuttingPlane -> CameraToCuttingPlane -> PatientRFToCuttingPlane -> ImagetoCuttingPlane

	// 1. convert surface's coordinate: tibiaBoneGuideImage -> tibiaBoneGuideRF, using invert(m_T_tibiaBoneGuideRFtoTBGImage), cal by bone registration/calibration
	vtkSmartPointer<vtkMatrix4x4> T_TBGImageTotibiaBoneGuideRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(m_T_tibiaBoneGuideRFtoTBGImage, T_TBGImageTotibiaBoneGuideRF); 

	// 2. convert surface's coordinate: tibiaBoneGuideRF -> Camera
	mitk::NavigationData::Pointer nd_CameraToBoneGuideRf = m_CameraSource->GetOutput(m_ToolStorage->GetToolIndexByName("TibiaBoneGuideRF")); // target Bone Guide RF
	auto T_BoneGuideRFToCamera = getVtkMatrix4x4(nd_CameraToBoneGuideRf);

	// 3. convert surface's coordinate: Camera -> patient/Object RF, known: CameraToCuttingPlane,  
	mitk::NavigationData::Pointer nd_CameraToObjectRf = m_CameraSource->GetOutput(m_ToolStorage->GetToolIndexByName("TibiaRF"));
	auto T_CameraToPatientRf = getVtkMatrix4x4(nd_CameraToObjectRf);
	vtkSmartPointer<vtkMatrix4x4> T_PatientRfToCamera = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(T_CameraToPatientRf, T_PatientRfToCamera);

	// 4. convert surface's coordinate: patient/Object RF -> Image, using m_T_patientRFtoImage (cal by image registration)
	// m_T_patientRFtoImage

	// do vtk transform 
	vtkSmartPointer<vtkTransform> tmpVtkTransform = vtkTransform::New();
	tmpVtkTransform->PostMultiply();
	tmpVtkTransform->Identity();
	tmpVtkTransform->SetMatrix(TibiaBoneGuideCuttingPlaneSurface->GetGeometry()->GetVtkMatrix());
	tmpVtkTransform->Concatenate(T_TBGImageTotibiaBoneGuideRF); // coordinate: tibiaBoneGuideImage->tibiaBoneGuideRF
	tmpVtkTransform->Concatenate(T_BoneGuideRFToCamera); // coordinate: tibiaBoneGuideRF -> camera 
	tmpVtkTransform->Concatenate(T_PatientRfToCamera); // coordinate: camera -> patient/Object Rf 
	tmpVtkTransform->Concatenate(m_T_patientRFtoImage); // coordinate: patient/Object Rf -> Image
	
	//PrintVtkMatrix(tmpVtkTransform->GetMatrix(), "tmpVtkTransform");// debug
	

	// set position, rendering.
	GetDataStorage()->GetNamedNode("TibiaBoneGuideCuttingPlane")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(tmpVtkTransform->GetMatrix()); // move probe
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();

	// cal the error(normal angle, point-to-surface distance) between planning plane and bone guide cutting plane. 
	auto TibiaPlanningPlaneSurface = dynamic_cast<mitk::Surface*>(TibiaPlanningPlane->GetData());
	//vtkSmartPointer<vtkPolyData> TibiaPlanningPlanePolyData = TibiaPlanningPlaneSurface->GetVtkPolyData();
	ComputeTwoSurfaceError(TibiaPlanningPlaneSurface, tmpVtkTransform->GetMatrix());
	
	*/


	//// visualization, Create an Arrow: guide direction for Bone Guide

	//double normal_dir[3];
	//CalculateVector(TibiaPlanningPlaneSurface, tmpVtkTransform->GetMatrix(), normal_dir);

	//if (!AxesActor) {
	//	//VisualizeNormalAndCenter(tmpVtkTransform->GetMatrix());
	//	/*vtkSmartPointer<vtkActor> actor = GenerateAxesActor(tmpVtkTransform->GetMatrix());
	//	AddActor(this->GetRenderWindowPart(), actor);*/
	//	AxesActor = vtkSmartPointer<vtkAxesActor>::New();
	//	//AxesActor = GenerateAxesActor(75.0, tmpVtkTransform->GetMatrix(), normal_dir);
	//	UpdateAxesActor(50.0, AxesActor, tmpVtkTransform->GetMatrix(), normal_dir);
	//	AddActor(this->GetRenderWindowPart(), AxesActor);
	//}
	//else {
	//	UpdateAxesActor(50.0, AxesActor, tmpVtkTransform->GetMatrix(), normal_dir);
	//}

	
	///// ******* original code for NDI Camera, need to MODIFY ************////

	/******* New Method, using TKAN Class ********/
	/*
	mitk::NavigationData::Pointer nd_CameraToProbe = m_CameraSource->GetOutput(m_ToolStorage->GetToolIndexByName("Probe")); // get data of Probe
	mitk::NavigationData::Pointer nd_CameraToBoneGuideRf = m_CameraSource->GetOutput(m_ToolStorage->GetToolIndexByName("TibiaBoneGuideRF")); // target Bone Guide RF
	mitk::NavigationData::Pointer nd_CameraToObjectRf = m_CameraSource->GetOutput(m_ToolStorage->GetToolIndexByName("TibiaRF"));
	
	// Tibia Bone Guide Navigation
	auto TibiaBoneGuideCuttingPlaneSurface = dynamic_cast<mitk::Surface*>(TibiaBoneGuideCuttingPlane->GetData());
	auto TibiaBoneGuideTransform = Tkan.TibiaBoneGuideNavigation(nd_CameraToBoneGuideRf, nd_CameraToObjectRf);
	GetDataStorage()->GetNamedNode("TibiaBoneGuideSurface")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(TibiaBoneGuideTransform); //
	GetDataStorage()->GetNamedNode("TibiaBoneGuideCuttingPlane")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(TibiaBoneGuideTransform); //

	// Probe Navigation
	auto probePos = Tkan.ProbeNavigationInTibiaBone(nd_CameraToProbe, nd_CameraToObjectRf);
	GetDataStorage()->GetNamedNode("Probe")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(probePos); //
	
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();


	// cal the error(normal angle, point-to-surface distance) between planning plane and bone guide cutting plane. 
	auto TibiaPlanningPlaneSurface = dynamic_cast<mitk::Surface*>(TibiaPlanningPlane->GetData());
	//vtkSmartPointer<vtkPolyData> TibiaPlanningPlanePolyData = TibiaPlanningPlaneSurface->GetVtkPolyData();
	ComputeTwoSurfaceError(TibiaPlanningPlaneSurface, TibiaBoneGuideCuttingPlaneSurface, TibiaBoneGuideTransform);


	//// visualization, Create an Arrow: guide direction for Bone Guide

	double normal_dir[3];
	CalculateVector(TibiaPlanningPlaneSurface, TibiaBoneGuideTransform, normal_dir);

	// test
	/// target plane
	double targetCenter[3];
	double targetNormal[3];
	ComputeCenterAndNormalFromSurface(TibiaPlanningPlaneSurface, targetCenter, targetNormal);

	/// cutting plane
	double tmp_cuttingCenter[3], cuttingCenter[3];
	double tmp_cuttingNormal[3], cuttingNormal[3];
	ComputeCenterAndNormalFromSurface(TibiaBoneGuideCuttingPlaneSurface, tmp_cuttingCenter, tmp_cuttingNormal);

	// convert center and normal with vtkMatrix4x4
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->SetMatrix(TibiaBoneGuideTransform);
	transform->TransformPoint(tmp_cuttingCenter, cuttingCenter);
	transform->TransformNormal(tmp_cuttingNormal, cuttingNormal);
	vtkMath::Normalize(cuttingNormal);
	for (int i = 0; i < 3; i++) {
		cuttingNormal[i] *= -1.0;
	}

	/// set the cutting plane center as a point, set planning plane center and normal
	double pointToPlaneDis = CalculatePointToPlaneDistance(cuttingCenter, targetCenter, targetNormal); 
	m_Controls.lineEdit_AxialAngle_Diff->setText(QString::number(pointToPlaneDis));

	// set planning center as a point, set cutting plane center and normal
	double pointToPlaneDis_1 = CalculatePointToPlaneDistance(targetCenter, cuttingCenter, cuttingNormal);
	m_Controls.lineEdit_SagittalAngle_Diff->setText(QString::number(pointToPlaneDis_1));

	if (!AxesActor) {
		AxesActor = vtkSmartPointer<vtkAxesActor>::New();
		//AxesActor = GenerateAxesActor(75.0, tmpVtkTransform->GetMatrix(), normal_dir);
		UpdateAxesActor(50, AxesActor, cuttingCenter, cuttingNormal);
		//UpdateAxesActor(50.0, AxesActor, TibiaBoneGuideTransform, normal_dir);
		AddActor(this->GetRenderWindowPart(), AxesActor);
	}
	else {
		UpdateAxesActor(50, AxesActor, cuttingCenter, cuttingNormal);
		//UpdateAxesActor(50.0, AxesActor, TibiaBoneGuideTransform, normal_dir);
	}

	*/
}

void TotalKneeArthroplasty::saveTransformMatrix()
{
	std::string path = desktopPath + TransMtxSavePath;

	if (!std::filesystem::exists(path))
	{
		std::filesystem::create_directories(path);
	}

	std::string info;
	if (radioButtonID == 1) { // TibiaRF
		info = saveTransMtxInTxtFile(path, "TibiaRF", Tkan.GetTibiaRFtoImageTrans()->GetData());
	}
	else if (radioButtonID == 2) { // FemurRF
		info = saveTransMtxInTxtFile(path, "FemurRF", Tkan.GetFemurRFtoImageTrans()->GetData());
	}

	else if (radioButtonID == 3) { // TibiaBoneGuideRF
		info = saveTransMtxInTxtFile(path, "TibiaBoneGuideRF", Tkan.GetTBGRFtoTBGImageTrans()->GetData());
	}
	else if (radioButtonID == 4) { // FemurBoneGuideRF
		info = saveTransMtxInTxtFile(path, "FemurBoneGuideRF", Tkan.GetFBGRFtoFBGImageTrans()->GetData());
	}
	else if (radioButtonID == 5) { // 4in1BoneGuideRF
		info = saveTransMtxInTxtFile(path, "FourInOneBoneGuideRF", Tkan.GetFIOBGRFtoFIOBGImageTrans()->GetData());
	}

	else if (radioButtonID == 6) { // Plane Validator
		info = saveTransMtxInTxtFile(path, "PlaneValidatorRF", Tkan.GetPVRFtoPVImageTrans()->GetData());
	}

	else if (radioButtonID == 7) { // precision block
		info = saveTransMtxInTxtFile(path, "PrecisionBlockRF", Tkan.GetPBRFtoPBImageTrans()->GetData());
	}
	else if (radioButtonID == 8) { // precision bone guide
		info = saveTransMtxInTxtFile(path, "PrecisionBoneGuideRF", Tkan.GetPBGRFtoPBGImageTrans()->GetData());
	}

	m_Controls.textBrowser->append(info.c_str());
}

void TotalKneeArthroplasty::saveNaviTransformMatrix()
{
	Tkan.SetPBGNaviMtxInTargetPos(Tkan.GetPBGtoTargetImageTrans()); // assign the PBG navi transMtx 

	std::string path = desktopPath + TransMtxSavePath;

	if (!std::filesystem::exists(path))
	{
		std::filesystem::create_directories(path);
	}
	std::string info;
	info = saveTransMtxInTxtFile(path, "PrecisionBoneGuideNavi", Tkan.GetPBGtoTargetImageTrans()->GetData());
	m_Controls.textBrowser->append(info.c_str());
}

void TotalKneeArthroplasty::reloadTransformMatrix()
{
	std::string path = desktopPath + TransMtxSavePath;

	std::string info;
	double tmp_transMtx[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	if (radioButtonID == 1) { // TibiaRF
		info = reloadTransMtxInTxtFile(path, "TibiaRF", tmp_transMtx);
		Tkan.SetTibiaRFtoImageTrans(DoubleToVtkMatrix(tmp_transMtx));
	}
	else if (radioButtonID == 2) { // FemurRF
		info = reloadTransMtxInTxtFile(path, "FemurRF", tmp_transMtx);
		Tkan.SetFemurRFtoImageTrans(DoubleToVtkMatrix(tmp_transMtx));
	}

	else if (radioButtonID == 3) { // TibiaBoneGuideRF
		info = reloadTransMtxInTxtFile(path, "TibiaBoneGuideRF", tmp_transMtx);
		Tkan.SetTBGRFtoTBGImageTrans(DoubleToVtkMatrix(tmp_transMtx));
	}
	else if (radioButtonID == 4) { // FemurBoneGuideRF
		info = reloadTransMtxInTxtFile(path, "FemurBoneGuideRF", tmp_transMtx);
		Tkan.SetFBGRFtoFBGImageTrans(DoubleToVtkMatrix(tmp_transMtx));
	}
	else if (radioButtonID == 5) { // 4in1BoneGuideRF
		info = reloadTransMtxInTxtFile(path, "FourInOneBoneGuideRF", tmp_transMtx);
		Tkan.SetFIOBGRFtoFIOBGImageTrans(DoubleToVtkMatrix(tmp_transMtx));
	}

	else if (radioButtonID == 6) { // Plane Validator
		info = reloadTransMtxInTxtFile(path, "PlaneValidatorRF", tmp_transMtx);
		Tkan.SetPVRFtoPVImageTrans(DoubleToVtkMatrix(tmp_transMtx));
	}

	else if (radioButtonID == 7) { // precision block
		info = reloadTransMtxInTxtFile(path, "PrecisionBlockRF", tmp_transMtx);
		Tkan.SetPBRFtoPBImageTrans(DoubleToVtkMatrix(tmp_transMtx));
	}
	else if (radioButtonID == 8) { // precision bone guide
		info = reloadTransMtxInTxtFile(path, "PrecisionBoneGuideRF", tmp_transMtx);
		Tkan.SetPBGRFtoPBGImageTrans(DoubleToVtkMatrix(tmp_transMtx));
	}

	m_Controls.textBrowser->append(info.c_str());
}

void TotalKneeArthroplasty::reloadNaviTransformMatrix()
{
	std::string path = desktopPath + TransMtxSavePath;

	std::string info;
	double tmp_transMtx[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

	info = reloadTransMtxInTxtFile(path, "PrecisionBoneGuideNavi", tmp_transMtx);
	Tkan.SetPBGNaviMtxInTargetPos(DoubleToVtkMatrix(tmp_transMtx)); // assign the PBG navi transMtx 
	
	m_Controls.textBrowser->append(info.c_str());
}

mitk::Point3D TotalKneeArthroplasty::getMitkPoint3D(double x, double y, double z)
{
	mitk::Point3D point;
	point[0] = x;
	point[1] = y;
	point[2] = z;
	return point;
}


void TotalKneeArthroplasty::generateLaserPointsNPlanes()
{
	// precision block laser coordinate
	double PB_p1_x = m_Controls.lineEdit_PB_p1_x->text().toDouble(), PB_p1_y = m_Controls.lineEdit_PB_p1_y->text().toDouble(), PB_p1_z = m_Controls.lineEdit_PB_p1_z->text().toDouble();
	double PB_p2_x = m_Controls.lineEdit_PB_p2_x->text().toDouble(), PB_p2_y = m_Controls.lineEdit_PB_p2_y->text().toDouble(), PB_p2_z = m_Controls.lineEdit_PB_p2_z->text().toDouble();
	double PB_p3_x = m_Controls.lineEdit_PB_p3_x->text().toDouble(), PB_p3_y = m_Controls.lineEdit_PB_p3_y->text().toDouble(), PB_p3_z = m_Controls.lineEdit_PB_p3_z->text().toDouble();
	
	// precision bone guide laser coordinate
	double PBG_p1_x = m_Controls.lineEdit_PBG_p1_x->text().toDouble(), PBG_p1_y = m_Controls.lineEdit_PBG_p1_y->text().toDouble(), PBG_p1_z = m_Controls.lineEdit_PBG_p1_z->text().toDouble();
	double PBG_p2_x = m_Controls.lineEdit_PBG_p2_x->text().toDouble(), PBG_p2_y = m_Controls.lineEdit_PBG_p2_y->text().toDouble(), PBG_p2_z = m_Controls.lineEdit_PBG_p2_z->text().toDouble();
	double PBG_p3_x = m_Controls.lineEdit_PBG_p3_x->text().toDouble(), PBG_p3_y = m_Controls.lineEdit_PBG_p3_y->text().toDouble(), PBG_p3_z = m_Controls.lineEdit_PBG_p3_z->text().toDouble();

	mitk::PointSet::Pointer PBLaserPointSet = mitk::PointSet::New();
	mitk::PointSet::Pointer PBGLaserPointSet = mitk::PointSet::New();
	PBLaserPointSet->Clear(); PBGLaserPointSet->Clear();

	PBLaserPointSet->InsertPoint(getMitkPoint3D( PB_p1_x, PB_p1_y, PB_p1_z ));
	PBLaserPointSet->InsertPoint(getMitkPoint3D( PB_p2_x, PB_p2_y, PB_p2_z ));
	PBLaserPointSet->InsertPoint(getMitkPoint3D( PB_p3_x, PB_p3_y, PB_p3_z ));

	PBGLaserPointSet->InsertPoint(getMitkPoint3D( PBG_p1_x, PBG_p1_y, PBG_p1_z ));
	PBGLaserPointSet->InsertPoint(getMitkPoint3D( PBG_p2_x, PBG_p2_y, PBG_p2_z ));
	PBGLaserPointSet->InsertPoint(getMitkPoint3D( PBG_p3_x, PBG_p3_y, PBG_p3_z ));

	generatePointSet(PBLaserPointSet, "PBLaserPointSet"); // draw the pointset
	generatePointSet(PBGLaserPointSet, "PBGLaserPointSet"); // draw the pointset

	generatePlane(PBLaserPointSet, "PBLaserPlane", "green"); // draw the plane by pointset
	generatePlane(PBGLaserPointSet, "PBGLaserPlane", "orange"); // draw the plane by pointset

	mitk::RenderingManager::GetInstance()->RequestUpdateAll();

	// compute the Precision Errors; point-surface dis error & surface-surface angle error; use TKAN functions.
	mitk::PointSet::Pointer PBLaserPointset = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("PBLaserPointSet")->GetData()); // precision block
	mitk::Surface::Pointer PBLaserPlane = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("PBLaserPlane")->GetData());
	mitk::PointSet::Pointer PBGLaserPointset = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("PBGLaserPointSet")->GetData()); // precision bone guide
	mitk::Surface::Pointer PBGLaserPlane = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("PBGLaserPlane")->GetData());

	vtkSmartPointer<vtkMatrix4x4> identityMtx = vtkSmartPointer<vtkMatrix4x4>::New();
	double dis = -1.0, angleErr = -1.0, pitch = -1.0, roll = -1.0, dis1 = -1.0, dis2 = -1.0, dis3 = -1.0, yaw = -1.0; 
	std::tie(dis, angleErr, pitch, roll, yaw) = Tkan.ComputePrecisionBGSurfaceError(PBLaserPlane, PBGLaserPlane, identityMtx);  // S2S angle error
	std::tie(dis1, dis2, dis3) = Tkan.ComputePointToSurfaceDisError(PBLaserPointset, PBGLaserPointset, identityMtx); // P2S dis error

	m_Controls.lineEdit_Dis_p2s_1->setText(QString::number(dis1));
	m_Controls.lineEdit_Dis_p2s_2->setText(QString::number(dis2));
	m_Controls.lineEdit_Dis_p2s_3->setText(QString::number(dis3));
	m_Controls.lineEdit_Angle_s2s->setText(QString::number(angleErr));

	m_Controls.lineEdit_distanceErr->setText(QString::number(dis));
	m_Controls.lineEdit_surfaceError->setText(QString::number(angleErr));
	m_Controls.lineEdit_pitchErr->setText(QString::number(pitch));
	m_Controls.lineEdit_rollErr->setText(QString::number(roll));

	m_Controls.lineEdit_AxialAngle_Diff->setText(QString::number(pitch));
	m_Controls.lineEdit_SagittalAngle_Diff->setText(QString::number(roll));
	m_Controls.lineEdit_CoronalAngle_Diff->setText(QString::number(yaw));
}


void TotalKneeArthroplasty::generatePointSet(mitk::PointSet::Pointer pointSet, std::string pointSetName)
{
	if (!pointSet) {
		MITK_INFO << "pointSet Invalid";
		return;
	}

	//// set only 3D window can visibility
	//GetDataStorage()->GetNamedNode("stdmulti.widget0.plane")->SetVisibility(false);
	//GetDataStorage()->GetNamedNode("stdmulti.widget1.plane")->SetVisibility(false);
	//GetDataStorage()->GetNamedNode("stdmulti.widget2.plane")->SetVisibility(false);

	mitk::DataNode::Pointer pointSetNode = mitk::DataNode::New();

	//set point size
	float newPointSize = 10.0; // Example size
	pointSet->SetProperty("pointsize", mitk::FloatProperty::New(newPointSize));

	pointSetNode->SetData(pointSet);
	pointSetNode->SetName(pointSetName);

	// already inut point interaction
	GetDataStorage()->Add(pointSetNode);
}

void TotalKneeArthroplasty::generatePlane(mitk::PointSet::Pointer pointSet, std::string pointSetName, std::string colorName)
{
	// check the pointset 
	if (pointSet.IsNull() || pointSet->GetSize() < 3)
	{
		MITK_INFO << "PointSet must contain at least 3 points to define a plane!" ;
		return;
	}

	// get the point from pointset.
	mitk::Point3D point1 = pointSet->GetPoint(0);
	mitk::Point3D point2 = pointSet->GetPoint(1);
	mitk::Point3D point3 = pointSet->GetPoint(2);

	// mitk::point3D -> double[3]
	double p1[3] = { point1[0], point1[1], point1[2] };
	double p2[3] = { point2[0], point2[1], point2[2] };
	double p3[3] = { point3[0], point3[1], point3[2] };

	// cal the two vectors, p2-p1 & p3-p1
	double v1[3], v2[3], normal[3];
	vtkMath::Subtract(p2, p1, v1);
	vtkMath::Subtract(p3, p1, v2);

	// cal the normal of two vectors/plane 
	vtkMath::Cross(v1, v2, normal);
	vtkMath::Normalize(normal);

	double centroid[3], rectPt1[3], rectPt2[3], rectPt3[3];
	centroid[0] = (p1[0] + p2[0] + p3[0]) / 3.0;
	centroid[1] = (p1[1] + p2[1] + p3[1]) / 3.0;
	centroid[2] = (p1[2] + p2[2] + p3[2]) / 3.0;
	double len = std::max({EuclideanDistance(centroid, p1), EuclideanDistance(centroid, p2), EuclideanDistance(centroid, p3)}) * 2 ;
	generateRectanglePoints(centroid, normal, len, len, rectPt1, rectPt2, rectPt3);

	vtkSmartPointer<vtkPlaneSource> planeSource = vtkSmartPointer<vtkPlaneSource>::New();
	planeSource->SetOrigin(rectPt1);  // origin
	planeSource->SetPoint1(rectPt2);  // one side from plane
	planeSource->SetPoint2(rectPt3);  // another side from plane
	planeSource->Update();

	// generated plane to mitk::Surface
	mitk::Surface::Pointer planeSurface = mitk::Surface::New();
	planeSurface->SetVtkPolyData(planeSource->GetOutput());

	// create dataNode for plane
	mitk::DataNode::Pointer planeNode = mitk::DataNode::New();
	planeNode->SetData(planeSurface);
	if (colorMap.count(colorName)) planeNode->SetColor(colorMap[colorName][0], colorMap[colorName][1], colorMap[colorName][2]);
	else planeNode->SetColor(0.0, 1.0, 0.0);
	
	planeNode->SetName(pointSetName);

	GetDataStorage()->Add(planeNode);

}

void TotalKneeArthroplasty::generateRectanglePoints(double centroid[3], double normal[3], double& width, double& height, double p1[3], double p2[3], double p3[3])
{
	// create a vector that perpendicular to normal vector
	double u[3] = { 1.0, 0.0, 0.0 };
	if (fabs(normal[0]) > 0.9) {
		u[0] = 0.0;
		u[1] = 1.0;
	}

	double v[3];
	vtkMath::Cross(normal, u, v);
	vtkMath::Normalize(v);
	vtkMath::Cross(v, normal, u);
	vtkMath::Normalize(u);

	// 3 corner points in the plane
	for (int i = 0; i < 3; ++i) {
		p1[i] = centroid[i] + (width / 2.0) * u[i] + (height / 2.0) * v[i];
		p2[i] = centroid[i] - (width / 2.0) * u[i] + (height / 2.0) * v[i];
		p3[i] = centroid[i] + (width / 2.0) * u[i] - (height / 2.0) * v[i];
	}
}

void TotalKneeArthroplasty::calibrateWithPointID()
{
	// generated Laser pointset and plane
	mitk::PointSet::Pointer PBLaserPointset = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("PBLaserPointSet")->GetData()); // precision block
	mitk::Surface::Pointer PBLaserPlane = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("PBLaserPlane")->GetData());
	mitk::PointSet::Pointer PBGLaserPointset = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("PBGLaserPointSet")->GetData()); // precision bone guide
	mitk::Surface::Pointer PBGLaserPlane = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("PBGLaserPlane")->GetData());

	// get srcPointset (selected pointset) from "CT steelball center" pointset, A-F
	int PB_p1_ID = m_Controls.lineEdit_PB_p1_ID->text()[0].toUpper().unicode() - 'A'; // get ID number, A -> 0
	int PB_p2_ID = m_Controls.lineEdit_PB_p2_ID->text()[0].toUpper().unicode() - 'A';
	int PB_p3_ID = m_Controls.lineEdit_PB_p3_ID->text()[0].toUpper().unicode() - 'A';

	//m_Controls.textBrowser->append(QString::number(PB_p1_ID) + " " + QString::number(PB_p2_ID) + " " + QString::number(PB_p3_ID));
	mitk::PointSet::Pointer originalPointSet = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("CT steelball centers")->GetData());
	mitk::PointSet::Pointer SeletedCTPointset = mitk::PointSet::New();
	SeletedCTPointset->Clear();
	SeletedCTPointset->InsertPoint(getMitkPoint3D(originalPointSet->GetPoint(PB_p1_ID).GetDataPointer()[0], originalPointSet->GetPoint(PB_p1_ID).GetDataPointer()[1], originalPointSet->GetPoint(PB_p1_ID).GetDataPointer()[2]));
	SeletedCTPointset->InsertPoint(getMitkPoint3D(originalPointSet->GetPoint(PB_p2_ID).GetDataPointer()[0], originalPointSet->GetPoint(PB_p2_ID).GetDataPointer()[1], originalPointSet->GetPoint(PB_p2_ID).GetDataPointer()[2]));
	SeletedCTPointset->InsertPoint(getMitkPoint3D(originalPointSet->GetPoint(PB_p3_ID).GetDataPointer()[0], originalPointSet->GetPoint(PB_p3_ID).GetDataPointer()[1], originalPointSet->GetPoint(PB_p3_ID).GetDataPointer()[2]));


	Tkan.PBLaserRegistration(SeletedCTPointset, PBLaserPointset, PBLaserPlane); // cal the calibration(PBLaser->PBImage) transform matrix

	PrintVtkMatrix(Tkan.GetPBLasertoPBImageTrans(), "GetPBLasertoPBImageTrans");
	//GetDataStorage()->GetNamedNode("PBLaserPlane")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(Tkan.GetPBLasertoPBImageTrans()); // move PBLaserPlane
	//GetDataStorage()->GetNamedNode("PBLaserPointSet")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(Tkan.GetPBLasertoPBImageTrans());
	//GetDataStorage()->GetNamedNode("PBGLaserPlane")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(Tkan.GetPBLasertoPBImageTrans()); // move PBGLaserPlane
	//GetDataStorage()->GetNamedNode("PBGLaserPointSet")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(Tkan.GetPBLasertoPBImageTrans());
	//mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void TotalKneeArthroplasty::onPrecCuttingPlaneCheckButtonClicked()
{
	if (m_Controls.checkBox_precCuttingPlaneSwitch->isChecked()) { //  LASER PBG Cutting Plane
		// 1. replace the "precCuttingPointset" to the "PBGLaserPointSet"(recorded Laser PBG Cutting plane pointset)
		// 2. replace the "BoneGuideCuttingPlaneSurface" to the "PBGLaserPlane"

		//precCuttingPointSet->Clear();
		//mitk::PointSet::Pointer PBGLaserPointset = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("PBGLaserPointSet")->GetData()); // precision bone guide
		//if (PBGLaserPointset.IsNotNull())
		//{
		//	for (unsigned int i = 0; i < PBGLaserPointset->GetSize(); ++i)
		//	{
		//		precCuttingPointSet->InsertPoint(PBGLaserPointset->GetPoint(i));
		//	}
		//}
		precCuttingPointSet = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("PBGLaserPointSet")->GetData()); // Calibrated Laser Cutting Pointset.
		cuttingPlane = GetDataStorage()->GetNamedNode("PBGLaserPlane"); // PBG Laser Plane
		m_Controls.textBrowser->append("Switch to the LASER PBG Cutting Plane");
	}
	else { // Original PBG Cutting Plane  
		precCuttingPointSet = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("PrecBoneGuideCuttingPlanePointset")->GetData()); 
		cuttingPlane = GetDataStorage()->GetNamedNode(planeDataName["PrecisionBGCuttingPlane"]); // precision BG cutting plane
		m_Controls.textBrowser->append("Switch to the Original PBG Cutting Plane");
	}
}

//void TotalKneeArthroplasty::CalculateVector(mitk::Surface::Pointer targetPlaneSurface, vtkSmartPointer<vtkMatrix4x4> cuttingPlaneMatrix, double normal_dir[3]) {
//	double targetCenter[3];
//	double n2[3];
//	ComputeCenterAndNormalFromSurface(targetPlaneSurface, targetCenter, n2);
//
//	double n1[3] = {
//		-1.0 * cuttingPlaneMatrix->GetElement(0, 2),
//		-1.0 * cuttingPlaneMatrix->GetElement(1, 2),
//		-1.0 * cuttingPlaneMatrix->GetElement(2, 2)
//	};
//
//	double cuttingPlaneCenter[3] = {
//		cuttingPlaneMatrix->GetElement(0, 3),
//		cuttingPlaneMatrix->GetElement(1, 3),
//		cuttingPlaneMatrix->GetElement(2, 3)
//	};
//
//	normal_dir[0] = targetCenter[0] - cuttingPlaneCenter[0];
//	normal_dir[1] = targetCenter[1] - cuttingPlaneCenter[1];
//	normal_dir[2] = targetCenter[2] - cuttingPlaneCenter[2];
//
//	double magnitude = std::sqrt(normal_dir[0] * normal_dir[0] + normal_dir[1] * normal_dir[1] + normal_dir[2] * normal_dir[2]);
//
//	for (int i = 0; i < 3; ++i) {
//		normal_dir[i] /= magnitude;
//	}
//}

vtkSmartPointer<vtkActor> TotalKneeArthroplasty::AddArrowActor() {
	vtkSmartPointer<vtkArrowSource> arrowSource = vtkSmartPointer<vtkArrowSource>::New();

	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(arrowSource->GetOutputPort());

	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	return actor;
}

vtkSmartPointer<vtkAxesActor> TotalKneeArthroplasty::GenerateAxesActor(double axesLength, vtkSmartPointer<vtkMatrix4x4> matrix, double normal_dir[3])
{
	// get the matrixSurface Normal and center(a point in surface)
	/*double normal[3] = {
		matrix->GetElement(0, 2),
		matrix->GetElement(1, 2),
		matrix->GetElement(2, 2)
	};*/
	double center[3] = {
		matrix->GetElement(0, 3),
		matrix->GetElement(1, 3),
		matrix->GetElement(2, 3)
	};

	// set original point and orientation
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->Translate(center);
	transform->RotateX(normal_dir[0]);
	transform->RotateY(normal_dir[1]);
	transform->RotateZ(normal_dir[2]);

	vtkSmartPointer<vtkAxesActor> axesActor = vtkSmartPointer<vtkAxesActor>::New();
	//axesActor->SetConeRadius(100);
	//axesActor->SetCylinderRadius(50);
	axesActor->SetTotalLength(axesLength, axesLength, axesLength);
	axesActor->SetUserTransform(transform);
	axesActor->SetAxisLabels(false);

	axesActor->GetXAxisTipProperty()->SetOpacity(0.0);
	axesActor->GetXAxisShaftProperty()->SetOpacity(0.0);
	axesActor->GetYAxisTipProperty()->SetOpacity(0.0);
	axesActor->GetYAxisShaftProperty()->SetOpacity(0.0);

	axesActor->Modified();

	return axesActor;
}

void TotalKneeArthroplasty::UpdateAxesActor(double axesLength, vtkSmartPointer<vtkAxesActor> actor, double center[3], double normal_dir[3])
{
	// compute the rotation and angle
	double zAxis[3] = { 0.0, 0.0, 1.0 };
	double rotationAxis[3];
	vtkMath::Cross(zAxis, normal_dir, rotationAxis);
	double angle = vtkMath::DegreesFromRadians(acos(vtkMath::Dot(zAxis, normal_dir)));

	// set original point and orientation
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->Translate(center);
	transform->RotateWXYZ(angle, rotationAxis);
	actor->SetUserTransform(transform);

	actor->SetTotalLength(axesLength, axesLength, axesLength);
	actor->SetAxisLabels(false);

	// show only z axis
	actor->GetXAxisTipProperty()->SetOpacity(0.0);
	actor->GetXAxisShaftProperty()->SetOpacity(0.0);
	actor->GetYAxisTipProperty()->SetOpacity(0.0);
	actor->GetYAxisShaftProperty()->SetOpacity(0.0);

	actor->Modified(); // update
}

void TotalKneeArthroplasty::UpdateAxesActor(double axesLength, vtkSmartPointer<vtkAxesActor> actor, vtkSmartPointer<vtkMatrix4x4> matrix, double normal_dir[3])
{
	// get the matrixSurface Normal and center(a point in surface)
	double normal[3] = {
		-1.0 * matrix->GetElement(0, 2),
		-1.0 * matrix->GetElement(1, 2),
		-1.0 * matrix->GetElement(2, 2)
	};

	double center[3] = {
		matrix->GetElement(0, 3),
		matrix->GetElement(1, 3),
		matrix->GetElement(2, 3)
	};

	m_Controls.textBrowser->append("normal: " + QString::number(normal[0]) + " " + QString::number(normal[1]) + " " + QString::number(normal[2]));
	m_Controls.textBrowser->append(QString::number(normal_dir[0]) + " " + QString::number(normal_dir[1]) + " " + QString::number(normal_dir[2]));

	// compute the rotation and angle
	double zAxis[3] = { 0.0, 0.0, 1.0 };
	double rotationAxis[3];
	vtkMath::Cross(zAxis, normal_dir, rotationAxis);
	double angle = vtkMath::DegreesFromRadians(acos(vtkMath::Dot(zAxis, normal_dir)));

	// set original point and orientation
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->Translate(center);
	transform->RotateWXYZ(angle, rotationAxis);
	actor->SetUserTransform(transform);

	actor->SetTotalLength(axesLength, axesLength, axesLength);
	actor->SetAxisLabels(false);

	// show only z axis
	actor->GetXAxisTipProperty()->SetOpacity(0.0);
	actor->GetXAxisShaftProperty()->SetOpacity(0.0);
	actor->GetYAxisTipProperty()->SetOpacity(0.0);
	actor->GetYAxisShaftProperty()->SetOpacity(0.0);

	actor->Modified(); // update
}

void TotalKneeArthroplasty::AddActor(mitk::IRenderWindowPart* renderWindowPart3D, vtkSmartPointer<vtkAxesActor> actor)
{
	QmitkRenderWindow* mitkRenderWindow = renderWindowPart3D->GetQmitkRenderWindow("3d");

	vtkRenderWindow* renderWindow = mitkRenderWindow->GetVtkRenderWindow();

	vtkSmartPointer<vtkRenderer> renderer;

	renderer = renderWindow->GetRenderers()->GetFirstRenderer();

	renderer->AddActor(actor);
}

void TotalKneeArthroplasty::ComputeTwoSurfaceError(mitk::Surface::Pointer targetPlane, mitk::Surface::Pointer cuttingPlane, vtkSmartPointer<vtkMatrix4x4> cuttingPlaneTransform)
{
	// this part (target surface) should be computed only once.
	double targetCenter[3];
	double targetNormal[3];
	ComputeCenterAndNormalFromSurface(targetPlane, targetCenter, targetNormal);
	
	//// convert surface to vtkmatrix4x4, then convert to  Axial,Sagittal,Coronal
	auto targetPlaneMatrix = SurfaceToMatrix(targetPlane);
	//double targetPlaneAxial[3], targetPlaneSagittal[3], targetPlaneCoronal[3];
	//MatrixToAxialSagittalCoronal(targetPlaneMatrix, targetPlaneAxial, targetPlaneSagittal, targetPlaneCoronal);

	//// convert cuttingMatrix to Axial,Sagittal,Coronal
	//double cuttingPlaneAxial[3], cuttingPlaneSagittal[3], cuttingPlaneCoronal[3];
	//MatrixToAxialSagittalCoronal(cuttingPlaneTransform, cuttingPlaneAxial, cuttingPlaneSagittal, cuttingPlaneCoronal);

	// get the Normal and Centroid of Cutting Plane 
	double tmp_cuttingCenter[3], cuttingCenter[3], tmp_cuttingNormal[3], cuttingNormal[3];
	ComputeCenterAndNormalFromSurface(cuttingPlane, tmp_cuttingCenter, tmp_cuttingNormal);

	m_Controls.textBrowser->append(QString::number(tmp_cuttingCenter[0]) + " " + QString::number(tmp_cuttingCenter[1]) + " " + QString::number(tmp_cuttingCenter[2]) );
	m_Controls.textBrowser->append(QString::number(tmp_cuttingNormal[0]) + " " + QString::number(tmp_cuttingNormal[1]) + " " + QString::number(tmp_cuttingNormal[2]));

	// convert center and normal with vtkMatrix4x4, Coordinate transform: Bone Guide Image -> Bone Image
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->SetMatrix(cuttingPlaneTransform);
	transform->TransformPoint(tmp_cuttingCenter, cuttingCenter);
	transform->TransformNormal(tmp_cuttingNormal, cuttingNormal);
	vtkMath::Normalize(cuttingNormal);

	vtkSmartPointer<vtkMatrix4x4> cuttingPlaneMtx = vtkSmartPointer<vtkMatrix4x4>::New();
	cuttingPlaneMtx->Identity();
	for (int i = 0; i < 3; i++) { // set elements
		cuttingPlaneMtx->SetElement(i, 3, cuttingCenter[i]);
	}
	cuttingPlaneMtx->SetElement(0, 2, cuttingNormal[0]);
	cuttingPlaneMtx->SetElement(1, 2, cuttingNormal[1]);
	cuttingPlaneMtx->SetElement(2, 2, cuttingNormal[2]);

	double targetPlaneEulerAngles[3], cuttingPlaneEulerAngles[3], EulerAnglesDifference[3];
	vtkTransform::GetOrientation(cuttingPlaneEulerAngles, cuttingPlaneMtx);
	vtkTransform::GetOrientation(targetPlaneEulerAngles, targetPlaneMatrix);
	CalculateEulerAnglesDifference(targetPlaneEulerAngles, cuttingPlaneEulerAngles, EulerAnglesDifference);
	
	/*if (cuttingPlaneYaw > 90.0) cuttingPlaneYaw = 180.0 - cuttingPlaneYaw;
	if (cuttingPlaneRoll > 90.0) cuttingPlaneRoll = 180.0 - cuttingPlaneRoll;
	if (cuttingPlanePitch > 90.0) cuttingPlanePitch = 180.0 - cuttingPlanePitch;*/
	//m_Controls.lineEdit_CuttingPlane_x->setText(QString::number(EulerAnglesDifference[0])); // Pitch
	//m_Controls.lineEdit_CuttingPlane_y->setText(QString::number(EulerAnglesDifference[1])); // Roll
	//m_Controls.lineEdit_CuttingPlane_z->setText(QString::number(EulerAnglesDifference[2])); // Yaw


	//// comapre the angle diff between "targetPlane" and "cuttingPlane"
	//double axialAngle = CalculateAngleBetweenVectors(targetPlaneAxial, cuttingPlaneAxial);
	//double sagittalAngle = CalculateAngleBetweenVectors(targetPlaneSagittal, cuttingPlaneSagittal);
	//double coronalAngle = CalculateAngleBetweenVectors(targetPlaneCoronal, cuttingPlaneCoronal);

	/*if (axialAngle > 90.0) axialAngle = 180.0 - axialAngle;
	if (sagittalAngle > 90.0) sagittalAngle = 180.0 - sagittalAngle;
	if (coronalAngle > 90.0) coronalAngle = 180.0 - coronalAngle;

	m_Controls.lineEdit_AxialAngle_Diff->setText(QString::number(axialAngle));
	m_Controls.lineEdit_SagittalAngle_Diff->setText(QString::number(sagittalAngle));
	m_Controls.lineEdit_CoronalAngle_Diff->setText(QString::number(coronalAngle));*/

	// compute the Point-to-Plane distance between "targetPlane" center and "cuttingPlane" surface
	auto P2SDistance = CalculatePointToPlaneDistance(cuttingPlaneMtx, targetPlaneMatrix);
	m_Controls.lineEdit_Dis_Point2Surface->setText(QString::number(P2SDistance));
}

void TotalKneeArthroplasty::CalculateEulerAnglesDifference(const double eulerAngles1[3], const double eulerAngles2[3], double difference[3]) {
	for (int i = 0; i < 3; ++i) {
		difference[i] = eulerAngles2[i] - eulerAngles1[i];
	}
}

vtkSmartPointer<vtkMatrix4x4> TotalKneeArthroplasty::SurfaceToMatrix(mitk::Surface::Pointer surface)
{
	vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	matrix->Identity(); 

	// get VTK PolyData
	vtkSmartPointer<vtkPolyData> polyData = surface->GetVtkPolyData();

	// compute Center
	vtkSmartPointer<vtkCenterOfMass> centerOfMassFilter = vtkSmartPointer<vtkCenterOfMass>::New();
	centerOfMassFilter->SetInputData(polyData);
	centerOfMassFilter->SetUseScalarsAsWeights(false);
	centerOfMassFilter->Update();
	double center[3];
	centerOfMassFilter->GetCenter(center);

	// compute Normal
	vtkSmartPointer<vtkPolyDataNormals> normalsFilter = vtkSmartPointer<vtkPolyDataNormals>::New();
	normalsFilter->SetInputData(polyData);
	normalsFilter->ComputeCellNormalsOn();
	normalsFilter->ComputePointNormalsOn();
	normalsFilter->Update();
	vtkDataArray* normals = normalsFilter->GetOutput()->GetCellData()->GetNormals();
	double normal[3];
	normals->GetTuple(0, normal);

	// set elements
	for (int i = 0; i < 3; i++) {
		matrix->SetElement(i, 3, center[i]); 
	}

	matrix->SetElement(0, 2, normal[0]);
	matrix->SetElement(1, 2, normal[1]);
	matrix->SetElement(2, 2, normal[2]);

	return matrix;
}

void TotalKneeArthroplasty::ComputeCenterAndNormalFromSurface(mitk::Surface::Pointer surface, double center[3], double normal[3])
{
	// get VTK PolyData
	vtkSmartPointer<vtkPolyData> polyData = surface->GetVtkPolyData();

	// Compute the center point
	vtkSmartPointer<vtkCenterOfMass> centerOfMassFilter = vtkSmartPointer<vtkCenterOfMass>::New();
	centerOfMassFilter->SetInputData(polyData);
	centerOfMassFilter->SetUseScalarsAsWeights(false);
	centerOfMassFilter->Update();
	centerOfMassFilter->GetCenter(center);

	// Compute the normal
	vtkSmartPointer<vtkPolyDataNormals> normals = vtkSmartPointer<vtkPolyDataNormals>::New();
	normals->SetInputData(polyData);
	normals->ComputePointNormalsOff();
	normals->ComputeCellNormalsOn();
	normals->Update();

	vtkSmartPointer<vtkPolyData> normalData = normals->GetOutput();
	normalData->GetCellData()->GetNormals()->GetTuple(0, normal);
}

void TotalKneeArthroplasty::MatrixToAxialSagittalCoronal(vtkSmartPointer<vtkMatrix4x4> matrix, double axial[3], double sagittal[3], double coronal[3])
{
	// Axial plane normal is typically aligned with the Z-axis
	axial[0] = matrix->GetElement(0, 2);
	axial[1] = matrix->GetElement(1, 2);
	axial[2] = matrix->GetElement(2, 2);

	// Sagittal plane normal is typically aligned with the X-axis
	sagittal[0] = matrix->GetElement(0, 0);
	sagittal[1] = matrix->GetElement(1, 0);
	sagittal[2] = matrix->GetElement(2, 0);

	// Coronal plane normal is typically aligned with the Y-axis
	coronal[0] = matrix->GetElement(0, 1);
	coronal[1] = matrix->GetElement(1, 1);
	coronal[2] = matrix->GetElement(2, 1);
}


double TotalKneeArthroplasty::CalculateAngleBetweenVectors(const double u[3], const double v[3]) 
{
	double dotProduct = vtkMath::Dot(u, v);
	double magnitudeU = vtkMath::Norm(u);
	double magnitudeV = vtkMath::Norm(v);
	double cosineOfAngle = dotProduct / (magnitudeU * magnitudeV);
	cosineOfAngle = std::max(-1.0, std::min(1.0, cosineOfAngle)); // [-1, 1];
	double angle = std::acos(cosineOfAngle);
	return vtkMath::DegreesFromRadians(angle); // convert radians to degrees
}

double TotalKneeArthroplasty::CalculatePointToPlaneDistance(double point[3], double center[3], double normal[3])
{
	/*
	input: cutting surface: center
	input: planning surface: center and normal
	output: point to plane distance
	*/

	double normalMagnitude = vtkMath::Norm(normal);

	double pointToPlaneVector[3];
	pointToPlaneVector[0] = point[0] - center[0];
	pointToPlaneVector[1] = point[1] - center[1];
	pointToPlaneVector[2] = point[2] - center[2];

	double distance = std::abs(vtkMath::Dot(pointToPlaneVector, normal)) / normalMagnitude;

	return distance;
}

double TotalKneeArthroplasty::CalculatePointToPlaneDistance(vtkSmartPointer<vtkMatrix4x4> matrixPoint, vtkSmartPointer<vtkMatrix4x4> matrixSurface)
{
	// get the center point from target plane vtk matrix 4x4
	double center[3]
	{
		matrixPoint->GetElement(0,3),
		matrixPoint->GetElement(1,3),
		matrixPoint->GetElement(2,3),
	};

	// get the matrixSurface Normal and center(a point in surface)
	double normal[3];
	normal[0] = matrixSurface->GetElement(0, 2);
	normal[1] = matrixSurface->GetElement(1, 2);
	normal[2] = matrixSurface->GetElement(2, 2);

	double planePoint[3];
	planePoint[0] = matrixSurface->GetElement(0, 3);
	planePoint[1] = matrixSurface->GetElement(1, 3);
	planePoint[2] = matrixSurface->GetElement(2, 3);
	
	// cal the distance 
	double vectorPtoQ[3];
	vectorPtoQ[0] = center[0] - planePoint[0];
	vectorPtoQ[1] = center[1] - planePoint[1];
	vectorPtoQ[2] = center[2] - planePoint[2];

	double dotProduct = vtkMath::Dot(normal, vectorPtoQ);
	double normalMagnitude = vtkMath::Norm(normal);
	double distance = std::abs(dotProduct) / normalMagnitude;

	return distance;

}

void TotalKneeArthroplasty::VisualizeNormalAndCenter(vtkSmartPointer<vtkMatrix4x4> matrix)
{
	// get the matrixSurface Normal and center(a point in surface)
	double normal[3] = {
		matrix->GetElement(0, 2),
		matrix->GetElement(1, 2),
		matrix->GetElement(2, 2)
	};

	double center[3] = {
		matrix->GetElement(0, 3),
		matrix->GetElement(1, 3),
		matrix->GetElement(2, 3)
	};

	vtkSmartPointer<vtkArrowSource> arrowSource = vtkSmartPointer<vtkArrowSource>::New();

	// normal & center 
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->Translate(center);
	transform->RotateWXYZ(0, normal[0], normal[1], normal[2]); // arrow direction
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilter->SetTransform(transform);
	transformFilter->SetInputConnection(arrowSource->GetOutputPort());

	// mapping data to arrow
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(transformFilter->GetOutputPort());
	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(1.0, 0.0, 0.0); // rgb

	// create a mitk node
	mitk::Surface::Pointer surface = mitk::Surface::New();
	surface->SetVtkPolyData(vtkPolyData::SafeDownCast(actor->GetMapper()));
	mitk::DataNode::Pointer node = mitk::DataNode::New();
	node->SetName("Normal");
	node->SetData(surface);
	GetDataStorage()->Add(node);
	
	// rendering
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void TotalKneeArthroplasty::onGuideNavigationCheckButtonClicked() 
{
	if (m_Controls.checkBox_BoneGuideNavigationPermission->isChecked()) {
		m_Controls.textBrowser->append("Bone Guide Navigation - Navigation Permission Allowed");
	}
	else {
		m_Controls.textBrowser->append("Bone Guide Navigation - Navigation Permission Denied");
	}
}

void TotalKneeArthroplasty::ShowToolStatus()
{
	m_NavigationData.clear();
	for (std::size_t i = 0; i < m_CameraSource->GetNumberOfOutputs(); i++)
	{
		m_NavigationData.push_back(m_CameraSource->GetOutput(i));
	}
	//initialize widget
	m_Controls.m_StatusWidgetToolToShow->RemoveStatusLabels();
	m_Controls.m_StatusWidgetToolToShow->SetShowPositions(true);
	m_Controls.m_StatusWidgetToolToShow->SetTextAlignment(Qt::AlignLeft);
	m_Controls.m_StatusWidgetToolToShow->SetNavigationDatas(&m_NavigationData);
	m_Controls.m_StatusWidgetToolToShow->ShowStatusLabels();
}

// debug function
void TotalKneeArthroplasty::PrintVtkMatrix(vtkSmartPointer<vtkMatrix4x4> matrix, QString message)
{
	if (!matrix)
	{
		std::cerr << "Invalid matrix pointer!" << std::endl;
		return;
	}

	m_Controls.textBrowser->append(message);
	for (int i = 0; i < 4; ++i)
	{
		QString s;
		for (int j = 0; j < 4; ++j)
		{
			s += QString::number(matrix->GetElement(i, j));
			if (i != 3 || j != 3) s += QString(", ");
		}
		m_Controls.textBrowser->append(s);
	}
}

void TotalKneeArthroplasty::PrintTransformMatrix()
{
	if (radioButtonID == 1) { // TibiaRF
		PrintVtkMatrix(Tkan.GetTibiaRFtoImageTrans(), "TibiaRF to Tibia Image");
	}
	else if (radioButtonID == 2) { // FemurRF
		PrintVtkMatrix(Tkan.GetFemurRFtoImageTrans(), "FemurRF to Femur Image");
	}

	else if (radioButtonID == 3) { // TibiaBoneGuideRF
		PrintVtkMatrix(Tkan.GetTBGRFtoTBGImageTrans(), "TibiaBoneGuideRF to TBG Image");
	}
	else if (radioButtonID == 4) { // DistalFemurBoneGuide
		PrintVtkMatrix(Tkan.GetFBGRFtoFBGImageTrans(), "FemurBoneGuide to FBG Image");
	}
	else if (radioButtonID == 5) { // 4in1BoneGuide
		PrintVtkMatrix(Tkan.GetFIOBGRFtoFIOBGImageTrans(), "4in1BoneGuide to FIO Image");
	}

	else if (radioButtonID == 6) { // Plane Validator
		PrintVtkMatrix(Tkan.GetPVRFtoPVImageTrans(), "Plane Validator to PV Image");
	}

	else if (radioButtonID == 7) { // precision block
		PrintVtkMatrix(Tkan.GetPBRFtoPBImageTrans(), "Precision Block to PB Image");
	}
	else if (radioButtonID == 8) { // precision bone guide
		PrintVtkMatrix(Tkan.GetPBGRFtoPBGImageTrans(), "Plane Bone Guide to PBG Image");
	}
}


void TotalKneeArthroplasty::printNaviTransformMatrix()
{
	PrintVtkMatrix(Tkan.GetPBGNaviMtxInTargetPos(), "Precision Bone Guide Navi Mtx in Target Pos");
}

/***************** Plane Generation *****************/

// Generate a vtk Plane and save as .stl file (at least 3 Points)
void TotalKneeArthroplasty::GeneratePlaneByPointset()
{
	auto cuttingPlanePointset = dynamic_cast<mitk::PointSet*>(m_Controls.mitkNodeSelectWidget_CuttingPlanePoints->GetSelectedNode()->GetData());

	// get absPath
	mitk::DataNode::Pointer selectedNode = m_Controls.mitkNodeSelectWidget_CuttingPlanePoints->GetSelectedNode();
	std::string absPath = "", filename = "";
	//selectedNode->GetStringProperty("path", absPath);
	absPath = m_Controls.lineEdit_ResultPath->text().toStdString();
	//selectedNode->GetStringProperty("name", filename);
	//m_Controls.textBrowser->append(QString::fromStdString(absPath));
	//m_Controls.textBrowser->append(QString::fromStdString(filename));

	if (cuttingPlanePointset->GetSize() < 3) {
		m_Controls.textBrowser->append("Not enough points to define a plane.");
		return;
	}

	// Get the three points
	mitk::Point3D p1 = cuttingPlanePointset->GetPoint(0);
	mitk::Point3D p2 = cuttingPlanePointset->GetPoint(1);
	mitk::Point3D p3 = cuttingPlanePointset->GetPoint(2);

	// Compute the vectors defining the plane
	mitk::Vector3D v1 = p2 - p1;
	mitk::Vector3D v2 = p3 - p1;

	// Compute the normal to the plane
	mitk::Vector3D normal = itk::CrossProduct(v1, v2);
	normal.Normalize();
	m_Controls.textBrowser->append(QString::number(normal[0]) + " " + QString::number(normal[1]) + " " + QString::number(normal[2]));

	// Compute the centroid of the plane
	mitk::Point3D centroid;
	centroid[0] = (p1[0] + p2[0] + p3[0]) / 3.0;
	centroid[1] = (p1[1] + p2[1] + p3[1]) / 3.0;
	centroid[2] = (p1[2] + p2[2] + p3[2]) / 3.0;

	// Create a plane 
	vtkSmartPointer<vtkPlaneSource> planeSource = vtkSmartPointer<vtkPlaneSource>::New();

	// set Normal and Center
	planeSource->SetCenter(centroid[0], centroid[1], centroid[2]);
	planeSource->SetNormal(normal[0], normal[1], normal[2]);
	planeSource->SetXResolution(50);  // Increase the resolution as needed
	planeSource->SetYResolution(50);  // Increase the resolution as needed

	// set the plane width and height
	double width = 40.0, height = 30.0;
	if (m_Controls.lineEdit_CuttingPlaneWidth) width = m_Controls.lineEdit_CuttingPlaneWidth->text().toDouble();
	if (m_Controls.lineEdit_CuttingPlaneHeight) height = m_Controls.lineEdit_CuttingPlaneHeight->text().toDouble();

	double corner_pt1[3], corner_pt2[3], corner_pt3[3]; // 3 corner points of the plane
	CalculatePlanePoints(centroid, normal, height, width, corner_pt1, corner_pt2, corner_pt3);

	// make sure the vec(pt1-pt2) Perpendicular to vec(pt1-pt3), pt1-pt2: width, pt1-pt3: height
	planeSource->SetOrigin(corner_pt1);
	planeSource->SetPoint1(corner_pt2);
	planeSource->SetPoint2(corner_pt3);
	planeSource->Update();

	// Save as .stl file
	vtkSmartPointer<vtkSTLWriter> stlWriter = vtkSmartPointer<vtkSTLWriter>::New();
	filename = "TibiaCuttingPlanPlane.stl";
	if (m_Controls.lineEdit_Filename) filename = m_Controls.lineEdit_Filename->text().toStdString();
	std::string path = absPath + "/" + filename + ".stl";
	m_Controls.textBrowser->append("Save path: " + QString::fromStdString(path));
	stlWriter->SetFileName(path.c_str());
	stlWriter->SetInputConnection(planeSource->GetOutputPort());
	stlWriter->Write();
}

// Calculate corner points in Plane by normals and centriod, for setting the width and height.
void TotalKneeArthroplasty::CalculatePlanePoints(const mitk::Point3D& center, const mitk::Vector3D& normal, double height, double width, double point1[3], double point2[3], double point3[3])
{
	// create a vector that perpendicular to normal vector
	double u[3] = { 1.0, 0.0, 0.0 };
	if (fabs(normal[0]) > 0.9) {
		u[0] = 0.0;
		u[1] = 1.0;
	}

	double v[3];
	vtkMath::Cross(normal.GetDataPointer(), u, v);
	vtkMath::Normalize(v);
	vtkMath::Cross(v, normal.GetDataPointer(), u);
	vtkMath::Normalize(u);

	// 3 corner points in the plane
	for (int i = 0; i < 3; ++i) {
		point1[i] = center[i] + (width / 2.0) * u[i] + (height / 2.0) * v[i];
		point2[i] = center[i] - (width / 2.0) * u[i] + (height / 2.0) * v[i];
		point3[i] = center[i] + (width / 2.0) * u[i] - (height / 2.0) * v[i];
	}
}
/***************** Plane Generation *****************/


//// Coordinate Conversion, convert the nd data to nd_ref Coordinate.
//mitk::NavigationData::Pointer TotalKneeArthroplasty::GetNavigationDataInRef(mitk::NavigationData::Pointer nd,
//	mitk::NavigationData::Pointer nd_ref)
//{
//	mitk::NavigationData::Pointer res = mitk::NavigationData::New();
//	res->Graft(nd);
//	res->Compose(nd_ref->GetInverse());
//	return res;
//}


void TotalKneeArthroplasty::OnSelectionChanged(berry::IWorkbenchPart::Pointer /*source*/,
	const QList<mitk::DataNode::Pointer>& nodes)
{
	//// iterate all selected objects, adjust warning visibility
	//foreach (mitk::DataNode::Pointer node, nodes)
	//{
	//  /*if (node.IsNotNull() && dynamic_cast<mitk::Image *>(node->GetData()))
	//  {
	//    m_Controls.labelWarning->setVisible(false);
	//    m_Controls.buttonPerformImageProcessing->setEnabled(true);
	//    return;
	//  }*/
	//}
}