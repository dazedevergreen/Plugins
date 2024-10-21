/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


//Aimooe Camera
#include "AimPositionAPI.h"
#include "AimPositionDef.h"

// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qmitk
#include "DentalRobot.h"

// Qt
#include <QMessageBox>

// mitk image
#include <mitkImage.h>
#include <QFileDialog>
#include <vtkAppendPolyData.h>
#include <vtkCamera.h>
#include <vtkCardinalSpline.h>
#include <vtkCellArray.h>
#include <vtkCleanPolyData.h>
#include <vtkClipPolyData.h>
#include <vtkConnectivityFilter.h>
#include <vtkImageAppend.h>
#include <vtkImageCast.h>
#include <vtkImageIterator.h>
#include <vtkImplicitPolyDataDistance.h>
#include <vtkPlanes.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkProbeFilter.h>
#include <vtkRendererCollection.h>
#include <vtkSplineFilter.h>
#include <ep/include/vtk-9.1/vtkTransformFilter.h>

#include "lancetTrackingDeviceSourceConfigurator.h"
#include "lancetVegaTrackingDevice.h"
#include "leastsquaresfit.h"
#include "mitkGizmo.h"
#include "mitkImageToSurfaceFilter.h"
#include "mitkMatrixConvert.h"
#include "mitkNavigationToolStorageDeserializer.h"
#include "mitkPointSet.h"
#include "QmitkDataStorageTreeModel.h"
#include "QmitkRenderWindow.h"
#include "surfaceregistraion.h"

#include <fstream>
#include <iostream>
#include <string>
#include <cstdlib>
#include <filesystem>

//Aimooe
//AimHandle aimHandle = NULL;
E_Interface EI;
T_MarkerInfo markerSt;
T_AimPosStatusInfo statusSt;
E_ReturnValue rlt;
QTimer* m_ServoPTimer{ nullptr };
QTimer* m_AimoeVisualizeTimer{ nullptr };
QTimer* m_timer{ nullptr };

//��׼��������ϵ��ͼ������ϵ
//m_steelBalls_cmm��U�͵����ϵĽ�����   m_probeDitchPset_cmm��U�͵���ģ���ϵİ��۵�����   steelball_image��ͼƬ�еĽ�����
//void DentalRobot::on_pushButton_imageRegisNew_clicked()
//{	
//	if(m_steelBalls_cmm == nullptr || m_probeDitchPset_cmm == nullptr || GetDataStorage()->GetNamedNode("steelball_image") == nullptr)
//	{
//		m_Controls.textBrowser->append("Image steelBall extraction should be conducted first!");
//		return;
//	}
//	
//	if(m_steelBalls_cmm->GetSize() != dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("steelball_image")->GetData())->GetSize())
//	{
//		m_Controls.textBrowser->append("Image steelBall extraction is not complete!");
//		return;
//	}
//
//	// Step 1: Calculate the transform from steelballs_cmm to steelballs_image
//	auto landmarkRegistrator = mitk::SurfaceRegistration::New();
//	landmarkRegistrator->SetLandmarksSrc(m_steelBalls_cmm);
//	landmarkRegistrator->SetLandmarksTarget(dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("steelball_image")->GetData()));
//
//	landmarkRegistrator->ComputeLandMarkResult();
//
//	auto tmpMatrix = landmarkRegistrator->GetResult();
//
//	// Step 2: Apply tmpMatrix to 'probePoints_cmm' to get 'probePoints_image'
//	auto probePoints_cmm = mitk::PointSet::New();
//	auto probePoints_image = mitk::PointSet::New();
//
//	for (int i{ 0 }; i < m_probeDitchPset_cmm->GetSize(); i++)
//	{
//		probePoints_cmm->InsertPoint(m_probeDitchPset_cmm->GetPoint(i));//��U�͵��������㼯���Ƶ�probePoints_cmm��
//	}
//	probePoints_cmm->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(tmpMatrix);//���任����Ӧ�õ�probePoints_cmm��
//	probePoints_cmm->GetGeometry()->Modified();
//
//	for (int i{ 0 }; i < probePoints_cmm->GetSize(); i++)
//	{
//		probePoints_image->InsertPoint(probePoints_cmm->GetPoint(i));
//	}
//
//
//	// Step 3: Check if enough probe ditch points have been collected
//	if (m_probeDitchPset_rf == nullptr)
//	{
//		m_Controls.textBrowser->append("No probe ditch point has been captured");
//		return;
//	}
//
//	if (m_probeDitchPset_rf->GetSize() < 5)
//	{
//		m_Controls.textBrowser->append("At least 5 probe ditch points should be captured");
//		return;
//	}
//
//	// Step 4: Calculate m_T_patientRFtoImage
//	//���ռ����İ��۵��������
//	int collectedPointNum = m_probeDitchPset_rf->GetSize();
//	int totalPointNum = probePoints_image->GetSize();
//
//	auto sorted_probeDitchPset_rf = mitk::PointSet::New();
//	SortPointSetByDistance(m_probeDitchPset_rf, sorted_probeDitchPset_rf);
//
//	// Generate the tmp landmark_src
//	auto tmp_landmark_src = mitk::PointSet::New();
//	auto sorted_landmark_src = mitk::PointSet::New();
//	double maxError{ 1000 };
//	double avgError{ 1000 };
//
//	std::vector<std::vector<int>> combinations = GenerateAllCombinations(totalPointNum-1, collectedPointNum);
//
//	for (const auto& combination : combinations) {
//
//		tmp_landmark_src->Clear();
//
//		for (int value : combination) {
//			tmp_landmark_src->InsertPoint(probePoints_image->GetPoint(value));
//		}
//
//		SortPointSetByDistance(tmp_landmark_src, sorted_landmark_src);//��tmp_landmark_src����Ϊsorted_landmark_src
//
//		auto tmpLandmarkRegistrator = mitk::SurfaceRegistration::New();
//		tmpLandmarkRegistrator->SetLandmarksSrc(sorted_landmark_src);
//		tmpLandmarkRegistrator->SetLandmarksTarget(sorted_probeDitchPset_rf);
//
//		m_Controls.textBrowser->append("sorted_landmark_src Pnum: " + QString::number(sorted_landmark_src->GetSize()));
//
//		m_Controls.textBrowser->append("sorted_probeDitchPset_rf Pnum: " + QString::number(sorted_probeDitchPset_rf->GetSize()));
//
//		tmpLandmarkRegistrator->ComputeLandMarkResult();
//		double tmpMaxError = tmpLandmarkRegistrator->GetmaxLandmarkError();
//		double tmpAvgError = tmpLandmarkRegistrator->GetavgLandmarkError();
//
//		if(tmpMaxError < maxError && tmpAvgError < avgError)
//		{
//			maxError = tmpMaxError;
//			avgError = tmpAvgError;
//
//			m_Controls.lineEdit_maxError->setText(QString::number(tmpMaxError));
//			m_Controls.lineEdit_avgError->setText(QString::number(tmpAvgError));
//
//			memcpy_s(m_T_patientRFtoImage, sizeof(double) * 16, tmpLandmarkRegistrator->GetResult()->GetData(), sizeof(double) * 16);
//		}
//
//		if (maxError < 0.5)
//		{
//			break;
//		}
//
//	}
//
//	if(maxError < 1.5 && avgError < 1.5)
//	{
//		m_Controls.textBrowser->append("Image registration succeeded");
//		m_Stat_patientRFtoImage = true;
//
//	}else
//	{
//		m_Controls.textBrowser->append("Image registration failed, please collect more points or reset!");
//		m_Stat_patientRFtoImage = false;
//
//		// Clear m_T_patientRFtoImage
//		auto identityMatrix = vtkMatrix4x4::New();
//		identityMatrix->Identity();
//		memcpy_s(m_T_patientRFtoImage, sizeof(double) * 16, identityMatrix->GetData(), sizeof(double) * 16);
//		
//	}

	// old pipeline style
	// // The surface node should have no offset, i.e., should have an identity matrix!
	// auto surfaceNode = GetDataStorage()->GetNamedNode("Reconstructed CBCT surface");
	//
	// if (surfaceNode == nullptr)
	// {
	// 	m_Controls.textBrowser->append("Reconstructed CBCT surface is missing!");
	// 	return;
	// }
	//
	// m_NavigatedImage = lancet::NavigationObject::New();
	//
	// auto matrix = dynamic_cast<mitk::Surface*>(surfaceNode->GetData())->GetGeometry()->GetVtkMatrix();
	//
	// if (matrix->IsIdentity() == false)
	// {
	// 	vtkNew<vtkMatrix4x4> identityMatrix;
	// 	identityMatrix->Identity();
	// 	dynamic_cast<mitk::Surface*>(surfaceNode->GetData())->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(identityMatrix);
	//
	// 	m_Controls.textBrowser->append("Warning: the initial surface has a non-identity offset matrix; the matrix has been reset to identity!");
	// }
	//
	// m_NavigatedImage->SetDataNode(surfaceNode);
	//
	// m_NavigatedImage->SetLandmarks(probePoints_image);
	//
	// m_NavigatedImage->SetReferencFrameName(surfaceNode->GetName());
	//
	// m_Controls.textBrowser->append("--- NavigatedImage has been set up ---");
	//
	// m_NavigatedImage->SetLandmarks_probe(m_probeDitchPset_rf);
	//
	// /// Apply image registration
	// m_SurfaceRegistrationStaticImageFilter = lancet::ApplySurfaceRegistratioinStaticImageFilter::New();
	// m_SurfaceRegistrationStaticImageFilter->ConnectTo(m_VegaSource);
	//
	// m_NavigatedImage->UpdateObjectToRfMatrix();
	// m_Controls.textBrowser->append("Avg landmark error:" + QString::number(m_NavigatedImage->GetlandmarkRegis_avgError()));
	// m_Controls.textBrowser->append("Max landmark error:" + QString::number(m_NavigatedImage->GetlandmarkRegis_maxError()));
	//
	// m_ImageRegistrationMatrix->DeepCopy(m_NavigatedImage->GetT_Object2ReferenceFrame());
	//
	// auto tmpMatrix_1 = mitk::AffineTransform3D::New();
	//
	// mitk::TransferVtkMatrixToItkTransform(m_ImageRegistrationMatrix, tmpMatrix_1.GetPointer());
	//
	// m_VegaToolStorage->GetToolByName("patientRF")->SetToolRegistrationMatrix(tmpMatrix_1);
	//
	// m_SurfaceRegistrationStaticImageFilter->SetRegistrationMatrix(m_VegaToolStorage->GetToolByName("patientRF")->GetToolRegistrationMatrix());
	//
	// m_SurfaceRegistrationStaticImageFilter->SetNavigationDataOfRF(m_VegaSource->GetOutput("patientRF"));
	//
	// m_VegaVisualizeTimer->stop();
	// m_VegaVisualizer->ConnectTo(m_SurfaceRegistrationStaticImageFilter);
	// m_VegaVisualizeTimer->start();


//}

void DentalRobot::GenerateCombinations(int m, int n, int index, std::vector<int>& currentCombination, std::vector<std::vector<int>>& result)
{
	if (currentCombination.size() == n) {
		result.push_back(currentCombination);
		return;
	}
	for (int i = index; i <= m; ++i) {
		currentCombination.push_back(i);
		GenerateCombinations(m, n, i + 1, currentCombination, result);
		currentCombination.pop_back();
	}
}

std::vector<std::vector<int>> DentalRobot::GenerateAllCombinations(int m, int n)
{
	std::vector<std::vector<int>> result;
	std::vector<int> currentCombination;
	GenerateCombinations(m, n, 0, currentCombination, result);
	return result;
}



// Function to calculate the Euclidean distance between two points
double DentalRobot::CalculateDistance(const mitk::Point3D& point1, const mitk::Point3D& point2)
{
	return std::sqrt(std::pow(point1[0] - point2[0], 2) +
		std::pow(point1[1] - point2[1], 2) +
		std::pow(point1[2] - point2[2], 2));
}

// Function to calculate the mass center of a mitk::PointSet
mitk::Point3D DentalRobot::CalculateMassCenter(const mitk::PointSet::Pointer& pointSet)
{
	mitk::Point3D massCenter;

	// Calculate the sum of coordinates
	for (int i{ 0 }; i < pointSet->GetSize() ; i++)
	{
		massCenter[0] += pointSet->GetPoint(i)[0];
		massCenter[1] += pointSet->GetPoint(i)[1];
		massCenter[2] += pointSet->GetPoint(i)[2];
	}

	// Divide by the number of points to get the average (mass center)
	int numberOfPoints = pointSet->GetSize();
	massCenter[0] /= numberOfPoints;
	massCenter[1] /= numberOfPoints;
	massCenter[2] /= numberOfPoints;

	return massCenter;
}

// Comparator function for sorting points based on distance from mass center
bool DentalRobot::ComparePointsByDistance(const mitk::Point3D& massCenter, const mitk::Point3D& point1, const mitk::Point3D& point2)
{
	return CalculateDistance(massCenter, point1) < CalculateDistance(massCenter, point2);
}

void DentalRobot::SortPointSetByDistance(mitk::PointSet::Pointer inputPointSet, mitk::PointSet::Pointer outputPointSet)
{
	// Calculate the mass center
	mitk::Point3D massCenter = CalculateMassCenter(inputPointSet);

	// Get the points container
	// mitk::PointSet::PointsContainer::Pointer pointsContainer = inputPointSet->GetPoints();

	// Convert the points container to a vector for sorting
	std::vector<mitk::Point3D> pointsVector;
	for (int i{0}; i < inputPointSet->GetSize(); i++)
	{
		auto point = inputPointSet->GetPoint(i);
		pointsVector.push_back(point);
	}

	// Sort the points based on their distance from the mass center
	std::sort(pointsVector.begin(), pointsVector.end(),
		[&](const mitk::Point3D& point1, const mitk::Point3D& point2)
		{
			return ComparePointsByDistance(massCenter, point1, point2);
		});

	// Clear existing points in the point set
	outputPointSet->Clear();

	// Add sorted points back to the point set
	for (const auto& sortedPoint : pointsVector)
	{
		outputPointSet->InsertPoint(sortedPoint);
	}
}


void DentalRobot::on_pushButton_resetImageRegis_clicked()
{
	//m_Controls.label_15->setText("0");
	//m_probeDitchPset_rf = mitk::PointSet::New();

	//auto tmpMatrix = vtkMatrix4x4::New();
	//tmpMatrix->Identity();

	//memcpy_s(m_T_patientRFtoImage, sizeof(double) * 16, tmpMatrix->GetData(), sizeof(double) * 16);

	//m_Stat_patientRFtoImage = false;

	////disconnect(m_VegaVisualizeTimer, &QTimer::timeout, this, &DentalRobot::UpdateDrillVisual);
	//disconnect(m_AimoeVisualizeTimer, &QTimer::timeout, this, &DentalRobot::UpdateDrillVisual);

	//m_Controls.lineEdit_maxError->setText("NaN");
	//m_Controls.lineEdit_avgError->setText("NaN");

	//// m_ImageRegistrationMatrix->Identity();
}


void DentalRobot::on_pushButton_collectDitch_clicked()
{

	// m_probeDitchPset_cmm = GetDataStorage()->GetNamedObject<mitk::PointSet>("probePoints_cmm");

	//if(m_probeDitchPset_cmm == nullptr)
	//{
	//	m_Controls.textBrowser->append("SteelBall extraction should be conducted first!");
	//	return;
	//}

	//if (m_probeDitchPset_cmm->GetSize() == 0)
	//{
	//	m_Controls.textBrowser->append("SteelBall extraction should be conducted first!");
	//	return;
	//}

	//if (m_probeDitchPset_rf == nullptr)
	//{
	//	m_probeDitchPset_rf = mitk::PointSet::New();
	//}

	//if (m_probeDitchPset_rf->GetSize() == m_probeDitchPset_cmm->GetSize())
	//{
	//	m_Controls.textBrowser->append("Enough points have been captured");
	//	return;
	//}

	//m_Controls.label_16->setText("/" + QString::number(m_probeDitchPset_cmm->GetSize()));

	//if(m_Stat_handpieceRFtoDrill == false)
	//{
	//	m_Controls.textBrowser->append("Handpiece has not been calibrated !");
	//	return;
	//}

	////if(m_Stat_cameraToHandpieceRF == false || m_Stat_cameraToPatientRF == false)
	////{
	////	m_Controls.textBrowser->append("RF is not visible !");
	////	return;
	////}

	//auto T_handpieceRFtoDrill = vtkMatrix4x4::New();
	//T_handpieceRFtoDrill->DeepCopy(m_T_handpieceRFtoDrill);

	//auto T_patientRFtoCamera = vtkMatrix4x4::New();
	//T_patientRFtoCamera->DeepCopy(m_T_cameraToPatientRF);
	//T_patientRFtoCamera->Invert();

	//auto T_cameraToHandpieceRF = vtkMatrix4x4::New();
	//T_cameraToHandpieceRF->DeepCopy(m_T_cameraToHandpieceRF);

	//auto trans_patientRFtoDrill = vtkTransform::New();
	//trans_patientRFtoDrill->Identity();
	//trans_patientRFtoDrill->PostMultiply();
	//trans_patientRFtoDrill->SetMatrix(T_handpieceRFtoDrill);
	//trans_patientRFtoDrill->Concatenate(T_cameraToHandpieceRF);
	//trans_patientRFtoDrill->Concatenate(T_patientRFtoCamera);
	//trans_patientRFtoDrill->Update();

	//auto patientRFtoDrillMatrix = trans_patientRFtoDrill->GetMatrix();

	////�Ӿ�������ȡ����ͷ����ڻ��߲ο�����µ�λ��
	//mitk::Point3D tipPointUnderPatientRF;
	//tipPointUnderPatientRF[0] = patientRFtoDrillMatrix->GetElement(0, 3);
	//tipPointUnderPatientRF[1] = patientRFtoDrillMatrix->GetElement(1, 3);
	//tipPointUnderPatientRF[2] = patientRFtoDrillMatrix->GetElement(2, 3);

	//double minDistance{ 100 };
	//for(int i{0}; i < m_probeDitchPset_rf->GetSize() ; i++)
	//{
	//	mitk::Point3D testPoint = m_probeDitchPset_rf->GetPoint(i);
	//	double tmpDistance = sqrt(pow(testPoint[0]-tipPointUnderPatientRF[0],2)+
	//		pow(testPoint[1] - tipPointUnderPatientRF[1], 2)+
	//		pow(testPoint[2] - tipPointUnderPatientRF[2], 2));

	//	if(tmpDistance < minDistance)
	//	{
	//		minDistance = tmpDistance;
	//	}
	//}

	//if(minDistance > 2.5)
	//{
	//	m_probeDitchPset_rf->InsertPoint(tipPointUnderPatientRF);

	//	m_Controls.label_15->setText(QString::number(m_probeDitchPset_rf->GetSize()));
	//}else
	//{
	//	m_Controls.textBrowser->append("Don't capture the same point");
	//}
	//

	// old pipeline style: Calculate T_drillDRFtoCalibratorDRF
	// auto patientRFindex = m_VegaToolStorage->GetToolIndexByName("patientRF");
	// auto drillDRFindex = m_VegaToolStorage->GetToolIndexByName("drillRF");
	// if (patientRFindex == -1 || drillDRFindex == -1)
	// {
	// 	m_Controls.textBrowser->append("There is no 'patientRF' or 'drillDRF' in the toolStorage!");
	// 	return;
	// }
	//
	// mitk::NavigationData::Pointer nd_ndiToPatientRF = m_VegaSource->GetOutput(patientRFindex);
	// mitk::NavigationData::Pointer nd_ndiTodrillDRF = m_VegaSource->GetOutput(drillDRFindex);
	//
	// if (nd_ndiToPatientRF->IsDataValid() == 0 || nd_ndiTodrillDRF->IsDataValid() == 0)
	// {
	// 	m_Controls.textBrowser->append("'patientRF' or 'drillDRF' is missing");
	// 	return;
	// }
	//
	// mitk::NavigationData::Pointer nd_patientRFtoDrillRF = GetNavigationDataInRef(nd_ndiTodrillDRF, nd_ndiToPatientRF);
	//
	// vtkMatrix4x4* T_patientRFtoDrillDRF = vtkMatrix4x4::New();
	// mitk::TransferItkTransformToVtkMatrix(nd_patientRFtoDrillRF->GetAffineTransform3D().GetPointer(), T_patientRFtoDrillDRF);
	//
	// auto drillRFtoTipMatrix = vtkMatrix4x4::New();
	// mitk::TransferItkTransformToVtkMatrix(m_VegaToolStorage->GetToolByName("drillRF")->GetToolRegistrationMatrix().GetPointer(), drillRFtoTipMatrix);
	//
	// if (drillRFtoTipMatrix->IsIdentity())
	// {
	// 	m_Controls.textBrowser->append("The drill has not been calibrated yet.");
	// 	return;
	// }
	//
	// auto tmpTrans = vtkTransform::New();
	// tmpTrans->Identity();
	// tmpTrans->PostMultiply();
	// tmpTrans->SetMatrix(drillRFtoTipMatrix);
	// tmpTrans->Concatenate(T_patientRFtoDrillDRF);
	// tmpTrans->Update();
	//
	// auto patientRFtoDrillTipMatrix = tmpTrans->GetMatrix();
	//
	// mitk::Point3D tipPointUnderPatientRF;
	// tipPointUnderPatientRF[0] = patientRFtoDrillTipMatrix->GetElement(0, 3);
	// tipPointUnderPatientRF[1] = patientRFtoDrillTipMatrix->GetElement(1, 3);
	// tipPointUnderPatientRF[2] = patientRFtoDrillTipMatrix->GetElement(2, 3);
	//
	// m_probeDitchPset_rf->InsertPoint(tipPointUnderPatientRF);
	//
	// m_Controls.textBrowser->append("Captured point " + QString::number(m_probeDitchPset_rf->GetSize()));
	//
	// m_Controls.label_15->setText(QString::number(m_probeDitchPset_rf->GetSize()));

}

void DentalRobot::on_pushButton_connectVega_clicked()  // TODO change the data source from NDI to Aimooe
{
	//T_AIMPOS_DATAPARA mPosDataPara;
	//Aim_API_Initial(aimHandle);
	//Aim_SetEthernetConnectIP(aimHandle, 192, 168, 31, 10);
	//rlt = Aim_ConnectDevice(aimHandle, I_ETHERNET, mPosDataPara);

	//if (rlt == AIMOOE_OK)
	//{
	//	qDebug() << "connect success";
	//}
	//else 
	//{
	//	qDebug() << "connect failed";
	//}
	//QString defaultPath = "C:\\Users\\haiti\\Desktop\\mark"; 
	//QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", defaultPath);
	//if (filename.isNull()) return;
	//filename.append("/");
	//qDebug() << "The selected folder address :" << filename;
	//rlt = Aim_SetToolInfoFilePath(aimHandle, filename.toLatin1().data());
	//if (rlt == AIMOOE_OK)
	//{
	//	qDebug() << "set filenemae success";
	//}
	//else 
	//{
	//	qDebug() << "set filenemae failed";
	//}
	//int size = 0;
	//Aim_GetCountOfToolInfo(aimHandle, size);
	//if (size != 0)
	//{
	//	t_ToolBaseInfo* toolarr = new t_ToolBaseInfo[size];
	//	rlt = Aim_GetAllToolFilesBaseInfo(aimHandle, toolarr);
	//	if (rlt == AIMOOE_OK)
	//	{
	//		for (int i = 0; i < size; i++)
	//		{
	//			char* ptool = toolarr[i].name;
	//			QString toolInfo = QString(ptool);
	//			m_Controls.textBrowser_2->append(toolInfo);
	//			/*		char* ptool = toolarr[i].name;
	//					QString toolInfo = QString("Tool Name��") + QString::fromLocal8Bit(ptool);
	//					m_Controls.textBrowser->append(toolInfo);*/
	//		}
	//	}
	//	delete[] toolarr;
	//}
	//else {
	//	std::cout << "There are no tool identification files in the current directory:";
	//}
	//std::cout << "End of connection";
	//rlt = AIMOOE_OK;
} 

void DentalRobot::on_pushButton_disaplayProbe_clicked()
{
	//if (m_AimoeVisualizeTimer == nullptr)
	//{
	//	m_AimoeVisualizeTimer = new QTimer(this);
	//}
	///*connect(m_AimoeVisualizeTimer, SIGNAL(timeout()), this, SLOT(getMatrix()));*/
	//connect(m_AimoeVisualizeTimer, SIGNAL(timeout()), this, SLOT(getMatrix_1()));


	//m_AimoeVisualizeTimer->start(100);
}



void DentalRobot::OnVegaVisualizeTimer() // TODO change the data source from NDI to Aimooe
{
	//Here we call the Update() method from the Visualization Filter. Internally the filter checks if
	//new NavigationData is available. If we have a new NavigationData the cone position and orientation
	//will be adapted.

	//if (m_VegaVisualizer.IsNotNull())
	//{
	//	m_VegaVisualizer->Update();
	//	// auto geo = this->GetDataStorage()->ComputeBoundingGeometry3D(this->GetDataStorage()->GetAll());
	//	// mitk::RenderingManager::GetInstance()->InitializeViews(geo);
	//	this->RequestRenderWindowUpdate();
	//}

	//m_Controls.m_StatusWidgetVegaToolToShow->Refresh();

	// Update the global variables which hold the camera data
	/*auto calibratorRFindex = m_VegaToolStorage->GetToolIndexByName("calibratorRF");
	auto handpieceRFindex = m_VegaToolStorage->GetToolIndexByName("handpieceRF");
	auto patientRFindex = m_VegaToolStorage->GetToolIndexByName("patientRF");

	if (calibratorRFindex == -1 || handpieceRFindex == -1 || patientRFindex == -1)
	{
		return;
	}

	mitk::NavigationData::Pointer nd_cameraToCalibratorRF = m_VegaSource->GetOutput(calibratorRFindex);
	mitk::NavigationData::Pointer nd_cameraToHandpieceRF = m_VegaSource->GetOutput(handpieceRFindex);
	mitk::NavigationData::Pointer nd_cameraToPatientRF = m_VegaSource->GetOutput(patientRFindex);

	m_Stat_cameraToCalibratorRF = nd_cameraToCalibratorRF->IsDataValid();
	m_Stat_cameraToHandpieceRF = nd_cameraToHandpieceRF->IsDataValid();
	m_Stat_cameraToPatientRF = nd_cameraToPatientRF->IsDataValid();

	if (m_Stat_cameraToCalibratorRF)
	{
		auto tmpMatrix = vtkMatrix4x4::New();
		mitk::TransferItkTransformToVtkMatrix(nd_cameraToCalibratorRF->GetAffineTransform3D().GetPointer(), tmpMatrix);
		memcpy_s(m_T_cameraToCalibratorRF, sizeof(double) * 16, tmpMatrix->GetData(), sizeof(double) * 16);
	}

	if (m_Stat_cameraToHandpieceRF)
	{
		auto tmpMatrix = vtkMatrix4x4::New();
		mitk::TransferItkTransformToVtkMatrix(nd_cameraToHandpieceRF->GetAffineTransform3D().GetPointer(), tmpMatrix);
		memcpy_s(m_T_cameraToHandpieceRF, sizeof(double) * 16, tmpMatrix->GetData(), sizeof(double) * 16);
	}

	if (m_Stat_cameraToPatientRF)
	{
		auto tmpMatrix = vtkMatrix4x4::New();
		mitk::TransferItkTransformToVtkMatrix(nd_cameraToPatientRF->GetAffineTransform3D().GetPointer(), tmpMatrix);
		memcpy_s(m_T_cameraToPatientRF, sizeof(double) * 16, tmpMatrix->GetData(), sizeof(double) * 16);
	}*/

} 

void DentalRobot::CombineRotationTranslation(float Rto[3][3], float Tto[3], vtkMatrix4x4* resultMatrix)
{
	// Set rotation part
	/*for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			resultMatrix->SetElement(i, j, Rto[i][j]);
		}
	}
	for (int i = 0; i < 3; ++i)
	{
		resultMatrix->SetElement(i, 3, Tto[i]);
	}*/
}

void DentalRobot::on_pushButton_calibrateDrill_clicked()
{
	// This function is aimed to calculate m_T_handpieceRFtoDrill

	// Step 1: calculate T_calibratorRFtoDrill from the
	// PointSet probe_head_tail_mandible or probe_head_tail_maxilla in the dataStorage
//	if(GetDataStorage()->GetNamedNode("probe_head_tail_mandible") == nullptr ||
//		GetDataStorage()->GetNamedNode("probe_head_tail_maxilla") == nullptr)
//	{
//		m_Controls.textBrowser->append("probe_head_tail_mandible or probe_head_tail_maxilla is missing!");
//		return;
//	}
//
//	auto probe_head_tail_mandible = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("probe_head_tail_mandible")->GetData());
//	auto probe_head_tail_maxilla = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("probe_head_tail_maxilla")->GetData());
//
//	if(probe_head_tail_mandible->GetSize() != 2 || probe_head_tail_maxilla->GetSize() != 2)
//	{
//		m_Controls.textBrowser->append("probe_head_tail_mandible or probe_head_tail_maxilla is problematic!");
//		return;
//	}
//
//	auto probe_head_tail = mitk::PointSet::New();
//
//	if(m_Controls.radioButton_maxilla->isChecked())
//	{
//		probe_head_tail = probe_head_tail_maxilla;
//	}else
//	{
//		probe_head_tail = probe_head_tail_mandible;
//	}
//
//	auto probe_head = probe_head_tail->GetPoint(0);
//	auto probe_tail = probe_head_tail->GetPoint(1);
//
//	Eigen::Vector3d z_probeInCalibratorRF;
//	z_probeInCalibratorRF[0] = probe_tail[0] - probe_head[0];
//	z_probeInCalibratorRF[1] = probe_tail[1] - probe_head[1];
//	z_probeInCalibratorRF[2] = probe_tail[2] - probe_head[2];
//	z_probeInCalibratorRF.normalize();
//
//	Eigen::Vector3d z_std{0,0,1};//������һ����׼��������ʾz��ķ��򣬼���0��0��1��
//
//	Eigen::Vector3d rotAxis = z_std.cross(z_probeInCalibratorRF);//����z_std��z_probeInCalibratorRF���������Ĳ�����õ���ֱ����������������������ʾ��ת��
//
//	rotAxis.normalize();//��һ����ת�ᣬȷ����ת��ֻ��ʾ��������ܴ�СӰ��
//
//	if(rotAxis.norm() < 0.00001) // in case the rotAxis becomes a zero vector
//	{
//		rotAxis[0] = 1;
//		rotAxis[1] = 0;
//		rotAxis[2] = 0;
//	}
//
//	double rotAngle = 180 * acos(z_std.dot(z_probeInCalibratorRF)) / 3.141592654; //������������֮��ļн�
//
//	auto trans_calibratorRFtoDrill = vtkTransform::New();
//	trans_calibratorRFtoDrill->Identity();
//	trans_calibratorRFtoDrill->PostMultiply();
//	trans_calibratorRFtoDrill->RotateWXYZ(rotAngle, rotAxis[0], rotAxis[1], rotAxis[2]);
//	trans_calibratorRFtoDrill->Update();
//
//	auto T_calibratorRFtoDrill = trans_calibratorRFtoDrill->GetMatrix();
//	T_calibratorRFtoDrill->SetElement(0, 3, probe_head[0]);
//	T_calibratorRFtoDrill->SetElement(1, 3, probe_head[1]);
//	T_calibratorRFtoDrill->SetElement(2, 3, probe_head[2]);
//
//	// for (int i{ 0 }; i < 16; i++)
//	// {
//	// 	m_T_calibratorRFtoDrill[i] = T_calibratorRFtoDrill->GetData()[i];
//	// }
//
//	memcpy_s(m_T_calibratorRFtoDrill, sizeof(double) * 16, T_calibratorRFtoDrill->GetData(), sizeof(double) * 16);
//
//
//	m_Stat_calibratorRFtoDrill = true;
//
//	// Step 2: Obtain the camera data and assemble the matrix:
//	// T_handpieceRFtoDrill = (T_cameraTohandpieceRF)^-1 * T_cameraToCalibratorRF * T_calibratorRFtoDrill
//	//if (m_Stat_cameraToCalibratorRF == false || m_Stat_cameraToHandpieceRF == false)
//	//{
//	//	m_Controls.textBrowser->append("calibratorRF or handpieceRF is invisible");
//	//	return;
//	//}
//	   
//
//	if (m_Stat_calibratorRFtoDrill == false)
//	{
//		m_Controls.textBrowser->append("m_T_calibratorRFtoDrill from hardware design is not ready");
//		return;
//	}
//
//	auto T_handpieceRFtoCamera = vtkMatrix4x4::New();
//	auto T_cameraToCalibratorRF = vtkMatrix4x4::New();
//	
//
//	T_handpieceRFtoCamera->DeepCopy(m_T_cameraToHandpieceRF);
//	T_handpieceRFtoCamera->Invert();
//
//	T_cameraToCalibratorRF->DeepCopy(m_T_cameraToCalibratorRF);
//
//	auto trans_handpieceRFtoDrill = vtkTransform::New();
//	trans_handpieceRFtoDrill->Identity();
//	trans_handpieceRFtoDrill->PostMultiply();
//	trans_handpieceRFtoDrill->SetMatrix(T_calibratorRFtoDrill);
//	trans_handpieceRFtoDrill->Concatenate(T_cameraToCalibratorRF);
//	trans_handpieceRFtoDrill->Concatenate(T_handpieceRFtoCamera);
//	trans_handpieceRFtoDrill->Update();
//
//	// Todo: the matrix below should be averaged for a time span before being stored into m_T_handpieceRFtoDrill
//	auto T_handpieceRFtoDrill = trans_handpieceRFtoDrill->GetMatrix();
//
//	memcpy_s(m_T_handpieceRFtoDrill, sizeof(double) * 16, T_handpieceRFtoDrill->GetData(), sizeof(double) * 16);
//
//	m_Stat_handpieceRFtoDrill = true;
//
//	m_Controls.textBrowser->append("Handpiece calibration succeeded!");
//
//	m_Controls.textBrowser->append("Drill tip in handpieceRF:"  
//		+ QString::number(m_T_handpieceRFtoDrill[3])+" / "
//		+ QString::number(m_T_handpieceRFtoDrill[7]) + " / "
//		+ QString::number(m_T_handpieceRFtoDrill[11])+ " / "
//		+ QString::number(m_T_handpieceRFtoDrill[15]));
//	
//}
//
//void DentalRobot::UpdateDeviation()
//{
//	// planned entry point and apex point in image frame
//	auto plan_tip_pts = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("plan_tip_pts")->GetData());
//	mitk::Point3D apex_plan = plan_tip_pts->GetPoint(0);//����
//	mitk::Point3D entry_plan = plan_tip_pts->GetPoint(1);//�����
//
//	Eigen::Vector3d axis_plan;
//	axis_plan[0] = entry_plan[0] - apex_plan[0];
//	axis_plan[1] = entry_plan[1] - apex_plan[1];
//	axis_plan[2] = entry_plan[2] - apex_plan[2];
//	axis_plan.normalize();//��ʾ���򣬲���ʾ����
//
//	// T_imageToProbe
//	auto T_cameraToHandpieceRF = vtkMatrix4x4::New();
//	T_cameraToHandpieceRF->DeepCopy(m_T_cameraToHandpieceRF);
//
//	auto T_handpieceRFtoDrill = vtkMatrix4x4::New();
//	T_handpieceRFtoDrill->DeepCopy(m_T_handpieceRFtoDrill);
//
//	auto T_patientRFtoCamera = vtkMatrix4x4::New();
//	T_patientRFtoCamera->DeepCopy(m_T_cameraToPatientRF);
//	T_patientRFtoCamera->Invert();
//
//	auto T_imageToPatientRF = vtkMatrix4x4::New();
//	T_imageToPatientRF->DeepCopy(m_T_patientRFtoImage);
//	T_imageToPatientRF->Invert();
//
//	auto tmpTrans = vtkTransform::New();
//	tmpTrans->Identity();
//	tmpTrans->PostMultiply();
//	tmpTrans->SetMatrix(T_handpieceRFtoDrill);
//	tmpTrans->Concatenate(T_cameraToHandpieceRF);
//	tmpTrans->Concatenate(T_patientRFtoCamera);
//	tmpTrans->Concatenate(T_imageToPatientRF);
//	tmpTrans->Update();
//
//	auto T_imageToProbe = tmpTrans->GetMatrix(); // image to probe actually
//
//	Eigen::Vector3d axis_probe;//��ͷ��z���ϵ�λ��
//	axis_probe[0] = T_imageToProbe->GetElement(0, 2);
//	axis_probe[1] = T_imageToProbe->GetElement(1, 2);
//	axis_probe[2] = T_imageToProbe->GetElement(2, 2);
//
//	double angleDevi = 180 * acos(axis_plan.dot(axis_probe)) / 3.1415926; //����Ƕ�ƫ��
//
//	m_Controls.lineEdit_angleError->setText(QString::number(angleDevi));
//
//	//******** m_NaviMode = 0 , i.e, Drilling mode, only update the Drill deviation *************
//	if(m_NaviMode == 0)
//	{
//		m_Controls.lineEdit_entryTotalError->setText("NaN");
//		m_Controls.lineEdit_entryVertError->setText("NaN");
//		m_Controls.lineEdit_entryHoriError->setText("NaN");
//		m_Controls.lineEdit_apexTotalError->setText("NaN");
//		m_Controls.lineEdit_apexVertError->setText("NaN");
//		m_Controls.lineEdit_apexHoriError->setText("NaN");
//
//		// Tip to planned apex
//		mitk::Point3D drillTip;
//		drillTip[0] = m_T_imageToInputDrill[3];
//		drillTip[1] = m_T_imageToInputDrill[7];
//		drillTip[2] = m_T_imageToInputDrill[11];
//		double tipToPlannedApex = GetPointDistance(drillTip, apex_plan);
//		m_Controls.lineEdit_drillTipTotalError->setText(QString::number(tipToPlannedApex));
//
//		// Vertical deviation
//		Eigen::Vector3d plannedApexToDrillTip;
//		plannedApexToDrillTip[0] = drillTip[0] - apex_plan[0];
//		plannedApexToDrillTip[1] = drillTip[1] - apex_plan[1];
//		plannedApexToDrillTip[2] = drillTip[2] - apex_plan[2];
//
//		double verticalDevi = plannedApexToDrillTip.dot(axis_plan);
//		m_Controls.lineEdit_drillTipVertError->setText(QString::number(verticalDevi));
//
//		// Horizontal deviation
//		double horizontalDevi = plannedApexToDrillTip.cross(axis_plan).norm();
//		m_Controls.lineEdit_drillTipHoriError->setText(QString::number(horizontalDevi));
//
//	}
//
//	//******** m_NaviMode = 1 , i.e, Implantation mode, only update the implant deviation and angle deviation *************
//	if (m_NaviMode == 1)
//	{
//		m_Controls.lineEdit_drillTipTotalError->setText("NaN");
//		m_Controls.lineEdit_drillTipVertError->setText("NaN");
//		m_Controls.lineEdit_drillTipHoriError->setText("NaN");
//
//		//******* real implant entry ************
//		// Total distance to  planned entry
//		mitk::Point3D entry_real = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("implant_tip_pts")->GetData())->GetPoint(1);
//		double totalEntryDis = GetPointDistance(entry_real, entry_plan);
//		m_Controls.lineEdit_entryTotalError->setText(QString::number(totalEntryDis));
//
//		// Vertical deviation
//		Eigen::Vector3d entry_planToReal;
//		entry_planToReal[0] = entry_real[0] - entry_plan[0];
//		entry_planToReal[1] = entry_real[1] - entry_plan[1];
//		entry_planToReal[2] = entry_real[2] - entry_plan[2];
//
//		double verticalDevi_entry = entry_planToReal.dot(axis_plan);
//		m_Controls.lineEdit_entryVertError->setText(QString::number(verticalDevi_entry));
//
//		// Horizontal deviation
//		double horizontalDevi_entry = entry_planToReal.cross(axis_plan).norm();
//		m_Controls.lineEdit_entryHoriError->setText(QString::number(horizontalDevi_entry));
//
//
//		//******* real implant apex ************
//		// Total distance to  planned apex
//		mitk::Point3D apex_real = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("implant_tip_pts")->GetData())->GetPoint(0);
//		double totalApexDis = GetPointDistance(apex_real, apex_plan);
//		m_Controls.lineEdit_apexTotalError->setText(QString::number(totalApexDis));
//
//		// Vertical deviation
//		Eigen::Vector3d apex_planToReal;
//		apex_planToReal[0] = apex_real[0] - apex_plan[0];
//		apex_planToReal[1] = apex_real[1] - apex_plan[1];
//		apex_planToReal[2] = apex_real[2] - apex_plan[2];
//
//		double verticalDevi_apex = apex_planToReal.dot(axis_plan);
//		m_Controls.lineEdit_apexVertError->setText(QString::number(verticalDevi_apex));
//
//		// Horizontal deviation
//		double horizontalDevi_apex = apex_planToReal.cross(axis_plan).norm();
//		m_Controls.lineEdit_apexHoriError->setText(QString::number(horizontalDevi_apex));
//
//	}



}


void DentalRobot::on_pushButton_startNaviImplant_clicked()
{
	////disconnect(m_VegaVisualizeTimer, &QTimer::timeout, this, &DentalRobot::UpdateDrillVisual);
	//disconnect(m_AimoeVisualizeTimer, &QTimer::timeout, this, &DentalRobot::UpdateDrillVisual);
	////disconnect(m_VegaVisualizeTimer, &QTimer::timeout, this, &DentalRobot::UpdateImplantAndCarrierVisual);
	//disconnect(m_AimoeVisualizeTimer, &QTimer::timeout, this, &DentalRobot::UpdateImplantAndCarrierVisual);

	//if (m_Stat_handpieceRFtoDrill == false)
	//{
	//	m_Controls.textBrowser->append("Handpiece calibration is not ready!");
	//	return;
	//}

	//if (m_Stat_patientRFtoImage == false)
	//{
	//	m_Controls.textBrowser->append("Image registration is not ready!");
	//	return;
	//}


	//m_NaviMode = 1;

	////********** Modify the length of the carrier/implant surface and m_T_handpieceRFtoDrill *********** 

	//auto identityMatrix = vtkMatrix4x4::New();
	//identityMatrix->Identity();

	//auto probe_head_tail_mandible = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("probe_head_tail_mandible")->GetData());

	//double probeDrillLength = GetPointDistance(probe_head_tail_mandible->GetPoint(0), probe_head_tail_mandible->GetPoint(1));

	//// Carrier part
	//double inputCarrierLength = m_Controls.lineEdit_carrierLength->text().toDouble();

	//auto carrier_tip_pts_default = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("carrier_tip_pts")->GetData());

	//carrier_tip_pts_default->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(identityMatrix);

	//carrier_tip_pts_default->GetGeometry()->Modified();

	//double carrierDefaultLength = GetPointDistance(carrier_tip_pts_default->GetPoint(0), carrier_tip_pts_default->GetPoint(1));

	//double z_carrier_scale = inputCarrierLength / carrierDefaultLength;

	//auto T_handpieceRFtoDrill_init = vtkMatrix4x4::New();
	//T_handpieceRFtoDrill_init->DeepCopy(m_T_handpieceRFtoDrill);

	//auto T_probeDrilltoInputCarrier = vtkMatrix4x4::New();
	//T_probeDrilltoInputCarrier->Identity();
	//T_probeDrilltoInputCarrier->SetElement(2, 2, z_carrier_scale);
	//T_probeDrilltoInputCarrier->SetElement(2, 3, probeDrillLength - inputCarrierLength);

	//auto Trans_handpieceRFtoInputCarrier = vtkTransform::New();
	//Trans_handpieceRFtoInputCarrier->PostMultiply();
	//Trans_handpieceRFtoInputCarrier->SetMatrix(T_probeDrilltoInputCarrier);
	//Trans_handpieceRFtoInputCarrier->Concatenate(T_handpieceRFtoDrill_init);
	//Trans_handpieceRFtoInputCarrier->Update();

	//auto T_handpieceRFtoInputCarrier = Trans_handpieceRFtoInputCarrier->GetMatrix();

	//memcpy_s(m_T_handpieceRFtoInputCarrier, sizeof(double) * 16, T_handpieceRFtoInputCarrier->GetData(), sizeof(double) * 16);

	//// Implant part
	//double inputImplantLength = m_Controls.lineEdit_implantLength->text().toDouble();

	//auto implant_tip_pts_default = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("implant_tip_pts")->GetData());

	//implant_tip_pts_default->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(identityMatrix);

	//implant_tip_pts_default->GetGeometry()->Modified();

	//double implantDefaultLength = GetPointDistance(implant_tip_pts_default->GetPoint(0), implant_tip_pts_default->GetPoint(1));

	//double z_implant_scale = inputImplantLength / implantDefaultLength;

	//auto T_probeDrilltoInputImplant = vtkMatrix4x4::New();
	//T_probeDrilltoInputImplant->Identity();
	//T_probeDrilltoInputImplant->SetElement(2, 2, z_implant_scale);
	//T_probeDrilltoInputImplant->SetElement(2, 3, probeDrillLength - inputCarrierLength- inputImplantLength);

	//auto Trans_handpieceRFtoInputImplant = vtkTransform::New();
	//Trans_handpieceRFtoInputImplant->PostMultiply();
	//Trans_handpieceRFtoInputImplant->SetMatrix(T_probeDrilltoInputImplant);
	//Trans_handpieceRFtoInputImplant->Concatenate(T_handpieceRFtoDrill_init);
	//Trans_handpieceRFtoInputImplant->Update();

	//auto T_handpieceRFtoInputImplant = Trans_handpieceRFtoInputImplant->GetMatrix();

	//memcpy_s(m_T_handpieceRFtoInputImplant, sizeof(double) * 16, T_handpieceRFtoInputImplant->GetData(), sizeof(double) * 16);


	//TurnOffAllNodesVisibility();
	//GetDataStorage()->GetNamedNode("CBCT Bounding Shape_cropped")->SetVisibility(true);
	//GetDataStorage()->GetNamedNode("carrierSurface")->SetVisibility(true);
	//GetDataStorage()->GetNamedNode("implantSurface")->SetVisibility(true);

	//GetDataStorage()->GetNamedNode("stdmulti.widget0.plane")->SetVisibility(false);
	//GetDataStorage()->GetNamedNode("stdmulti.widget1.plane")->SetVisibility(false);
	//GetDataStorage()->GetNamedNode("stdmulti.widget2.plane")->SetVisibility(false);

	////connect(m_VegaVisualizeTimer, &QTimer::timeout, this, &DentalRobot::UpdateImplantAndCarrierVisual);
	//connect(m_AimoeVisualizeTimer, &QTimer::timeout, this, &DentalRobot::UpdateImplantAndCarrierVisual);
}

void DentalRobot::UpdateImplantAndCarrierVisual()
{
	//if (m_Stat_cameraToHandpieceRF == false || m_Stat_cameraToPatientRF == false)
	//{
	//	// m_Controls.textBrowser->append("RF is not visible!");
	//	return;
	//}

	//auto T_cameraToHandpieceRF = vtkMatrix4x4::New();
	//T_cameraToHandpieceRF->DeepCopy(m_T_cameraToHandpieceRF);

	//auto T_handpieceRFtoInputCarrier = vtkMatrix4x4::New();
	//T_handpieceRFtoInputCarrier->DeepCopy(m_T_handpieceRFtoInputCarrier);

	//auto T_handpieceRFtoInputImplant = vtkMatrix4x4::New();
	//T_handpieceRFtoInputImplant->DeepCopy(m_T_handpieceRFtoInputImplant);

	//auto T_patientRFtoCamera = vtkMatrix4x4::New();
	//T_patientRFtoCamera->DeepCopy(m_T_cameraToPatientRF);
	//T_patientRFtoCamera->Invert();

	//auto T_imageToPatientRF = vtkMatrix4x4::New();
	//T_imageToPatientRF->DeepCopy(m_T_patientRFtoImage);
	//T_imageToPatientRF->Invert();

	//auto tmpTrans_carrier = vtkTransform::New();
	//tmpTrans_carrier->Identity();
	//tmpTrans_carrier->PostMultiply();
	//tmpTrans_carrier->SetMatrix(T_handpieceRFtoInputCarrier);
	//tmpTrans_carrier->Concatenate(T_cameraToHandpieceRF);
	//tmpTrans_carrier->Concatenate(T_patientRFtoCamera);
	//tmpTrans_carrier->Concatenate(T_imageToPatientRF);
	//tmpTrans_carrier->Update();

	//auto T_imageToInputCarrier = tmpTrans_carrier->GetMatrix();

	//GetDataStorage()->GetNamedNode("carrierSurface")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(T_imageToInputCarrier);
	//GetDataStorage()->GetNamedNode("carrierSurface")->GetData()->GetGeometry()->Modified();

	//GetDataStorage()->GetNamedNode("carrier_tip_pts")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(T_imageToInputCarrier);
	//GetDataStorage()->GetNamedNode("carrier_tip_pts")->GetData()->GetGeometry()->Modified();

	//auto tmpTrans_implant = vtkTransform::New();
	//tmpTrans_implant->Identity();
	//tmpTrans_implant->PostMultiply();
	//tmpTrans_implant->SetMatrix(T_handpieceRFtoInputImplant);
	//tmpTrans_implant->Concatenate(T_cameraToHandpieceRF);
	//tmpTrans_implant->Concatenate(T_patientRFtoCamera);
	//tmpTrans_implant->Concatenate(T_imageToPatientRF);
	//tmpTrans_implant->Update();

	//auto T_imageToInputImplant = tmpTrans_implant->GetMatrix();

	//GetDataStorage()->GetNamedNode("implantSurface")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(T_imageToInputImplant);
	//GetDataStorage()->GetNamedNode("implantSurface")->GetData()->GetGeometry()->Modified();

	//GetDataStorage()->GetNamedNode("implant_tip_pts")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(T_imageToInputImplant);
	//GetDataStorage()->GetNamedNode("implant_tip_pts")->GetData()->GetGeometry()->Modified();


	//UpdateDeviation();
}


void DentalRobot::on_pushButton_startNavi_clicked()
{
	////Դ�����Ǵӻ�е�������ݵ�
	///*disconnect(m_VegaVisualizeTimer, &QTimer::timeout, this, &DentalRobot::UpdateDrillVisual);
	//disconnect(m_VegaVisualizeTimer, &QTimer::timeout, this, &DentalRobot::UpdateImplantAndCarrierVisual);*/
	//disconnect(m_AimoeVisualizeTimer, &QTimer::timeout, this, &DentalRobot::UpdateDrillVisual);
	//disconnect(m_AimoeVisualizeTimer, &QTimer::timeout, this, &DentalRobot::UpdateImplantAndCarrierVisual);

	//if (m_Stat_handpieceRFtoDrill == false)
	//{
	//	m_Controls.textBrowser->append("Handpiece calibration is not ready!");
	//	return;
	//}

	//if (m_Stat_patientRFtoImage == false)
	//{
	//	m_Controls.textBrowser->append("Image registration is not ready!");
	//	return;
	//}


	//m_NaviMode = 0;

	//// Modify the length of the drill/probe surface and m_T_handpieceRFtoInputDrill based on the input length and m_T_handpieceRFtoDrill
	////�������볤�Ⱥ� m_T_handpieceRFtoInputDrill �޸���ͷ/̽�����ĳ��Ⱥ� m_T_handpieceRFtoInputDrill
	//double inputDrillLength = m_Controls.lineEdit_drillLength->text().toDouble();//��ͷ���ȣ�ʵ�ʣ�

	//auto probe_head_tail_mandible = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("probe_head_tail_mandible")->GetData());

	//double probeDrillLength = GetPointDistance(probe_head_tail_mandible->GetPoint(0), probe_head_tail_mandible->GetPoint(1));//̽�볤�ȣ��趨��

	//double z_scale = inputDrillLength / probeDrillLength;//������������

	//auto T_handpieceRFtoDrill_init = vtkMatrix4x4::New();
	//T_handpieceRFtoDrill_init->DeepCopy(m_T_handpieceRFtoDrill);

	//auto T_probeDrilltoInputDrill = vtkMatrix4x4::New();
	//T_probeDrilltoInputDrill->Identity();//��ʼ��Ϊ��λ����
	//T_probeDrilltoInputDrill->SetElement(2, 2, z_scale);//���õ����е�����z���ֵΪ��������
	//T_probeDrilltoInputDrill->SetElement(2, 3, probeDrillLength - inputDrillLength);//���õ����е�����z���ƽ�Ʋ���Ϊ��probeDrillLength - inputDrillLength

	////QString matrixString = MatrixToString(T_probeDrilltoInputDrill);
	////m_Controls.textBrowser->append("T_probeDrilltoInputDrill:\n"+matrixString);

	//auto tmpTrans = vtkTransform::New();
	//tmpTrans->PostMultiply();
	//tmpTrans->SetMatrix(T_probeDrilltoInputDrill);
	//tmpTrans->Concatenate(T_handpieceRFtoDrill_init);
	//tmpTrans->Update();

	//auto T_handpieceRFtoDrill_new = tmpTrans->GetMatrix();

	//memcpy_s(m_T_handpieceRFtoInputDrill, sizeof(double) * 16, T_handpieceRFtoDrill_new->GetData(), sizeof(double) * 16);

	//TurnOffAllNodesVisibility();
	//GetDataStorage()->GetNamedNode("CBCT Bounding Shape_cropped")->SetVisibility(true);
	//GetDataStorage()->GetNamedNode("drillSurface")->SetVisibility(true);

	//GetDataStorage()->GetNamedNode("stdmulti.widget0.plane")->SetVisibility(false);
	//GetDataStorage()->GetNamedNode("stdmulti.widget1.plane")->SetVisibility(false);
	//GetDataStorage()->GetNamedNode("stdmulti.widget2.plane")->SetVisibility(false);


	//connect(m_AimoeVisualizeTimer, &QTimer::timeout, this, &DentalRobot::UpdateDrillVisual);

}


//void DentalRobot::UpdateDrillVisual()
//{
	//if(GetDataStorage()->GetNamedNode("drillSurface") == nullptr)
	//{
	//	m_Controls.textBrowser->append("drillSurface is missing");
	//	return;
	//}

	////if(m_Stat_cameraToHandpieceRF == false || m_Stat_cameraToPatientRF == false)
	////{
	////	 m_Controls.textBrowser->append("RF is not visible!");
	////	return;
	////}


	//auto T_handpieceRFtoDrill = vtkMatrix4x4::New();
	//T_handpieceRFtoDrill->DeepCopy(m_T_handpieceRFtoInputDrill);

	//auto T_cameraToHandpieceRF = vtkMatrix4x4::New();
	//T_cameraToHandpieceRF->DeepCopy(m_T_cameraToHandpieceRF);

	//auto T_patientRFtoCamera = vtkMatrix4x4::New();
	//T_patientRFtoCamera->DeepCopy(m_T_cameraToPatientRF);
	//T_patientRFtoCamera->Invert();

	//auto T_imageToPatientRF = vtkMatrix4x4::New();
	//T_imageToPatientRF->DeepCopy(m_T_patientRFtoImage);
	//T_imageToPatientRF->Invert();

	//auto tmpTrans = vtkTransform::New();
	//tmpTrans->Identity();
	//tmpTrans->PostMultiply();
	//tmpTrans->SetMatrix(T_handpieceRFtoDrill);
	//tmpTrans->Concatenate(T_cameraToHandpieceRF);
	//tmpTrans->Concatenate(T_patientRFtoCamera);
	//tmpTrans->Concatenate(T_imageToPatientRF);
	//tmpTrans->Update();

	//auto T_imageToDrill = tmpTrans->GetMatrix();

	//memcpy_s(m_T_imageToInputDrill, sizeof(double) * 16, T_imageToDrill->GetData(), sizeof(double) * 16);
	//
	//GetDataStorage()->GetNamedNode("drillSurface")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(T_imageToDrill);
	//GetDataStorage()->GetNamedNode("drillSurface")->GetData()->GetGeometry()->Modified();
	//mitk::RenderingManager::GetInstance()->RequestUpdateAll();

	//UpdateDeviation();
//}



void DentalRobot::on_pushButton_imageRegis_clicked()
{
	//auto extractedBall_node = GetDataStorage()->GetNamedNode("steelball_image");

	//auto stdBall_node = GetDataStorage()->GetNamedNode("steelball_rf");

	//if (extractedBall_node == nullptr)
	//{
	//	m_Controls.textBrowser->append("steelball_image is missing");
	//	return;
	//}

	//if (stdBall_node == nullptr)
	//{
	//	m_Controls.textBrowser->append("steelball_rf is missing");
	//	return;
	//}

	//auto landmarkRegistrator = mitk::SurfaceRegistration::New();

	//auto sourcePointset = dynamic_cast<mitk::PointSet*>(extractedBall_node->GetData());
	//auto targetPointset = dynamic_cast<mitk::PointSet*>(stdBall_node->GetData());
	//landmarkRegistrator->SetLandmarksSrc(sourcePointset);
	//landmarkRegistrator->SetLandmarksTarget(targetPointset);
	//landmarkRegistrator->ComputeLandMarkResult();

	//auto T_patientRFtoImage = landmarkRegistrator->GetResult();

	//memcpy_s(m_T_patientRFtoImage, sizeof(double) * 16, T_patientRFtoImage->GetData(), sizeof(double) * 16);

	//m_Stat_patientRFtoImage = true;

	//m_Controls.textBrowser->append("Image registration succeeded!");

	//// Realization with pipeline
	//// if (m_ImageRegistrationMatrix->IsIdentity() == false)
	//// {
	//// 	m_Controls.textBrowser->append("Image registration has been done.");
	//// 	return;
	//// }
	////
	//// auto extractedBall_node = GetDataStorage()->GetNamedNode("steelball_image");
	////
	//// auto stdBall_node = GetDataStorage()->GetNamedNode("steelball_rf");
	////
	//// if (extractedBall_node == nullptr)
	//// {
	//// 	m_Controls.textBrowser->append("steelball_image is missing");
	//// 	return;
	//// }
	////
	//// if (stdBall_node == nullptr)
	//// {
	//// 	m_Controls.textBrowser->append("steelball_rf is missing");
	//// 	return;
	//// }
	////
	//// auto extractedBall_pset = GetDataStorage()->GetNamedObject<mitk::PointSet>("steelball_image");
	//// auto stdball_pset = GetDataStorage()->GetNamedObject<mitk::PointSet>("steelball_rf");
	//// int extracted_num = extractedBall_pset->GetSize();
	////
	//// if (extracted_num < stdball_pset->GetSize())
	//// {
	//// 	m_Controls.textBrowser->append("steelball_image extraction incomplete");
	//// 	return;
	//// }
	////
	////
	//// // The surface node should have no offset, i.e., should have an identity matrix!
	//// auto surfaceNode = GetDataStorage()->GetNamedNode("Reconstructed CBCT surface");
	////
	//// if (surfaceNode == nullptr)
	//// {
	//// 	m_Controls.textBrowser->append("Reconstructed CBCT surface is missing!");
	//// 	return;
	//// }
	////
	//// m_NavigatedImage = lancet::NavigationObject::New();
	////
	//// auto matrix = dynamic_cast<mitk::Surface*>(surfaceNode->GetData())->GetGeometry()->GetVtkMatrix();
	////
	//// if (matrix->IsIdentity() == false)
	//// {
	//// 	vtkNew<vtkMatrix4x4> identityMatrix;
	//// 	identityMatrix->Identity();
	//// 	dynamic_cast<mitk::Surface*>(surfaceNode->GetData())->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(identityMatrix);
	////
	//// 	m_Controls.textBrowser->append("Warning: the initial surface has a non-identity offset matrix; the matrix has been reset to identity!");
	//// }
	////
	//// m_NavigatedImage->SetDataNode(surfaceNode);
	////
	//// m_NavigatedImage->SetLandmarks(extractedBall_pset);
	////
	//// m_NavigatedImage->SetReferencFrameName(surfaceNode->GetName());
	////
	//// m_Controls.textBrowser->append("--- NavigatedImage has been set up ---");
	////
	//// m_NavigatedImage->SetLandmarks_probe(stdball_pset);
	////
	//// /// Apply image registration
	//// m_SurfaceRegistrationStaticImageFilter = lancet::ApplySurfaceRegistratioinStaticImageFilter::New();
	//// m_SurfaceRegistrationStaticImageFilter->ConnectTo(m_VegaSource);
	////
	//// m_NavigatedImage->UpdateObjectToRfMatrix();
	//// m_Controls.textBrowser->append("Avg landmark error:" + QString::number(m_NavigatedImage->GetlandmarkRegis_avgError()));
	//// m_Controls.textBrowser->append("Max landmark error:" + QString::number(m_NavigatedImage->GetlandmarkRegis_maxError()));
	////
	//// m_ImageRegistrationMatrix->DeepCopy(m_NavigatedImage->GetT_Object2ReferenceFrame());
	////
	//// auto tmpMatrix = mitk::AffineTransform3D::New();
	////
	//// mitk::TransferVtkMatrixToItkTransform(m_ImageRegistrationMatrix, tmpMatrix.GetPointer());
	////
	//// m_VegaToolStorage->GetToolByName("patientRF")->SetToolRegistrationMatrix(tmpMatrix);
	////
	//// m_SurfaceRegistrationStaticImageFilter->SetRegistrationMatrix(m_VegaToolStorage->GetToolByName("patientRF")->GetToolRegistrationMatrix());
	////
	//// m_SurfaceRegistrationStaticImageFilter->SetNavigationDataOfRF(m_VegaSource->GetOutput("patientRF"));
	////
	//// m_VegaVisualizeTimer->stop();
	//// m_VegaVisualizer->ConnectTo(m_SurfaceRegistrationStaticImageFilter);
	//// m_VegaVisualizeTimer->start();

}

void DentalRobot::on_pushButton_updateData_clicked()
{

	//if (m_AimoeVisualizeTimer == nullptr)
	//{
	//	m_AimoeVisualizeTimer = new QTimer(this); 
	//}
	//connect(m_AimoeVisualizeTimer, SIGNAL(timeout()), this, SLOT(get_ToolInfor()));
	////connect(m_AimoeVisualizeTimer, SIGNAL(timeout()), this, SLOT(CollectProbeData()));
	//m_AimoeVisualizeTimer->start(50);

	////update visualize filter by timer
	////if (m_VegaVisualizeTimer == nullptr)
	////{
	////	m_VegaVisualizeTimer = new QTimer(this); //create a new timer
	////}
	////connect(m_VegaVisualizeTimer, &QTimer::timeout, this, &DentalRobot::OnVegaVisualizeTimer);

	////ShowToolStatus_Vega();

	////m_VegaVisualizeTimer->start(200); //Every 200ms the method OnTimer()
}

void DentalRobot::get_ToolInfor() // myMarkers &marker
{
	////E_ReturnValue rlt = Aim_SetToolInfoFilePath(aimHandle, "C:\\Users\\haiti\\Desktop\\mark\\");
	//QString position_text;
	//std::vector<std::string> toolidarr;
	//toolidarr.push_back("DI_PatientRF");
	//toolidarr.push_back("DI_Calibrator");
	////toolidarr.push_back("DI_Probe");
	//toolidarr.push_back("DI_HandpieceRF");

	//auto prlt = GetNewToolData();

	//if (prlt->validflag)//�ж��Ƿ�ɼ��ɹ�
	//{
	//	do
	//	{
	//		//��ȡ����
	//		UpdateCameraToToolMatrix(prlt, "DI_PatientRF", m_T_cameraToPatientRF, m_Controls.Dental_PatientRFDataLabel);
	//		UpdateCameraToToolMatrix(prlt, "DI_Calibrator", m_T_cameraToCalibratorRF, m_Controls.Dental_CalibratorRFDataLabel);
	//		//UpdateCameraToToolMatrix(prlt, "DI_Probe", m_T_cameraToProbe, m_Stat_cameraToProbe, m_Controls.Dental_ProbeRFDataLabel);
	//		UpdateCameraToToolMatrix(prlt, "DI_HandpieceRF", m_T_cameraToHandpieceRF, m_Controls.Dental_HandpieceRFDataLabel);
	//		//qDebug() << "UpdateCameraToToolMatrix is ok.";


	//		//��ȡProbe����
	//		//if (strcmp(prlt->toolname, "THA_Probe") == 0)
	//		if (strcmp(prlt->toolname, "Spine_Probe") == 0)
	//		{
	//			if (prlt->validflag)
	//			{
	//				ProbeTop[0] = prlt->tooltip[0];
	//				ProbeTop[1] = prlt->tooltip[1];
	//				ProbeTop[2] = prlt->tooltip[2];
	//			}
	//		}
	//		T_AimToolDataResult* pnext = prlt->next;
	//		delete prlt;
	//		prlt = pnext;
	//	} while (prlt != NULL);
	//	/*cout << endl;*/

	//}
	//else
	//{
	//	delete prlt;
	//}
}

void DentalRobot::CollectProbeData() {
	/*if (PIController.CanCollectData(ProbeMarkerCnt)) {
		on_pushButton_collectDitch_clicked();
	}*/
}

T_AimToolDataResult* DentalRobot::GetNewToolData()
{
	rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, I_ETHERNET, markerSt, statusSt);
	if (rlt == AIMOOE_NOT_REFLASH)
	{
		std::cout << "camera get data failed";
	}
	T_AimToolDataResult* mtoolsrlt = new T_AimToolDataResult;
	mtoolsrlt->next = NULL;
	mtoolsrlt->validflag = false;
	rlt = Aim_FindToolInfo(aimHandle, markerSt, mtoolsrlt, 3);
	T_AimToolDataResult* prlt = mtoolsrlt;
	return prlt;
}

void DentalRobot::UpdateCameraToToolMatrix(T_AimToolDataResult* ToolData, const char* Name, double* aCamera2Tool, QLabel* label)
{

	//if (strcmp(ToolData->toolname, Name) == 0)
	//{
	//	if (ToolData->validflag)
	//	{
	//		//��ȡ�������
	//		this->t_tran[0] = ToolData->Tto[0];
	//		this->t_tran[1] = ToolData->Tto[1];
	//		this->t_tran[2] = ToolData->Tto[2];
	//		for (int i = 0; i < 3; ++i)
	//		{
	//			for (int j = 0; j < 3; ++j)
	//			{
	//				this->R_tran[i][j] = ToolData->Rto[i][j];
	//			}
	//		}
	//		//ƴ�Ӿ���
	//		vtkNew<vtkMatrix4x4> matrix;
	//		CombineRotationTranslation(R_tran, t_tran, matrix);
	//		memcpy_s(aCamera2Tool, sizeof(double) * 16, matrix->GetData(), sizeof(double) * 16);

	//		if (label != nullptr)
	//		{
	//			QString str = "x:" + QString::number(ToolData->tooltip[0]) + " "
	//				+ "y:" + QString::number(ToolData->tooltip[1]) + " "
	//				+ "z:" + QString::number(ToolData->tooltip[2]);
	//			label->setText(str);
	//			label->setStyleSheet("QLabel { color : green; }");
	//			/*std::cout << str << std::endl;*/
	//		}
	//	}
	//	else
	//	{
	//		if (label != nullptr)
	//		{
	//			QString str = "x:nan  y:nan   z:nan";
	//			label->setText(str);
	//			label->setStyleSheet("QLabel { color : red; }");
	//		}
	//	}
	//}
}


void DentalRobot::OnAutoPositionStart()
{
	// ȷ��Ŀ���ߣ����ȡ�����ݵ��뵽P2��P3��
	auto targetLinePoints = dynamic_cast<mitk::PointSet*>(m_Controls.mitkNodeSelectWidget_imageTargetLine->GetSelectedNode()->GetData());
	auto targetPoint_0 = targetLinePoints->GetPoint(0); // TCP frame origin should move to this point
	auto targetPoint_1 = targetLinePoints->GetPoint(1);

	//auto targetPoint_0 = targetLinePoints->GetPoint(1); // TCP frame origin should move to this point
	//auto targetPoint_1 = targetLinePoints->GetPoint(0);//test
	std::cout << "targetPoint_0: (" << targetPoint_0[0] << ", " << targetPoint_0[1] << ", " << targetPoint_0[2] << ")" << std::endl;
	std::cout << "targetPoint_1: (" << targetPoint_1[0] << ", " << targetPoint_1[1] << ", " << targetPoint_1[2] << ")" << std::endl;


	double targetPointUnderBase_0[3]{ 0 };
	double targetPointUnderBase_1[3]{ 0 };

	for (int i{ 0 }; i < 20; i++)
	{
		//��ȡ��е����׼����T_BaseToBaseRF
		vtkMatrix4x4* vtkT_BaseToBaseRF = vtkMatrix4x4::New();
		vtkT_BaseToBaseRF->DeepCopy(m_T_BaseToBaseRF);


		//��ȡT_BaseRFToCamera
		auto vtkT_BaseRFToCamera = vtkMatrix4x4::New();
		vtkT_BaseRFToCamera->DeepCopy(m_T_CamToBaseRF);
		vtkT_BaseRFToCamera->Invert();

		//��ȡT_CameraToPatientRF
		auto vtkT_CameraToPatientRF = vtkMatrix4x4::New();
		vtkT_CameraToPatientRF->DeepCopy(m_T_CamToPatientRF);

		//��ȡT_PatientRFToImage
		auto vtkT_PatientRFToImage = vtkMatrix4x4::New();
		//vtkT_PatientRFToImage->DeepCopy(T_PatientRFtoImage_SPI);
		vtkT_PatientRFToImage->DeepCopy(m_T_PatientRFtoImage);

		//����T_BaseToImage
		vtkNew<vtkTransform> Transform;
		Transform->Identity();
		Transform->PostMultiply();
		Transform->SetMatrix(vtkT_PatientRFToImage);
		PrintMatrix("T_PatientRFToImage", vtkT_PatientRFToImage->GetData());

		Transform->Concatenate(vtkT_CameraToPatientRF);
		PrintMatrix("T_CameraToPatientRF", vtkT_CameraToPatientRF->GetData());

		Transform->Concatenate(vtkT_BaseRFToCamera);
		PrintMatrix("T_BaseRFToCamera", vtkT_BaseRFToCamera->GetData());

		Transform->Concatenate(vtkT_BaseToBaseRF);
		PrintMatrix("T_BaseToBaseRF", vtkT_BaseToBaseRF->GetData());

		Transform->Update();
		auto vtkT_BaseToImage = Transform->GetMatrix();
		//PrintMatrix("T_BaseToImage", vtkT_BaseToImage->GetData());


		//��ȡP0�������
		auto TargetMatrix_0 = vtkMatrix4x4::New();
		TargetMatrix_0->SetElement(0, 3, targetPoint_0[0]);
		TargetMatrix_0->SetElement(1, 3, targetPoint_0[1]);
		TargetMatrix_0->SetElement(2, 3, targetPoint_0[2]);

		vtkNew<vtkTransform> Trans;
		Trans->Identity();
		Trans->PostMultiply();
		Trans->SetMatrix(TargetMatrix_0);
		Trans->Concatenate(vtkT_BaseToImage);
		Trans->Update();
		auto vtkT_BaseToTarget_0 = Trans->GetMatrix();

		//��ȡP1�������
		auto TargetMatrix_1 = vtkMatrix4x4::New();
		TargetMatrix_1->SetElement(0, 3, targetPoint_1[0]);
		TargetMatrix_1->SetElement(1, 3, targetPoint_1[1]);
		TargetMatrix_1->SetElement(2, 3, targetPoint_1[2]);

		vtkNew<vtkTransform> Trans1;
		Trans1->Identity();
		Trans1->PostMultiply();
		Trans1->SetMatrix(TargetMatrix_1);
		Trans1->Concatenate(vtkT_BaseToImage);
		Trans1->Update();
		auto vtkT_BaseToTarget_1 = Trans1->GetMatrix();

		//����20����
		targetPointUnderBase_0[0] += vtkT_BaseToTarget_0->GetElement(0, 3);
		targetPointUnderBase_0[1] += vtkT_BaseToTarget_0->GetElement(1, 3);
		targetPointUnderBase_0[2] += vtkT_BaseToTarget_0->GetElement(2, 3);

		targetPointUnderBase_1[0] += vtkT_BaseToTarget_1->GetElement(0, 3);
		targetPointUnderBase_1[1] += vtkT_BaseToTarget_1->GetElement(1, 3);
		targetPointUnderBase_1[2] += vtkT_BaseToTarget_1->GetElement(2, 3);

	}
	//ȡƽ��
	targetPointUnderBase_0[0] = targetPointUnderBase_0[0] / 20;
	targetPointUnderBase_0[1] = targetPointUnderBase_0[1] / 20;
	targetPointUnderBase_0[2] = targetPointUnderBase_0[2] / 20;

	targetPointUnderBase_1[0] = targetPointUnderBase_1[0] / 20;
	targetPointUnderBase_1[1] = targetPointUnderBase_1[1] / 20;
	targetPointUnderBase_1[2] = targetPointUnderBase_1[2] / 20;
	std::cout << "targetPointUnderBase_0 [0]" << targetPointUnderBase_0[0] << std::endl;
	std::cout << "targetPointUnderBase_0 [1]" << targetPointUnderBase_0[1] << std::endl;
	std::cout << "targetPointUnderBase_0 [2]" << targetPointUnderBase_0[2] << std::endl;

	std::cout << "targetPointUnderBase_1 [0]" << targetPointUnderBase_1[0] << std::endl;
	std::cout << "targetPointUnderBase_1 [1]" << targetPointUnderBase_1[1] << std::endl;
	std::cout << "targetPointUnderBase_1 [2]" << targetPointUnderBase_1[2] << std::endl;
	//��ȡ��е�۵�T_BaseToFlanger
	double dX1 = 0; double dY1 = 0; double dZ1 = 0;
	double dRx1 = 0; double dRy1 = 0; double dRz1 = 0;
	int nRet = HRIF_ReadActTcpPos(0, 0, dX1, dY1, dZ1, dRx1, dRy1, dRz1);

	auto tmpTrans = vtkTransform::New();
	tmpTrans->PostMultiply();
	tmpTrans->RotateX(dRx1);
	tmpTrans->RotateY(dRy1);
	tmpTrans->RotateZ(dRz1);
	tmpTrans->Translate(dX1, dY1, dZ1);
	tmpTrans->Update();
	vtkSmartPointer<vtkMatrix4x4> VTKT_BaseToFlanger = tmpTrans->GetMatrix();

	//���÷�����Z�᷽��(������)
	Eigen::Vector3d currentXunderBase;
	currentXunderBase[0] = VTKT_BaseToFlanger->GetElement(0, 2);
	currentXunderBase[1] = VTKT_BaseToFlanger->GetElement(1, 2);
	currentXunderBase[2] = VTKT_BaseToFlanger->GetElement(2, 2);
	currentXunderBase.normalize();


	std::cout << "currentxunderBase: " << currentXunderBase << std::endl;

	//��Base����ϵ��Ŀ������ϵZ��ķ�������
	Eigen::Vector3d targetXunderBase;
	targetXunderBase[0] = targetPointUnderBase_1[0] - targetPointUnderBase_0[0];
	targetXunderBase[1] = targetPointUnderBase_1[1] - targetPointUnderBase_0[1];
	targetXunderBase[2] = targetPointUnderBase_1[2] - targetPointUnderBase_0[2];
	targetXunderBase.normalize();

	MITK_INFO << "targetXunderBase" << targetXunderBase;

	Eigen::Vector3d  rotationAxis;
	rotationAxis = currentXunderBase.cross(targetXunderBase);


	double rotationAngle;//������ת��
	if (currentXunderBase.dot(targetXunderBase) > 0) //����������ڻ�����0��cos����0����ǣ�
	{
		rotationAngle = 180 * asin(rotationAxis.norm()) / 3.1415926;//��������ģ����sin[rotationAngle]��,��ȡ������
	}
	else //����������ڻ�С��0��cosС��0���۽ǣ�
	{
		rotationAngle = 180 - 180 * asin(rotationAxis.norm()) / 3.1415926;
	}

	vtkNew<vtkTransform> tmpTransform;
	tmpTransform->PostMultiply();
	tmpTransform->Identity();
	tmpTransform->SetMatrix(VTKT_BaseToFlanger);
	tmpTransform->RotateWXYZ(rotationAngle, rotationAxis[0], rotationAxis[1], rotationAxis[2]);//��ת�Ƕȣ�����ת����
	tmpTransform->Update();

	auto testMatrix = tmpTransform->GetMatrix();
	PrintMatrix("targetMatrix", testMatrix->GetData());

	Eigen::Matrix3d Re;

	Re << testMatrix->GetElement(0, 0), testMatrix->GetElement(0, 1), testMatrix->GetElement(0, 2),
		testMatrix->GetElement(1, 0), testMatrix->GetElement(1, 1), testMatrix->GetElement(1, 2),
		testMatrix->GetElement(2, 0), testMatrix->GetElement(2, 1), testMatrix->GetElement(2, 2);

	Eigen::Vector3d eulerAngle = Re.eulerAngles(2, 1, 0);
	double x = targetPointUnderBase_0[0];
	double y = targetPointUnderBase_0[1];
	double z = targetPointUnderBase_0[2];
	double rx = 180 * eulerAngle[2] / 3.1415;
	double ry = 180 * eulerAngle[1] / 3.1415;
	double rz = 180 * eulerAngle[0] / 3.1415;

	m_Controls.textBrowser->append("-------------------------------------------------------------------------------------------");
	m_Controls.textBrowser->append("TCP_move");
	m_Controls.textBrowser->append("dx=" + QString::number(x));
	m_Controls.textBrowser->append("dy=" + QString::number(y));
	m_Controls.textBrowser->append("dz=" + QString::number(z));
	m_Controls.textBrowser->append("dRx=" + QString::number(rx));
	m_Controls.textBrowser->append("dRy=" + QString::number(ry));
	m_Controls.textBrowser->append("dRz=" + QString::number(rz));
	/*("-------------------------------------------------------------------------------------------");*/

	dX = x;
	dY = y;
	dZ = z;
	dRx = rx;
	dRy = ry;
	dRz = rz;

	//dX = -400
	//dY = -200
	//dZ = -200
	//dRx = 0
	//dRy = 0
	//dRy = 0

	//int nMoveType = 0;

	//// ���幤���������
	//double dTcp_X = 0; double dTcp_Y = 0; double dTcp_Z = 0;
	//double dTcp_Rx = 0; double dTcp_Ry = 0; double dTcp_Rz = 0;
	//// �����û��������
	//double dUcs_X = 0; double dUcs_Y = 0; double dUcs_Z = 0;
	//double dUcs_Rx = 0; double dUcs_Ry = 0; double dUcs_Rz = 0;

	//int nTargetPoint = HRIF_WayPointEx(0, 0, nMoveType, dX, dY, dZ, dRx, dRy, dRz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz, dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek, nIOBit,

	//	dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);//��е���ƶ�
	int nRet_1 = HRIF_ReadActJointPos(0, 0, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
	//int nTargetPoint = HRIF_MoveL(0, 0, dX, dY, dZ, dRx, dRy, dRz,
	//	dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName, sUcsName, dVelocity, dAcc, dRadius,
	//	nIsSeek, nIOBit, nIOState, strCmdID);

	int nTargetPoint = HRIF_WayPointEx(0, 0, 1, dX, dY, dZ, dRx, dRy, dRz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, finaltcp[0], finaltcp[1], finaltcp[2], finaltcp[3], finaltcp[4], finaltcp[5], 0, 0, 0, 0, 0, 0, dVelocity, dAcc, dRadius, 0,
		nIsSeek, nIOBit, nIOState, strCmdID);

	std::cout << "nTargetPoint: " << nTargetPoint << std::endl;
	if (nTargetPoint == 0) {
		m_Controls.textBrowser->append("MOVE OK");
	}
	else {
		m_Controls.textBrowser->append("MOVE FAILED");
		std::cout << nTargetPoint << std::endl;
	}
	m_Controls.textBrowser->append("-------------------------------------------------------------------------------------------");

}

void DentalRobot::PrintMatrix(std::string matrixName, double* matrix)
{
	m_Controls.textBrowser->append("---------------------------------------------------");
	m_Controls.textBrowser->append(QString::fromStdString(matrixName + ":"));
	/*std::cout << matrixName + ": " << std::endl;*/
	for (int i = 0; i < 4; ++i)
	{
		std::string row;
		for (int j = 0; j < 4; ++j)
		{
			row += std::to_string(matrix[i * 4 + j]) + " ";
		}
		m_Controls.textBrowser->append(QString::fromStdString(row) + "\n");
	}
	m_Controls.textBrowser->append("---------------------------------------------------");
}


//else if (this->imp->robotTrackToolFiltering.sign() == "onPushbtnVerifyClicked")
//		{
//		this->imp->isUpdateCurrentPoseError = true;
//
//		if (!this->imp->ui.widgetTrackingTools->isToolValid(LMediatorLinkDevice::RobotBaseRF) ||
//			!this->imp->ui.widgetTrackingTools->isToolValid(LMediatorLinkDevice::Probe))
//		{
//			QMessageBox::information(nullptr, strHint, strRobotBaseRFOrProbeInvisible, strClose);
//			return;
//		}
//
//		// TODO: step 1: calculate T_flangeToVerifyPoint with hardware engineering data
//		auto T_flangeToVerifyPoint1 = vtkSmartPointer<vtkMatrix4x4>::New();
//		auto T_flangeToVerifyPoint2 = vtkSmartPointer<vtkMatrix4x4>::New();
//		auto T_flangeToVerifyPoint3 = vtkSmartPointer<vtkMatrix4x4>::New();
//		T_flangeToVerifyPoint1->Identity();
//		T_flangeToVerifyPoint2->Identity();
//		T_flangeToVerifyPoint3->Identity();
//
//		T_flangeToVerifyPoint1->SetElement(0, 3, -8.982);
//		T_flangeToVerifyPoint1->SetElement(1, 3, -6.750);
//		T_flangeToVerifyPoint1->SetElement(2, 3, 180.365);
//
//		T_flangeToVerifyPoint2->SetElement(0, 3, 2.294);
//		T_flangeToVerifyPoint2->SetElement(1, 3, -6.750);
//		T_flangeToVerifyPoint2->SetElement(2, 3, 184.469);
//
//		T_flangeToVerifyPoint3->SetElement(0, 3, 2.294);
//		T_flangeToVerifyPoint3->SetElement(1, 3, 6.750);
//		T_flangeToVerifyPoint3->SetElement(2, 3, 184.469);
//
//		auto T_flangeToVerifyPoint = vtkSmartPointer<vtkMatrix4x4>::New();
//		T_flangeToVerifyPoint->DeepCopy(T_flangeToVerifyPoint2);
//
//		// step 2: T_robotBaseToVerifyPoint = T_robotBaseToFlange * T_flangeToVerifyPoint
//		auto trans_robotBaseToVerifyPoint = vtkTransform::New();
//		trans_robotBaseToVerifyPoint->Identity();
//		trans_robotBaseToVerifyPoint->PostMultiply();
//		trans_robotBaseToVerifyPoint->SetMatrix(T_flangeToVerifyPoint);
//		trans_robotBaseToVerifyPoint->Concatenate(tracking::G_robotBaseToFlange.data());
//		trans_robotBaseToVerifyPoint->Update();
//		auto T_robotBaseToVerifyPoint = trans_robotBaseToVerifyPoint->GetMatrix();
//
//		//// step 3: T_robotBaseToProbe = T_robotBaseToRobotBaseRF * T_robotBaseRFToCamera * T_cameraToProbe
//		//auto T_robotBaseToRobotBaseRF = vtkSmartPointer<vtkMatrix4x4>::New();
//		//T_robotBaseToRobotBaseRF->DeepCopy(tracking::G_robotBaseRFtoRobotBase.data());
//		//T_robotBaseToRobotBaseRF->Invert();
//
//		//auto T_robotBaseRFToCamera = vtkSmartPointer<vtkMatrix4x4>::New();
//		//T_robotBaseRFToCamera->DeepCopy(tracking::G_cameraToRobotBaseRF.data());
//		//T_robotBaseRFToCamera->Invert();
//
//		//auto trans_robotBaseToProbe = vtkTransform::New();
//		//trans_robotBaseToProbe->Identity();
//		//trans_robotBaseToProbe->PostMultiply();
//		//trans_robotBaseToProbe->SetMatrix(tracking::G_cameraToProbe.data());
//		//trans_robotBaseToProbe->Concatenate(T_robotBaseRFToCamera);
//		//trans_robotBaseToProbe->Concatenate(T_robotBaseToRobotBaseRF);
//		//auto T_robotBaseToProbe = trans_robotBaseToProbe->GetMatrix();
//
//		// step 3: T_robotBaseToProbe = T_robotBaseToFlange * T_FlangeToEndRF * T_EndRFToCamera * T_CameraToProbe
//		auto T_RobotEndRFToCamera = vtkSmartPointer<vtkMatrix4x4>::New();
//		T_RobotEndRFToCamera->DeepCopy(tracking::G_cameraToRobotEndRF.data());
//		T_RobotEndRFToCamera->Invert();
//
//		auto trans_robotBaseToProbe = vtkTransform::New();
//		trans_robotBaseToProbe->Identity();
//		trans_robotBaseToProbe->PostMultiply();
//		trans_robotBaseToProbe->SetMatrix(tracking::G_cameraToProbe.data());
//		trans_robotBaseToProbe->Concatenate(tracking::T_RobotEndRFToCamera);
//		trans_robotBaseToProbe->Concatenate(tracking::G_FlangeToEndRF);
//		trans_robotBaseToProbe->Concatenate(tracking::G_robotBaseToFlange);
//		auto T_robotBaseToProbe = trans_robotBaseToProbe->GetMatrix();
//
//		// step 4: calculate probe verification point error using T_robotBaseToVerifyPoint and T_robotBaseToProbe
//		auto error = tracking::distanceToPoint3D(T_robotBaseToVerifyPoint, T_robotBaseToProbe);
//		this->imp->ui.labelProbeVerifyAccuracyValue->setText(QString::number(error, 'f', 2));
//		}