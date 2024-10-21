﻿
// Qmitk
#include "DentalWidget.h"

// mitk image
#include <mitkImage.h>
#include <vtkCleanPolyData.h>
#include <vtkImplicitPolyDataDistance.h>
#include <ep/include/vtk-9.1/vtkTransformFilter.h>
#include <mitkGeometry3D.h>
#include <mitkRegistrationHelper.h>

#include "mitkBoundingShapeCropper.h"
#include "mitkImageToSurfaceFilter.h"
#include "mitkNodePredicateDataType.h"
#include "mitkSurface.h"
#include "surfaceregistraion.h"
#include "mitkMAPRegistrationWrapper.h"

bool DentalWidget::PrepareCbctImages()
{
	// Crop the CBCT images so that only the mandible part is left
	auto preOpCtImageNode = m_Controls.mitkNodeSelectWidget_preopCT_reg->GetSelectedNode();
	auto intraOpCtImageNode = m_Controls.mitkNodeSelectWidget_intraopCT_reg->GetSelectedNode();

	if(preOpCtImageNode == nullptr || intraOpCtImageNode == nullptr)
	{
		m_Controls.textBrowser->append("Please select the preop CT and the intraop CT");
		return false;
	}

	auto preOpCtImage = dynamic_cast<mitk::Image*>(preOpCtImageNode->GetData());
	auto intraOpCtImage = dynamic_cast<mitk::Image*>(intraOpCtImageNode->GetData());

	// Crop the preop CBCT
	const auto imageGeometry = preOpCtImage->GetGeometry();
	auto boundingBox = mitk::GeometryData::New();
	boundingBox->SetGeometry(static_cast<mitk::Geometry3D*>(this->InitializeWithImageGeometry(imageGeometry)));

	auto croppedImageNode = mitk::DataNode::New();
	auto cutter = mitk::BoundingShapeCropper::New();
	cutter->SetGeometry(boundingBox);
	cutter->SetUseWholeInputRegion(false); //either mask (mask=true) or crop (mask=false)
	cutter->SetInput(preOpCtImage);
	cutter->Update();

	croppedImageNode->SetData(cutter->GetOutput());
	croppedImageNode->SetName("cropped preop CBCT");
	GetDataStorage()->Add(croppedImageNode,preOpCtImageNode);

	// Crop the intraop CBCT
	const auto imageGeometry2 = intraOpCtImage->GetGeometry();
	auto boundingBox2 = mitk::GeometryData::New();
	boundingBox2->SetGeometry(static_cast<mitk::Geometry3D*>(this->InitializeWithImageGeometry(imageGeometry2)));

	auto croppedImageNode2 = mitk::DataNode::New();
	auto cutter2 = mitk::BoundingShapeCropper::New();
	cutter2->SetGeometry(boundingBox2);
	cutter2->SetUseWholeInputRegion(false); //either mask (mask=true) or crop (mask=false)
	cutter2->SetInput(intraOpCtImage);
	cutter2->Update();

	croppedImageNode2->SetData(cutter2->GetOutput());
	croppedImageNode2->SetName("cropped intraop CBCT");
	GetDataStorage()->Add(croppedImageNode2, intraOpCtImageNode);

	return true;
}

mitk::Geometry3D::Pointer DentalWidget::InitializeWithImageGeometry(const mitk::BaseGeometry* geometry) {
	// convert a BaseGeometry into a Geometry3D (otherwise IO is not working properly)
	if (geometry == nullptr)
		mitkThrow() << "Geometry is not valid.";

	auto boundingGeometry = mitk::Geometry3D::New();

	// boundingGeometry->SetBounds(geometry->GetBounds());

	auto bounds = geometry->GetBounds();

	if(m_Controls.radioButton_mandible_cbctReg->isChecked())
	{
		// bounds[3] = bounds[2] + 0.5 * (bounds[3] - bounds[2]);
		bounds[5] = bounds[4] + (3.0 / 7.0) * (bounds[5] - bounds[4]);
	}else
	{
		// bounds[3] = bounds[2] + 0.5 * (bounds[3] - bounds[2]);
		bounds[4] = bounds[5] - (4.0 / 7.0) * (bounds[5] - bounds[4]);
	}
	
	
	boundingGeometry->SetBounds(bounds);

	boundingGeometry->SetImageGeometry(geometry->GetImageGeometry());
	boundingGeometry->SetOrigin(geometry->GetOrigin());
	boundingGeometry->SetSpacing(geometry->GetSpacing());
	boundingGeometry->SetIndexToWorldTransform(geometry->GetIndexToWorldTransform()->Clone());
	boundingGeometry->Modified();
	return boundingGeometry;
}

bool DentalWidget::RetrieveRegistrationMatrix()
{
	auto registrationResultNode = GetDataStorage()->GetNamedNode("Reg #1");
	if(registrationResultNode == nullptr)
	{
		m_Controls.textBrowser->append("Registration result empty, please register first!");
		return false;
	}

	// auto tmpMatrix = dynamic_cast<mitk::MAPRegistrationWrapper*> (registrationResultNode->GetData())->GetGeometry()->GetVtkMatrix();

	auto wrapper = dynamic_cast<mitk::MAPRegistrationWrapper*> (registrationResultNode->GetData());
	
	auto matrix = mitk::MITKRegistrationHelper::getAffineMatrix(wrapper, true)->GetMatrix();
	auto translation = mitk::MITKRegistrationHelper::getAffineMatrix(wrapper, false)->GetTranslation();

	m_Controls.lineEdit_offsetMatrix_0->setText(QString::number(matrix[0][0])); 
	m_Controls.lineEdit_offsetMatrix_1->setText(QString::number(matrix[1][0]));
	m_Controls.lineEdit_offsetMatrix_2->setText(QString::number(matrix[2][0]));
	m_Controls.lineEdit_offsetMatrix_4->setText(QString::number(matrix[0][1]));
	m_Controls.lineEdit_offsetMatrix_5->setText(QString::number(matrix[1][1]));
	m_Controls.lineEdit_offsetMatrix_6->setText(QString::number(matrix[2][1]));
	m_Controls.lineEdit_offsetMatrix_8->setText(QString::number(matrix[0][2]));
	m_Controls.lineEdit_offsetMatrix_9->setText(QString::number(matrix[1][2]));
	m_Controls.lineEdit_offsetMatrix_10->setText(QString::number(matrix[2][2]));
	m_Controls.lineEdit_offsetMatrix_12->setText(QString::number(translation.GetElement(0)));
	m_Controls.lineEdit_offsetMatrix_13->setText(QString::number(translation.GetElement(1)));
	m_Controls.lineEdit_offsetMatrix_14->setText(QString::number(translation.GetElement(2)));

	for(int i{0}; i < 3; i++)
	{
		for(int j{0}; j < 3; j++)
		{
			m_preOpCtToIntraOpCtMatrix[4 * j + i] = matrix[i][j];
		}
	}

	m_preOpCtToIntraOpCtMatrix[3] = translation.GetElement(0);
	m_preOpCtToIntraOpCtMatrix[7] = translation.GetElement(1);
	m_preOpCtToIntraOpCtMatrix[11] = translation.GetElement(2);

	// m_Controls.textBrowser->append(QString::number(m_preOpCtToIntraOpCtMatrix[9]));


	return true;
}


bool DentalWidget::AppendMatrix_regisCbct()
{
	// auto tmpNode = m_Controls.mitkNodeSelectWidget_appendMatrix->GetSelectedNode();
	// if(tmpNode == nullptr)
	// {
	// 	m_Controls.textBrowser->append("Please select a node first!");
	// 	return false;
	// }
	//
	// auto tmpBaseData = tmpNode->GetData();
	// vtkNew<vtkMatrix4x4> tmpMatrix;
	// tmpMatrix->DeepCopy(m_preOpCtToIntraOpCtMatrix);
	//
	// vtkNew<vtkTransform> tmpTransform;
	// tmpTransform->Identity();
	// tmpTransform->PostMultiply();
	// tmpTransform->SetMatrix(tmpBaseData->GetGeometry()->GetVtkMatrix());
	// tmpTransform->Concatenate(tmpMatrix);
	// tmpTransform->Update();
	//
	// tmpBaseData->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(tmpTransform->GetMatrix());
	//
	// mitk::RenderingManager::GetInstance()->RequestUpdateAll();

	//--------------------------------------------
	vtkNew<vtkMatrix4x4> tmpMatrix;
	tmpMatrix->DeepCopy(m_preOpCtToIntraOpCtMatrix);
	m_currentSelectedNode = m_Controls.mitkNodeSelectWidget_appendMatrix->GetSelectedNode();

	QModelIndex parentIndex = m_NodetreeModel->GetIndex(m_currentSelectedNode);

	int parentRowCount = m_NodetreeModel->rowCount(parentIndex);
	int parentColumnCount = m_NodetreeModel->columnCount(parentIndex);

	for (int i = 0; i < (parentRowCount + 1); i++)
	{
		if (i == parentRowCount)
		{
			m_baseDataToMove = m_NodetreeModel->GetNode(parentIndex)->GetData();

		}
		else
		{
			QModelIndex tmpIndex = m_NodetreeModel->index(i, 0, parentIndex);
			m_baseDataToMove = m_NodetreeModel->GetNode(tmpIndex)->GetData();
		}

		if (m_baseDataToMove != nullptr)
		{
			auto tmpVtkTransform = vtkTransform::New();
			tmpVtkTransform->PostMultiply();
			tmpVtkTransform->Identity();
			tmpVtkTransform->SetMatrix(m_baseDataToMove->GetGeometry()->GetVtkMatrix());
			tmpVtkTransform->Concatenate(tmpMatrix);

			m_baseDataToMove->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(tmpVtkTransform->GetMatrix());
			mitk::RenderingManager::GetInstance()->RequestUpdateAll();


		}
		else
		{
			m_Controls.textBrowser->append("Moving object is empty ~~");
		}

		// if the m_baseDataToMove is pointSet, rewrite the pointSet members and keep geometry matrix as identity
		if (dynamic_cast<mitk::PointSet*>(m_baseDataToMove) != nullptr)
		{
			auto tmpPointSet = dynamic_cast<mitk::PointSet*>(m_baseDataToMove);
			int size = tmpPointSet->GetSize();

			for (int i{ 0 }; i < size; i++)
			{
				auto tmpPoint = tmpPointSet->GetPoint(i);
				vtkNew<vtkTransform> tmpTrans;
				tmpTrans->Identity();
				tmpTrans->PostMultiply();
				tmpTrans->Translate(tmpPoint[0], tmpPoint[1], tmpPoint[2]);
				tmpTrans->Concatenate(tmpPointSet->GetGeometry()->GetVtkMatrix());
				tmpTrans->Update();
				auto tmpMatrix = tmpTrans->GetMatrix();

				mitk::Point3D newPoint;
				newPoint[0] = tmpMatrix->GetElement(0, 3);
				newPoint[1] = tmpMatrix->GetElement(1, 3);
				newPoint[2] = tmpMatrix->GetElement(2, 3);

				tmpPointSet->SetPoint(i, newPoint);

			}
			vtkNew<vtkMatrix4x4> identityMatrix;
			identityMatrix->Identity();
			tmpPointSet->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(identityMatrix);
		}


	}




	return true;
}


bool DentalWidget::AssembleRegistrationPoints()
{
	if (GetDataStorage()->GetNamedNode("Steelball centers") == nullptr && GetDataStorage()->GetNamedNode("Guide plate points") == nullptr)
	{
		m_Controls.textBrowser->append("Warning: 'Steelball centers' or 'Guide plate points' is missing");
		return false;
	}
	
	auto steelballs = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("Steelball centers")->GetData());
	auto guidePlatePoints = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("Guide plate points")->GetData());

	auto registrationPointset = mitk::PointSet::New();
	for (int i{0}; i< steelballs->GetSize(); i++)
	{
		registrationPointset->InsertPoint(steelballs->GetPoint(i));
	}

	for (int i{0}; i < guidePlatePoints->GetSize(); i++)
	{
		registrationPointset->InsertPoint(guidePlatePoints->GetPoint(i));
	}

	auto tmpNode = mitk::DataNode::New();
	tmpNode->SetData(registrationPointset);
	tmpNode->SetName("RegistrationPointSet");

	GetDataStorage()->Add(tmpNode);

	return true;
}
