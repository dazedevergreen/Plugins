
// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qmitk
#include "HTOrobot.h"

// Qt
#include <QMessageBox>
#include <mutex>
// mitk image
#include <mitkImage.h>
#include "mitkPointSet.h"
#include <mitkSurface.h>
#include "surfaceregistraion.h"
//vtk
#include <vtkLandmarkTransform.h>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkMatrix4x4.h>
#include <vtkPointData.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTransformFilter.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <ep\include\vtk-9.1\vtkPolyDataNormals.h>
#include <QtCore\qmath.h>
#include <new.h>
#include <ep\include\vtk-9.1\vtkKdTree.h>
//tibiaImageRegistration
void HTOrobot::getImageBallcenter()
{

	mitk::PointSet::Pointer ballCentersImagePointSet = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("ballCenters_imagePointSet")->GetData());
	if (ballCentersImagePointSet.IsNotNull())
	{

		// get size of PointSet
		unsigned int numberOfPoints = ballCentersImagePointSet->GetSize();
		for (unsigned int i = 0; i < numberOfPoints; ++i)
		{
			// get position
			mitk::PointSet::PointType point = ballCentersImagePointSet->GetPoint(i);

			QString pointText = QString("Point %1: (%2, %3, %4)\n").arg(i + 1).arg(point[0]).arg(point[1]).arg(point[2]);
			m_Controls.textBrowser_HTO->append(pointText);
		}
	}
}
void HTOrobot::tibiaRegistration()
{
	mitk::PointSet::Pointer ballCentersImagePointSet = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("ballCenters_imagePointSet")->GetData());
	double designValues[15] // 基于metalBallRF的钢球坐标（硬件工程数据）
	{

		6.36, 24.18, 9.80,
		-7.82, 46.69, 9.80,
		16.25, 61.75, 9.80,
		24.95, 26.89, 9.80,
		43.78, 15.51, 9.80,
	};
	mitk::PointSet::Pointer ballCenters_ballRF = mitk::PointSet::New();
	
	for (int i{ 0 }; i < 5; i++)
	{
		double tmpDouble[3]{ designValues[3 * (i)],designValues[3 * (i)+1],designValues[3 * (i)+2] };
		mitk::Point3D tmpPoint{ tmpDouble };
		ballCenters_ballRF->InsertPoint(tmpPoint);
	
	}

	T_imageFrameToSteelballRF = ComputeTransformMartix(ballCentersImagePointSet, ballCenters_ballRF);
	QString outputText_new = "T_imageFrameToSteelballRF:\n";
	for (int row = 0; row < 4; ++row)
	{
		for (int col = 0; col < 4; ++col)
		{
			outputText_new += QString::number(T_imageFrameToSteelballRF->GetElement(row, col)) + "\t";
		}
		outputText_new += "\n";
	}
	m_Controls.textBrowser_HTO->append(outputText_new);
	mitk::DataNode::Pointer pointSetNode2 = mitk::DataNode::New();
	pointSetNode2->SetData(ballCenters_ballRF);
	pointSetNode2->SetName("ballCenters_ballRFPointSet");
	GetDataStorage()->Add(pointSetNode2);

}
void HTOrobot::saveTibiaRegistration()
{

	mitk::DataStorage::Pointer dataStorage = GetDataStorage();
	mitk::PointSet::Pointer ballCenter_ballRF = dynamic_cast<mitk::PointSet*>(dataStorage->GetNamedNode("ballCenters_ballRFPointSet")->GetData());
	vtkSmartPointer<vtkPoints>  vtkballCenter_ballRF = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints>  vtkballCenter_ballRFinImage = vtkSmartPointer<vtkPoints>::New();
	for (int i=0; i < 5; i++)
	{
		mitk::PointSet::PointType point = ballCenter_ballRF->GetPoint(i);
		vtkballCenter_ballRF->InsertNextPoint(point[0], point[1], point[2]);
	}
	vtkSmartPointer<vtkMatrix4x4> m_vtkregistrationMatrixInverse = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(T_imageFrameToSteelballRF, m_vtkregistrationMatrixInverse);
	auto transform = vtkTransform::New();
	transform->SetMatrix(m_vtkregistrationMatrixInverse);
	transform->TransformPoints(vtkballCenter_ballRF, vtkballCenter_ballRFinImage);
	mitk::Point3D transformedPoint;
	mitk::PointSet::Pointer m_PointSet_ballRF = mitk::PointSet::New();
	for (vtkIdType i = 0; i < vtkballCenter_ballRFinImage->GetNumberOfPoints(); ++i)
	{
		transformedPoint = vtkballCenter_ballRFinImage->GetPoint(i);
		
		QString transformedPointText = "Transformed Point: x=" + QString::number(transformedPoint[0]) +
			", y=" + QString::number(transformedPoint[1]) + ", z=" + QString::number(transformedPoint[2]);
		m_Controls.textBrowser_HTO->append(transformedPointText);
		m_PointSet_ballRF->InsertPoint(transformedPoint);
	}
	mitk::DataNode::Pointer pointSetNode3 = mitk::DataNode::New();
	pointSetNode3->SetData(m_PointSet_ballRF);
	pointSetNode3->SetName("ballRFToImagePointSet");
	GetDataStorage()->Add(pointSetNode3);
	// 保存矩阵到文件
	std::string filename = "C:/Users/Admin/Desktop/save/T_imageFrameToSteelballRF.txt";
	std::ofstream file(filename);

	if (!file.is_open())
	{
		std::cerr << "Failed to open file: " << filename << std::endl;
		return;
	}

	for (int row = 0; row < 4; ++row)
	{
		for (int col = 0; col < 4; ++col)
		{
			

			file << T_imageFrameToSteelballRF->GetElement(row, col) << " ,";
		}
		file << "\n";
		
	}
	file.close();
	std::cout << "T_imageFrameToSteelballRF saved to file: " << filename << std::endl;
	m_Controls.textBrowser_HTO->append("saveArmMatrix");
}
void HTOrobot::reuseMatrix()
{
	//qDebug() << "1";
	//int data[4][4];
	//// 指定文件路径
	//std::string filePath = "C:/Users/Admin/Desktop/save/T_imageFrameToSteelballRF.txt";
	//fstream in.open(FilePath, ios::in);//打开一个file
	//if (!in.is_open()) {
	//	cout << "Can not find " << filePath << endl;
	//	system("pause");
	//}
	//string buff;
	//int i = 0;//行数i
	//while (getline(in, buff)) {
	//	vector<double> nums;
	//	// string->char *
	//	char* s_input = (char*)buff.c_str();
	//	const char* split = "，";
	//	// 以‘，’为分隔符拆分字符串
	//	char* p = strtok(s_input, split);
	//	double a;
	//	while (p != NULL) {
	//		// char * -> int
	//		a = atof(p);
	//		//cout << a << endl;
	//		nums.push_back(a);
	//		p = strtok(NULL, split);
	//	}//end while
	//	for (int b = 0; b < nums.size(); b++) {
	//		data[i][b] = nums[b];
	//	}//end for
	//	i++;
	//}//end while
	//in.close();
	//cout << "get  data" << endl;

	//

	//for (int row = 0; row < 4; ++row)
	//{
	//	for (int col = 0; col < 4; ++col)
	//	{


	//		std::cout << T_imageFrameToSteelballRF->GetElement(row, col) << ","<<std::endl;
	//		m_Controls.textBrowser_HTO->append(QString::number(T_imageFrameToSteelballRF->GetElement(row, col))+",");
	//	}
	//	m_Controls.textBrowser_HTO->append("\n");

	//}
	//
	//std::cout << "T_imageFrameToSteelballRF read from file: " << filePath << std::endl;
	
}
vtkSmartPointer<vtkMatrix4x4> HTOrobot::ComputeTransformMartix(mitk::PointSet::Pointer points_set1, mitk::PointSet::Pointer points_set2)
{
	// Creat vtkPointset
	vtkSmartPointer<vtkPoints> vtkPoints1 = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> vtkPoints2 = vtkSmartPointer<vtkPoints>::New();
	// Convert mitk::PointSet to vtkPoints
	for (unsigned int i = 0; i < points_set1->GetSize(); ++i)
	{
		mitk::PointSet::PointType point = points_set1->GetPoint(i);
		vtkPoints1->InsertNextPoint(point[0], point[1], point[2]);
	}

	for (unsigned int i = 0; i < points_set2->GetSize(); ++i)
	{
		mitk::PointSet::PointType point = points_set2->GetPoint(i);
		vtkPoints2->InsertNextPoint(point[0], point[1], point[2]);
	}


	vtkSmartPointer<vtkLandmarkTransform> tmpTrans = vtkSmartPointer<vtkLandmarkTransform>::New();
	tmpTrans->SetModeToRigidBody();

	// Set source point and target point into vtkLandmarkTransform
	tmpTrans->SetSourceLandmarks(vtkPoints1);
	tmpTrans->SetTargetLandmarks(vtkPoints2);

	tmpTrans->Update();

	vtkSmartPointer<vtkMatrix4x4> transformationMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	transformationMatrix->DeepCopy(tmpTrans->GetMatrix());

	return transformationMatrix;
}

/**
*采集粗配准点
 * @brief 使用探针收集LandMark点
 * @note 本质上是是将探针的尖端点转移到F_PatientRF下
 */
 //---------------------------------------------------------------------------------------------------------------
void HTOrobot::collectLandmark()
{
	m_IndexOfLandmark++;
	m_Controls.lineEdit_collectedLandmark->setText(QString::number(m_IndexOfLandmark));
	m_Controls.textBrowser_HTO->append(QString(" m_IndexOfLandmark: ") + QString::number(m_IndexOfLandmark));
	//获取T_patientToProbeRF矩阵
	auto vtkT_CameraToProbe = vtkMatrix4x4::New();
	auto vtkT_PatientRFToCamera = vtkMatrix4x4::New();
	vtkT_CameraToProbe->DeepCopy(T_CamToProbe);
	vtkT_PatientRFToCamera->DeepCopy(T_CamToPatientRF);
	vtkT_PatientRFToCamera->Invert();

	auto transform = vtkTransform::New();
	transform->Identity();
	transform->PostMultiply();
	transform->SetMatrix(vtkT_CameraToProbe);
	transform->Concatenate(vtkT_PatientRFToCamera);
	transform->Update();
	auto T_patientToProbeRF = transform->GetMatrix();

	//保存T_patientrfToProbeRF
	memcpy_s(T_PatientRFToProbe, sizeof(double) * 16, T_patientToProbeRF->GetData(), sizeof(double) * 16);

	//获取probe尖端的点在patinentRF坐标系下的坐标
	auto tmptrans = vtkTransform::New();
	tmptrans->Identity();
	tmptrans->PostMultiply();
	tmptrans->SetMatrix(vtkT_PatientRFToCamera);
	//
	tmptrans->MultiplyPoint(ProbeTop, nd_tip_FpatientRF);
	vtkProbeTip_onObjRf->InsertNextPoint(nd_tip_FpatientRF[0], nd_tip_FpatientRF[1], nd_tip_FpatientRF[2]);

	m_Controls.textBrowser_HTO->append(QString("Probe Point Landmark: (") + QString::number(nd_tip_FpatientRF[0]) + ", " + QString::number(nd_tip_FpatientRF[1]) + ", "
		+ QString::number(nd_tip_FpatientRF[2]) + ")");

}
/**
*采集精配准点
 * @brief 使用探针收集ICP点
 * @note 本质上是是将探针的尖端点转移到F_Image下
 */
 //---------------------------------------------------------------------------------------------------------------
void HTOrobot::collectICP()
{
	//计数
	m_IndexOfICP++;
	m_Controls.lineEdit_collectedICP->setText(QString::number(m_IndexOfICP));
	m_Controls.textBrowser_HTO->append(QString(" m_IndexOfICP: ") + QString::number(m_IndexOfICP));

	//获取T_patientToProbeRF矩阵
	auto vtkT_cameraToprobeRF = vtkMatrix4x4::New();
	vtkT_cameraToprobeRF->DeepCopy(T_CamToProbe);

	auto vtkT_patientRFTocamera = vtkMatrix4x4::New();
	vtkT_patientRFTocamera->DeepCopy(T_CamToPatientRF);
	vtkT_patientRFTocamera->Invert();

	auto T_ImageTopatientRF_landmark = vtkMatrix4x4::New();
	T_ImageTopatientRF_landmark->DeepCopy(T_PatientRFtoImage);
	T_ImageTopatientRF_landmark->Invert();

	auto transform = vtkTransform::New();
	transform->Identity();
	transform->PostMultiply();
	transform->SetMatrix(vtkT_cameraToprobeRF);
	transform->Concatenate(vtkT_patientRFTocamera);
	transform->Concatenate(T_ImageTopatientRF_landmark);
	transform->Update();
	auto T_ImageToProbeRF = transform->GetMatrix();

	//获取探针在image下的坐标
	auto tmptrans = vtkTransform::New();
	tmptrans->Identity();
	tmptrans->PostMultiply();
	//tmptrans->SetMatrix(T_ImageToProbeRF);//这里有问题
	tmptrans->SetMatrix(vtkT_patientRFTocamera);
	tmptrans->Concatenate(T_ImageTopatientRF_landmark);
	tmptrans->MultiplyPoint(ProbeTop, nd_tip_FImage_icp);

	vtkProbeTip_onObjRf_icp->InsertNextPoint(nd_tip_FImage_icp[0], nd_tip_FImage_icp[1], nd_tip_FImage_icp[2]);
	m_Controls.textBrowser_HTO->append(QString("Probe Point ICP: (") + QString::number(nd_tip_FImage_icp[0]) + ", " + QString::number(nd_tip_FImage_icp[1]) + ", "
		+ QString::number(nd_tip_FImage_icp[2]) + ")");

}
//---------------------------------------------------------------------------------------------------------------
/**
 * @brief LandMark配准（粗配准、点点配准）
 * @note 重点要注意配的src数据和tar数据，决定了后面配准矩阵的指向
 * @note auto vtkT_patientRFtoImage = ComputeHomogeneousTransform_vtk(Patient坐标系下的点, Image坐标系下的点);
 */
 //---------------------------------------------------------------------------------------------------------------
void HTOrobot::landmarkRegistration()
{
	//获取立柱工装的landmark的点
	auto landmarkSrcNode = m_Controls.mitkNodeSelectWidget_landmark_src->GetSelectedNode();
	mitk::PointSet::Pointer pointSet_Src = dynamic_cast<mitk::PointSet*>(landmarkSrcNode->GetData());
	if (pointSet_Src.IsNotNull())
	{
		unsigned int numberOfPoints = pointSet_Src->GetSize();
		for (unsigned int i = 0; i < numberOfPoints; ++i)
		{
			// get position
			mitk::PointSet::PointType point = pointSet_Src->GetPoint(i);

			QString pointText = QString("Point %1: (%2, %3, %4)\n").arg(i + 1).arg(point[0]).arg(point[1]).arg(point[2]);
			m_Controls.textBrowser_HTO->append(pointText);
		}
	}

	//获取采集的点
	mitk::PointSet::Pointer pointSet_Tar = mitk::PointSet::New();
	vtkIdType numPoints = vtkProbeTip_onObjRf->GetNumberOfPoints();
	for (vtkIdType i = 0; i < numPoints; ++i)
	{
		double point[3];
		vtkProbeTip_onObjRf->GetPoint(i, point);

		mitk::PointSet::PointType mitkPoint;
		mitkPoint[0] = point[0];
		mitkPoint[1] = point[1];
		mitkPoint[2] = point[2];
		pointSet_Tar->SetPoint(i, mitkPoint);
	}

	if (pointSet_Tar.IsNotNull())
	{
		unsigned int numberOfPoints = pointSet_Tar->GetSize();

		for (unsigned int i = 0; i < numberOfPoints; ++i)
		{
			mitk::PointSet::PointType point = pointSet_Tar->GetPoint(i);

			QString pointText2 = QString("Point %1: (%2, %3, %4)\n").arg(i + 1).arg(point[0]).arg(point[1]).arg(point[2]);
			m_Controls.textBrowser_HTO->append(pointText2);
		}
	}

	//计算Tpatienttoimage
	auto vtkT_patientRFtoImage = ComputeTransformMartix(pointSet_Src, pointSet_Tar);
	memcpy_s(T_PatientRFtoImage, sizeof(double) * 16, vtkT_patientRFtoImage->GetData(), sizeof(double) * 16);

	// Create a Data Node for the second point set
	// show point in image space
	mitk::DataNode::Pointer pointSetNode2 = mitk::DataNode::New();
	pointSetNode2->SetData(pointSet_Tar);
	pointSetNode2->SetName("Simulated_PointSet_Target");

	GetDataStorage()->Add(pointSetNode2);
	QString matrixText;



}

//---------------------------------------------------------------------------------------------------------------
/**
 * @brief 使用探针收集ICP点（软件全流程）
 * @note 本质上是是将探针的尖端点转移到F_Image下
 */
 //---------------------------------------------------------------------------------------------------------------
void HTOrobot::collectICP_SPI()
{
	////计数
	//m_IndexOfICP++;
	//m_Controls.lineEdit_collectedICP->setText(QString::number(m_IndexOfICP));
	//m_Controls.textBrowser_HTO->append(QString(" m_IndexOfICP: ") + QString::number(m_IndexOfICP));

	////获取在图像坐标系下的探针尖端点的坐标 T_imageToCamera * 探针尖端点在相机坐标系下的坐标

	//auto T_ImageToCamera_landmark = vtkMatrix4x4::New();
	//T_ImageToCamera_landmark->DeepCopy(T_imageFrameToSteelballRF);
	//T_ImageToCamera_landmark->Invert();


	////获取探针在image下的坐标
	//auto tmptrans = vtkTransform::New();
	//tmptrans->Identity();
	//tmptrans->PostMultiply();
	//tmptrans->SetMatrix(T_ImageToCamera_landmark);
	//tmptrans->MultiplyPoint(ProbeTop, nd_tip_FImage_icp);

	//vtkProbeTip_onObjRf_icp_SPI->InsertNextPoint(nd_tip_FImage_icp[0], nd_tip_FImage_icp[1], nd_tip_FImage_icp[2]);
	//m_Controls.textBrowser_HTO->append(QString("Probe Point ICP: (") + QString::number(nd_tip_FImage_icp[0]) + ", " + QString::number(nd_tip_FImage_icp[1]) + ", "
	//	+ QString::number(nd_tip_FImage_icp[2]) + ")");

}
/**
*ICP精配准
*@brief 执行源表面和目标点集之间的ICP配准[点面配准]
*@note 配准后的ICP矩阵要右乘到landmark配准矩阵进行补偿
*/
//---------------------------------------------------------------------------------------------------------------
void HTOrobot::ICPRegistration()
{
	auto icpRegistrator = mitk::SurfaceRegistration::New();

	m_IcpSourceSurface = m_Controls.mitkNodeSelectWidget_surface_regis->GetSelectedNode();

	auto icpTargetPointset = mitk::PointSet::New();

	for (vtkIdType i = 0; i < vtkProbeTip_onObjRf_icp->GetNumberOfPoints(); ++i)
	{
		double* pointCoords = vtkProbeTip_onObjRf_icp->GetPoint(i);

		mitk::Point3D point;
		point[0] = pointCoords[0];
		point[1] = pointCoords[1];
		point[2] = pointCoords[2];


		icpTargetPointset->InsertPoint(i, point);
	}


	if (m_IcpSourceSurface != nullptr && icpTargetPointset != nullptr)
	{
		auto icpSrcSurface = dynamic_cast<mitk::Surface*>(m_IcpSourceSurface->GetData());
		icpRegistrator->SetIcpPoints(icpTargetPointset);
		icpRegistrator->SetSurfaceSrc(icpSrcSurface);
		icpRegistrator->ComputeIcpResult();

		double rms = GetRegisrationRMS(icpTargetPointset, icpSrcSurface, icpRegistrator->GetResult());
		m_Controls.textBrowser_HTO->append("rms" + QString::number(rms));
		Eigen::Matrix4d tmpRegistrationResult{ icpRegistrator->GetResult()->GetData() };
		tmpRegistrationResult.transposeInPlace();

		auto T_imageToImage_icp = vtkMatrix4x4::New();
		for (int row = 0; row < 4; ++row)
		{
			for (int col = 0; col < 4; ++col)
			{
				T_imageToImage_icp->SetElement(row, col, tmpRegistrationResult(row, col));
			}
		}

		QString output;

		output.append("Eigen::Matrix4d content:\n");
		for (int row = 0; row < tmpRegistrationResult.rows(); ++row)
		{
			for (int col = 0; col < tmpRegistrationResult.cols(); ++col)
			{
				output.append(QString::number(tmpRegistrationResult(row, col)) + " ");
			}
			output.append("\n");
		}

		m_Controls.textBrowser_HTO->append(output);

		memcpy_s(T_ImageToImage_icp, sizeof(double) * 16, T_imageToImage_icp->GetData(), sizeof(double) * 16);
	}
}
double HTOrobot::GetRegisrationRMS(mitk::PointSet* points, mitk::Surface* surface, vtkMatrix4x4* matrix)
{
	vtkSmartPointer<vtkPoints> pSource =
		vtkSmartPointer<vtkPoints>::New();

	for (int i = 0; i < points->GetSize(); i++)
	{
		auto pointVega = points->GetPoint(i);
		pSource->InsertNextPoint(pointVega[0], pointVega[1], pointVega[2]);
	}
	vtkSmartPointer<vtkPoints> transformedPoints =
		vtkSmartPointer<vtkPoints>::New();
	transformedPoints->DeepCopy(TransformVTKPoints(pSource, matrix));

	vtkSmartPointer<vtkKdTree> KDTree = vtkSmartPointer<vtkKdTree>::New();
	KDTree->BuildLocatorFromPoints(surface->GetVtkPolyData()->GetPoints());
	double sum = 0.0;
	for (int i = 0; i < transformedPoints->GetNumberOfPoints(); ++i)
	{
		double distance;
		double pos[3];
		transformedPoints->GetPoint(i, pos);
		KDTree->FindClosestPoint(pos, distance);
		sum += distance;
	}

	double result = std::sqrt(sum / transformedPoints->GetNumberOfPoints());
	return result;
}
vtkSmartPointer<vtkPoints> HTOrobot::TransformVTKPoints(vtkPoints* in, vtkMatrix4x4* m)
{
	vtkSmartPointer<vtkPoints> tmpPoints = vtkSmartPointer<vtkPoints>::New();
	tmpPoints->DeepCopy(in);
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	polyData->SetPoints(tmpPoints);
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	vtkSmartPointer<vtkMatrix4x4> tmpMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	tmpMatrix->DeepCopy(m);
	vtkSmartPointer<vtkTransform> trans = vtkSmartPointer<vtkTransform>::New();
	trans->SetMatrix(tmpMatrix);
	transformFilter->SetInputData(polyData);
	transformFilter->SetTransform(trans);
	transformFilter->Update();
	vtkSmartPointer<vtkPoints> result = vtkSmartPointer<vtkPoints>::New();
	result->DeepCopy(transformFilter->GetOutput()->GetPoints());
	return result;
}
//---------------------------------------------------------------------------------------------------------------
/**
*@brief 重新进行图像配准
*@note  清空的东西
*/
//---------------------------------------------------------------------------------------------------------------
void HTOrobot::ResetImageConfiguration()
{
	m_IndexOfLandmark = 0;
	m_IndexOfICP = 0;

	m_Controls.lineEdit_collectedLandmark->setText(QString::number(0));
	m_Controls.lineEdit_collectedICP->setText(QString::number(0));

	vtkProbeTip_onObjRf->Reset();
	vtkProbeTip_onObjRf_icp->Reset();

	if (vtkProbeTip_onObjRf->GetNumberOfPoints() == 0 && vtkProbeTip_onObjRf_icp->GetNumberOfPoints() == 0)
	{
		m_Controls.textBrowser_HTO->append("Replace image configuration");
	}
	else
	{
		m_Controls.textBrowser_HTO->append("Replace image configuration failed");
	}
}
//---------------------------------------------------------------------------------------------------------------
/**
*@brief 保存图像配准矩阵
*@note  保存两个矩阵T_PatientRFtoImage、T_ImageToImage_icp
*/
//---------------------------------------------------------------------------------------------------------------
void HTOrobot::saveImageMatrix()
{
	//存T_PatientRFtoImage
	std::ofstream robotMatrixFile(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\T_PatientRFtoImage.txt");
	for (int i = 0; i < 16; i++) {
		robotMatrixFile << T_PatientRFtoImage[i];
		if (i != 15) {
			robotMatrixFile << ",";
		}
		else {
			robotMatrixFile << ";";
		}
	}
	robotMatrixFile << std::endl;
	robotMatrixFile.close();

	//存T_ImageToImage_icp
	std::ofstream robotMatrixFile1(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\T_ImageToImage_icp.txt");
	for (int i = 0; i < 16; i++) {
		robotMatrixFile1 << T_ImageToImage_icp[i];
		if (i != 15) {
			robotMatrixFile1 << ",";
		}
		else {
			robotMatrixFile1 << ";";
		}
	}
	robotMatrixFile1 << std::endl;
	robotMatrixFile1.close();
	m_Controls.textBrowser_HTO->append("saveImageMatrix");

}
//---------------------------------------------------------------------------------------------------------------
/**
*@brief 复用图像配准矩阵
*@note  复用两个矩阵T_PatientRFtoImage、T_ImageToImage_icp
*/
//---------------------------------------------------------------------------------------------------------------
void HTOrobot::reuseImageMatrix()
{
	//导入T_PatientRFtoImage
	std::ifstream inputFile(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\T_PatientRFtoImage.txt");
	if (inputFile.is_open()) {
		std::string line;
		if (std::getline(inputFile, line)) {
			std::stringstream ss(line);
			std::string token;
			int index = 0;
			while (std::getline(ss, token, ',')) {
				T_PatientRFtoImage[index] = std::stod(token);
				index++;
			}
		}
		inputFile.close();
	}
	else {

		m_Controls.textBrowser_HTO->append("无法打开文件:T_PatientRFtoImage.txt");
	}


	PrintDataHelper::AppendTextBrowserMatrix(m_Controls.textBrowser_HTO, "T_PatientRFtoImage", T_PatientRFtoImage);
	PrintArray16ToMatrix("T_PatientRFtoImage", T_PatientRFtoImage);


	std::ifstream inputFile2(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\T_ImageToImage_icp.txt");
	if (inputFile2.is_open()) {
		std::string line2;
		if (std::getline(inputFile2, line2)) {
			std::stringstream ss2(line2);
			std::string token2;
			int index2 = 0;
			while (std::getline(ss2, token2, ',')) {
				T_ImageToImage_icp[index2] = std::stod(token2);
				index2++;
			}
		}
		inputFile2.close();
	}
	else {
		m_Controls.textBrowser_HTO->append("无法打开文件：T_ImageToImage_icp.txt");
	}


	PrintDataHelper::AppendTextBrowserMatrix(m_Controls.textBrowser_HTO, "T_ImageToImage_icp", T_ImageToImage_icp);
	PrintArray16ToMatrix("T_ImageToImage_icp", T_ImageToImage_icp);
}
void HTOrobot::PrintArray16ToMatrix(const std::string& title, double* Array)
{
	std::cout << "+-----------------------------------------------+" << std::endl;
	std::cout << "|               " << title << "              |" << std::endl;
	std::cout << "+-----------+-----------+----------+------------+" << std::endl;
	for (int i = 0; i < 4; i++) {
		std::cout << "|  ";
		for (int j = 0; j < 4; j++) {
			std::cout << std::fixed << std::setprecision(6) << Array[i * 4 + j] << " ";
		}
		std::cout << "|" << std::endl;
	}
	std::cout << "+-----------+-----------+----------+------------+" << std::endl;
}

/*----------检查点验证-Checkpoint validation-------*/
void HTOrobot::testProximalPoint()
{
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToObjRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToObjRF = GetArray2vtkMatrix(T_CamToMetalBallRF);
	mitk::Point3D outputPoint;
	verifyPoint(vtkMatrixFromT_CamToObjRF,outputPoint);
	double distance = sqrt(pow(outputPoint[0] - proximalStaplePoint[0], 2) +
		pow(outputPoint[1] - proximalStaplePoint[1], 2) +
		pow(outputPoint[2] - proximalStaplePoint[2], 2));

		
	std::cout << "Root Mean Square Error (RMSE): " << distance << std::endl;
	m_Controls.lineEdit_proximalTest->setText(QString::number(distance));

}
void HTOrobot::testDistalPoint()
{
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToObjRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToObjRF = GetArray2vtkMatrix(T_CamToPatientRF);
	mitk::Point3D outputPoint;
	verifyPoint(vtkMatrixFromT_CamToObjRF, outputPoint);
	double distance = sqrt(pow(outputPoint[0] - distalStaplePoint[0], 2) +
		pow(outputPoint[1] - distalStaplePoint[1], 2) +
		pow(outputPoint[2] - distalStaplePoint[2], 2));


	std::cout << "Root Mean Square Error (RMSE): " << distance << std::endl;
	m_Controls.lineEdit_distalTest->setText(QString::number(distance));

}
void HTOrobot::testSawPoint()
{
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToObjRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToObjRF = GetArray2vtkMatrix(T_CamToSaw);
	mitk::Point3D outputPoint;
	verifyPoint(vtkMatrixFromT_CamToObjRF, outputPoint);
	double distance = sqrt(pow(outputPoint[0] - SawTestPoint[0], 2) +
		pow(outputPoint[1] - SawTestPoint[1], 2) +
		pow(outputPoint[2] - SawTestPoint[2], 2));


	std::cout << "Root Mean Square Error (RMSE): " << distance << std::endl;
	m_Controls.lineEdit_sawTest->setText(QString::number(distance));
}
void HTOrobot::collectProximalStaplePoint()
{
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToObjRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToObjRF = GetArray2vtkMatrix(T_CamToMetalBallRF);
	verifyPoint(vtkMatrixFromT_CamToObjRF, proximalStaplePoint);

}
void HTOrobot::collectDistalStaplePoint()
{
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToObjRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToObjRF = GetArray2vtkMatrix(T_CamToPatientRF);
	verifyPoint(vtkMatrixFromT_CamToObjRF, distalStaplePoint);

}
void HTOrobot::collectSawStaplePoint()
{
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToObjRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToObjRF = GetArray2vtkMatrix(T_CamToSaw);
	verifyPoint(vtkMatrixFromT_CamToObjRF, SawTestPoint);

}
void HTOrobot::verifyPoint(vtkSmartPointer<vtkMatrix4x4> T_CamToRF, mitk::Point3D& outputPoint)
{
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToProbe = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToProbe = GetArray2vtkMatrix(T_CamToProbe);

	auto tmp_transform = vtkTransform::New();
	vtkSmartPointer<vtkMatrix4x4> m_aimToObjectRfInverse = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(T_CamToRF, m_aimToObjectRfInverse);
	tmp_transform->SetMatrix(m_aimToObjectRfInverse);

	auto tmpmatrix = tmp_transform->GetMatrix();
	double inputPoint[4] = { Cam_ProbeTop[0], Cam_ProbeTop[1], Cam_ProbeTop[2], 1.0 };
	QString outputText5 = "probe Point on Cam:\n";
	for (int i = 0; i < 3; ++i)
	{
		outputText5 += QString::number(Cam_ProbeTop[i]) + "\t";

	}
	outputText5 += "\n";
	m_Controls.textBrowser_HTO->append(outputText5);
	double probe_outputPoint[4] = { 0.0, 0.0, 0.0, 0.0 };
	tmp_transform->MultiplyPoint(inputPoint, probe_outputPoint);

	outputPoint[0] = probe_outputPoint[0];
	outputPoint[1] = probe_outputPoint[1];
	outputPoint[2] = probe_outputPoint[2];
	QString outputText = "probe Point on FobjRf:\n";
	for (int i = 0; i < 3; ++i)
	{
		outputText += QString::number(probe_outputPoint[i]) + "\t";

	}
	outputText += "\n";
	m_Controls.textBrowser_HTO->append(outputText);
	
}

void HTOrobot::sawStlRegistration()
{

	mitk::DataStorage::Pointer dataStorage = GetDataStorage();
	mitk::PointSet::Pointer m_landmarks = dynamic_cast<mitk::PointSet*>(dataStorage->GetNamedNode("SawPointSet")->GetData());
	m_Controls.textBrowser_output->append("-------------SawPointSet-------------");
	if (m_landmarks.IsNotNull())
	{
		// get size of PointSet
		unsigned int numberOfPoints = m_landmarks->GetSize();
		for (unsigned int i = 0; i < numberOfPoints; ++i)
		{
			// get position
			mitk::PointSet::PointType point = m_landmarks->GetPoint(i);

			QString pointText = "SawPointSet: x=" + QString::number(point[0]) +
				", y=" + QString::number(point[1]) + ", z=" + QString::number(point[2]);
			m_Controls.textBrowser_output->append(pointText);
		}
	}
	
	//锯片三点应用图像配准矩阵转移到image坐标系下的点
	if (!pointOnSawRF_onImage) {
		return;
	}
	mitk::PointSet::Pointer m_landmarks_probe = dynamic_cast<mitk::PointSet*>(dataStorage->GetNamedNode("SawRFpointSet_toImage")->GetData());
	
	//mitk::PointSet::Pointer m_landmarks_probe = mitk::PointSet::New();


	QString pointText2 = "----------SawRFpointSet_toImage:\n";
	m_Controls.textBrowser_output->append(pointText2);
	if (m_landmarks_probe.IsNotNull())
	{
		unsigned int numberOfPoints = m_landmarks_probe->GetSize();
		for (unsigned int i = 0; i < numberOfPoints; ++i)
		{
			mitk::PointSet::PointType point = m_landmarks_probe->GetPoint(i);

			
			 pointText2 += "SawRFpointSet_toImage: x=" + QString::number(point[0]) +
				 ", y=" + QString::number(point[1]) + ", z=" + QString::number(point[2]);
			m_Controls.textBrowser_output->setText(pointText2);
		}
		
	}

	//将图像坐标系中锯片stl模型与应用图像配准矩阵导入影像空间坐标系中的锯片上三点坐标进行配准对齐，求解R、T
	//组合成齐次变换矩阵
	
	m_vtkregistrationMatrix_Saw = ComputeTransformMartix(m_landmarks, m_landmarks_probe);
	QString outputText_new = "m_vtkregistrationMatrix_Saw:\n";


	for (int row = 0; row < 4; ++row)
	{
		for (int col = 0; col < 4; ++col)
		{
			outputText_new += QString::number(m_vtkregistrationMatrix_Saw->GetElement(row, col)) + "\t";
		}
		outputText_new += "\n";
	}
	m_Controls.textBrowser_output->append(outputText_new);
	

	//image中锯片模型上应用转换矩阵m_vtkregistrationMatrix_Saw
	

	auto mitkSawStl = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("Saw")->GetData());
	auto tmpVtkSurface_initial = mitkSawStl->GetVtkPolyData();
	vtkNew<vtkPolyData> tmpVtkSurface;
	tmpVtkSurface->DeepCopy(tmpVtkSurface_initial);

	vtkNew<vtkTransform> SawTransform;
	SawTransform->SetMatrix(m_vtkregistrationMatrix_Saw);
	vtkNew<vtkTransformFilter> cutPlaneTransformFilter;
	cutPlaneTransformFilter->SetTransform(SawTransform);
	cutPlaneTransformFilter->SetInputData(tmpVtkSurface);
	cutPlaneTransformFilter->Update();
	auto sawSurface1 = mitk::Surface::New();
	sawSurface1->SetVtkPolyData(cutPlaneTransformFilter->GetPolyDataOutput());
	
	if (!sawNode1)
	{
		qDebug() << "sawNode1";
		sawNode1 = mitk::DataNode::New();
		sawNode1->SetName("SawPlane");
		sawNode1->SetData(sawSurface1);
		dataStorage->Add(sawNode1);
		dataStorage->Modified();
	}
	else
	{
		
		sawNode1->SetData(sawSurface1);      // Add poinset into datanode
		sawNode1->Modified();
	}
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	mitk::RenderingManager::GetInstance()->ForceImmediateUpdateAll();
	
}
//get point on SawRF.Saw calibrator;
void HTOrobot::getPointOnSawRF()
{
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToProbe = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToProbe = GetArray2vtkMatrix(T_CamToProbe);
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToSaw = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToSaw = GetArray2vtkMatrix(T_CamToSaw);
	//用探针采集锯片上的标记点，将其转换到Saw的Marker下
	auto tmp_transform = vtkTransform::New();
	vtkSmartPointer<vtkMatrix4x4> m_aimToObjectRfInverse = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(vtkMatrixFromT_CamToSaw, m_aimToObjectRfInverse);
	tmp_transform->SetMatrix(m_aimToObjectRfInverse);

	auto tmpmatrix = tmp_transform->GetMatrix();
	double inputPoint[4] = { Cam_ProbeTop[0], Cam_ProbeTop[1], Cam_ProbeTop[2], 1.0 };
	QString outputText5 = "probe Point on Cam:\n";
	for (int i = 0; i < 3; ++i)
	{
		outputText5 += QString::number(Cam_ProbeTop[i]) + "\t";

	}
	outputText5 += "\n";
	m_Controls.textBrowser_HTO->append(outputText5);
	double probe_outputPoint[4] = { 0.0, 0.0, 0.0, 0.0 };
	tmp_transform->MultiplyPoint(inputPoint, probe_outputPoint);
	// 将probe_outputPoint的前三个元素赋值给全局变量
	PointOnSawRF[0] = probe_outputPoint[0];
	PointOnSawRF[1] = probe_outputPoint[1];
	PointOnSawRF[2] = probe_outputPoint[2];

	QString outputText = "probe Point on FobjRf:\n";
	for (int i = 0; i < 3; ++i)
	{
		outputText += QString::number(PointOnSawRF[i]) + "\t";

	}
	outputText += "\n";
	m_Controls.textBrowser_HTO->append(outputText);

	vtkProbeTip_onObjRf->InsertNextPoint(probe_outputPoint[0], probe_outputPoint[1], probe_outputPoint[2]);
	clickCount++;

	if (clickCount == 4) {
		// 当点击次数达到3次时执行的操作
		qDebug() << "Button has been clicked three times!";
		// 重置计数器
		
		getSawError();
		clickCount = 0;
	}

}
void HTOrobot::getSawError()
{
	
	mitk::PointSet::Pointer  pointSet_Src = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("sawPointSet")->GetData());
	auto imageCheckPoint = pointSet_Src->GetPoint(0);
	if (pointSet_Src.IsNotNull())
	{
		unsigned int numberOfPoints = pointSet_Src->GetSize();
		for (unsigned int i = 0; i < numberOfPoints; ++i)
		{
			// get position
			mitk::PointSet::PointType point = pointSet_Src->GetPoint(i);

			QString pointText = QString("Point %1: (%2, %3, %4)\n").arg(i + 1).arg(point[0]).arg(point[1]).arg(point[2]);
			m_Controls.textBrowser_HTO->append(pointText);
		}
	}

	//获取采集的点
	mitk::PointSet::Pointer pointSet_Tar = mitk::PointSet::New();
	vtkIdType numPoints = vtkProbeTip_onObjRf->GetNumberOfPoints();
	for (vtkIdType i = 0; i < numPoints; ++i)
	{
		double point[3];
		vtkProbeTip_onObjRf->GetPoint(i, point);

		mitk::PointSet::PointType mitkPoint;
		mitkPoint[0] = point[0];
		mitkPoint[1] = point[1];
		mitkPoint[2] = point[2];
		pointSet_Tar->SetPoint(i, mitkPoint);
	}

	if (pointSet_Tar.IsNotNull())
	{
		unsigned int numberOfPoints = pointSet_Tar->GetSize();

		for (unsigned int i = 0; i < numberOfPoints; ++i)
		{
			mitk::PointSet::PointType point = pointSet_Tar->GetPoint(i);

			QString pointText2 = QString("Point %1: (%2, %3, %4)\n").arg(i + 1).arg(point[0]).arg(point[1]).arg(point[2]);
			m_Controls.textBrowser_HTO->append(pointText2);
		}
	}
	//计算Tsawtoimage
	auto vtkT_sawRFtoImage = ComputeTransformMartix(pointSet_Tar, pointSet_Src);
	//应用该矩阵，计算两组点的误差；
	vtkSmartPointer<vtkPoints>vtkProbeTip_onObjRfInImage = vtkSmartPointer<vtkPoints>::New();
	vtkNew<vtkTransform> tmpTransform;
	tmpTransform->SetMatrix(vtkT_sawRFtoImage);

	tmpTransform->TransformPoints(vtkProbeTip_onObjRf, vtkProbeTip_onObjRfInImage);
	tmpTransform->Update();
	/*memcpy_s(T_SawRFtoImage, sizeof(double) * 16, vtkT_sawRFtoImage->GetData(), sizeof(double) * 16);*/
	double distance = sqrt(pow(vtkProbeTip_onObjRfInImage->GetPoint(0)[0] - imageCheckPoint[0], 2) +
		pow(vtkProbeTip_onObjRfInImage->GetPoint(0)[1] - imageCheckPoint[1], 2) +
		pow(vtkProbeTip_onObjRfInImage->GetPoint(0)[2] - imageCheckPoint[2], 2));
	if (distance > 1)
	{
		m_Controls.LineEdit_sawError->setText(tr("Calibration error:>")+QString::number(distance));
	}
	else {
		m_Controls.LineEdit_sawError->setText(QString::number(distance));
	}
	
}

//SawNavigation on MetalRF;
void HTOrobot::startSawPositionCapture()
{
	if (m_timer_saw == nullptr)
	{
		m_timer_saw = new QTimer(this); 
	}

	connect(m_timer_saw, SIGNAL(timeout()), this, SLOT(captureSawPosition()));
	
	m_timer_saw->start(100);

	qDebug() << "Mouse position capture started.";

}
void HTOrobot::captureSawPosition()
{
	pointOnSawRF_onImage->Reset();
	if (!m_PointSet_saw)
	{
		m_PointSet_saw = mitk::PointSet::New();
	}
	ReadPositionSawOnMetalBallRF_realtime();
	auto transform = vtkTransform::New();

	vtkSmartPointer<vtkMatrix4x4> m_vtkregistrationMatrixInverse = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(T_imageFrameToSteelballRF, m_vtkregistrationMatrixInverse);
	transform->SetMatrix(m_vtkregistrationMatrixInverse);
	mitk::Point3D transformedPoint;
	QString transformedPointText;

	m_Controls.textBrowser_HTO->clear();
	m_PointSet_saw->Clear();
	transform->TransformPoints(pointOnSawRF_onMetalRF, pointOnSawRF_onImage);
	
	for (vtkIdType i = 0; i < pointOnSawRF_onImage->GetNumberOfPoints(); ++i)
	{
		transformedPoint=pointOnSawRF_onImage->GetPoint(i);

		transformedPointText = "Transformed Point: x=" + QString::number(transformedPoint[0]) +
			", y=" + QString::number(transformedPoint[1]) + ", z=" + QString::number(transformedPoint[2])+"\n";
		m_Controls.textBrowser_HTO->append(transformedPointText);
		// Insert the point into the PointSet's first point position
		m_PointSet_saw->SetPoint(i, transformedPoint);
	}
	
	
	if (!saw_DataNode)
	{
		saw_DataNode = mitk::DataNode::New();
		saw_DataNode->SetName("SawRFpointSet_toImage");
		saw_DataNode->SetColor(1.0, 0.0, 0.0);                                 
		saw_DataNode->SetData(m_PointSet_saw);                                
		saw_DataNode->SetProperty("pointsize", mitk::FloatProperty::New(10.0)); 
		mitk::DataStorage::Pointer dataStorage = this->GetDataStorage();
		dataStorage->Add(saw_DataNode);
		dataStorage->Modified();
		qDebug() << "saw_DataNode";
	}
	else
	{
		
		saw_DataNode->SetData(m_PointSet_saw);      // Add poinset into datanode
		saw_DataNode->Modified();
		
	}

	

	// Update the MITK rendering
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	mitk::RenderingManager::GetInstance()->ForceImmediateUpdateAll();
	sawStlRegistration();
	navigationSawCut();
}

//get pointOnSawRF On MetalBallRF;
void HTOrobot::ReadPositionSawOnMetalBallRF_realtime()
{
	pointOnSawRF_onMetalRF->Reset();
	
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToSaw = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToSaw = GetArray2vtkMatrix(T_CamToSaw);
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToMetalBallRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToMetalBallRF = GetArray2vtkMatrix(T_CamToMetalBallRF);
	vtkSmartPointer<vtkMatrix4x4> m_CamToMetalBallRFInverse = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(vtkMatrixFromT_CamToMetalBallRF, m_CamToMetalBallRFInverse);
	//将摆锯末端marker下的锯片上的标记点转化到MetalRF下 
	auto tmp_transform = vtkTransform::New();
	tmp_transform->PostMultiply();
	tmp_transform->SetMatrix(vtkMatrixFromT_CamToSaw);
	tmp_transform->Concatenate(m_CamToMetalBallRFInverse);
	tmp_transform->Update();
	tmp_transform->TransformPoints(vtkProbeTip_onObjRf, pointOnSawRF_onMetalRF);

	QString outputText3 = "PointOnSawRF on MetalRF:\n";
	qDebug() << "PointOnSawRF on MetalRF numbers:" << pointOnSawRF_onMetalRF->GetNumberOfPoints();
	for (vtkIdType i = 0; i < pointOnSawRF_onMetalRF->GetNumberOfPoints(); ++i)
	{
		double point[3];
		pointOnSawRF_onMetalRF->GetPoint(i, point);
		outputText3 += "PointOnSawRF on MetalRF numbers:"+QString::number(point[i]) + "\t";
	}
	outputText3 += "\n";
	m_Controls.textBrowser_HTO->append(outputText3);
}

void HTOrobot::getPointOnPatientRF()
{
	getStrechT();
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToPatientRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToPatientRF = GetArray2vtkMatrix(T_CamToPatientRF);
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToMetalBallRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToMetalBallRF = GetArray2vtkMatrix(T_CamToMetalBallRF);

	mitk::PointSet::Pointer pointONLateral = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("tibiaMalleolusPointSet")->GetData());
	auto point1 = pointONLateral->GetPoint(0);
	auto point2 = pointONLateral->GetPoint(1);
	mitk::Point3D ankleCenter;
	ankleCenter[0] = (point1[0] + point2[0]) / 2;
	ankleCenter[1] = (point1[1] + point2[1]) / 2;
	ankleCenter[2] = (point1[2] + point2[2]) / 2;
	auto transform = vtkTransform::New();
	vtkSmartPointer<vtkMatrix4x4> T_CamToPatientRFInverse = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(vtkMatrixFromT_CamToPatientRF, T_CamToPatientRFInverse);
	transform->PostMultiply();
	transform->SetMatrix(T_imageFrameToSteelballRF); 
	transform->Concatenate(vtkMatrixFromT_CamToMetalBallRF);
	transform->Concatenate(T_CamToPatientRFInverse);
	transform->Update();
	double  ankleCenterOnImage[4] = { ankleCenter[0], ankleCenter[1],ankleCenter[2], 1.0 };
	transform->MultiplyPoint(ankleCenterOnImage, ankleCenterOnPatientRF);
	QString transformedPointText = "------------------ankleCenterOnPatientRF: x=" + QString::number(ankleCenterOnPatientRF[0]) +
		", y=" + QString::number(ankleCenterOnPatientRF[1]) + ", z=" + QString::number(ankleCenterOnPatientRF[2]);
	m_Controls.textBrowser_output->setText(transformedPointText);

}
void HTOrobot::getStrechT_realtime()
{
	m_Controls.textBrowser_output->clear();
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToPatientRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToPatientRF = GetArray2vtkMatrix(T_CamToPatientRF);
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToMetalBallRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToMetalBallRF = GetArray2vtkMatrix(T_CamToMetalBallRF);
	vtkSmartPointer<vtkMatrix4x4> m_CamToMetalBallRFInverse = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(vtkMatrixFromT_CamToMetalBallRF, m_CamToMetalBallRFInverse);
	vtkSmartPointer<vtkMatrix4x4> T_imageFrameToSteelballRFInverse = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(T_imageFrameToSteelballRF, T_imageFrameToSteelballRFInverse);
	auto MetalRFtoPatientRF_transform = vtkTransform::New();
	MetalRFtoPatientRF_transform->PostMultiply();
	MetalRFtoPatientRF_transform->SetMatrix(vtkMatrixFromT_CamToPatientRF);
	MetalRFtoPatientRF_transform->Concatenate(m_CamToMetalBallRFInverse);
	MetalRFtoPatientRF_transform->Concatenate(T_imageFrameToSteelballRFInverse);
	MetalRFtoPatientRF_transform->Update();

	MetalRFtoPatientRF_transform->MultiplyPoint(ankleCenterOnPatientRF, ankleCenterOnPatientRFtoImagePoint);
	MetalRFtoPatientRF_transform->MultiplyPoint(normalOnPatientRF, normalOnPatientRFtoImage);


	auto transform1 = vtkTransform::New();
	transform1->PostMultiply();
	transform1->SetMatrix(vtkMatrixFromT_CamToPatientRF);
	transform1->Concatenate(m_CamToMetalBallRFInverse);
	T_MetalRFtoPatientRF_realTime = transform1->GetMatrix();
	
}
void HTOrobot::getStrechT()
{
	m_Controls.textBrowser_output->clear();
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToPatientRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToPatientRF = GetArray2vtkMatrix(T_CamToPatientRF);
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToMetalBallRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToMetalBallRF = GetArray2vtkMatrix(T_CamToMetalBallRF);
	vtkSmartPointer<vtkMatrix4x4> m_CamToMetalBallRFInverse = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(vtkMatrixFromT_CamToMetalBallRF, m_CamToMetalBallRFInverse);
	
	auto MetalRFtoPatientRF_transform = vtkTransform::New();
	MetalRFtoPatientRF_transform->PostMultiply();
	MetalRFtoPatientRF_transform->SetMatrix(vtkMatrixFromT_CamToPatientRF);
	MetalRFtoPatientRF_transform->Concatenate(m_CamToMetalBallRFInverse);
	MetalRFtoPatientRF_transform->Update();
	T_MetalRFtoPatientRF = MetalRFtoPatientRF_transform->GetMatrix();//金属球阵列到胫骨柄阵列的转换矩阵
	

	vtkSmartPointer<vtkMatrix4x4> T_imageFrameToSteelballRFInverse = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(T_imageFrameToSteelballRF, T_imageFrameToSteelballRFInverse);
	auto tmp_transform = vtkTransform::New();
	tmp_transform->PostMultiply();
	tmp_transform->SetMatrix(T_MetalRFtoPatientRF);
	tmp_transform->Concatenate(T_imageFrameToSteelballRFInverse);
	tmp_transform->Update();
	T_PatientRFToimageFrame = tmp_transform->GetMatrix();//胫骨柄阵列到图像坐标系的转换矩阵
	
	

	QString outputText_new = "-------------T_PatientRFToimageFrame-------------:\n";

	for (int row = 0; row < 4; ++row)
	{
		for (int col = 0; col < 4; ++col)
		{
			outputText_new += QString::number(T_PatientRFToimageFrame->GetElement(row, col)) + "\t";
		}
		outputText_new += "\n";
	}
	m_Controls.textBrowser_output->append(outputText_new);

	QString outputText = "-------------T_MetalRFtoPatientRF-------------:\n";

	for (int row = 0; row < 4; ++row)
	{
		for (int col = 0; col < 4; ++col)
		{
			outputText += QString::number(T_MetalRFtoPatientRF->GetElement(row, col)) + "\t";
		}
		outputText += "\n";
	}
	m_Controls.textBrowser_output->append(outputText);
}

