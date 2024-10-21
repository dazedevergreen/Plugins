
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
#include <mitkRemeshing.h>
#include "mitkVtkInterpolationProperty.h"
#include "mitkSurfaceToImageFilter.h"
#include "mitkBoundingShapeCropper.h"
#include "mitkRotationOperation.h"
#include "mitkInteractionConst.h"
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
#include <vtkPlane.h>
#include <vtkClipPolyData.h>
#include <vtkFillHolesFilter.h>
#include <ep\include\vtk-9.1\vtkCutter.h>

void HTOrobot::startNavigationStrech()
{
	
	if (m_timer_strech == nullptr)
	{
		m_timer_strech = new QTimer(this); 
	}
	connect(m_timer_strech, SIGNAL(timeout()), this, SLOT(strechAngleNavigation()));


	
	m_timer_strech->start(100); 

	qDebug() << "Strech Angle navigation started.";

}



void HTOrobot::strechAngleNavigation()
{
	getStrechT_realtime();

	if (!m_PointSet_ankle)
	{
		m_PointSet_ankle = mitk::PointSet::New();
	}
	

	
	m_PointSet_ankle->Clear();

	QString outputText3 = "--------------ankleCenterOnPatientRF to Image------------:\n";
	
	outputText3 += QString::number(ankleCenterOnPatientRFtoImagePoint[0]) + "\t"+ QString::number(ankleCenterOnPatientRFtoImagePoint[1])+"\t"+ QString::number(ankleCenterOnPatientRFtoImagePoint[2]);
	
	outputText3 += "\n";
	m_Controls.textBrowser_output->setText(outputText3);
	m_PointSet_ankle->SetPoint(0, ankleCenterOnPatientRFtoImagePoint);
	if (!ankle_DataNode)
	{
		ankle_DataNode = mitk::DataNode::New();
		ankle_DataNode->SetName("ankleCenterOnPatientRFtoImagePoint");
		ankle_DataNode->SetColor(1.0, 0.0, 0.0);                                  
		ankle_DataNode->SetData(m_PointSet_ankle);                                
		ankle_DataNode->SetProperty("pointsize", mitk::FloatProperty::New(10.0)); 
		mitk::DataStorage::Pointer dataStorage = this->GetDataStorage();
		dataStorage->Add(ankle_DataNode);
		dataStorage->Modified();
		
	}
	else
	{

		ankle_DataNode->SetData(m_PointSet_ankle);      
		ankle_DataNode->Modified();

	}
	
	getStrechAngleAndHeight();
	distalTibia(); 
	updatePowerLine();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	mitk::RenderingManager::GetInstance()->ForceImmediateUpdateAll();

}

void HTOrobot::updatePowerLine()
{
	
	mitk::DataNode::Pointer femurCenter = GetDataStorage()->GetNamedNode("hipCenterPoint");
	auto tmpPointSet_1 = dynamic_cast<mitk::PointSet*>(femurCenter->GetData());
	auto point1 = tmpPointSet_1->GetPoint(0);
	mitk::Point3D anklecenterPointOnImage;
	anklecenterPointOnImage[0] = ankleCenterOnPatientRFtoImagePoint[0];
	anklecenterPointOnImage[1] = ankleCenterOnPatientRFtoImagePoint[1];
	anklecenterPointOnImage[2] = ankleCenterOnPatientRFtoImagePoint[2];
	mitk::PointSet::Pointer tmpPointSet_2 = mitk::PointSet::New();
	tmpPointSet_2->SetPoint(0, point1);
	tmpPointSet_2->SetPoint(1, anklecenterPointOnImage);
	
	mitk::DataNode::Pointer existingForceLineNode = GetDataStorage()->GetNamedNode("Line_realTime");
	if (existingForceLineNode.IsNotNull())
	{

		existingForceLineNode->SetData(tmpPointSet_2); 
	}
	else
	{
		
		mitk::DataNode::Pointer line = mitk::DataNode::New();
		line->SetData(tmpPointSet_2); 
		line->SetName("Line_realTime");
		line->SetBoolProperty("show contour", true);
		line->SetFloatProperty("contoursize", 1.0);
		GetDataStorage()->Add(line);

	}

	GetDataStorage()->Modified();
	updateProportation_ReallTime(existingForceLineNode);

}
void HTOrobot::updateProportation_ReallTime(mitk::DataNode::Pointer existingForceLineNode)
{
	
	//auto tmpPointSet= dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("Line_realTime")->GetData());
	auto tmpPointSet = dynamic_cast<mitk::PointSet*>(existingForceLineNode->GetData());
	auto imageCheckPoint_0 = tmpPointSet->GetPoint(0);
	auto imageCheckPoint_1 = tmpPointSet->GetPoint(1);
	auto tmpPointSet2 = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("tibiaCondylesPointSet")->GetData());
	auto Point_0 = tmpPointSet2->GetPoint(0);
	auto Point_1 = tmpPointSet2->GetPoint(1);
	double tibia_length = sqrt(pow(Point_0[0] - Point_1[0], 2));

	double line1ProjStart[3] = { imageCheckPoint_0[0], 0.0, imageCheckPoint_0[2] };
	double line1ProjEnd[3] = { imageCheckPoint_1[0], 0.0,imageCheckPoint_1[2] };
	double line2ProjStart[3] = { Point_0[0], 0.0, Point_0[2] };
	double line2ProjEnd[3] = { Point_1[0], 0.0, Point_1[2] };

	// Calculate the intersection of two line segments in two dimensions
	double intersection3D[3];
	if (calculateIntersection(line1ProjStart, line1ProjEnd, line2ProjStart, line2ProjEnd, intersection3D)) {
		std::cout << "Intersection in the coronal plane: (" << intersection3D[0] << ", " << intersection3D[1] << ", " << intersection3D[2] << ")" << std::endl;
		//ÏÔÊ¾Õ¼±È
		double propatation = sqrt(pow(intersection3D[0] - Point_0[0], 2)) / tibia_length;
		propatation = ceil(propatation * 100);
		m_Controls.LineEdit_proportationRealTime->setText(QString::number(propatation));
	}
	else {

		std::cout << "The lines do not intersect in the coronal plane." << std::endl;
		m_Controls.LineEdit_proportationRealTime->setText(QString::number(0));
	}
	double vx1 = Point_1[0] - Point_0[0];
	double vy1 = Point_1[1] - Point_0[1];
	double vz1 = Point_1[2] - Point_0[2];
	double length1 = sqrt(pow(vx1, 2) + pow(vy1, 2) + pow(vz1, 2));
	double ux1 = vx1 / length1;
	double uy1 = vy1 / length1;
	double uz1 = vz1 / length1;
	double normal1[3]{ ux1,uy1,uz1 };

	double vx = imageCheckPoint_1[0] - imageCheckPoint_0[0];
	double vy = imageCheckPoint_1[1] - imageCheckPoint_0[1];
	double vz = imageCheckPoint_1[2] - imageCheckPoint_0[2];
	double length = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));
	double ux = vx / length;
	double uy = vy / length;
	double uz = vz / length;
	double normal2[3]{ ux,uy,uz };
	// ¼ÆËãÁ½¸ö·½ÏòÏòÁ¿Ö®¼äµÄ¼Ð½Ç
	double dotProduct = normal1[0] * normal2[0] + normal1[1] * normal2[1] + normal1[2] * normal[2]; // µã»ý
	double mPTA = round(acos(dotProduct) * (180.0 / M_PI));
	m_Controls.LineEdit_caculateMPTA->setText(QString::number(mPTA));

}
void HTOrobot::getStrechAngleAndHeight()
{

	//Calculate spreading angle
	double dotProduct = normalOnPatientRFtoImage[0] * normal[0] + normalOnPatientRFtoImage[1] * normal[1]+normalOnPatientRFtoImage[2] * normal[2] ;
	double magnitudeA = sqrt(normalOnPatientRFtoImage[0] * normalOnPatientRFtoImage[0] + normalOnPatientRFtoImage[1] * normalOnPatientRFtoImage[1]+ normalOnPatientRFtoImage[2] * normalOnPatientRFtoImage[2]);
	double magnitudeB = sqrt(normal[0] * normal[0] + normal[1] * normal[1]+ normal[2] * normal[2]);


	if (magnitudeA == 0 || magnitudeB == 0) {
		std::cerr << "One of the vectors has zero magnitude. Cannot compute angle." << std::endl;
		return;
	}

	double cosTheta = dotProduct / (magnitudeA * magnitudeB);
	double angleInRadians = acos(cosTheta); 
	double angleInDegrees = round(angleInRadians * (180.0 / M_PI)); 
	m_Controls.LineEdit_strechAngle->setText(QString::number(angleInDegrees));
	double depth = m_Controls.LineEdit_sawDistance->text().toDouble();
	std::cout << "angleInDegrees: " << angleInDegrees << std::endl;
	double angle_rad = angleInDegrees * M_PI / 180.0;
	std::cout << "angle_rad: " << angle_rad << std::endl;
	double height = round(sin(angle_rad) * depth);
	m_Controls.LineEdit_strechHeight->setText(QString::number(height));
}
void HTOrobot::distalTibia()
{
	vtkSmartPointer<vtkMatrix4x4> T_MetalRFtoPatientRFInverse = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(T_MetalRFtoPatientRF, T_MetalRFtoPatientRFInverse);
	vtkSmartPointer<vtkMatrix4x4>T_PatientRFNewToPatientRFOld= vtkSmartPointer<vtkMatrix4x4>::New();
	vtkNew<vtkTransform> Transform1;
	Transform1->SetMatrix(T_MetalRFtoPatientRFInverse);
	Transform1->Concatenate(T_MetalRFtoPatientRF_realTime);
	T_PatientRFNewToPatientRFOld=Transform1->GetMatrix();


	vtkSmartPointer<vtkMatrix4x4> T_PatientRFToimageFrameInverse = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(T_PatientRFToimageFrame, T_PatientRFToimageFrameInverse);
	//½«Í¼ÏñÖÐµÄëÖ¹ÇÔ¶¶ËÄ£ÐÍÓ¦ÓÃT_PatientRFToimageFrame£»
	auto mitkdistaltibiaSurface = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("Distal Tibia")->GetData());
	auto tmpVtkSurface_initial = mitkdistaltibiaSurface->GetVtkPolyData();
	vtkNew<vtkPolyData> tmpVtkSurface;
	tmpVtkSurface->DeepCopy(tmpVtkSurface_initial);
	

	vtkNew<vtkTransform> Transform;
	Transform->SetMatrix(mitkdistaltibiaSurface->GetGeometry()->GetVtkMatrix());
	Transform->Concatenate(T_PatientRFToimageFrame);
	Transform->Concatenate(T_PatientRFNewToPatientRFOld);
	Transform->Concatenate(T_PatientRFToimageFrameInverse);

	auto distalTibiaSurface = mitk::Surface::New();
	distalTibiaSurface->SetVtkPolyData(tmpVtkSurface);
	distalTibiaSurface->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(Transform->GetMatrix());

	if (!distalTibiaNode)
	{
		distalTibiaNode = mitk::DataNode::New();
		distalTibiaNode->SetName("distalTibiaNode");
		distalTibiaNode->SetData(distalTibiaSurface);
		GetDataStorage()->Add(distalTibiaNode);
		GetDataStorage()->Modified();
		qDebug() << "distalTibia2";
	}
	else
	{
		qDebug() << "distalTibia3";

		distalTibiaNode->SetData(distalTibiaSurface);      
		distalTibiaNode->Modified();
	}
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	mitk::RenderingManager::GetInstance()->ForceImmediateUpdateAll();
}


void HTOrobot::navigationSawCut()
{
	//Obtain the normal vector of the sawplane in image space and the normal vector of the planning horizontal osteotomy plane;
	auto mitkCutPlane_1 = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("1st cut plane")->GetData());

	auto planCutPlane = mitkCutPlane_1->GetVtkPolyData();

	
	vtkNew<vtkTransform> cutPlaneTransform;
	cutPlaneTransform->SetMatrix(mitkCutPlane_1->GetGeometry()->GetVtkMatrix());
	vtkNew<vtkTransformFilter> cutPlaneTransformFilter;
	cutPlaneTransformFilter->SetTransform(cutPlaneTransform);
	cutPlaneTransformFilter->SetInputData(planCutPlane);
	cutPlaneTransformFilter->Update();

	vtkNew<vtkPolyData> tmpVtkSurface;
	tmpVtkSurface->DeepCopy(cutPlaneTransformFilter->GetPolyDataOutput());

	double surfaceNormal[3];
	double cutPlaneCenter[3];

	GetPlaneProperty(tmpVtkSurface, surfaceNormal, cutPlaneCenter);
	Eigen::Vector3d Normal(surfaceNormal[0], surfaceNormal[1], surfaceNormal[2]);
	Normal.normalize();
	//saw stl Normal
	vtkIdType numPoints = pointOnSawRF_onImage->GetNumberOfPoints();
	Eigen::Vector3d point1(pointOnSawRF_onImage->GetPoint(0)[0], pointOnSawRF_onImage->GetPoint(0)[1], pointOnSawRF_onImage->GetPoint(0)[2]);
	Eigen::Vector3d point2(pointOnSawRF_onImage->GetPoint(1)[0], pointOnSawRF_onImage->GetPoint(1)[1], pointOnSawRF_onImage->GetPoint(1)[2]);
	Eigen::Vector3d point3(pointOnSawRF_onImage->GetPoint(2)[0], pointOnSawRF_onImage->GetPoint(2)[1], pointOnSawRF_onImage->GetPoint(2)[2]);

	Eigen::Vector3d vector1 = point2 - point1;
	Eigen::Vector3d vector2 = point3 - point1;

	Eigen::Vector3d axis_plan = vector2.cross(vector1);
	axis_plan.normalize();
	double dotProduct = Normal.dot(axis_plan); 

	double cosTheta = dotProduct;
	double angleRad = acos(cosTheta);
	double angleDeg = angleRad * (180.0 / M_PI); //Get the angle between the sawstl and the plan surface;
	mitk::PointSet::Pointer  planeAndTibiaIntersectionPoint = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("planeAndTibiaIntersectionPointSet")->GetData());
	mitk::Point3D kerfPoint;
	mitk::Point3D vergePoint;
	
	double distance;//Distance of the current saw blade from the osteotomy entrance
	double cutDepth;
	double retainHinge;
	vergePoint = planeAndTibiaIntersectionPoint->GetPoint(0);//Get the osteotomy and model the diplomatic points
	kerfPoint=planeAndTibiaIntersectionPoint->GetPoint(1);//Get the intersection of the osteotomy and the model.

	if (judgModel_flag = 1)
	{
		if (kerfPoint[0] >(pointOnSawRF_onImage->GetPoint(0)[0] - 10))
		{
			//The distance between the first point on the saw blade and the entry point of the horizontal planning surface;
			cutDepth = abs( (kerfPoint[0]-pointOnSawRF_onImage->GetPoint(0)[0] - 10)  );
			m_Controls.LineEdit_cutTibiaDepth->setText(QString::number(cutDepth));
			retainHinge = abs(vergePoint[0] - ((pointOnSawRF_onImage->GetPoint(0)[0] - 10)));
		}
		else
		{
			distance = abs(kerfPoint[0] - (pointOnSawRF_onImage->GetPoint(0)[0] - 10));
			m_Controls.LineEdit_sawDistance->setText(QString::number(distance));
			
		}
		 
	}
	else
	{
		if(vergePoint[0] < (pointOnSawRF_onImage->GetPoint(0)[0] + 10))
		{
			cutDepth = abs((pointOnSawRF_onImage->GetPoint(0)[0] +10)-vergePoint[0]);
			m_Controls.LineEdit_cutTibiaDepth->setText(QString::number(cutDepth));
			retainHinge = abs(kerfPoint[0] - ((pointOnSawRF_onImage->GetPoint(0)[0] + 10)));
        }
		else{
			distance = abs(vergePoint[0] - (pointOnSawRF_onImage->GetPoint(0)[0] + 10));
			m_Controls.LineEdit_sawDistance->setText(QString::number(distance));
		}
	}

	if (retainHinge < 15)
	{
		m_Controls.lineEdit_warn->setText("Saw blade osteotomy boundary protection warning:<20mm");
	}
	else
	{
		m_Controls.lineEdit_warn->clear();
		m_Controls.LineEdit_sawAngle->setText(QString::number(angleDeg));
	}
	

}

void HTOrobot::navigationCutTibia()
{
	//sawSTL Nomal

	Eigen::Vector3d point1(pointOnSawRF_onImage->GetPoint(0)[0], pointOnSawRF_onImage->GetPoint(0)[1], pointOnSawRF_onImage->GetPoint(0)[2]);
	Eigen::Vector3d point2(pointOnSawRF_onImage->GetPoint(1)[0], pointOnSawRF_onImage->GetPoint(1)[1], pointOnSawRF_onImage->GetPoint(1)[2]);
	Eigen::Vector3d point3(pointOnSawRF_onImage->GetPoint(2)[0], pointOnSawRF_onImage->GetPoint(2)[1], pointOnSawRF_onImage->GetPoint(2)[2]);
	Eigen::Vector3d vector1 = point2 - point1;
	Eigen::Vector3d vector2 = point3 - point1;

	Eigen::Vector3d axis_plan = vector1.cross(vector2);
	axis_plan.normalize();

	double origin_0[3];
	origin_0[0] = point1[0];
	origin_0[1] = point1[1];
	origin_0[2] = point1[2];

	normal[0] = axis_plan[0];
	normal[1] = axis_plan[1];
	normal[2] = axis_plan[2];

	
	bindNormalToTibiaRF();
	
	vtkSmartPointer<vtkPlane> cutPlane = vtkSmartPointer<vtkPlane>::New();
	cutPlane->SetNormal(normal); 
	cutPlane->SetOrigin(origin_0); 
	auto tibiaSurface = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("tibiaSurface")->GetData());

	auto initialTibiaPolyData = tibiaSurface->GetVtkPolyData();
	vtkNew<vtkTransform> tibiaTransform;
	tibiaTransform->SetMatrix(tibiaSurface->GetGeometry()->GetVtkMatrix());
	vtkNew<vtkTransformFilter> tmpFilter;
	tmpFilter->SetTransform(tibiaTransform);
	tmpFilter->SetInputData(initialTibiaPolyData);
	tmpFilter->Update();

	vtkNew<vtkPolyData> tibiaPolyData;
	tibiaPolyData->DeepCopy(tmpFilter->GetPolyDataOutput());
	
	vtkSmartPointer<vtkClipPolyData> clipper = vtkSmartPointer<vtkClipPolyData>::New();
	clipper->SetInputData(tibiaPolyData);
	clipper->SetClipFunction(cutPlane);
	clipper->InsideOutOn(); //proximal tibia
	clipper->Update();
	vtkSmartPointer<vtkPolyData> clippedTibiaPolyData=clipper->GetOutput();
	vtkSmartPointer<vtkFillHolesFilter> fillHoles = vtkSmartPointer<vtkFillHolesFilter>::New();

	fillHoles->SetHoleSize(500);
	fillHoles->SetInputData(clippedTibiaPolyData);
	fillHoles->Update();
	mitk::Surface::Pointer mitkSurface = mitk::Surface::New();
	mitkSurface->SetVtkPolyData(fillHoles->GetOutput());



	mitk::DataNode::Pointer clippedTibiaNode;
	if (!clippedTibiaNode) 
	{
		clippedTibiaNode = mitk::DataNode::New();
		clippedTibiaNode->SetName("proximal tibia"); 
		clippedTibiaNode->SetData(mitkSurface); 
		clippedTibiaNode->SetColor(0.0, 1.0, 0.0);
		GetDataStorage()->Add(clippedTibiaNode);

	}
	else
	{
		clippedTibiaNode->SetData(mitkSurface); 
		GetDataStorage()->Modified();
	}


	vtkSmartPointer<vtkClipPolyData> clipperExternal = vtkSmartPointer<vtkClipPolyData>::New();
	clipperExternal->SetInputData(initialTibiaPolyData);
	clipperExternal->SetClipFunction(cutPlane);
	clipperExternal->InsideOutOff(); //distal tibia
	clipperExternal->Update();


	vtkSmartPointer<vtkPolyData> clippedTibiaPolyDataExternal = clipperExternal->GetOutput();
	
	vtkSmartPointer<vtkFillHolesFilter> fillHoles1 = vtkSmartPointer<vtkFillHolesFilter>::New();
	fillHoles1->SetHoleSize(500);
	fillHoles1->SetInputData(clippedTibiaPolyDataExternal);
	fillHoles1->Update();
	mitk::Surface::Pointer mitkSurfaceExternal = mitk::Surface::New();
	mitkSurfaceExternal->SetVtkPolyData(fillHoles1->GetOutput());

	mitk::DataNode::Pointer clippedTibiaNodeExternal;
	if (!clippedTibiaNodeExternal)
	{
		clippedTibiaNodeExternal = mitk::DataNode::New();
		clippedTibiaNodeExternal->SetName("Distal Tibia"); 
		clippedTibiaNodeExternal->SetData(mitkSurfaceExternal); 
		clippedTibiaNode->SetColor(0.0, 0.0, 1.0);
		GetDataStorage()->Add(clippedTibiaNodeExternal); 

	}
	else
	{
		clippedTibiaNodeExternal->SetData(mitkSurfaceExternal); 

		GetDataStorage()->Modified();
	}


	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	mitk::RenderingManager::GetInstance()->ForceImmediateUpdateAll();


}

void HTOrobot::bindNormalToTibiaRF()
{
	double normalOnSawInImage[4] = { normal[0],normal[1],normal[2],1.0 };
	
	/*QString transformedPointText = "------------------normalOnSaw: x=" + QString::number(normalOnSaw[0]) +
		", y=" + QString::number(normalOnSaw[1]) + ", z=" + QString::number(normalOnSaw[2]);
	m_Controls.textBrowser_output->append(transformedPointText);*/
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToPatientRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToPatientRF = GetArray2vtkMatrix(T_CamToPatientRF);
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToMetalBallRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToMetalBallRF = GetArray2vtkMatrix(T_CamToMetalBallRF);
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToSaw = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToSaw = GetArray2vtkMatrix(T_CamToSaw);
	vtkSmartPointer<vtkMatrix4x4> m_CamToPatientRFInverse = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(vtkMatrixFromT_CamToPatientRF, m_CamToPatientRFInverse);

	auto tmp_transform = vtkTransform::New();
	tmp_transform->PostMultiply();
	tmp_transform->SetMatrix(T_imageFrameToSteelballRF);
	tmp_transform->Concatenate(vtkMatrixFromT_CamToMetalBallRF);
	tmp_transform->Concatenate(m_CamToPatientRFInverse);
	tmp_transform->Update();
	tmp_transform->MultiplyPoint(normalOnSawInImage, normalOnPatientRF);// get normalOnMetalRF


	
	QString transformedPointText1 = "------------------normalOnPatientRF: x=" + QString::number(normalOnPatientRF[0]) +
		", y=" + QString::number(normalOnPatientRF[1]) + ", z=" + QString::number(normalOnPatientRF[2]);
	m_Controls.textBrowser_output->append(transformedPointText1);
}
void HTOrobot::modifyPointCoordinate()
{
	/*Eigen::Vector3d P1(0, 0, 0);
	Eigen::Vector3d P2(-0.57,-7,12.5);
	Eigen::Vector3d translationVector = P2 - P1;
	mitk::Point3D vector;
	vector[0] = translationVector[0];
	vector[1] = translationVector[1];
	vector[2] = translationVector[2];
	qDebug()<<"------------------translationVector: x="<<  QString::number(translationVector[0]) <<
		", y=" <<QString::number(translationVector[1]) << ", z=" <<QString::number(translationVector[2]);
	double sourcePointDesignValues[42]
	{
		14.89, 0, 6,
		0, 0, 0,
		-20, 0, 0,
		-0.57, -7, 12.5,
		33.14,-1.56,5.43,
		34.56,-3.38,9.33,
		31.45,-2.77,0.78,
		-14.89,-18.90,6,
		0,-18.90,0,
		-20,-18.90,0,
		-0.57,-11.90,12.50,
		34.057,-18.07,8.13,
		32.57,-15.29,4.05,
		30.96,-17.19,-0.385,
	};
	mitk::PointSet::Pointer PointSetOnDaoban = mitk::PointSet::New();
	mitk::DataNode::Pointer pointSetNode = mitk::DataNode::New();
	
	for (int i{ 0 }; i < 14; i++)
	{
		double tmpDouble[3]{ sourcePointDesignValues[3 * (i)],sourcePointDesignValues[3 * (i)+1],sourcePointDesignValues[3 * (i)+2] };
		mitk::Point3D tmpPoint{ tmpDouble };
		mitk::Point3D tmpPoint1;
		tmpPoint1[0] = tmpPoint[0] - vector[0];
		tmpPoint1[1] = tmpPoint[1] - vector[1];
		tmpPoint1[2] = tmpPoint[2] - vector[2];

		PointSetOnDaoban->InsertPoint(tmpPoint1);
	}


	mitk::DataNode::Pointer pointSetNode1 = mitk::DataNode::New();
	pointSetNode1->SetData(PointSetOnDaoban);
	pointSetNode1->SetName("PointSetOnDaoban");
	GetDataStorage()->Add(pointSetNode1);*/
	//Eigen::Vector3d n_x (1, 0, 0);
	//Eigen::Vector3d n_y(0, 1, 0);
	//Eigen::Vector3d n_z(0, 0, 1);
	//Eigen::Vector3d orgin(0.57, 7, -12.5);
	//vtkSmartPointer<vtkMatrix4x4> coordinate = vtkSmartPointer<vtkMatrix4x4>::New();
	//CreateCoordinateSystemMatrix(n_x, n_y, n_z, orgin, coordinate);
	//auto daobanStl = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("daoban")->GetData());
	//auto tmpVtkSurface_initial = daobanStl->GetVtkPolyData();
	//vtkNew<vtkPolyData> daobanVtkSurface;
	//daobanVtkSurface->DeepCopy(tmpVtkSurface_initial);

	//// transform tmpVtkSurface
	//vtkNew<vtkTransform> daobanTransform;
	//daobanTransform->PostMultiply();
	//daobanTransform->SetMatrix(daobanStl->GetGeometry()->GetVtkMatrix());
	//daobanTransform->Concatenate(coordinate);
	//daobanTransform->Update();
	//auto daobanSurface1 = mitk::Surface::New();
	//daobanSurface1->SetVtkPolyData(daobanVtkSurface);
	//daobanSurface1->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(daobanTransform->GetMatrix());
	////mitk::DataNode::Pointer daobanNode = GetDataStorage()->GetNamedNode("daoban");
	//mitk::DataNode::Pointer daobanNode;
	//if (!daobanNode)
	//{
	//	//GetDataStorage()->Remove(daobanNode);
	//	daobanNode = mitk::DataNode::New();
	//	daobanNode->SetName("daoban_copy");
	//	daobanNode->SetData(daobanSurface1);
	//	GetDataStorage()->Add(daobanNode);
	//	GetDataStorage()->Modified();
	//}
	//else
	//{
	//	daobanNode = mitk::DataNode::New();
	//	daobanNode->SetName("daoban_copy");
	//	daobanNode->SetData(daobanSurface1);      // Add poinset into datanode
	//	daobanNode->Modified();
	//}
}
void HTOrobot::getPointOnDaoban()
{
	double sourcePointDesignValues[42]
	{
		14.89, 0, 6,
		0, 0, 0,
		-20, 0, 0,
		-0.57, -7, 12.5,
		33.14,-1.56,5.43,
		34.56,-3.38,9.33,
		31.45,-2.77,0.78,
		-14.89,-18.90,6,
		0,-18.90,0,
		-20,-18.90,0,
		-0.57,-11.90,12.50,
		34.057,-18.07,8.13,
		32.57,-15.29,4.05,
		30.96,-17.19,-0.385,
	};
	mitk::PointSet::Pointer PointSetOnDaoban = mitk::PointSet::New();
	mitk::DataNode::Pointer pointSetNode = mitk::DataNode::New();

	for (int i{ 0 }; i < 14; i++)
	{
		double tmpDouble[3]{ sourcePointDesignValues[3 * (i)],sourcePointDesignValues[3 * (i)+1],sourcePointDesignValues[3 * (i)+2] };
		mitk::Point3D tmpPoint{ tmpDouble };
		PointSetOnDaoban->InsertPoint(tmpPoint);
	}


	mitk::DataNode::Pointer pointSetNode1 = mitk::DataNode::New();
	pointSetNode1->SetData(PointSetOnDaoban);
	pointSetNode1->SetName("DaobanPointSet");
	GetDataStorage()->Add(pointSetNode1);
}
bool HTOrobot::judgeTibiaShowPlanPosition()
{
	if (judgModel_flag==1)
	{
		showPlanCutPlanePosition();
		return true;

	}
	else
	{
		showPlanCutPlanePositionL();
		return true;
	}
	return false;
}
void HTOrobot::showPlanCutPlanePosition()
{
	getPointOnDaoban();
	auto  PointSetOnDaobanR = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("DaobanPointSet")->GetData());

	qDebug() << "------------------PointSetOnDaoban: x=" << QString::number(PointSetOnDaobanR->GetPoint(3)[0]) <<
		", y=" << QString::number(PointSetOnDaobanR->GetPoint(3)[1]) << ", z=" << QString::number(PointSetOnDaobanR->GetPoint(3)[2]);

	//Establish the coordinate system of the faces on the guide plate.
	Eigen::Vector3d point1(PointSetOnDaobanR->GetPoint(0)[0], PointSetOnDaobanR->GetPoint(0)[1], PointSetOnDaobanR->GetPoint(0)[2]);
	Eigen::Vector3d point2(PointSetOnDaobanR->GetPoint(1)[0], PointSetOnDaobanR->GetPoint(1)[1], PointSetOnDaobanR->GetPoint(1)[2]);
	Eigen::Vector3d point3(PointSetOnDaobanR->GetPoint(2)[0], PointSetOnDaobanR->GetPoint(2)[1], PointSetOnDaobanR->GetPoint(2)[2]);
	Eigen::Vector3d vector1 = point2 - point1;
	Eigen::Vector3d vector2 = point3 - point1;
	Eigen::Vector3d axis_z = -vector1.cross(vector2);
	axis_z.normalize();
	Eigen::Vector3d point4(PointSetOnDaobanR->GetPoint(4)[0], PointSetOnDaobanR->GetPoint(4)[1], PointSetOnDaobanR->GetPoint(4)[2]);
	Eigen::Vector3d point5(PointSetOnDaobanR->GetPoint(5)[0], PointSetOnDaobanR->GetPoint(5)[1], PointSetOnDaobanR->GetPoint(5)[2]);
	Eigen::Vector3d point6(PointSetOnDaobanR->GetPoint(6)[0], PointSetOnDaobanR->GetPoint(6)[1], PointSetOnDaobanR->GetPoint(6)[2]);
	Eigen::Vector3d vector3 = point5 - point4;
	Eigen::Vector3d vector4 = point6 - point4;
	Eigen::Vector3d axis_y = vector3.cross(vector4);
	//Eigen::Vector3d axis_y = axis.cross(axis_z);
	axis_y.normalize();
	Eigen::Vector3d axis_x = axis_z.cross(axis_y);
	axis_x.normalize();

	Eigen::Vector3d originDaobanR(37.26, -7, 12.5);

	vtkSmartPointer<vtkMatrix4x4> daoBanCoordinateR = vtkSmartPointer<vtkMatrix4x4>::New();
	CreateCoordinateSystemMatrix(axis_x, axis_y, axis_z, originDaobanR, daoBanCoordinateR);

	QString outputText_new="\n";
	for (int row = 0; row < 4; ++row)
	{
		for (int col = 0; col < 4; ++col)
		{
			outputText_new += QString::number(daoBanCoordinateR->GetElement(row, col)) + "\t";
		}
		outputText_new += "\n";
	}
	std::cout << "-----------daoBanCoordinate--------:"<<outputText_new << std::endl;
	
	// Establish the coordinate system of the plan surface


	auto planPlane = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("PlanCutPlane1PointSet")->GetData());
	
	double vx = planPlane->GetPoint(1)[0] - planPlane->GetPoint(2)[0];
	double vy = planPlane->GetPoint(1)[1] - planPlane->GetPoint(2)[1];
	double vz = planPlane->GetPoint(1)[2] - planPlane->GetPoint(2)[2];
	double length = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));
	Eigen::Vector3d origin(planPlane->GetPoint(2)[0], planPlane->GetPoint(2)[1], planPlane->GetPoint(2)[2]);
	double direction[3]{ vx / length,vy / length,vz / length };

	auto cutPlane = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("1st cut plane")->GetData());
	auto tmpVtkSurface_initial = cutPlane->GetVtkPolyData();

	// transform tmpVtkSurface
	vtkNew<vtkTransform> cutPlaneTransform;
	cutPlaneTransform->SetMatrix(cutPlane->GetGeometry()->GetVtkMatrix());
	vtkNew<vtkTransformFilter> cutPlaneTransformFilter;
	cutPlaneTransformFilter->SetTransform(cutPlaneTransform);
	cutPlaneTransformFilter->SetInputData(tmpVtkSurface_initial);
	cutPlaneTransformFilter->Update();

	vtkNew<vtkPolyData> tmpVtkSurface;
	tmpVtkSurface->DeepCopy(cutPlaneTransformFilter->GetPolyDataOutput());

	double surfaceNormal[3];
	double cutPlaneCenter[3];

	GetPlaneProperty(tmpVtkSurface, surfaceNormal, cutPlaneCenter);
	Eigen::Vector3d n_z(-surfaceNormal[0], -surfaceNormal[1], -surfaceNormal[2]);
	n_z.normalize();
	Eigen::Vector3d n_x(direction[0], direction[1],direction[2]);
	n_x.normalize();
	Eigen::Vector3d n_y = -n_z.cross(n_x);
	n_y.normalize();

	vtkSmartPointer<vtkMatrix4x4> PlanPlanecoordinate = vtkSmartPointer<vtkMatrix4x4>::New();
	CreateCoordinateSystemMatrix(n_x, n_y, n_z, origin, PlanPlanecoordinate);

	QString outputText="\n";
	for (int row = 0; row < 4; ++row)
	{
		for (int col = 0; col < 4; ++col)
		{
			outputText += QString::number(PlanPlanecoordinate->GetElement(row, col)) + "\t";
		}
		outputText += "\n";
	}
	std::cout <<"-----------PlanPlanecoordinate-------------"<< outputText << std::endl;
	
	// Calculate the transformation matrix between the two coordinate systems
	auto transform = vtkTransform::New();
	vtkSmartPointer<vtkMatrix4x4> daoBanCoordinateInverse = vtkSmartPointer<vtkMatrix4x4>::New();
	T_toolToPlanPlane = vtkSmartPointer<vtkMatrix4x4>::New();
	transform->PostMultiply();
	vtkMatrix4x4::Invert(daoBanCoordinateR, daoBanCoordinateInverse);
	transform->SetMatrix(daoBanCoordinateInverse);
	transform->Concatenate(PlanPlanecoordinate);
	
	transform->Update();
	T_toolToPlanPlane =transform->GetMatrix();
	QString outputText1="\n";
	for (int row = 0; row < 4; ++row)
	{
		for (int col = 0; col < 4; ++col)
		{
			outputText1 += QString::number(T_toolToPlanPlane->GetElement(row, col)) + "\t";
		}
		outputText1 += "\n";
	}
	std::cout << "-----------TransformMatrix-------------"<<outputText1 << std::endl;


	applyTransformMatrix(T_toolToPlanPlane);
}

void HTOrobot::guideToObjPosition()
{
	Eigen::Vector3d n_x (1, 0, 0);
	Eigen::Vector3d n_y(0, 1, 0);
	Eigen::Vector3d n_z(0, 0, 1);
	Eigen::Vector3d orgin(-0.57, -7, 12.5);
	vtkSmartPointer<vtkMatrix4x4> tcpCoordinate = vtkSmartPointer<vtkMatrix4x4>::New();
	CreateCoordinateSystemMatrix(n_x, n_y, n_z, orgin, tcpCoordinate);

	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_BaseToBaseRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_BaseToBaseRF = GetArray2vtkMatrix(T_BaseToBaseRF);
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToBaseRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToBaseRF = GetArray2vtkMatrix(T_CamToBaseRF);
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToMetalBallRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToMetalBallRF = GetArray2vtkMatrix(T_CamToMetalBallRF);
	vtkSmartPointer<vtkMatrix4x4> T_CamToBaseRFReverse = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(vtkMatrixFromT_CamToBaseRF, T_CamToBaseRFReverse);

	vtkSmartPointer<vtkMatrix4x4>T_toolToPlanPlaneReverse = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(T_toolToPlanPlane, T_toolToPlanPlaneReverse);

	vtkNew<vtkTransform> TransformBaseToTool;
	TransformBaseToTool->PreMultiply();
	TransformBaseToTool->SetMatrix(vtkMatrixFromT_BaseToBaseRF);
	TransformBaseToTool->Concatenate(T_CamToBaseRFReverse);

	TransformBaseToTool->Concatenate(vtkMatrixFromT_CamToMetalBallRF);




	vtkSmartPointer<vtkMatrix4x4> T_imageFrameToSteelballRFReverse = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(T_imageFrameToSteelballRF, T_imageFrameToSteelballRFReverse);
	TransformBaseToTool->Concatenate(T_imageFrameToSteelballRFReverse);

	//image 2 plane*plane2tool
	//TtoolToPlan=tool2image*image2plane
	//£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿
	//£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿
	//ÊÇ²»ÊÇÉÙÁËÒ»¸öflange2T_baseºÍtoolmatrix
	//»¹ÓÐimagetotool
	//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
	TransformBaseToTool->Concatenate(T_toolToPlanPlane);
	TransformBaseToTool->Concatenate(tcpCoordinate);
	TransformBaseToTool->Update();
	T_BaseToTool = vtkSmartPointer<vtkMatrix4x4>::New();
	T_BaseToTool =TransformBaseToTool->GetMatrix();
	
	QString outputText1 = "-----------TransformMatrix------------\n";
	// ±£´æ¾ØÕóµ½ÎÄ¼þ
	std::string filename = "C:/Users/zhu/Desktop/save/T_BaseToTool.txt";
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
			outputText1 += QString::number(T_BaseToTool->GetElement(row, col)) + "\t";
			m_Controls.textBrowser_HTO->append(outputText1);

			file << T_BaseToTool->GetElement(row, col) << " ,";
		}
		outputText1 += "\n";
		m_Controls.textBrowser_HTO->append("\n");
		file << "\n";
	}
	file.close();
	std::cout << "T_055 saved to file: " << filename << std::endl;
}

void HTOrobot::startNavigationGuidePosition()
{
	
	if (m_timer_navigationGuidePosition == nullptr)
	{
		m_timer_navigationGuidePosition = new QTimer(this); // create a new timer
	}
	
	connect(m_timer_navigationGuidePosition, SIGNAL(timeout()), this, SLOT(navigationGuidePosition()));


	// Set the interval for capturing mouse position (in milliseconds)
	m_timer_navigationGuidePosition->start(100); // Set the interval as needed (e.g., every second)

	qDebug() << "Mouse position capture started.";
	
}
void HTOrobot::navigationGuidePosition()
{
	Eigen::Vector3d n_x(1, 0, 0);
	Eigen::Vector3d n_y(0, 1, 0);
	Eigen::Vector3d n_z(0, 0, 1);
	Eigen::Vector3d orgin(-0.57, -7, 12.5);
	vtkSmartPointer<vtkMatrix4x4> tcpCoordinate = vtkSmartPointer<vtkMatrix4x4>::New();
	CreateCoordinateSystemMatrix(n_x, n_y, n_z, orgin, tcpCoordinate);
	tcpCoordinate->Invert();
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_BaseToBaseRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_BaseToBaseRF = GetArray2vtkMatrix(T_BaseToBaseRF);
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToBaseRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToBaseRF = GetArray2vtkMatrix(T_CamToBaseRF);
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToMetalBallRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToMetalBallRF = GetArray2vtkMatrix(T_CamToMetalBallRF);
	getBaseToFlangeMatrix();
	
	vtkSmartPointer<vtkMatrix4x4> T_CamToMetalBallRFReverse = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(vtkMatrixFromT_CamToMetalBallRF, T_CamToMetalBallRFReverse);
	vtkSmartPointer<vtkMatrix4x4>T_imageFrameToSteelballRFReverse = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(T_imageFrameToSteelballRF, T_imageFrameToSteelballRFReverse);
	vtkSmartPointer<vtkMatrix4x4>T_BaseToBaseRFReverse = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(vtkMatrixFromT_BaseToBaseRF, T_BaseToBaseRFReverse);
	
	vtkNew<vtkTransform> TransformBaseToTool;
	//ÕâÀïµÄ¾ØÕó¿ÉÄÜÊÇ·´ÁË£¬ÁíÒ»·½Ãæ£¬ÕâÀïµÄ¾ØÕóºÃÏñÒ²È±¶«Î÷£¬ÎÒÈÏÎªÊÇÈ±ÁË£¬flangetotcpµÄ
	//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
	TransformBaseToTool->PostMultiply();
	TransformBaseToTool->SetMatrix(tcpCoordinate);
	TransformBaseToTool->Concatenate(VTKT_BaseTotool_read);
	TransformBaseToTool->Concatenate(T_BaseToBaseRFReverse);
	TransformBaseToTool->Concatenate(vtkMatrixFromT_CamToBaseRF);
	TransformBaseToTool->Concatenate(T_CamToMetalBallRFReverse);
	TransformBaseToTool->Concatenate(T_imageFrameToSteelballRFReverse);
	//TransformBaseToTool->Concatenate(T_toolToPlanPlane);
	
	TransformBaseToTool->Update();
	TransformBaseToTool->GetMatrix();


	//ÄÇÕâ¸öÊÇ¸ÉÊ²Ã´£¬½ðÊôÇòµ½tcpÆ«ÖÃµÄ¾ØÕó¶ÔstlÓÐÊ²Ã´Ó°ÏìÂð
	applyTransformMatrix(TransformBaseToTool->GetMatrix());

}

void HTOrobot::showPlanCutPlanePositionL()
{
	getPointOnDaoban();

	auto  PointSetOnDaobanL = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("PointSetOnDaoban")->GetData());

	//½¨Á¢µ¼°åÉÏµÄÃæ×ø±êÏµ
	Eigen::Vector3d point1(PointSetOnDaobanL->GetPoint(7)[0], PointSetOnDaobanL->GetPoint(0)[1], PointSetOnDaobanL->GetPoint(0)[2]);
	Eigen::Vector3d point2(PointSetOnDaobanL->GetPoint(8)[0], PointSetOnDaobanL->GetPoint(1)[1], PointSetOnDaobanL->GetPoint(1)[2]);
	Eigen::Vector3d point3(PointSetOnDaobanL->GetPoint(9)[0], PointSetOnDaobanL->GetPoint(2)[1], PointSetOnDaobanL->GetPoint(2)[2]);
	Eigen::Vector3d vector1 = point2 - point1;
	Eigen::Vector3d vector2 = point3 - point1;
	Eigen::Vector3d axis_z = -vector1.cross(vector2);
	axis_z.normalize();
	Eigen::Vector3d point4(PointSetOnDaobanL->GetPoint(11)[0], PointSetOnDaobanL->GetPoint(4)[1], PointSetOnDaobanL->GetPoint(4)[2]);
	Eigen::Vector3d point5(PointSetOnDaobanL->GetPoint(12)[0], PointSetOnDaobanL->GetPoint(5)[1], PointSetOnDaobanL->GetPoint(5)[2]);
	Eigen::Vector3d point6(PointSetOnDaobanL->GetPoint(13)[0], PointSetOnDaobanL->GetPoint(6)[1], PointSetOnDaobanL->GetPoint(6)[2]);
	Eigen::Vector3d vector3 = point5 - point4;
	Eigen::Vector3d vector4 = point6 - point4;
	Eigen::Vector3d axis_y = vector3.cross(vector4);
	//Eigen::Vector3d axis_y = axis.cross(axis_z);
	Eigen::Vector3d axis_x = axis_z.cross(axis_y);
	axis_x.normalize();
	axis_y.normalize();
	Eigen::Vector3d originDaobanL(37.83, -14, 12.50);

	vtkSmartPointer<vtkMatrix4x4> daoBanCoordinateL = vtkSmartPointer<vtkMatrix4x4>::New();
	CreateCoordinateSystemMatrix(axis_x, axis_y, axis_z, originDaobanL, daoBanCoordinateL);
	// Êä³ö±ä»»¾ØÕó
	QString outputText_new = "\n";
	for (int row = 0; row < 4; ++row)
	{
		for (int col = 0; col < 4; ++col)
		{
			outputText_new += QString::number(daoBanCoordinateL->GetElement(row, col)) + "\t";
		}
		outputText_new += "\n";
	}
	std::cout << "-----------daoBanCoordinate--------:" << outputText_new << std::endl;

	//½¨Á¢¹æ»®ÃæµÄ×ø±êÏµ
	
	auto planPlane = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("PlanCutPlane1PointSet")->GetData());

	double vx = planPlane->GetPoint(2)[0] - planPlane->GetPoint(1)[0];
	double vy = planPlane->GetPoint(2)[1] - planPlane->GetPoint(1)[1];
	double vz= planPlane->GetPoint(2)[2] - planPlane->GetPoint(1)[2];
	double length = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));

	Eigen::Vector3d origin(planPlane->GetPoint(1)[0], planPlane->GetPoint(1)[1], planPlane->GetPoint(1)[2]);
	
	
	double direction[3]{ vx / length,vy / length,vz / length };
	auto cutPlane = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("1st cut plane")->GetData());
	auto tmpVtkSurface_initial = cutPlane->GetVtkPolyData();

	// transform tmpVtkSurface
	vtkNew<vtkTransform> cutPlaneTransform;
	cutPlaneTransform->SetMatrix(cutPlane->GetGeometry()->GetVtkMatrix());
	vtkNew<vtkTransformFilter> cutPlaneTransformFilter;
	cutPlaneTransformFilter->SetTransform(cutPlaneTransform);
	cutPlaneTransformFilter->SetInputData(tmpVtkSurface_initial);
	cutPlaneTransformFilter->Update();

	vtkNew<vtkPolyData> tmpVtkSurface;
	tmpVtkSurface->DeepCopy(cutPlaneTransformFilter->GetPolyDataOutput());

	double surfaceNormal[3];
	double cutPlaneCenter[3];

	GetPlaneProperty(tmpVtkSurface, surfaceNormal, cutPlaneCenter);
	Eigen::Vector3d n_z(-surfaceNormal[0], -surfaceNormal[1], -surfaceNormal[2]);
	n_z.normalize();
	Eigen::Vector3d n_x(direction[0], direction[1], direction[2]);
	n_x.normalize();
	Eigen::Vector3d n_y = -n_z.cross(n_x);
	n_y.normalize();
	//Æ´½Ó×ø±êÏµ¾ØÕó

	vtkSmartPointer<vtkMatrix4x4> PlanPlanecoordinate = vtkSmartPointer<vtkMatrix4x4>::New();
	CreateCoordinateSystemMatrix(n_x, n_y, n_z, origin, PlanPlanecoordinate);
	// Êä³ö±ä»»¾ØÕó
	QString outputText = "\n";
	for (int row = 0; row < 4; ++row)
	{
		for (int col = 0; col < 4; ++col)
		{
			outputText += QString::number(PlanPlanecoordinate->GetElement(row, col)) + "\t";
		}
		outputText += "\n";
	}
	std::cout << "-----------PlanPlanecoordinate-------------" << outputText << std::endl;

	// ¼ÆËãÁ½¸ö×ø±êÏµÖ®¼äµÄ×ª»»¾ØÕó
	auto transform = vtkTransform::New();
	vtkSmartPointer<vtkMatrix4x4> daoBanCoordinateInverse = vtkSmartPointer<vtkMatrix4x4>::New();
	T_toolToPlanPlane = vtkSmartPointer<vtkMatrix4x4>::New();
	transform->PostMultiply();
	vtkMatrix4x4::Invert(daoBanCoordinateL, daoBanCoordinateInverse);
	transform->SetMatrix(daoBanCoordinateInverse);
	transform->Concatenate(PlanPlanecoordinate);
	transform->Update();
	T_toolToPlanPlane = transform->GetMatrix();
	QString outputText1 = "\n";
	for (int row = 0; row < 4; ++row)
	{
		for (int col = 0; col < 4; ++col)
		{
			outputText1 += QString::number(T_toolToPlanPlane->GetElement(row, col)) + "\t";
		}
		outputText1 += "\n";
	}
	std::cout << "-----------TransformMatrix-------------" << outputText1 << std::endl;


	applyTransformMatrix(T_toolToPlanPlane);


}

//°Ñµ¼°å×ª»»µ½ÊÀ½ç×ø±êÏµÏÂ£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿£¿ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
void HTOrobot::applyTransformMatrix(vtkMatrix4x4* transMatrix)
{

	auto daobanStl = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("daoban")->GetData());
	auto tmpVtkSurface_initial = daobanStl->GetVtkPolyData();
	vtkNew<vtkPolyData> daobanVtkSurface;
	daobanVtkSurface->DeepCopy(tmpVtkSurface_initial);
	// transform tmpVtkSurface
	vtkNew<vtkTransform> daobanTransform;
	daobanTransform->PreMultiply();
	daobanTransform->SetMatrix(daobanStl->GetGeometry()->GetVtkMatrix());
	daobanTransform->Concatenate(transMatrix);
	daobanTransform->Update();
	auto daobanSurface1 = mitk::Surface::New();
	daobanSurface1->SetVtkPolyData(daobanVtkSurface);
	daobanSurface1->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(daobanTransform->GetMatrix());

	if (!daobanNode1)
	{
	
		daobanNode1 = mitk::DataNode::New();
		daobanNode1->SetName("ObjGuide");
		daobanNode1->SetData(daobanSurface1);
		GetDataStorage()->Add(daobanNode1);
		
	}
	else
	{

		daobanNode1->SetData(daobanSurface1);      // Add poinset into datanode
		daobanNode1->Modified();
		GetDataStorage()->Modified();
	}
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	mitk::RenderingManager::GetInstance()->ForceImmediateUpdateAll();
}
// ´´½¨Ò»¸ö×ø±êÏµµÄ±ä»»¾ØÕó
void HTOrobot::CreateCoordinateSystemMatrix(const Eigen::Vector3d& xAxis, const Eigen::Vector3d& yAxis, const Eigen::Vector3d& zAxis, const Eigen::Vector3d& origin, vtkMatrix4x4* resultMatrix) 
{

	// Set rotation part
	resultMatrix->SetElement(0, 0, xAxis(0));
	resultMatrix->SetElement(0, 1, yAxis(0));
	resultMatrix->SetElement(0, 2, zAxis(0));
	resultMatrix->SetElement(1, 0, xAxis(1));
	resultMatrix->SetElement(1, 1, yAxis(1));
	resultMatrix->SetElement(1, 2, zAxis(1));
	resultMatrix->SetElement(2, 0, xAxis(2));
	resultMatrix->SetElement(2, 1, yAxis(2));
	resultMatrix->SetElement(2, 2, zAxis(2));

	resultMatrix->SetElement(0, 3, origin(0));
	resultMatrix->SetElement(1, 3, origin(1));
	resultMatrix->SetElement(2, 3, origin(2));


	// set last col
	resultMatrix->SetElement(3, 0, 0.0);
	resultMatrix->SetElement(3, 1, 0.0);
	resultMatrix->SetElement(3, 2, 0.0);
	resultMatrix->SetElement(3, 3, 1.0);
}

void HTOrobot::PostOperativeVerification()
{
	auto hipCenterPoint = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("hipCenterPoint")->GetData());

	
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToPatientRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToPatientRF = GetArray2vtkMatrix(T_CamToPatientRF);
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToMetalBallRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToMetalBallRF = GetArray2vtkMatrix(T_CamToMetalBallRF);
	vtkSmartPointer<vtkMatrix4x4> m_aimToObjectRfInverse = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(vtkMatrixFromT_CamToMetalBallRF, m_aimToObjectRfInverse);
	vtkSmartPointer<vtkMatrix4x4> T_imageFrameToSteelballRFInverse = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(T_imageFrameToSteelballRF, T_imageFrameToSteelballRFInverse);

	auto tmp_transform = vtkTransform::New();
	
	tmp_transform->PostMultiply();
	tmp_transform->SetMatrix(vtkMatrixFromT_CamToPatientRF);
	tmp_transform->Concatenate(m_aimToObjectRfInverse);
	tmp_transform->Concatenate(T_imageFrameToSteelballRFInverse);
	tmp_transform->Update();
	vtkSmartPointer<vtkPoints> vtkProbeTip_onObjRfInImage = vtkSmartPointer<vtkPoints>::New();
	tmp_transform->TransformPoints(shieldProbeTip_onObjRf, vtkProbeTip_onObjRfInImage);
	
	mitk::Point3D anklecenterPointOnImage;
	anklecenterPointOnImage[0] = (vtkProbeTip_onObjRfInImage->GetPoint(0)[0]+ vtkProbeTip_onObjRfInImage->GetPoint(1)[0])/2;
	anklecenterPointOnImage[1] = (vtkProbeTip_onObjRfInImage->GetPoint(0)[1] + vtkProbeTip_onObjRfInImage->GetPoint(1)[1]) / 2;
	anklecenterPointOnImage[2] = (vtkProbeTip_onObjRfInImage->GetPoint(0)[2] + vtkProbeTip_onObjRfInImage->GetPoint(1)[2]) / 2;
	mitk::PointSet::Pointer tmpPointSet_2 = mitk::PointSet::New();
	tmpPointSet_2->SetPoint(0, hipCenterPoint->GetPoint(0));
	tmpPointSet_2->SetPoint(1, anklecenterPointOnImage);

	mitk::DataNode::Pointer existingForceLineNode = GetDataStorage()->GetNamedNode("Intraoperative_Line");
	if (existingForceLineNode.IsNotNull())
	{

		existingForceLineNode->SetData(tmpPointSet_2);
	}
	else
	{

		mitk::DataNode::Pointer line = mitk::DataNode::New();
		line->SetData(tmpPointSet_2);
		line->SetName("Intraoperative_Line");
		line->SetBoolProperty("show contour", true);
		line->SetFloatProperty("contoursize", 1.0);
		line->SetColor(0.0,1.0,0.0);
		GetDataStorage()->Add(line);

	}
	updateProportation_ReallTime(existingForceLineNode);
	double proportation=m_Controls.LineEdit_proportationRealTime->text().toDouble();
	m_Controls.LineEdit_proportationRealTime->setText(QString::number(proportation));
	double mPTA=m_Controls.LineEdit_caculateMPTA->text().toDouble();
	m_Controls.LineEdit_proportationRealTime->setText(QString::number(mPTA));
	double angle = m_Controls.LineEdit_strechAngle->text().toDouble();
	m_Controls.LineEdit_proportationRealTime->setText(QString::number(angle));
}
void HTOrobot::captureMalleolusPoint()
{

	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToProbe = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToProbe = GetArray2vtkMatrix(T_CamToProbe);
	vtkSmartPointer<vtkMatrix4x4> vtkMatrixFromT_CamToPatientRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrixFromT_CamToPatientRF = GetArray2vtkMatrix(T_CamToPatientRF);
	//ÓÃÌ½Õë²É¼¯¾âÆ¬ÉÏµÄ±ê¼Çµã£¬½«Æä×ª»»µ½SawµÄMarkerÏÂ
	auto tmp_transform = vtkTransform::New();
	vtkSmartPointer<vtkMatrix4x4> m_aimToObjectRfInverse = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(vtkMatrixFromT_CamToPatientRF, m_aimToObjectRfInverse);
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
	shieldProbeTip_onObjRf->InsertNextPoint(probe_outputPoint[0], probe_outputPoint[1], probe_outputPoint[2]);

}
