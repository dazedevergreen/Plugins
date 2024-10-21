#include <iostream>
// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qmitk
#include "HTOrobot.h"

// Qt
#include <QMessageBox>

// mitk image
#include <mitkImage.h>
#include <mitkPlane.h>
#include <vtkVector.h>
#include <vtkAppendPolyData.h>
#include <vtkCleanPolyData.h>
#include <vtkClipPolyData.h>
#include <vtkFeatureEdges.h>
#include <vtkPlane.h>
#include <vtkPlaneSource.h>
#include <vtkCellIterator.h>
#include <vtkSmoothPolyDataFilter.h>
#include <vtkStripper.h>
#include <vtkTriangleFilter.h>
#include <ep/include/vtk-9.1/vtkTransformFilter.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyDataMapper.h>
#include "mitkBoundingShapeCropper.h"
#include <mitkRemeshing.h>
#include "mitkInteractionConst.h"
#include <vtkSmartPointer.h>
#include <mitkPointSet.h>
#include <mitkRenderingManager.h>
#include "mitkRotationOperation.h"
#include <vtkPolyDataPlaneClipper.h>
#include <vtkFillHolesFilter.h>
#include "mitkSurface.h"
#include "mitkSurfaceToImageFilter.h"
#include "mitkVtkInterpolationProperty.h"
#include <lancetNavigationObject.h>
#include "vtkOBBTree.h"
#include "vtkDelaunay2D.h"
#include <vtkCutter.h>
#include <ep\include\vtk-9.1\vtkPolyDataMapper.h>
#include <vtkIntersectionPolyDataFilter.h>
#include <QtCore\qmath.h>
#include <mitkGizmo.h>
bool HTOrobot::CreateOneCutPlane()
{
	// m_Controls.textBrowser->append("Creating cut plane(s)");
	auto dataNode_cutPlane = GetDataStorage()->GetNamedNode("tibia cut plane");
	auto dataNode_tibiaSurface = GetDataStorage()->GetNamedNode("tibiaSurface");

	if (dataNode_tibiaSurface == nullptr)
	{
		m_Controls.textBrowser_HTO->append("tibiaSurface is missing");
		return false;
	}

	auto tibiaSurface = dynamic_cast<mitk::Surface*>(dataNode_tibiaSurface->GetData());

	// Get the OBB of tibia surface

	auto initialTibiaPolyData = tibiaSurface->GetVtkPolyData();
	vtkNew<vtkTransform> tibiaTransform;
	tibiaTransform->SetMatrix(tibiaSurface->GetGeometry()->GetVtkMatrix());
	vtkNew<vtkTransformFilter> tmpFilter;
	tmpFilter->SetTransform(tibiaTransform);
	tmpFilter->SetInputData(initialTibiaPolyData);
	tmpFilter->Update();

	vtkNew<vtkPolyData> tibiaPolyData;
	tibiaPolyData->DeepCopy(tmpFilter->GetPolyDataOutput());


	vtkNew<vtkOBBTree> obbTree;
	obbTree->SetDataSet(tibiaPolyData);
	obbTree->SetMaxLevel(2);
	obbTree->BuildLocator();

	double corner[3] = { 0.0, 0.0, 0.0 };//包围盒的一个角点坐标
	double max[3] = { 0.0, 0.0, 0.0 };//包围盒的最大点坐标
	double mid[3] = { 0.0, 0.0, 0.0 };//包围盒的中间点坐标
	double min[3] = { 0.0, 0.0, 0.0 };//包围盒的最小点坐标
	double size[3] = { 0.0, 0.0, 0.0 };



	//获取到包围多边形数据集的OBB的所有关键信息，包括它的位置、大小和方向
	obbTree->ComputeOBB(tibiaPolyData, corner, max, mid, min, size);

	vtkNew<vtkPolyData> obbPolydata;
	obbTree->GenerateRepresentation(0, obbPolydata);

	// auto tmpNode = mitk::DataNode::New();
	// auto tmpSurface = mitk::Surface::New();
	// tmpSurface->SetVtkPolyData(obbPolydata);
	// tmpNode->SetData(tmpSurface);
	// tmpNode->SetName("OBB");
	// GetDataStorage()->Add(tmpNode);

	//创建截骨平面并定义位置和方向
	auto cutPlaneSource = vtkSmartPointer<vtkPlaneSource>::New();

	cutPlaneSource->SetOrigin(0, 0, 0);//平面的原点坐标
	cutPlaneSource->SetPoint1(0, 70, 0);
	cutPlaneSource->SetPoint2(70, 0, 0);

	cutPlaneSource->SetNormal(max);//设置法向量

	// 定义截骨末端合页点的初始坐标
	mitk::Point3D point0;//水平面左边中点
	mitk::Point3D point1;//水平面左下角
	mitk::Point3D point2;//水平面右下角
	mitk::Point3D point3;//水平面右边中点
	mitk::Point3D planeCenterPoint;//水平面中心点
	// Determine the optimal plane location
	//存储截骨得到的两个部分
	vtkNew<vtkPolyData> largerSubpart_0;
	vtkNew<vtkPolyData> smallerSubpart_0;
	//新的截骨平面中心点，通过对现有几何数据加权平均得到
	double origin_0[3]
	{
		corner[0] + 0.5 * (1.7 * max[0] + mid[0] + min[0]),
		corner[1] + 0.5 * (1.7 * max[1] + mid[1] + min[1]),
		corner[2] + 0.5 * (1.7 * max[2] + mid[2] + min[2])
	};
	std::cout << "origin_0： " << ": (" << origin_0[0] << ", " << origin_0[1] << ", " << origin_0[2] << ")" << std::endl;
	vtkNew<vtkPolyData> largerSubpart_1;
	vtkNew<vtkPolyData> smallerSubpart_1;
	double origin_1[3]
	{
		corner[0] + 0.5 * (0.3 * max[0] + mid[0] + min[0]),
		corner[1] + 0.5 * (0.3 * max[1] + mid[1] + min[1]),
		corner[2] + 0.5 * (0.3 * max[2] + mid[2] + min[2])
	};
	//执行截骨操作
	CutPolyDataWithPlane(tibiaPolyData, largerSubpart_0, smallerSubpart_0, origin_0, max);
	CutPolyDataWithPlane(tibiaPolyData, largerSubpart_1, smallerSubpart_1, origin_1, max);

	if (smallerSubpart_0->GetNumberOfCells() >= smallerSubpart_1->GetNumberOfCells())
	{
		cutPlaneSource->SetCenter(origin_0);//生成截骨平面
		cutPlaneSource->SetNormal(0,0,1);
		qDebug() << "11221";
		point0[0] = origin_0[0] - 35;
		point0[1] = origin_0[1];
		point0[2] = origin_0[2];
		point1[0] = origin_0[0] - 35;
		point1[1] = origin_0[1] - 35;
		point1[2] = origin_0[2];
		point2[0] = origin_0[0] + 35;
		point2[1] = origin_0[1] - 35;
		point2[2] = origin_0[2];
		point3[0] = origin_0[0] + 35;
		point3[1] = origin_0[1];
		point3[2] = origin_0[2];

		planeCenterPoint[0] = origin_0[0];
		planeCenterPoint[1] = origin_0[1];
		planeCenterPoint[2] = origin_0[2];
		//将坐标添加到mitk::PointSet中
		mitkPointSet1->InsertPoint(0, point0);//第一截骨面末端中点标记
		mitkPointSet1->InsertPoint(1, point1);//平面左上角
		mitkPointSet1->InsertPoint(2, point2);//平面左下角
		mitkPointSet1->InsertPoint(3, point3);//平面右边中点

		mitkPointSet1->InsertPoint(4, planeCenterPoint);//平面原点
		std::cout << "point3： " << ": (" << point3[0] << ", " << point3[1] << ", " << point3[2] << ")" << std::endl;
		std::cout << "origin_0： " << ": (" << origin_0[0] << ", " << origin_0[1] << ", " << origin_0[2] << ")" << std::endl;
	}
	else
	{
		cutPlaneSource->SetCenter(origin_1);
		cutPlaneSource->SetNormal(0, 0, 1);
		point0[0] = origin_1[0] - 35;
		point0[1] = origin_1[1];
		point0[2] = origin_1[2];
		point1[0] = origin_1[0] - 35;
		point1[1] = origin_1[1] - 35;
		point1[2] = origin_1[2];
		point2[0] = origin_1[0] + 35;
		point2[1] = origin_1[1] - 35;
		point2[2] = origin_1[2];
		point3[0] = origin_1[0] + 35;
		point3[1] = origin_1[1];
		point3[2] = origin_1[2];

		planeCenterPoint[0] = origin_0[0];
		planeCenterPoint[1] = origin_0[1];
		planeCenterPoint[2] = origin_0[2];
		//将坐标添加到mitk::PointSet中
		mitkPointSet1->InsertPoint(0, point0);
		mitkPointSet1->InsertPoint(1, point1);
		mitkPointSet1->InsertPoint(2, point2);
		mitkPointSet1->InsertPoint(3, point3);
		mitkPointSet1->InsertPoint(4, planeCenterPoint);
		std::cout << "point3： " << ": (" << point3[0] << ", " << point3[1] << ", " << point3[2] << ")" << std::endl;
		std::cout << "origin_1： " << ": (" << origin_1[0] << ", " << origin_1[1] << ", " << origin_1[2] << ")" << std::endl;

	}

	cutPlaneSource->Update();
	auto cutSurface = mitk::Surface::New();
	cutSurface->SetVtkPolyData(cutPlaneSource->GetOutput());
	auto planeNode = mitk::DataNode::New();
	planeNode->SetData(cutSurface);
	planeNode->SetName("tibia cut plane");
	GetDataStorage()->Add(planeNode);


	//GetDistancefromTibia();
	return true;

}

bool HTOrobot::GetIntersectionLine()
{
	/*实时获取截骨线*/
	auto cutPlaneNode = GetDataStorage()->GetNamedNode("1st cut plane");
	auto tibiaSurfaceNode = GetDataStorage()->GetNamedNode("tibiaSurface");
	if (cutPlaneNode == nullptr)
	{
		m_Controls.textBrowser_HTO->append("'tibia cut plane' is not ready");
		return false;
	}
	if (tibiaSurfaceNode == nullptr)
	{
		m_Controls.textBrowser_HTO->append("'tibiaSurface' is not ready");
		return false;
	}
	auto mitkCutSurface = dynamic_cast<mitk::Surface*>(cutPlaneNode->GetData());

	auto tibiaMitkSurface = dynamic_cast<mitk::Surface*>(tibiaSurfaceNode->GetData());

	auto tmpVtkSurface_initial = mitkCutSurface->GetVtkPolyData();

	auto tibiaVtkSurface_initial = tibiaMitkSurface->GetVtkPolyData();

	// transform tmpVtkSurface
	vtkNew<vtkTransform> cutPlaneTransform;
	cutPlaneTransform->SetMatrix(mitkCutSurface->GetGeometry()->GetVtkMatrix());
	vtkNew<vtkTransformFilter> cutPlaneTransformFilter;
	cutPlaneTransformFilter->SetTransform(cutPlaneTransform);
	cutPlaneTransformFilter->SetInputData(tmpVtkSurface_initial);
	cutPlaneTransformFilter->Update();

	vtkNew<vtkPolyData> tmpVtkSurface;
	tmpVtkSurface->DeepCopy(cutPlaneTransformFilter->GetPolyDataOutput());

	// transform tibiaVtkSurface
	vtkNew<vtkTransform> tibiaTransform;
	tibiaTransform->SetMatrix(tibiaMitkSurface->GetGeometry()->GetVtkMatrix());
	vtkNew<vtkTransformFilter> tibiaTransformFilter;
	tibiaTransformFilter->SetTransform(tibiaTransform);
	tibiaTransformFilter->SetInputData(tibiaVtkSurface_initial);
	tibiaTransformFilter->Update();


	vtkNew<vtkPolyData> tibiaVtkSurface;
	tibiaVtkSurface->DeepCopy(tibiaTransformFilter->GetPolyDataOutput());

	double surfaceNormal[3];
	double cutPlaneCenter[3];

	GetPlaneProperty(tmpVtkSurface, surfaceNormal, cutPlaneCenter);


	// 使用获取的法向量和原点创建一个vtkPlane对象
	vtkSmartPointer<vtkPlane> cutPlane = vtkSmartPointer<vtkPlane>::New();
	cutPlane->SetNormal(surfaceNormal);
	cutPlane->SetOrigin(cutPlaneCenter);

	// 然后将这个cutPlane设置给vtkCutter
	vtkSmartPointer<vtkCutter> cutter_plane = vtkSmartPointer<vtkCutter>::New();
	cutter_plane->SetCutFunction(cutPlane);
	//设置输入数据
	cutter_plane->SetInputData(tibiaVtkSurface); // tibiaModelPolyData 是表示胫骨的vtkPolyData对象
	//更新获取交线
	cutter_plane->Update();
	vtkSmartPointer<vtkPolyData> intersectionLine = cutter_plane->GetOutput();
	TraverseIntersectionLines(intersectionLine);//获取切割平面与tibiaPolyData的交线以及特定点

	qDebug() << "GetIntersectionLine ";
	return true;
}
void HTOrobot::GetDistancefromTibia()
{
	//获取平面位置到距离胫骨内外侧平台的距离以及保留外侧合页
	auto tmpPointSet2 = dynamic_cast<mitk::PointSet*>(m_Controls.mitkNodeSelectWidget_tibiaPoint->GetSelectedNode()->GetData());
	auto Point_0 = tmpPointSet2->GetPoint(0);
	auto Point_1 = tmpPointSet2->GetPoint(1);

	if (judgModel_flag == 0)
	{
		distance1 = Point_0[2] - maxPoint[2];//截骨面末端距外侧平台
		distance2 = sqrt(pow((Point_1[0] - minPoint[0]), 2) +
			pow((Point_1[1] - minPoint[1]), 2) +
			pow((Point_1[2] - minPoint[2]), 2));//截骨面入口内侧平台
		if (maxPoint[0]> mitkPointSet1->GetPoint(3)[0])
		{
			distance3 = sqrt(pow((maxPoint[0] - mitkPointSet1->GetPoint(3)[0]), 2) +
				pow((maxPoint[1] - mitkPointSet1->GetPoint(3)[1]), 2) +
				pow((maxPoint[2] - mitkPointSet1->GetPoint(3)[2]), 2));//保留外侧合页
		}
		else
		{
			distance3 = -sqrt(pow((maxPoint[0] - mitkPointSet1->GetPoint(3)[0]), 2) +
				pow((maxPoint[1] - mitkPointSet1->GetPoint(3)[1]), 2) +
				pow((maxPoint[2] - mitkPointSet1->GetPoint(3)[2]), 2));
		}
		if (mitkPointSet1->GetPoint(3)[0] > maxPoint[0]) {
			depth = abs(maxPoint[0] - minPoint[0]);//入骨深度
		}
		else {
			depth = abs(mitkPointSet1->GetPoint(3)[0] - minPoint[0]);//入骨深度
		}

		std::cout << "mitkPointSet1： " << ": (" << mitkPointSet1->GetPoint(3)[0] << ", " << mitkPointSet1->GetPoint(3)[1] << ", " << mitkPointSet1->GetPoint(3)[2] << ")" << std::endl;
		m_Controls.LineEdit_distance1->setText(QString::number(distance1));
		m_Controls.LineEdit_distance2->setText(QString::number(distance2));
		m_Controls.LineEdit_hy->setText(QString::number(distance3));
		m_Controls.LineEdit_depth->setText(QString::number(depth));
	}
	else {
		if (judgModel_flag == 1)
		{
			distance1 = Point_0[2] - minPoint[2];//截骨面末端距外侧平台
			distance2 = sqrt(pow((Point_1[0] - maxPoint[0]), 2) +
				pow((Point_1[1] - maxPoint[1]), 2) +
				pow((Point_1[2] - maxPoint[2]), 2));//截骨面入口距内侧平台
			if (minPoint[0] < mitkPointSet1->GetPoint(0)[0])
			{
				distance3 = sqrt(pow((minPoint[0] - mitkPointSet1->GetPoint(0)[0]), 2) +
					pow((minPoint[1] - mitkPointSet1->GetPoint(0)[1]), 2) +
					pow((minPoint[2] - mitkPointSet1->GetPoint(0)[2]), 2));//保留外侧合页
			}
			else {
				distance3 = -sqrt(pow((minPoint[0] - mitkPointSet1->GetPoint(0)[0]), 2) +
					pow((minPoint[1] - mitkPointSet1->GetPoint(0)[1]), 2) +
					pow((minPoint[2] - mitkPointSet1->GetPoint(0)[2]), 2));
			}
			if (mitkPointSet1->GetPoint(0)[0] < minPoint[0]) {
				depth = abs(minPoint[0] - maxPoint[0]);//截骨面入骨深度
			}
			else {
				depth = abs(mitkPointSet1->GetPoint(0)[0] - maxPoint[0]);//截骨面入骨深度
			}


			std::cout << "mitkPointSet1： " << ": (" << mitkPointSet1->GetPoint(0)[0] << ", " << mitkPointSet1->GetPoint(0)[1] << ", " << mitkPointSet1->GetPoint(0)[2] << ")" << std::endl;
			m_Controls.LineEdit_distance1->setText(QString::number(distance1));
			m_Controls.LineEdit_distance2->setText(QString::number(distance2));
			m_Controls.LineEdit_hy->setText(QString::number(distance3));
			m_Controls.LineEdit_depth->setText(QString::number(depth));
		}

	}

}
void HTOrobot::TraverseIntersectionLines(vtkSmartPointer<vtkPolyData> intersectionLine)
{
	qDebug() << "TraverseIntersectionLines1 ";
	if (!planeAndTibiaIntersectionPointSet)
	{
		planeAndTibiaIntersectionPointSet = mitk::PointSet::New();
	}
	planeAndTibiaIntersectionPointSet->Clear();
	// 获取点集
	vtkSmartPointer<vtkPoints> points = intersectionLine->GetPoints();

	// 获取点的数量
	int numberOfPoints = points->GetNumberOfPoints();
	double minX = std::numeric_limits<double>::max();
	int minXIndex = -1;
	double maxX = -std::numeric_limits<double>::max(); // 初始化为小于可能的最小x值
	int maxXIndex = -1;

	std::cout << "Intersection Line Points:" << std::endl;
	for (int i = 0; i < numberOfPoints; ++i)
	{
		double point[3];
		points->GetPoint(i, point); // 获取第i个点的坐标

		std::cout << "Point " << i << ": (" << point[0] << ", " << point[1] << ", " << point[2] << ")" << std::endl;
		//检查当前点的x坐标是否小于已知的最小x值
		if (point[0] < minX)
		{
			minX = point[0]; // 更新最小x值
			minXIndex = i;   // 记录最小x值对应的点的索引
		}
		if (point[0] > maxX)
		{
			maxX = point[0];
			maxXIndex = i;
		}
	}
	if (minXIndex != -1 && maxXIndex != -1)
	{
		std::cout << "The point with the smallest x-coordinate is Point " << minXIndex << ": ("
			<< minX << ", " << points->GetPoint(minXIndex)[1] << ", " << points->GetPoint(minXIndex)[2] << ")" << std::endl;

		std::cout << "The point with the largest x-coordinate is Point " << maxXIndex << ": ("
			<< maxX << ", " << points->GetPoint(maxXIndex)[1] << ", " << points->GetPoint(maxXIndex)[2] << ")" << std::endl;
	}
	else
	{
		std::cout << "No points were processed." << std::endl;
	}

	////存储找到的截骨面与胫骨模型的交线上的最左侧点和入口点；
	minPoint[0] = points->GetPoint(minXIndex)[0];
	minPoint[1] = points->GetPoint(minXIndex)[1];
	minPoint[2] = points->GetPoint(minXIndex)[2];

	maxPoint[0] = points->GetPoint(maxXIndex)[0];
	maxPoint[1] = points->GetPoint(maxXIndex)[1];
	maxPoint[2] = points->GetPoint(maxXIndex)[2];
	qDebug() << "TraverseIntersectionLines12";
	planeAndTibiaIntersectionPointSet->InsertPoint(0, minPoint);
	planeAndTibiaIntersectionPointSet->InsertPoint(1, maxPoint);

	if (!m_DataNode)
	{
		m_DataNode = mitk::DataNode::New();
		m_DataNode->SetName("planeAndTibiaIntersectionPointSet");
		m_DataNode->SetColor(1.0, 0.0, 0.0);                                  // Set color to red
		m_DataNode->SetData(planeAndTibiaIntersectionPointSet);                                // Add poinset into datanode
		m_DataNode->SetProperty("pointsize", mitk::FloatProperty::New(10.0)); // Change the point size
		mitk::DataStorage::Pointer dataStorage = this->GetDataStorage();
		dataStorage->Add(m_DataNode);
		dataStorage->Modified();
		qDebug() << "m_DataNode1";
	}
	else
	{

		m_DataNode->SetData(planeAndTibiaIntersectionPointSet);      // Add poinset into datanode
		m_DataNode->Modified();

	}
	qDebug() << "TraverseIntersectionLines2";
}


bool HTOrobot::CreateCutPlane()
{
	if (m_Controls.radioButton_oneCut->isChecked())
	{
		if (CreateOneCutPlane())
		{
			auto cutSurfaceNode = GetDataStorage()->GetNamedNode("tibia cut plane");
			cutSurfaceNode->SetColor(0, 1, 0);
			cutSurfaceNode->SetOpacity(0.5);
			cutSurfaceNode->SetName("1st cut plane");



			return true;

		}
	}


	if (m_Controls.radioButton_twoCuts->isChecked())
	{

		auto cutPlaneNode = GetDataStorage()->GetNamedNode("1st cut plane");
		auto mitkCutPlane_0 = dynamic_cast<mitk::Surface*>(cutPlaneNode->GetData());
		auto tmpVtkSurface_initial = mitkCutPlane_0->GetVtkPolyData();

		// transform tmpVtkSurface
		vtkNew<vtkTransform> cutPlaneTransform;
		cutPlaneTransform->SetMatrix(mitkCutPlane_0->GetGeometry()->GetVtkMatrix());
		vtkNew<vtkTransformFilter> cutPlaneTransformFilter;
		cutPlaneTransformFilter->SetTransform(cutPlaneTransform);
		cutPlaneTransformFilter->SetInputData(tmpVtkSurface_initial);
		cutPlaneTransformFilter->Update();

		vtkNew<vtkPolyData> tmpVtkSurface;
		tmpVtkSurface->DeepCopy(cutPlaneTransformFilter->GetPolyDataOutput());
		auto cutSurface1 = mitk::Surface::New();
		cutSurface1->SetVtkPolyData(tmpVtkSurface);
		auto planeNode1 = mitk::DataNode::New();
		planeNode1->SetData(cutSurface1);
		planeNode1->SetName("2 cut plane");
		GetDataStorage()->Add(planeNode1);
		auto cutSurfaceNode2 = GetDataStorage()->GetNamedNode("2 cut plane");
		cutSurfaceNode2->SetColor(1, 0, 0);
		cutSurfaceNode2->SetOpacity(0.5);
		cutSurfaceNode2->SetName("2nd cut plane");
		double vx;
		double vy;
		double vz;
		double length;
		//计算旋转轴单位向量
		if (judgModel_flag == 0) {
			double vx_1 = mitkPointSet1->GetPoint(2)[0] - mitkPointSet1->GetPoint(1)[0];
			double vy_1 = mitkPointSet1->GetPoint(2)[1] - mitkPointSet1->GetPoint(1)[1];
			double vz_1 = mitkPointSet1->GetPoint(2)[2] - mitkPointSet1->GetPoint(1)[2];
			double length_1 = sqrt(pow(vx_1, 2) + pow(vy_1, 2) + pow(vz_1, 2));
			vx = vx_1;
			vy = vy_1;
			vz = vz_1;
			length = length_1;
		}
		else {
			if (judgModel_flag == 1) {
				double vx_2 = mitkPointSet1->GetPoint(1)[0] - mitkPointSet1->GetPoint(2)[0];
				double vy_2 = mitkPointSet1->GetPoint(1)[1] - mitkPointSet1->GetPoint(2)[1];
				double vz_2 = mitkPointSet1->GetPoint(1)[2] - mitkPointSet1->GetPoint(2)[2];
				double length_2 = sqrt(pow(vx_2, 2) + pow(vy_2, 2) + pow(vz_2, 2));
				vx = vx_2;
				vy = vy_2;
				vz = vz_2;
				length = length_2;
			}

		}

		double direction[3]{ vx / length,vy / length,vz / length };
		double angle = -110.0;

		double center[3];//水平截骨线中点
		center[0] = (mitkPointSet1->GetPoint(1)[0] + mitkPointSet1->GetPoint(2)[0]) / 2.0;
		center[1] = (mitkPointSet1->GetPoint(1)[1] + mitkPointSet1->GetPoint(2)[1]) / 2.0;
		center[2] = (mitkPointSet1->GetPoint(1)[2] + mitkPointSet1->GetPoint(2)[2]) / 2.0;
		qDebug() << center[0] << "," << center[1] << "," << center[2];
		qDebug() << mitkPointSet1->GetPoint(1)[0] << "," << mitkPointSet1->GetPoint(1)[1] << "," << mitkPointSet1->GetPoint(1)[2];
		qDebug() << mitkPointSet1->GetPoint(2)[0] << "," << mitkPointSet1->GetPoint(2)[1] << "," << mitkPointSet1->GetPoint(2)[2];

		QModelIndex parentIndex = m_NodetreeModel->GetIndex(planeNode1);
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
			Rotate(center, direction, angle, m_baseDataToMove);
		}

	}
	m_DataNode = mitk::DataNode::New();
	mitk::PointSet::Pointer mitkPointSet = mitk::PointSet::New();
	for (int i = 0; i < 4; ++i)
	{
		mitkPointSet->InsertPoint(mitkPointSet1->GetPoint(i));
	}
	m_DataNode->SetName("PlanCutPlane1PointSet");
	m_DataNode->SetColor(1.0, 0.0, 0.0);                                  // Set color to red
	m_DataNode->SetData(mitkPointSet);                                // Add poinset into datanode
	m_DataNode->SetProperty("pointsize", mitk::FloatProperty::New(5.0)); // Change the point size
	mitk::DataStorage::Pointer dataStorage = this->GetDataStorage();
	dataStorage->Add(m_DataNode);
	dataStorage->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;

}

bool HTOrobot::CutPolyDataWithPlane(vtkSmartPointer<vtkPolyData> dataToCut,
	vtkSmartPointer<vtkPolyData> largerSubPart,
	vtkSmartPointer<vtkPolyData> smallerSubPart,
	double planeOrigin[3], double planeNormal[3])
{
	vtkNew<vtkPlane> implicitPlane;
	implicitPlane->SetNormal(planeNormal);
	implicitPlane->SetOrigin(planeOrigin);

	vtkNew<vtkClipPolyData> clipper;
	clipper->SetInputData(dataToCut);
	clipper->GenerateClippedOutputOn();
	clipper->SetClipFunction(implicitPlane);
	clipper->Update();
	vtkNew<vtkPolyData> tibiaPart_0;
	tibiaPart_0->DeepCopy(clipper->GetClippedOutput());
	int cellNum_0 = tibiaPart_0->GetNumberOfCells();

	clipper->Update();
	vtkNew<vtkPolyData> tibiaPart_1;
	tibiaPart_1->DeepCopy(clipper->GetOutput());
	int cellNum_1 = tibiaPart_1->GetNumberOfCells();


	// Fill holes
	vtkNew<vtkFillHolesFilter> holeFiller0;
	holeFiller0->SetInputData(tibiaPart_0);
	holeFiller0->SetHoleSize(500);
	holeFiller0->Update();
	vtkNew<vtkPolyData> tibia_filled_0;
	tibia_filled_0->DeepCopy(holeFiller0->GetOutput());


	vtkNew<vtkFillHolesFilter> holeFiller1;
	holeFiller1->SetInputData(tibiaPart_1);
	holeFiller1->SetHoleSize(500);
	holeFiller1->Update();
	vtkNew<vtkPolyData> tibia_filled_1;
	tibia_filled_1->DeepCopy(holeFiller1->GetOutput());

	if (cellNum_1 >= cellNum_0)
	{
		largerSubPart->DeepCopy(tibia_filled_1);
		smallerSubPart->DeepCopy(tibia_filled_0);
	}
	else
	{
		largerSubPart->DeepCopy(tibia_filled_0);
		smallerSubPart->DeepCopy(tibia_filled_1);
	}

	return true;
}


bool HTOrobot::CutTibiaWithOnePlane()
{
	m_Controls.textBrowser_HTO->append("Cutting with one plane");
	auto cutPlaneNode = GetDataStorage()->GetNamedNode("1st cut plane");
	auto tibiaSurfaceNode = GetDataStorage()->GetNamedNode("tibiaSurface");
	if (cutPlaneNode == nullptr)
	{
		m_Controls.textBrowser_HTO->append("'tibia cut plane' is not ready");
		return false;
	}
	if (tibiaSurfaceNode == nullptr)
	{
		m_Controls.textBrowser_HTO->append("'tibiaSurface' is not ready");
		return false;
	}

	auto mitkCutSurface = dynamic_cast<mitk::Surface*>(cutPlaneNode->GetData());

	auto tibiaMitkSurface = dynamic_cast<mitk::Surface*>(tibiaSurfaceNode->GetData());

	auto tmpVtkSurface_initial = mitkCutSurface->GetVtkPolyData();

	auto tibiaVtkSurface_initial = tibiaMitkSurface->GetVtkPolyData();

	// transform tmpVtkSurface
	vtkNew<vtkTransform> cutPlaneTransform;
	cutPlaneTransform->SetMatrix(mitkCutSurface->GetGeometry()->GetVtkMatrix());
	vtkNew<vtkTransformFilter> cutPlaneTransformFilter;
	cutPlaneTransformFilter->SetTransform(cutPlaneTransform);
	cutPlaneTransformFilter->SetInputData(tmpVtkSurface_initial);
	cutPlaneTransformFilter->Update();

	vtkNew<vtkPolyData> tmpVtkSurface;
	tmpVtkSurface->DeepCopy(cutPlaneTransformFilter->GetPolyDataOutput());

	// transform tibiaVtkSurface
	vtkNew<vtkTransform> tibiaTransform;
	tibiaTransform->SetMatrix(tibiaMitkSurface->GetGeometry()->GetVtkMatrix());
	vtkNew<vtkTransformFilter> tibiaTransformFilter;
	tibiaTransformFilter->SetTransform(tibiaTransform);
	tibiaTransformFilter->SetInputData(tibiaVtkSurface_initial);
	tibiaTransformFilter->Update();


	vtkNew<vtkPolyData> tibiaVtkSurface;
	tibiaVtkSurface->DeepCopy(tibiaTransformFilter->GetPolyDataOutput());

	double surfaceNormal[3];
	double cutPlaneCenter[3];

	GetPlaneProperty(tmpVtkSurface, surfaceNormal, cutPlaneCenter);


	vtkNew<vtkPolyData> proximalTibiaSurface;
	vtkNew<vtkPolyData> distalTibiaSurface;

	CutPolyDataWithPlane(tibiaVtkSurface, distalTibiaSurface, proximalTibiaSurface, cutPlaneCenter, surfaceNormal);

	auto mitkProximalSurface = mitk::Surface::New();
	auto mitkDistalSurface = mitk::Surface::New();
	mitkProximalSurface->SetVtkPolyData(proximalTibiaSurface);
	mitkDistalSurface->SetVtkPolyData(distalTibiaSurface);

	// for visualization purpose: mitkProximalSurface and mitkProximalSurface have rough surfaces
	auto proximal_remehsed = mitk::Remeshing::Decimate(mitkProximalSurface, 1, true, true);
	auto distal_remehsed = mitk::Remeshing::Decimate(mitkDistalSurface, 1, true, true);

	vtkNew<vtkFillHolesFilter> holeFiller0;
	holeFiller0->SetInputData(proximal_remehsed->GetVtkPolyData());
	holeFiller0->SetHoleSize(500);
	holeFiller0->Update();
	vtkNew<vtkPolyData> proximalTibia;
	proximalTibia->DeepCopy(holeFiller0->GetOutput());
	auto proximalSurface = mitk::Surface::New();
	proximalSurface->SetVtkPolyData(proximalTibia);

	vtkNew<vtkFillHolesFilter> holeFiller1;
	holeFiller1->SetInputData(distal_remehsed->GetVtkPolyData());
	holeFiller1->SetHoleSize(500);
	holeFiller1->Update();
	vtkNew<vtkPolyData> distalTibia;
	distalTibia->DeepCopy(holeFiller1->GetOutput());
	auto distalSurface = mitk::Surface::New();
	distalSurface->SetVtkPolyData(distalTibia);


	auto tmpNode0 = mitk::DataNode::New();
	auto tmpNode1 = mitk::DataNode::New();
	tmpNode0->SetData(proximalSurface);
	tmpNode0->SetName("proximal tibiaSurface");

	mitk::VtkInterpolationProperty::Pointer interpolationProp0;
	tmpNode0->GetProperty(interpolationProp0, "material.interpolation");
	interpolationProp0->SetInterpolationToFlat();
	tmpNode0->SetProperty("material.interpolation", interpolationProp0);


	tmpNode1->SetData(distalSurface);
	tmpNode1->SetName("distal tibiaSurface");

	mitk::VtkInterpolationProperty::Pointer interpolationProp1;
	tmpNode1->GetProperty(interpolationProp1, "material.interpolation");
	interpolationProp1->SetInterpolationToFlat();
	tmpNode1->SetProperty("material.interpolation", interpolationProp1);

	GetDataStorage()->Add(tmpNode0);
	GetDataStorage()->Add(tmpNode1);

	return true;
}

bool HTOrobot::CutTibiaWithTwoPlanes()
{
	auto cutplane_0 = GetDataStorage()->GetNamedNode("1st cut plane");
	auto cutplane_1 = GetDataStorage()->GetNamedNode("2nd cut plane");
	auto tibiaNode = GetDataStorage()->GetNamedNode("tibiaSurface");

	if (cutplane_0 == nullptr || cutplane_1 == nullptr)
	{
		m_Controls.textBrowser_HTO->append("'1st cut plane' or '2nd cut plane' is not ready");
		return false;
	}
	if (tibiaNode == nullptr)
	{
		m_Controls.textBrowser_HTO->append("'tibiaSurface' is not ready");
		return false;
	}

	auto mitkCutPlane_0 = dynamic_cast<mitk::Surface*>(cutplane_0->GetData());
	auto mitkCutPlane_1 = dynamic_cast<mitk::Surface*>(cutplane_1->GetData());
	auto mitkTibia = dynamic_cast<mitk::Surface*>(tibiaNode->GetData());

	vtkNew<vtkPolyData> vtkCutPlane_0;
	vtkNew<vtkPolyData> vtkCutPlane_1;
	vtkNew<vtkPolyData> vtkTibia;

	// Append the geometry offset matrices
	vtkNew<vtkTransform> cutPlaneTransform_0;
	cutPlaneTransform_0->SetMatrix(mitkCutPlane_0->GetGeometry()->GetVtkMatrix());

	vtkNew<vtkTransform> cutPlaneTransform_1;
	cutPlaneTransform_1->SetMatrix(mitkCutPlane_1->GetGeometry()->GetVtkMatrix());

	vtkNew<vtkTransform> tibiaTransform;
	tibiaTransform->SetMatrix(mitkTibia->GetGeometry()->GetVtkMatrix());

	vtkNew<vtkTransformFilter> cutPlaneTransformFilter_0;
	cutPlaneTransformFilter_0->SetTransform(cutPlaneTransform_0);
	cutPlaneTransformFilter_0->SetInputData(mitkCutPlane_0->GetVtkPolyData());
	cutPlaneTransformFilter_0->Update();
	vtkCutPlane_0->DeepCopy(cutPlaneTransformFilter_0->GetPolyDataOutput());

	vtkNew<vtkTransformFilter> cutPlaneTransformFilter_1;
	cutPlaneTransformFilter_1->SetTransform(cutPlaneTransform_1);
	cutPlaneTransformFilter_1->SetInputData(mitkCutPlane_1->GetVtkPolyData());
	cutPlaneTransformFilter_1->Update();
	vtkCutPlane_1->DeepCopy(cutPlaneTransformFilter_1->GetPolyDataOutput());

	vtkNew<vtkTransformFilter> tibiaTransformFilter;
	tibiaTransformFilter->SetTransform(tibiaTransform);
	tibiaTransformFilter->SetInputData(mitkTibia->GetVtkPolyData());
	tibiaTransformFilter->Update();
	vtkTibia->DeepCopy(tibiaTransformFilter->GetPolyDataOutput());

	double cutPlaneCenter_0[3];
	double cutPlaneNormal_0[3];
	double cutPlaneCenter_1[3];
	double cutPlaneNormal_1[3];

	GetPlaneProperty(vtkCutPlane_0, cutPlaneNormal_0, cutPlaneCenter_0);
	GetPlaneProperty(vtkCutPlane_1, cutPlaneNormal_1, cutPlaneCenter_1);

	vtkNew<vtkPolyData> largetPart;
	vtkNew<vtkPolyData> tmpMiddlePart;
	vtkNew<vtkPolyData> middlePart;
	vtkNew<vtkPolyData> smallPart;


	// Cut and merge
	CutPolyDataWithPlane(vtkTibia, largetPart, tmpMiddlePart, cutPlaneCenter_0, cutPlaneNormal_0);
	CutPolyDataWithPlane(tmpMiddlePart, middlePart, smallPart, cutPlaneCenter_1, cutPlaneNormal_1);

	vtkSmartPointer<vtkAppendPolyData> appendFilter =
		vtkSmartPointer<vtkAppendPolyData>::New();
	vtkSmartPointer<vtkCleanPolyData> cleanFilter =
		vtkSmartPointer<vtkCleanPolyData>::New();

	appendFilter->AddInputData(largetPart);
	appendFilter->AddInputData(smallPart);
	appendFilter->Update();

	cleanFilter->SetInputData(appendFilter->GetOutput());
	cleanFilter->Update();

	auto proximalSurface = mitk::Surface::New();
	auto distalSurface = mitk::Surface::New();

	proximalSurface->SetVtkPolyData(middlePart);
	distalSurface->SetVtkPolyData(cleanFilter->GetOutput());

	//// for visualization purpose: mitkProximalSurface and mitkProximalSurface have rough surfaces
	auto proximal_remehsed = mitk::Remeshing::Decimate(proximalSurface, 1, true, true);
	auto distal_remehsed = mitk::Remeshing::Decimate(distalSurface, 1, true, true);


	// Save into nodes
	auto proximalNode = mitk::DataNode::New();
	auto distalNode = mitk::DataNode::New();

	proximalNode->SetName("proximal tibiaSurface");
	proximalNode->SetData(proximalSurface);
	//proximalNode->SetData(proximal_remehsed);
	distalNode->SetName("distal tibiaSurface");
	distalNode->SetData(distalSurface);
	//distalNode->SetData(distal_remehsed);


	GetDataStorage()->Add(distalNode);
	GetDataStorage()->Add(proximalNode);

	return true;
}


bool HTOrobot::CutTibiaSurface()
{
	if (m_Controls.radioButton_oneCut->isChecked())
	{
		if (CutTibiaWithOnePlane())
		{
			return true;
		}
	}

	if (m_Controls.radioButton_twoCuts->isChecked())
	{
		if (CutTibiaWithTwoPlanes())
		{
			return true;
		}

	}

	return false;
}

bool HTOrobot::CutTibiaImage()
{
	qDebug()<< "CutTibiaImage";
	auto proximalNode = GetDataStorage()->GetNamedNode("proximal tibiaSurface");
	auto distalNode = GetDataStorage()->GetNamedNode("distal tibiaSurface");
	auto imageNode = GetDataStorage()->GetNamedNode("tibiaImage");

	if (proximalNode == nullptr || distalNode == nullptr)
	{
		m_Controls.textBrowser_HTO->append("'proximal tibiaSurface' or 'distal tibiaSurface' is missing");
		return false;
	}

	if (imageNode == nullptr)
	{
		m_Controls.textBrowser_HTO->append("'tibiaImage' is missing");
		return false;
	}

	auto mitkProximalSurface = dynamic_cast<mitk::Surface*>(proximalNode->GetData());
	auto proximalBounds = mitkProximalSurface->GetGeometry()->GetBounds();
	auto proximalOrigin = mitkProximalSurface->GetGeometry()->GetOrigin();

	auto mitkDistalSurface = dynamic_cast<mitk::Surface*>(distalNode->GetData());
	auto distalBounds = mitkDistalSurface->GetGeometry()->GetBounds();
	auto distalOrigin = mitkDistalSurface->GetGeometry()->GetOrigin();

	auto image = dynamic_cast<mitk::Image*>(imageNode->GetData());

	// Apply the geometric offset matrices
	vtkNew<vtkPolyData> vtkDistal;
	vtkNew<vtkPolyData> vtkProximal;

	vtkNew<vtkTransform> distalTransform;
	distalTransform->SetMatrix(mitkDistalSurface->GetGeometry()->GetVtkMatrix());

	vtkNew<vtkTransform> proximalTransform;
	proximalTransform->SetMatrix(mitkProximalSurface->GetGeometry()->GetVtkMatrix());

	vtkNew<vtkTransformFilter> distalTransformFilter;
	distalTransformFilter->SetTransform(distalTransform);
	distalTransformFilter->SetInputData(mitkDistalSurface->GetVtkPolyData());
	distalTransformFilter->Update();
	vtkDistal->DeepCopy(distalTransformFilter->GetPolyDataOutput());

	vtkNew<vtkTransformFilter> proximalTransformFilter;
	proximalTransformFilter->SetTransform(proximalTransform);
	proximalTransformFilter->SetInputData(mitkProximalSurface->GetVtkPolyData());
	proximalTransformFilter->Update();
	vtkProximal->DeepCopy(proximalTransformFilter->GetPolyDataOutput());

	auto imageClone = image->Clone();

	auto proximalImage = mitk::Image::New();
	auto distalImage = mitk::Image::New();

	auto surfaceToImageFilter = mitk::SurfaceToImageFilter::New();
	//surfaceToImageFilter->SetMakeOutputBinary(false);
	surfaceToImageFilter->SetBackgroundValue(0);
	//surfaceToImageFilter->SetUShortBinaryPixelType(false);
	surfaceToImageFilter->SetImage(imageClone);
	surfaceToImageFilter->SetInput(mitkProximalSurface);


	surfaceToImageFilter->Update();
	proximalImage = surfaceToImageFilter->GetOutput()->Clone();

	// Cut the proximal image
	auto proximalBoundingBox = mitk::GeometryData::New();
	proximalBoundingBox->SetGeometry(mitkProximalSurface->GetGeometry());

	auto cutter = mitk::BoundingShapeCropper::New();
	cutter->SetGeometry(proximalBoundingBox);
	cutter->SetUseWholeInputRegion(false);
	cutter->SetInput(proximalImage);
	cutter->Update();



	surfaceToImageFilter->SetInput(mitkDistalSurface);
	surfaceToImageFilter->Update();
	distalImage = surfaceToImageFilter->GetOutput()->Clone();

	// Cut the distal image
	auto distalBoundingBox = mitk::GeometryData::New();
	distalBoundingBox->SetGeometry(mitkDistalSurface->GetGeometry());

	auto cutterDistal = mitk::BoundingShapeCropper::New();
	cutterDistal->SetGeometry(distalBoundingBox);
	cutterDistal->SetUseWholeInputRegion(false);
	cutterDistal->SetInput(distalImage);
	cutterDistal->Update();


	auto tmpNode0 = mitk::DataNode::New();
	tmpNode0->SetName("distal tibiaImage");
	tmpNode0->SetData(cutterDistal->GetOutput());
	GetDataStorage()->Add(tmpNode0, distalNode);

	auto tmpNode1 = mitk::DataNode::New();
	tmpNode1->SetName("proximal tibiaImage");
	tmpNode1->SetData(cutter->GetOutput());
	GetDataStorage()->Add(tmpNode1, proximalNode);
	qDebug() << "CutTibiaImageYES";
	return true;
}


bool HTOrobot::CutTibia()
{
	if (CutTibiaSurface() && CutTibiaImage())
	{
		return true;
		qDebug() << "cutTibia1";
	}

	return false;
	qDebug() << "cutTibia222";
}



bool HTOrobot::GetPlaneProperty(vtkSmartPointer<vtkPolyData> plane, double normal[3], double center[3])
{
	auto tmpCenter = plane->GetCenter();

	center[0] = *tmpCenter;
	center[1] = *(tmpCenter + 1);
	center[2] = *(tmpCenter + 2);

	// Obtain the normal of the mitkSurface
	double p0[3]; double p1[3]; double p2[3];

	plane->GetCell(0)->GetPoints()->GetPoint(0, p0);
	plane->GetCell(0)->GetPoints()->GetPoint(1, p1);
	plane->GetCell(0)->GetPoints()->GetPoint(2, p2);

	Eigen::Vector3d a(*p0, *(p0 + 1), *(p0 + 2));
	Eigen::Vector3d b(*p1, *(p1 + 1), *(p1 + 2));
	Eigen::Vector3d c(*p2, *(p2 + 1), *(p2 + 2));

	Eigen::Vector3d tmpVector0 = b - a;
	Eigen::Vector3d tmpVector1 = c - a;

	Eigen::Vector3d normalVector = tmpVector0.cross(tmpVector1);

	normal[0] = normalVector[0];
	normal[1] = normalVector[1];
	normal[2] = normalVector[2];

	return true;
}

//adjust cutTibia
void HTOrobot::TranslateMinusX()
{
	double direction[3]{ -1,0,0 };

	QModelIndex parentIndex = m_NodetreeModel->GetIndex(m_currentSelectedNode);

	int parentRowCount = m_NodetreeModel->rowCount(parentIndex);
	int parentColumnCount = m_NodetreeModel->columnCount(parentIndex);

	if (m_currentSelectedNode->GetName() == "tibiaSurface")
	{
		mitk::DataStorage::Pointer dataStorage = GetDataStorage();
		mitk::DataNode::Pointer PointSet_Ie_tiabia = dataStorage->GetNamedNode("tibiaMalleolusPointSet");
		auto tmpPointSet_1 = dynamic_cast<mitk::PointSet*>(PointSet_Ie_tiabia->GetData());
		auto tmpPointSet2 = dynamic_cast<mitk::PointSet*>(m_Controls.mitkNodeSelectWidget_tibiaPoint->GetSelectedNode()->GetData());

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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), m_baseDataToMove);
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), tmpPointSet_1);
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), tmpPointSet2);
		}

		ShowLine();
	}
	else if (m_currentSelectedNode->GetName() == "1st cut plane")
	{
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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), m_baseDataToMove);
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), mitkPointSet1);

		}

		if (GetIntersectionLine())
		{

			GetDistancefromTibia();
			qDebug() << "10000";

		}
	}
	else
	{
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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), m_baseDataToMove);

		}
	}
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void HTOrobot::TranslateMinusY()
{
	double direction[3]{ 0,-1,0 };

	QModelIndex parentIndex = m_NodetreeModel->GetIndex(m_currentSelectedNode);

	int parentRowCount = m_NodetreeModel->rowCount(parentIndex);
	int parentColumnCount = m_NodetreeModel->columnCount(parentIndex);

	if (m_currentSelectedNode->GetName() == "tibiaSurface")
	{
		mitk::DataStorage::Pointer dataStorage = GetDataStorage();
		mitk::DataNode::Pointer PointSet_Ie_tiabia = dataStorage->GetNamedNode("tibiaMalleolusPointSet");
		auto tmpPointSet_1 = dynamic_cast<mitk::PointSet*>(PointSet_Ie_tiabia->GetData());
		auto tmpPointSet2 = dynamic_cast<mitk::PointSet*>(m_Controls.mitkNodeSelectWidget_tibiaPoint->GetSelectedNode()->GetData());
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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), m_baseDataToMove);
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), tmpPointSet_1);
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), tmpPointSet2);
		}

		ShowLine();
	}
	else if (m_currentSelectedNode->GetName() == "1st cut plane")
	{
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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), m_baseDataToMove);
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), mitkPointSet1);
		}

		if (GetIntersectionLine())
		{

			GetDistancefromTibia();
			qDebug() << "10000";

		}
	}
	else
	{
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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), m_baseDataToMove);
		}
	}
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}
void HTOrobot::TranslateMinusZ()
{
	double direction[3]{ 0,0,-1 };

	QModelIndex parentIndex = m_NodetreeModel->GetIndex(m_currentSelectedNode);

	int parentRowCount = m_NodetreeModel->rowCount(parentIndex);
	int parentColumnCount = m_NodetreeModel->columnCount(parentIndex);

	if (m_currentSelectedNode->GetName() == "tibiaSurface")
	{
		auto tmpPointSet2 = dynamic_cast<mitk::PointSet*>(m_Controls.mitkNodeSelectWidget_tibiaPoint->GetSelectedNode()->GetData());
		mitk::DataStorage::Pointer dataStorage = GetDataStorage();
		mitk::DataNode::Pointer PointSet_Ie_tiabia = dataStorage->GetNamedNode("tibiaMalleolusPointSet");
		auto tmpPointSet_1 = dynamic_cast<mitk::PointSet*>(PointSet_Ie_tiabia->GetData());
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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), m_baseDataToMove);
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), tmpPointSet2);
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), tmpPointSet_1);
		}

		ShowLine();
	}
	else if (m_currentSelectedNode->GetName() == "1st cut plane")
	{
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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), m_baseDataToMove);
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), mitkPointSet1);

		}

		if (GetIntersectionLine())
		{

			GetDistancefromTibia();
			qDebug() << "10000";

		}
	}
	else
	{
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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), m_baseDataToMove);

		}
	}
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}
void HTOrobot::TranslatePlusX()
{
	double direction[3]{ 1,0,0 };
	QModelIndex parentIndex = m_NodetreeModel->GetIndex(m_currentSelectedNode);//获取当前被选中的节点索引
	int parentRowCount = m_NodetreeModel->rowCount(parentIndex);
	int parentColumnCount = m_NodetreeModel->columnCount(parentIndex);
	if (m_currentSelectedNode->GetName() == "tibiaSurface")
	{
		auto tmpPointSet2 = dynamic_cast<mitk::PointSet*>(m_Controls.mitkNodeSelectWidget_tibiaPoint->GetSelectedNode()->GetData());
		mitk::DataStorage::Pointer dataStorage = GetDataStorage();
		mitk::DataNode::Pointer PointSet_Ie_tiabia = dataStorage->GetNamedNode("tibiaMalleolusPointSet");
		auto tmpPointSet_1 = dynamic_cast<mitk::PointSet*>(PointSet_Ie_tiabia->GetData());
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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), m_baseDataToMove);
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), tmpPointSet2);
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), tmpPointSet_1);
		}

		ShowLine();


	}
	else if (m_currentSelectedNode->GetName() == "1st cut plane")
	{

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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), m_baseDataToMove);
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), mitkPointSet1);
		}

		if (GetIntersectionLine())
		{
			qDebug() << "TransX ";
			GetDistancefromTibia();
			qDebug() << "10000";

		}
	}
	else
	{
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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), m_baseDataToMove);


		}
	}
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void HTOrobot::TranslatePlusY()
{
	double direction[3]{ 0,1,0 };

	QModelIndex parentIndex = m_NodetreeModel->GetIndex(m_currentSelectedNode);

	int parentRowCount = m_NodetreeModel->rowCount(parentIndex);
	int parentColumnCount = m_NodetreeModel->columnCount(parentIndex);

	if (m_currentSelectedNode->GetName() == "tibiaSurface")
	{
		mitk::DataStorage::Pointer dataStorage = GetDataStorage();
		mitk::DataNode::Pointer PointSet_Ie_tiabia = dataStorage->GetNamedNode("tibiaMalleolusPointSet");
		auto tmpPointSet_1 = dynamic_cast<mitk::PointSet*>(PointSet_Ie_tiabia->GetData());
		auto tmpPointSet2 = dynamic_cast<mitk::PointSet*>(m_Controls.mitkNodeSelectWidget_tibiaPoint->GetSelectedNode()->GetData());
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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), m_baseDataToMove);
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), tmpPointSet_1);
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), tmpPointSet2);
		}

		ShowLine();
	}
	else if (m_currentSelectedNode->GetName() == "1st cut plane")
	{
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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), m_baseDataToMove);
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), mitkPointSet1);

		}

		if (GetIntersectionLine())
		{

			GetDistancefromTibia();
			qDebug() << "10000";

		}
	}
	else
	{
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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), m_baseDataToMove);


		}
	}
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}
void HTOrobot::TranslatePlusZ()
{

	double direction[3]{ 0,0,1 };

	QModelIndex parentIndex = m_NodetreeModel->GetIndex(m_currentSelectedNode);

	int parentRowCount = m_NodetreeModel->rowCount(parentIndex);
	int parentColumnCount = m_NodetreeModel->columnCount(parentIndex);

	if (m_currentSelectedNode->GetName() == "tibiaSurface")
	{
		
		mitk::DataStorage::Pointer dataStorage = GetDataStorage();
		mitk::DataNode::Pointer PointSet_Ie_tiabia = dataStorage->GetNamedNode("tibiaMalleolusPointSet");
		auto tmpPointSet_1 = dynamic_cast<mitk::PointSet*>(PointSet_Ie_tiabia->GetData());
		auto tmpPointSet2 = dynamic_cast<mitk::PointSet*>(m_Controls.mitkNodeSelectWidget_tibiaPoint->GetSelectedNode()->GetData());
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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), m_baseDataToMove);
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), tmpPointSet_1);
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), tmpPointSet2);
	
		}

		ShowLine();
	}
	else if (m_currentSelectedNode->GetName() == "1st cut plane")
	{
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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), m_baseDataToMove);
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), mitkPointSet1);
		}

		if (GetIntersectionLine())
		{

			GetDistancefromTibia();
			qDebug() << "10000";

		}
	}
	else
	{
	
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
		
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), m_baseDataToMove);

		}
	}
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();

}
void HTOrobot::RotatePlusX()
{
	if (m_baseDataToMove == nullptr)
	{
		m_Controls.textBrowser_HTO->append("Empty input. Please select a node ~");
		return;
	}

	double direction[3]{ 1,0,0 };
	double angle = m_Controls.lineEdit_intuitiveValue_1->text().toDouble();
	mitk::Point<double, 3>::ValueType* center = m_currentSelectedNode->GetData()->GetGeometry()->GetCenter().GetDataPointer();

	QModelIndex parentIndex = m_NodetreeModel->GetIndex(m_currentSelectedNode);

	int parentRowCount = m_NodetreeModel->rowCount(parentIndex);
	int parentColumnCount = m_NodetreeModel->columnCount(parentIndex);

	if (m_currentSelectedNode->GetName() == "tibiaSurface")
	{
		mitk::DataStorage::Pointer dataStorage = GetDataStorage();
		mitk::DataNode::Pointer PointSet_Ie_tiabia = dataStorage->GetNamedNode("tibiaMalleolusPointSet");
		auto tmpPointSet_1 = dynamic_cast<mitk::PointSet*>(PointSet_Ie_tiabia->GetData());
		auto tmpPointSet2 = dynamic_cast<mitk::PointSet*>(m_Controls.mitkNodeSelectWidget_tibiaPoint->GetSelectedNode()->GetData());

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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Rotate(center, direction, angle, m_baseDataToMove);
			Rotate(center, direction, angle, tmpPointSet_1);
			Rotate(center, direction, angle, tmpPointSet2);
		}


		ShowLine();
	}
	else if (m_currentSelectedNode->GetName() == "1st cut plane")
	{

		double center[3];
		center[0] = mitkPointSet1->GetPoint(4)[0];
		center[1] = mitkPointSet1->GetPoint(4)[1];
		center[2] = mitkPointSet1->GetPoint(4)[2];
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
			Rotate(center, direction, angle, m_baseDataToMove);
			Rotate(center, direction, angle, mitkPointSet1);
		}

		if (GetIntersectionLine())
		{

			GetDistancefromTibia();
			qDebug() << "10000";

		}
	}
	else
	{
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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Rotate(center, direction, angle, m_baseDataToMove);


		}
	}
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();

}
void HTOrobot::RotatePlusY()
{
	if (m_baseDataToMove == nullptr)
	{
		m_Controls.textBrowser_HTO->append("Empty input. Please select a node ~");
		return;
	}

	double direction[3]{ 0,1,0 };
	double angle = m_Controls.lineEdit_intuitiveValue_1->text().toDouble();
	mitk::Point<double, 3>::ValueType* center = m_currentSelectedNode->GetData()->GetGeometry()->GetCenter().GetDataPointer();

	QModelIndex parentIndex = m_NodetreeModel->GetIndex(m_currentSelectedNode);

	int parentRowCount = m_NodetreeModel->rowCount(parentIndex);
	int parentColumnCount = m_NodetreeModel->columnCount(parentIndex);

	if (m_currentSelectedNode->GetName() == "tibiaSurface")
	{
		mitk::DataStorage::Pointer dataStorage = GetDataStorage();
		mitk::DataNode::Pointer PointSet_Ie_tiabia = dataStorage->GetNamedNode("tibiaMalleolusPointSet");
		auto tmpPointSet_1 = dynamic_cast<mitk::PointSet*>(PointSet_Ie_tiabia->GetData());
		auto tmpPointSet2 = dynamic_cast<mitk::PointSet*>(m_Controls.mitkNodeSelectWidget_tibiaPoint->GetSelectedNode()->GetData());

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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Rotate(center, direction, angle, m_baseDataToMove);
			Rotate(center, direction, angle, tmpPointSet_1);
			Rotate(center, direction, angle, tmpPointSet2);

		}

		ShowLine();
	}
	else if (m_currentSelectedNode->GetName() == "1st cut plane")
	{
		double center[3];
		center[0] = mitkPointSet1->GetPoint(4)[0];
		center[1] = mitkPointSet1->GetPoint(4)[1];
		center[2] = mitkPointSet1->GetPoint(4)[2];
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
			Rotate(center, direction, angle, m_baseDataToMove);
			Rotate(center, direction, angle, mitkPointSet1);
		}

		if (GetIntersectionLine())
		{

			GetDistancefromTibia();
			qDebug() << "10000";

		}
	}
	else
	{
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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Rotate(center, direction, angle, m_baseDataToMove);


		}

	}
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}
void HTOrobot::RotatePlusZ()
{
	if (m_baseDataToMove == nullptr)
	{
		m_Controls.textBrowser_HTO->append("Empty input. Please select a node ~");
		return;
	}

	double direction[3]{ 0,0,1 };
	double angle = m_Controls.lineEdit_intuitiveValue_1->text().toDouble();
	mitk::Point<double, 3>::ValueType* center = m_currentSelectedNode->GetData()->GetGeometry()->GetCenter().GetDataPointer();

	QModelIndex parentIndex = m_NodetreeModel->GetIndex(m_currentSelectedNode);

	int parentRowCount = m_NodetreeModel->rowCount(parentIndex);
	int parentColumnCount = m_NodetreeModel->columnCount(parentIndex);

	if (m_currentSelectedNode->GetName() == "tibiaSurface")
	{
		mitk::DataStorage::Pointer dataStorage = GetDataStorage();
		mitk::DataNode::Pointer PointSet_Ie_tiabia = dataStorage->GetNamedNode("tibiaMalleolusPointSet");
		auto tmpPointSet_1 = dynamic_cast<mitk::PointSet*>(PointSet_Ie_tiabia->GetData());
		auto tmpPointSet2 = dynamic_cast<mitk::PointSet*>(m_Controls.mitkNodeSelectWidget_tibiaPoint->GetSelectedNode()->GetData());

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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Rotate(center, direction, angle, m_baseDataToMove);
			Rotate(center, direction, angle, tmpPointSet_1);
			Rotate(center, direction, angle, tmpPointSet2);

		}

		ShowLine();
	}
	else if (m_currentSelectedNode->GetName() == "1st cut plane")
	{
		double center[3];
		center[0] = mitkPointSet1->GetPoint(4)[0];
		center[1] = mitkPointSet1->GetPoint(4)[1];
		center[2] = mitkPointSet1->GetPoint(4)[2];
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
			Rotate(center, direction, angle, m_baseDataToMove);
			Rotate(center, direction, angle, mitkPointSet1);

		}

		if (GetIntersectionLine())
		{

			GetDistancefromTibia();
			qDebug() << "10000";

		}
	}
	else
	{
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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Rotate(center, direction, angle, m_baseDataToMove);


		}

	}
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void HTOrobot::RotateMinusX()
{
	if (m_baseDataToMove == nullptr)
	{
		m_Controls.textBrowser_HTO->append("Empty input. Please select a node ~");
		return;
	}

	double direction[3]{ 1,0,0 };
	double angle = -m_Controls.lineEdit_intuitiveValue_1->text().toDouble();
	mitk::Point<double, 3>::ValueType* center = m_currentSelectedNode->GetData()->GetGeometry()->GetCenter().GetDataPointer();

	QModelIndex parentIndex = m_NodetreeModel->GetIndex(m_currentSelectedNode);

	int parentRowCount = m_NodetreeModel->rowCount(parentIndex);
	int parentColumnCount = m_NodetreeModel->columnCount(parentIndex);

	if (m_currentSelectedNode->GetName() == "tibiaSurface")
	{
		mitk::DataStorage::Pointer dataStorage = GetDataStorage();
		mitk::DataNode::Pointer PointSet_Ie_tiabia = dataStorage->GetNamedNode("tibiaMalleolusPointSet");
		auto tmpPointSet_1 = dynamic_cast<mitk::PointSet*>(PointSet_Ie_tiabia->GetData());
		auto tmpPointSet2 = dynamic_cast<mitk::PointSet*>(m_Controls.mitkNodeSelectWidget_tibiaPoint->GetSelectedNode()->GetData());

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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Rotate(center, direction, angle, m_baseDataToMove);
			Rotate(center, direction, angle, tmpPointSet_1);
			Rotate(center, direction, angle, tmpPointSet2);

		}

		ShowLine();
	}
	else if (m_currentSelectedNode->GetName() == "1st cut plane")
	{
		double center[3];
		center[0] = mitkPointSet1->GetPoint(4)[0];
		center[1] = mitkPointSet1->GetPoint(4)[1];
		center[2] = mitkPointSet1->GetPoint(4)[2];
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
			Rotate(center, direction, angle, m_baseDataToMove);
			Rotate(center, direction, angle, mitkPointSet1);
		}

		if (GetIntersectionLine())
		{

			GetDistancefromTibia();
			qDebug() << "10000";

		}
	}
	else
	{
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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Rotate(center, direction, angle, m_baseDataToMove);


		}

	}
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}
void HTOrobot::RotateMinusY()
{
	if (m_baseDataToMove == nullptr)
	{
		m_Controls.textBrowser_HTO->append("Empty input. Please select a node ~");
		return;
	}

	double direction[3]{ 0,1,0 };
	double angle = -m_Controls.lineEdit_intuitiveValue_1->text().toDouble();
	mitk::Point<double, 3>::ValueType* center = m_currentSelectedNode->GetData()->GetGeometry()->GetCenter().GetDataPointer();

	QModelIndex parentIndex = m_NodetreeModel->GetIndex(m_currentSelectedNode);

	int parentRowCount = m_NodetreeModel->rowCount(parentIndex);
	int parentColumnCount = m_NodetreeModel->columnCount(parentIndex);

	if (m_currentSelectedNode->GetName() == "tibiaSurface")
	{
		mitk::DataStorage::Pointer dataStorage = GetDataStorage();
		mitk::DataNode::Pointer PointSet_Ie_tiabia = dataStorage->GetNamedNode("tibiaMalleolusPointSet");
		auto tmpPointSet_1 = dynamic_cast<mitk::PointSet*>(PointSet_Ie_tiabia->GetData());
		auto tmpPointSet2 = dynamic_cast<mitk::PointSet*>(m_Controls.mitkNodeSelectWidget_tibiaPoint->GetSelectedNode()->GetData());

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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Rotate(center, direction, angle, m_baseDataToMove);
			Rotate(center, direction, angle, tmpPointSet_1);
			Rotate(center, direction, angle, tmpPointSet2);

		}

		ShowLine();
	}
	else if (m_currentSelectedNode->GetName() == "1st cut plane")
	{
		double center[3];
		center[0] = mitkPointSet1->GetPoint(4)[0];
		center[1] = mitkPointSet1->GetPoint(4)[1];
		center[2] = mitkPointSet1->GetPoint(4)[2];
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
			Rotate(center, direction, angle, m_baseDataToMove);
			Rotate(center, direction, angle, mitkPointSet1);
		}

		if (GetIntersectionLine())
		{

			GetDistancefromTibia();
			qDebug() << "10000";

		}
	}
	else
	{
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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Rotate(center, direction, angle, m_baseDataToMove);

		}
	}
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}
void HTOrobot::RotateMinusZ()
{
	if (m_baseDataToMove == nullptr)
	{
		m_Controls.textBrowser_HTO->append("Empty input. Please select a node ~");
		return;
	}

	double direction[3]{ 0,0,1 };
	double angle = -m_Controls.lineEdit_intuitiveValue_1->text().toDouble();
	mitk::Point<double, 3>::ValueType* center = m_currentSelectedNode->GetData()->GetGeometry()->GetCenter().GetDataPointer();

	QModelIndex parentIndex = m_NodetreeModel->GetIndex(m_currentSelectedNode);

	int parentRowCount = m_NodetreeModel->rowCount(parentIndex);
	int parentColumnCount = m_NodetreeModel->columnCount(parentIndex);

	if (m_currentSelectedNode->GetName() == "tibiaSurface")
	{
		mitk::DataStorage::Pointer dataStorage = GetDataStorage();
		mitk::DataNode::Pointer PointSet_Ie_tiabia = dataStorage->GetNamedNode("tibiaMalleolusPointSet");
		auto tmpPointSet_1 = dynamic_cast<mitk::PointSet*>(PointSet_Ie_tiabia->GetData());
		auto tmpPointSet2 = dynamic_cast<mitk::PointSet*>(m_Controls.mitkNodeSelectWidget_tibiaPoint->GetSelectedNode()->GetData());

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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Rotate(center, direction, angle, m_baseDataToMove);
			Rotate(center, direction, angle, tmpPointSet_1);
			Rotate(center, direction, angle, tmpPointSet2);


		}

		ShowLine();
	}
	else if (m_currentSelectedNode->GetName() == "1st cut plane")
	{
		double center[3];
		center[0] = mitkPointSet1->GetPoint(4)[0];
		center[1] = mitkPointSet1->GetPoint(4)[1];
		center[2] = mitkPointSet1->GetPoint(4)[2];
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
			Rotate(center, direction, angle, m_baseDataToMove);


		}
		Rotate(center, direction, angle, mitkPointSet1);

		if (GetIntersectionLine())
		{

			GetDistancefromTibia();
			qDebug() << "10000";

		}
	}
	else
	{
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
			//Translate(direction, m_Controls.lineEdit_intuitiveValue_1->text().toDouble(), point6);
			//std::cout << point6[0]<<std::endl;
			Rotate(center, direction, angle, m_baseDataToMove);


		}

	}
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	if (!m_Controls.LineEdit_angle->text().isEmpty() && m_Controls.LineEdit_angle->text().toDouble() != angleInDegrees)
	{
		angleInDegrees += angle;
		m_Controls.LineEdit_angle->setText(QString::number(angleInDegrees));
	}
}

void HTOrobot::on_pushButton_addGizmo_clicked()
{

	if (mitk::Gizmo::HasGizmoAttached(m_currentSelectedNode, GetDataStorage()) == 1)
	{
		mitk::Gizmo::RemoveGizmoFromNode(m_currentSelectedNode, GetDataStorage());
	}
	else
	{
		mitk::Gizmo::AddGizmoToNode(m_currentSelectedNode, GetDataStorage());
	}
}


//translate strech_angle
void HTOrobot::RotateMinus()
{
	if (m_baseDataToMove == nullptr)
	{
		m_Controls.textBrowser_HTO->append("Empty input. Please select a node ~");
		return;
	}
	double vx = mitkPointSet1->GetPoint(0)[0] - mitkPointSet1->GetPoint(1)[0];
	double vy = mitkPointSet1->GetPoint(0)[1] - mitkPointSet1->GetPoint(1)[1];
	double vz = mitkPointSet1->GetPoint(0)[2] - mitkPointSet1->GetPoint(1)[2];
	double length = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));
	double direction_cutPlane[3]{ vx / length,vy / length,vz / length };
	double angle = m_Controls.LineEdit_transAngle->text().toDouble();
	qDebug() << "mitkPointSet1->GetPoint(0):" << mitkPointSet1->GetPoint(0)[0] << "," << mitkPointSet1->GetPoint(0)[1] << "," << mitkPointSet1->GetPoint(0)[2];
	qDebug() << "mitkPointSet1->GetPoint(1):" << mitkPointSet1->GetPoint(1)[0] << "," << mitkPointSet1->GetPoint(1)[1] << "," << mitkPointSet1->GetPoint(1)[2];

	double center[3];
	center[0] = mitkPointSet1->GetPoint(0)[0];
	center[1] = mitkPointSet1->GetPoint(0)[1];
	center[2] = mitkPointSet1->GetPoint(0)[2];

	QModelIndex parentIndex = m_NodetreeModel->GetIndex(m_currentSelectedNode);

	int parentRowCount = m_NodetreeModel->rowCount(parentIndex);
	int parentColumnCount = m_NodetreeModel->columnCount(parentIndex);

	if (m_currentSelectedNode->GetName() == "distal tibiaSurface")
	{
		mitk::DataStorage::Pointer dataStorage = GetDataStorage();
		mitk::DataNode::Pointer PointSet_Ie_tiabia = dataStorage->GetNamedNode("tibiaMalleolusPointSet");
		auto tmpPointSet_1 = dynamic_cast<mitk::PointSet*>(PointSet_Ie_tiabia->GetData());
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

			Rotate(center, direction_cutPlane, angle, m_baseDataToMove);

		}
		//Rotate(center, direction_cutPlane, angle, dynamic_cast<mitk::Surface*>(m_currentSelectedNode->GetData()));
		Rotate(center, direction_cutPlane, angle, tmpPointSet_1);
		ShowLine();

	}
	else
	{
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

			Rotate(center, direction_cutPlane, angle, m_baseDataToMove);
		}

	}
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();

	if (!m_Controls.LineEdit_transAngle->text().isEmpty())
	{
		if (judgModel_flag == 0) {
			angleInDegrees -= angle;
		}
		else {
			if (judgModel_flag == 1) {
				angleInDegrees += angle;
			}

		}


		m_Controls.LineEdit_angle->setText(QString::number(angleInDegrees));
	}
	CaculateStrechHeigh();
	
}
void HTOrobot::RotatePlus()
{
	//if (!m_Controls.LineEdit_angle->text().isEmpty())
	//{
	//	double angle = angleInDegrees;
	//	/*angleInDegrees += angle;
	//	m_Controls.LineEdit_angle->setText(QString::number(angleInDegrees));*/
	//}

	if (m_baseDataToMove == nullptr)
	{
		m_Controls.textBrowser_HTO->append("Empty input. Please select a node ~");
		return;
	}

	double vx = mitkPointSet1->GetPoint(0)[0] - mitkPointSet1->GetPoint(1)[0];
	double vy = mitkPointSet1->GetPoint(0)[1] - mitkPointSet1->GetPoint(1)[1];
	double vz = mitkPointSet1->GetPoint(0)[2] - mitkPointSet1->GetPoint(1)[2];
	double length = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));
	double direction_cutPlane[3]{ vx / length,vy / length,vz / length };
	double angle = -m_Controls.LineEdit_transAngle->text().toDouble();
	double center[3];
	center[0] = mitkPointSet1->GetPoint(0)[0];
	center[1] = mitkPointSet1->GetPoint(0)[1];
	center[2] = mitkPointSet1->GetPoint(0)[2];


	QModelIndex parentIndex = m_NodetreeModel->GetIndex(m_currentSelectedNode);

	int parentRowCount = m_NodetreeModel->rowCount(parentIndex);
	int parentColumnCount = m_NodetreeModel->columnCount(parentIndex);
	if (m_currentSelectedNode->GetName() == "distal tibiaSurface")
	{

		mitk::DataStorage::Pointer dataStorage = GetDataStorage();
		mitk::DataNode::Pointer PointSet_Ie_tiabia = dataStorage->GetNamedNode("tibiaMalleolusPointSet");
		auto tmpPointSet_1 = dynamic_cast<mitk::PointSet*>(PointSet_Ie_tiabia->GetData());
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

			Rotate(center, direction_cutPlane, angle, m_baseDataToMove);

		}
		/*mitk::DataStorage::Pointer dataStorage = GetDataStorage();
		mitk::DataNode::Pointer PointSet_Ie_tiabia = dataStorage->GetNamedNode("PointSet1_tibia");
		auto tmpPointSet_1 = dynamic_cast<mitk::PointSet*>(PointSet_Ie_tiabia->GetData());
		Rotate(center, direction_cutPlane, angle, dynamic_cast<mitk::Surface*>(m_currentSelectedNode->GetData()));*/
		Rotate(center, direction_cutPlane, angle, tmpPointSet_1);
		ShowLine();
	}
	else
	{
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

			Rotate(center, direction_cutPlane, angle, m_baseDataToMove);
		}

	}
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	if (!m_Controls.LineEdit_transAngle->text().isEmpty())
	{
		if (judgModel_flag == 0) {
			angleInDegrees -= angle;
		}
		else {
			if (judgModel_flag == 1)
			{
				angleInDegrees += angle;
			}

		}


		m_Controls.LineEdit_angle->setText(QString::number(angleInDegrees));
	}
	CaculateStrechHeigh();
}
void HTOrobot::CaculateStrechAngle()
{
	mitk::DataStorage::Pointer dataStorage = GetDataStorage();
	mitk::DataNode::Pointer PointSet1 = dataStorage->GetNamedNode("hipCenterPoint");
	auto tmpPointSet_1 = dynamic_cast<mitk::PointSet*>(PointSet1->GetData());
	mitk::Point3D point0;
	point0[0] = tmpPointSet_1->GetPoint(0)[0];
	point0[1] = tmpPointSet_1->GetPoint(0)[1];
	point0[2] = tmpPointSet_1->GetPoint(0)[2];
	auto tmpPointSet2 = dynamic_cast<mitk::PointSet*>(dataStorage->GetNamedNode("tibiaCondylesPointSet")->GetData()); 
	double point1[3];
	double point2[3];
	point1[0] = tmpPointSet2->GetPoint(0)[0];
	point1[1] = tmpPointSet2->GetPoint(0)[1];
	point1[2] = tmpPointSet2->GetPoint(0)[2];
	point2[0] = tmpPointSet2->GetPoint(1)[0];
	point2[1] = tmpPointSet2->GetPoint(1)[1];
	point2[2] = tmpPointSet2->GetPoint(1)[2];
	double tibia_centerPoint[3];
	tibia_centerPoint[0] = (point1[0] + point2[0]) / 2;
	tibia_centerPoint[1] = (point1[1] + point2[1]) / 2;
	tibia_centerPoint[2] = (point1[2] + point2[2]) / 2;
	/*qDebug()<< "point0:" << point0[0] << "," << point0[1] << "," << point0[2] ;
	cout << "point1:" << point1[0] << "," << point1[1] << "," << point1[2];
	cout << "point2:" << point2[0] << "," << point2[1] << "," << point2[2];
	cout << "tibia_centerPoint:" << tibia_centerPoint[0] << "," << tibia_centerPoint[1] << "," << tibia_centerPoint[2];*/

	double vx_3 = point2[0] - point1[0];
	double vy_3 = point2[1] - point1[1];
	double vz_3 = point2[2] - point1[2];
	double length3 = sqrt(pow(vx_3, 2) + pow(vy_3, 2) + pow(vz_3, 2));
	double ux_3 = vx_3 / length3;
	double uy_3 = vy_3 / length3;
	double uz_3 = vz_3 / length3;
	double direction3[3]{ ux_3,uy_3,uz_3 };

	//计算目标力线延长线点的位置
	double vx = tibia_centerPoint[0] - point0[0];
	double vy = tibia_centerPoint[1] - point0[1];
	double vz = tibia_centerPoint[2] - point0[2];
	double length = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));
	double ux = vx / length;
	double uy = vy / length;
	double uz = vz / length;
	// 根据力线长度计算目标点的坐标
	double x3 = ux * Line_length + point0[0];
	double y3 = uy * Line_length + point0[1];
	double z3 = uz * Line_length + point0[2];
	cout << "目标延长线上的点C坐标为: (" << x3 << ", " << y3 << ", " << z3 << ")";
	//std::cout << Line_length << endl;
	if (judgModel_flag == 0) {
		cout << "合页点为: (" << mitkPointSet1->GetPoint(3)[0] << ", " << mitkPointSet1->GetPoint(3)[1] << ", " << mitkPointSet1->GetPoint(3)[2] << ")" ;
		auto linePointSet = dynamic_cast<mitk::PointSet*>(dataStorage->GetNamedNode("Line")->GetData());
		//计算当前力线的向量；
		double vx_1 = linePointSet->GetPoint(1)[0] - mitkPointSet1->GetPoint(3)[0];//当前力线终点-合页点
		double vy_1 = linePointSet->GetPoint(1)[1] - mitkPointSet1->GetPoint(3)[1];
		double vz_1 = linePointSet->GetPoint(1)[2] - mitkPointSet1->GetPoint(3)[2];
		double length_1 = sqrt(pow(vx_1, 2) + pow(vy_1, 2) + pow(vz_1, 2));
		double ux_1 = vx_1 / length_1;
		double uy_1 = vy_1 / length_1;
		double uz_1 = vz_1 / length_1;
		double direction1[3]{ ux_1,uy_1,uz_1 };
		//计算理想力线的向量
		double vx_2 = x3 - mitkPointSet1->GetPoint(3)[0];//理想力线终点-合页点
		double vy_2 = y3 - mitkPointSet1->GetPoint(3)[1];
		double vz_2 = z3 - mitkPointSet1->GetPoint(3)[2];
		double length_2 = sqrt(pow(vx_2, 2) + pow(vy_2, 2) + pow(vz_2, 2));
		double ux_2 = vx_2 / length_2;
		double uy_2 = vy_2 / length_2;
		double uz_2 = vz_2 / length_2;
		double direction2[3]{ ux_2,uy_2,uz_2 };
		// 计算两个方向向量之间的夹角
		double dotProduct = direction1[0] * direction2[0] + direction1[1] * direction2[1] + direction1[2] * direction2[2]; // 点积
		double angleInRadians = acos(dotProduct); // 使用反余弦函数得到角度的弧度值
		double angleInDegrees = round(angleInRadians * (180.0 / M_PI)); // 将弧度转换为度数
		m_Controls.LineEdit_angle->setText(QString::number(angleInDegrees));
		std::cout << "The angle between the two directions is: " << angleInDegrees << " degrees." << std::endl;
		m_Controls.LineEdit_transAngle->setText(QString::number(angleInDegrees));
		//caculate mPTA
		double dotProduct3 = direction3[0] * direction2[0] + direction3[1] * direction2[1] + direction3[2] * direction2[2];
		double mPTA = round(acos(dotProduct3) * (180.0 / M_PI));
		m_Controls.LineEdit_mPTA->setText(QString::number(mPTA));
	}
	else
	{
		if (judgModel_flag == 1)
		{
			cout << "合页点为: (" << mitkPointSet1->GetPoint(0)[0] << ", " << mitkPointSet1->GetPoint(0)[1] << ", " << mitkPointSet1->GetPoint(0)[2] << ")" ;
			auto linePointSet = dynamic_cast<mitk::PointSet*>(dataStorage->GetNamedNode("Line")->GetData());
			//计算当前力线的向量；
			double vx_1 = linePointSet->GetPoint(1)[0] - mitkPointSet1->GetPoint(0)[0];//当前力线终点-合页点
			double vy_1 = linePointSet->GetPoint(1)[1] - mitkPointSet1->GetPoint(0)[1];
			double vz_1 = linePointSet->GetPoint(1)[2] - mitkPointSet1->GetPoint(0)[2];
			double length_1 = sqrt(pow(vx_1, 2) + pow(vy_1, 2) + pow(vz_1, 2));
			double ux_1 = vx_1 / length_1;
			double uy_1 = vy_1 / length_1;
			double uz_1 = vz_1 / length_1;
			double direction1[3]{ ux_1,uy_1,uz_1 };
			//计算理想力线的向量
			double vx_2 = x3 - mitkPointSet1->GetPoint(0)[0];//理想力线终点-合页点
			double vy_2 = y3 - mitkPointSet1->GetPoint(0)[1];
			double vz_2 = z3 - mitkPointSet1->GetPoint(0)[2];
			double length_2 = sqrt(pow(vx_2, 2) + pow(vy_2, 2) + pow(vz_2, 2));
			double ux_2 = vx_2 / length_2;
			double uy_2 = vy_2 / length_2;
			double uz_2 = vz_2 / length_2;
			double direction2[3]{ ux_2,uy_2,uz_2 };
			// 计算两个方向向量之间的夹角
			double dotProduct = direction1[0] * direction2[0] + direction1[1] * direction2[1] + direction1[2] * direction2[2]; // 点积
			double angleInRadians = acos(dotProduct); // 使用反余弦函数得到角度的弧度值
			double angleInDegrees = round(angleInRadians * (180.0 / M_PI)); // 将弧度转换为度数
			m_Controls.LineEdit_angle->setText(QString::number(angleInDegrees));
			std::cout << "The angle between the two directions is: " << angleInDegrees << " degrees." << std::endl;
			m_Controls.LineEdit_transAngle->setText(QString::number(angleInDegrees));
			
			//caculate mPTA
			double dotProduct3 = direction3[0] * direction2[0] + direction3[1] * direction2[1] + direction3[2] * direction2[2];
			double mPTA = round(acos(dotProduct3) * (180.0 / M_PI));
			m_Controls.LineEdit_mPTA->setText(QString::number(mPTA));
			
		}

	}
	
	CaculateStrechHeigh();
	RotateMinus();

}
void HTOrobot::CaculateStrechHeigh()
{
	// 将角度转换为弧度
	std::cout << "angleInDegrees: " << angleInDegrees << std::endl;
	double angle_rad = angleInDegrees * M_PI / 180.0;
	std::cout << "angle_rad: " << angle_rad << std::endl;
	double height = round(sin(angle_rad) * depth);
	m_Controls.LineEdit_height->setText(QString::number(height));
}
void HTOrobot::Rotate(double center[3], double direction[3], double counterclockwiseDegree, mitk::BaseData* data)
{
	if (data != nullptr)
	{
		double normalizedDirection[3];
		double directionLength = sqrt((pow(direction[0], 2) + pow(direction[1], 2) + pow(direction[2], 2)));
		normalizedDirection[0] = direction[0] / directionLength;
		normalizedDirection[1] = direction[1] / directionLength;
		normalizedDirection[2] = direction[2] / directionLength;


		mitk::Point3D rotateCenter{ center };
		mitk::Vector3D rotateAxis{ normalizedDirection };
		auto* doOp = new mitk::RotationOperation(mitk::OpROTATE, rotateCenter, rotateAxis, counterclockwiseDegree);
		// execute the Operation
		// here no undo is stored, because the movement-steps aren't interesting.
		// only the start and the end is interesting to store for undo.
		data->GetGeometry()->ExecuteOperation(doOp);
		delete doOp;

		mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	}
	else
	{
		m_Controls.textBrowser_HTO->append("Empty input. Please select a node ~");
	}
	clock_t start = clock();
	TestCut2();
	MITK_WARN << "Test.Interface.time(TestCut2): " << (clock() - start);
	// TestCut3();
}
void HTOrobot::TestCut()
{
	if (m_Controls.radioButton_testCutting->isChecked() == false)
	{
		return;
	}

	if (m_growingCutterNode == nullptr)
	{
		m_growingCutterNode = mitk::DataNode::New();
		auto initSurface = dynamic_cast<mitk::Surface*>(m_currentSelectedNode->GetData());
		auto initPolyData = initSurface->GetVtkPolyData();

		vtkNew<vtkPolyData> movedPolyData;

		vtkNew<vtkTransformFilter> transformFilter;
		vtkNew<vtkTransform> tmpTransform;
		tmpTransform->SetMatrix(initSurface->GetGeometry()->GetVtkMatrix());
		transformFilter->SetTransform(tmpTransform);
		transformFilter->SetInputData(initPolyData);
		transformFilter->Update();

		movedPolyData->DeepCopy(transformFilter->GetPolyDataOutput());

		auto tmpSurface = mitk::Surface::New();
		tmpSurface->SetVtkPolyData(movedPolyData);
		m_growingCutterNode->SetData(tmpSurface);
		m_growingCutterNode->SetName("growingCutter");
		GetDataStorage()->Add(m_growingCutterNode);
	}

	// Append the current cutter
	auto initSurface = dynamic_cast<mitk::Surface*>(m_currentSelectedNode->GetData());
	auto initPolyData = initSurface->GetVtkPolyData();

	vtkNew<vtkPolyData> movedPolyData;

	vtkNew<vtkTransformFilter> transformFilter;
	vtkNew<vtkTransform> tmpTransform;
	tmpTransform->SetMatrix(initSurface->GetGeometry()->GetVtkMatrix());
	transformFilter->SetTransform(tmpTransform);
	transformFilter->SetInputData(initPolyData);
	transformFilter->Update();

	movedPolyData->DeepCopy(transformFilter->GetPolyDataOutput());

	auto initGrowingCutter = dynamic_cast<mitk::Surface*>(m_growingCutterNode->GetData())->GetVtkPolyData();

	vtkNew<vtkAppendPolyData> appendFilter;
	appendFilter->AddInputData(initGrowingCutter);
	appendFilter->AddInputData(movedPolyData);
	appendFilter->Update();

	auto tmpSurface = mitk::Surface::New();
	tmpSurface->SetVtkPolyData(appendFilter->GetOutput());
	m_growingCutterNode->SetData(tmpSurface);

}
void HTOrobot::TestCut2()
{
	if (m_Controls.radioButton_testCutting->isChecked() == false)
	{
		return;
	}

	auto initSurface = dynamic_cast<mitk::Surface*>(m_currentSelectedNode->GetData());

	if (initSurface == nullptr)
	{
		return;
	}

	auto initPolyData = initSurface->GetVtkPolyData();

	vtkNew<vtkPolyData> movedPolyData;

	vtkNew<vtkTransformFilter> transformFilter;
	vtkNew<vtkTransform> tmpTransform;
	tmpTransform->SetMatrix(initSurface->GetGeometry()->GetVtkMatrix());
	transformFilter->SetTransform(tmpTransform);
	transformFilter->SetInputData(initPolyData);
	transformFilter->Update();

	movedPolyData->DeepCopy(transformFilter->GetPolyDataOutput());

	auto movedSurface = mitk::Surface::New();
	movedSurface->SetVtkPolyData(movedPolyData);

	if (GetDataStorage()->GetNamedNode("imageToCut") == nullptr)
	{
		return;
	}

	auto imageToCut = GetDataStorage()->GetNamedObject<mitk::Image>("imageToCut");

	auto surfaceToImageFilter = mitk::SurfaceToImageFilter::New();
	//surfaceToImageFilter->SetMakeOutputBinary(false);
	surfaceToImageFilter->SetBackgroundValue(2150);
	//surfaceToImageFilter->SetUShortBinaryPixelType(false);
	surfaceToImageFilter->SetImage(imageToCut);
	surfaceToImageFilter->SetInput(movedSurface);
	surfaceToImageFilter->SetReverseStencil(true);
	surfaceToImageFilter->Update();

	GetDataStorage()->GetNamedNode("imageToCut")->SetData(surfaceToImageFilter->GetOutput());
}
void HTOrobot::OnSelectionChanged(berry::IWorkbenchPart::Pointer /*source*/, const QList<mitk::DataNode::Pointer>& nodes)
{
	std::string nodeName;
	// iterate all selected objects, adjust warning visibility
	foreach(mitk::DataNode::Pointer node, nodes)
	{
		if (node.IsNull())
		{
			return;
		}

		node->GetName(nodeName);

		if (node != nullptr)
		{
			m_currentSelectedNode = node;
			m_baseDataToMove = node->GetData();
		}

	}

}
void HTOrobot::Translate(double direction[3], double length, mitk::BaseData* data)
{
	if (data != nullptr)
	{
		// mitk::Point3D normalizedDirection;
		double directionLength = sqrt((pow(direction[0], 2) + pow(direction[1], 2) + pow(direction[2], 2)));
		// normalizedDirection[0] = direction[0] / directionLength;
		// normalizedDirection[1] = direction[1] / directionLength;
		// normalizedDirection[2] = direction[2] / directionLength;

		mitk::Point3D movementVector;
		movementVector[0] = length * direction[0] / directionLength;
		movementVector[1] = length * direction[1] / directionLength;
		movementVector[2] = length * direction[2] / directionLength;

		auto* doOp = new mitk::PointOperation(mitk::OpMOVE, 0, movementVector, 0);
		// execute the Operation
		// here no undo is stored, because the movement-steps aren't interesting.
		// only the start and the end is interesting to store for undo.
		data->GetGeometry()->ExecuteOperation(doOp);

		delete doOp;

		mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	}
	else
	{
		m_Controls.textBrowser_HTO->append("Empty input. Please select a node ~");
	}
	clock_t start = clock();
	TestCut2();
	MITK_WARN << "Test.Interface.time(TestCut2): " << (clock() - start);

	// TestCut3();

}

void HTOrobot::updateProportation()
{
	auto tmpPointSet = dynamic_cast<mitk::PointSet*>(m_Controls.mitkNodeSelectWidget_featurePoint->GetSelectedNode()->GetData());
	auto imageCheckPoint_0 = tmpPointSet->GetPoint(0);
	mitk::DataStorage::Pointer dataStorage = GetDataStorage();
	mitk::DataNode::Pointer PointSet_Ie_tiabia = dataStorage->GetNamedNode("Line");
	auto tmpPointSet_1 = dynamic_cast<mitk::PointSet*>(PointSet_Ie_tiabia->GetData());
	auto imageCheckPoint_1 = tmpPointSet_1->GetPoint(1);
	auto tmpPointSet2 = dynamic_cast<mitk::PointSet*>(m_Controls.mitkNodeSelectWidget_tibiaPoint->GetSelectedNode()->GetData());
	auto Point_0 = tmpPointSet2->GetPoint(0);
	auto Point_1 = tmpPointSet2->GetPoint(1);
	//double tibia_length = sqrt(pow(Point_0[0] - Point_1[0], 2)+ pow(Point_0[1] - Point_1[1], 2)+ pow(Point_0[2] - Point_1[2], 2));
	double tibia_length = sqrt(pow(Point_0[0] - Point_1[0], 2));
	//力线的长度
	Line_length = sqrt(pow(imageCheckPoint_1[0] - imageCheckPoint_0[0], 2)
		+ pow(imageCheckPoint_1[1] - imageCheckPoint_0[1], 2)
		+ pow(imageCheckPoint_1[2] - imageCheckPoint_0[2], 2));
	double coronalY = 0.0;
	// 计算每条直线在冠状面的投影点
	double line1ProjStart[3] = { imageCheckPoint_0[0], coronalY, imageCheckPoint_0[2] };
	double line1ProjEnd[3] = { imageCheckPoint_1[0], coronalY,imageCheckPoint_1[2] };
	double line2ProjStart[3] = { Point_0[0], coronalY, Point_0[2] };
	double line2ProjEnd[3] = { Point_1[0], coronalY, Point_1[2] };
	
	// 使用VTK数学库计算二维空间内两线段的交点
	double intersection3D[3];
	if (calculateIntersection(line1ProjStart, line1ProjEnd, line2ProjStart, line2ProjEnd, intersection3D)) {
		std::cout << "Intersection in the coronal plane: (" << intersection3D[0] << ", " << intersection3D[1] << ", " << intersection3D[2] << ")" << std::endl;
		//显示占比
		double propatation = sqrt(pow(intersection3D[0] - Point_0[0], 2)) / tibia_length;
		propatation = ceil(propatation * 100);
		m_Controls.lineEdit_proportation->setText(QString::number(propatation));
	}
	else {

		std::cout << "The lines do not intersect in the coronal plane." << std::endl;
		m_Controls.lineEdit_proportation->setText(QString::number(0));
	}
	
	//caculate mPTA
	double vx1 = Point_1[0] - Point_0[0];
	double vy1 = Point_1[1] - Point_0[1];
	double vz1 = Point_1[2] - Point_0[2];
	double length1 = sqrt(pow(vx1, 2) + pow(vy1, 2) + pow(vz1, 2));
	double ux1 = vx1/ length1;
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
	// 计算两个方向向量之间的夹角
	double dotProduct = normal1[0] * normal2[0] + normal1[1] * normal2[1] + normal1[2] * normal[2]; // 点积
	double mPTA = round(acos(dotProduct) * (180.0 / M_PI));
	m_Controls.LineEdit_mPTA->setText(QString::number(mPTA));

}
void HTOrobot::ShowLine()
{
	auto tmpPointSet = dynamic_cast<mitk::PointSet*>(m_Controls.mitkNodeSelectWidget_featurePoint->GetSelectedNode()->GetData());
	auto imageCheckPoint_0 = tmpPointSet->GetPoint(0);
	mitk::DataStorage::Pointer dataStorage = GetDataStorage();
	mitk::DataNode::Pointer PointSet_Ie_tiabia = dataStorage->GetNamedNode("tibiaMalleolusPointSet"); 
	auto tmpPointSet_1 = dynamic_cast<mitk::PointSet*>(PointSet_Ie_tiabia->GetData());
	auto point1 = tmpPointSet_1->GetPoint(0);
	auto point2 = tmpPointSet_1->GetPoint(1);
	mitk::Point3D centerPoint;
	centerPoint[0] = (point1[0] + point2[0]) / 2;
	centerPoint[1] = (point1[1] + point2[1]) / 2;
	centerPoint[2] = (point1[2] + point2[2]) / 2;

	mitk::PointSet::Pointer tmpPointSet_2 = mitk::PointSet::New();
	tmpPointSet_2->SetPoint(0, imageCheckPoint_0);
	tmpPointSet_2->SetPoint(1, centerPoint);

	//查询是否存在名为"Line"的节点
	mitk::DataNode::Pointer existingForceLineNode = dataStorage->GetNamedNode("Line");
	if (existingForceLineNode.IsNotNull())
	{

		existingForceLineNode->SetData(tmpPointSet_2); // 假设tmpPointSet_2是更新后的力线点集
	}
	else
	{

		mitk::DataNode::Pointer line = mitk::DataNode::New();
		line->SetData(tmpPointSet_2);
		line->SetName("Line");


		line->SetBoolProperty("show contour", true);
		line->SetFloatProperty("contoursize", 1.0);
		
		dataStorage->Add(line);

	}

	dataStorage->Modified();
	qDebug() << "line";
	updateProportation();
	return;
}
// 获取股骨头中心
// 表面选点 + 球面中心拟合
// 拟合球体，返回球心和半径
std::pair<Eigen::Vector3d, double> HTOrobot::FitSphere(const std::vector<Eigen::Vector3d>& points)
{
	int n = points.size();
	Eigen::MatrixXd A(n, 4);
	Eigen::VectorXd b(n);

	for (int i = 0; i < n; ++i) {
		const Eigen::Vector3d& p = points[i];
		A(i, 0) = 2 * p.x();
		A(i, 1) = 2 * p.y();
		A(i, 2) = 2 * p.z();
		A(i, 3) = 1;
		b(i) = p.squaredNorm();
	}

	Eigen::Vector4d x = A.colPivHouseholderQr().solve(b);
	Eigen::Vector3d center(x[0], x[1], x[2]);
	double radius = std::sqrt(x[3] + center.squaredNorm());

	return { center, radius };
}
bool HTOrobot::OnFemurCenterClicked()
{
	// Action
	m_Controls.textBrowser_HTO->append("Action: Get Femur Center.");

	try {
		// 从库中读取股骨头表面手动标记的点
		auto surfacePointSetNode = GetDataStorage()->GetNamedNode("femoralHead");
		if (!surfacePointSetNode) {
			throw std::runtime_error("Error: Could not find femoral head point set node.");
		}
		auto surfacePointSet = dynamic_cast<mitk::PointSet*>(surfacePointSetNode->GetData());
		if (!surfacePointSet) {
			throw std::runtime_error("Error: Could not get point set data from node.");
		}

		std::vector<Eigen::Vector3d> femoralHeadPoints;
		for (auto it = surfacePointSet->Begin(); it != surfacePointSet->End(); ++it) {
			auto point = it->Value();
			femoralHeadPoints.emplace_back(point[0], point[1], point[2]);
		}

		// 拟合球体并计算球心
		std::vector<Eigen::Vector3d> centers;
		int numPoints = femoralHeadPoints.size();
		for (int i = 0; i < numPoints; ++i) {
			std::vector<Eigen::Vector3d> subsetPoints;
			for (int j = 0; j < numPoints; ++j) {
				if (j != i) {
					subsetPoints.push_back(femoralHeadPoints[j]);
				}
			}
			auto [center, _] = FitSphere(subsetPoints);
			centers.push_back(center);
		}

		// 求所有球心的平均值
		if (centers.empty()) {
			throw std::runtime_error("Error: No valid groups of points to fit spheres.");
		}

		Eigen::Vector3d averageCenter(0, 0, 0);
		for (const auto& center : centers) {
			averageCenter += center;
		}
		averageCenter /= centers.size();

		// 输出股骨头中心
		m_Controls.textBrowser_HTO->append(QString("Femur Head Center: (%1, %2, %3)").arg(averageCenter.x()).arg(averageCenter.y()).arg(averageCenter.z()));
		// 删除之前的中心点节点
		auto previousCenterNode = GetDataStorage()->GetNamedNode("hipCenterPoint");
		if (previousCenterNode) {
			GetDataStorage()->Remove(previousCenterNode);
		}

		// 创建一个新的点集并添加中心点
		mitk::PointSet::Pointer centerPointSet = mitk::PointSet::New();
		mitk::Point3D centerPoint;
		centerPoint[0] = averageCenter.x();
		centerPoint[1] = averageCenter.y();
		centerPoint[2] = averageCenter.z();
		centerPointSet->InsertPoint(0, centerPoint);

		// 创建一个新的数据节点并设置点集
		mitk::DataNode::Pointer centerPointNode = mitk::DataNode::New();
		centerPointNode->SetData(centerPointSet);
		centerPointNode->SetName("hipCenterPoint");
		centerPointNode->SetProperty("pointsize", mitk::FloatProperty::New(5.0));
		// 将数据节点添加到数据存储中
		GetDataStorage()->Add(centerPointNode);

		// 设置模型的基本参数，股骨透明度、节点大小等
		auto femurSurface = GetDataStorage()->GetNamedNode("femurSurface");
		//femurSurface->SetColor(0.0, 1.0, 0.0);
		femurSurface->SetOpacity(0.5f);


	}
	catch (const std::exception & e) {
		m_Controls.textBrowser_HTO->append(QString("Error: %1").arg(e.what()));
		return false;
	}
	return true;
}

bool HTOrobot::calculateIntersection(double p1[3], double p2[3], double p3[3], double p4[3], double intersection[3])
{
	double denominator = ((p4[2] - p3[2]) * (p2[0] - p1[0])) - ((p4[0] - p3[0]) * (p2[2] - p1[2]));

	if (denominator == 0) {
		// 线段平行或重合，没有交点
		return false;
	}

	double ua = ((p4[0] - p3[0]) * (p1[2] - p3[2])) - ((p4[2] - p3[2]) * (p1[0] - p3[0]));
	double ub = ((p2[0] - p1[0]) * (p1[2] - p3[2])) - ((p2[2] - p1[2]) * (p1[0] - p3[0]));

	ua /= denominator;
	ub /= denominator;

	// 检查交点是否在线段内部
	if (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1) {
		intersection[0] = p1[0] + ua * (p2[0] - p1[0]);
		intersection[2] = p1[2] + ua * (p2[2] - p1[2]);
		// 对于冠状面，Z坐标已知，直接赋值
		intersection[1] = 0.0; // 假设coronalZ是您之前定义的冠状面Z值
		return true;
	}

	// 交点不在线段之间
	return false;
}

void HTOrobot::UnShowLine()
{
	mitk::DataStorage::Pointer dataStorage = GetDataStorage();
	mitk::DataNode::Pointer existingForceLineNode = dataStorage->GetNamedNode("Line");


	if (existingForceLineNode.IsNotNull()) {
		dataStorage->Remove(existingForceLineNode);
		m_Controls.lineEdit_proportation->clear();
		existingForceLineNode->SetBoolProperty("show contour", false);
	}



}