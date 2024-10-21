// mitk image
#include <mitkImage.h>
#include <mitkAffineTransform3D.h>
#include <mitkMatrixConvert.h>
#include <mitkPoint.h>
#include <mitkSurface.h>
#include <QmitkRenderWindow.h>
#include <mitkIRenderWindowPart.h>
#include "surfaceregistraion.h"
#include <mitkCameraController.h>

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

#include <windows.h>
#include <iostream>

#include <fstream>
#include <string>
#include <cstdlib>
#include <filesystem>

// Coordinate Conversion, convert the nd data to nd_ref Coordinate.
mitk::NavigationData::Pointer GetNavigationDataInRef(mitk::NavigationData::Pointer nd,
	mitk::NavigationData::Pointer nd_ref)
{
	mitk::NavigationData::Pointer res = mitk::NavigationData::New();
	res->Graft(nd);
	res->Compose(nd_ref->GetInverse());
	return res;
}

vtkSmartPointer<vtkMatrix4x4> getVtkMatrix4x4InRef(vtkSmartPointer<vtkMatrix4x4> matrix, vtkSmartPointer<vtkMatrix4x4> matrixRF)
{

	vtkSmartPointer<vtkMatrix4x4> inv_matrixRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Invert(matrixRF, inv_matrixRF);

	vtkSmartPointer<vtkTransform> tmpVtkTransform = vtkTransform::New();
	tmpVtkTransform->PostMultiply();
	tmpVtkTransform->Identity();
	tmpVtkTransform->SetMatrix(matrix);
	tmpVtkTransform->Concatenate(inv_matrixRF);

	return tmpVtkTransform->GetMatrix();
}

// Coordinate Conversion, convert the nd data to nd_ref Coordinate.


//// Data Type/Format Conversion  ////

// convert "nd data" to "vtkMatrix4x4"
vtkSmartPointer<vtkMatrix4x4> getVtkMatrix4x4(mitk::NavigationData::Pointer nd)
{
	auto o = nd->GetOrientation();
	double R[3][3];
	double* V = { nd->GetPosition().GetDataPointer() };
	vtkQuaterniond quaterniond{ o.r(), o.x(), o.y(), o.z() };
	quaterniond.ToMatrix3x3(R);

	vtkSmartPointer<vtkMatrix4x4> matrix = vtkMatrix4x4::New();
	matrix->SetElement(0, 0, R[0][0]);
	matrix->SetElement(0, 1, R[0][1]);
	matrix->SetElement(0, 2, R[0][2]);
	matrix->SetElement(1, 0, R[1][0]);
	matrix->SetElement(1, 1, R[1][1]);
	matrix->SetElement(1, 2, R[1][2]);
	matrix->SetElement(2, 0, R[2][0]);
	matrix->SetElement(2, 1, R[2][1]);
	matrix->SetElement(2, 2, R[2][2]);

	matrix->SetElement(0, 3, V[0]);
	matrix->SetElement(1, 3, V[1]);
	matrix->SetElement(2, 3, V[2]);

	//matrix->Print(std::cout);
	return matrix;
}

// Convert Double To VtkMatrix data type
vtkSmartPointer<vtkMatrix4x4> DoubleToVtkMatrix(const double elements[16])
{
	vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	matrix->DeepCopy(elements);
	/*for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			matrix->SetElement(i, j, elements[i * 4 + j]);
		}
	}*/
	return matrix;
}


// Convert "Translation[3] & Roatation[3][3]" To "vtkMatrix4x4"
void CombineRotationTranslation(float Rto[3][3], float Tto[3], vtkSmartPointer<vtkMatrix4x4> resultMatrix)
{
	// Set rotation part
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			resultMatrix->SetElement(i, j, Rto[i][j]);
		}
	}
	for (int i = 0; i < 3; ++i)
	{
		resultMatrix->SetElement(i, 3, Tto[i]);
	}

}

//// Data Type/Format Conversion  ///

/// Euclidean Distance
double EuclideanDistance(const mitk::Point3D& p1, const mitk::Point3D& p2) {
	return std::sqrt(
		std::pow(p2[0] - p1[0], 2) +
		std::pow(p2[1] - p1[1], 2) +
		std::pow(p2[2] - p1[2], 2)
	);
}

double EuclideanDistance(const double p1[3], const double p2[3]) {
	return std::sqrt(
		std::pow(p2[0] - p1[0], 2) +
		std::pow(p2[1] - p1[1], 2) +
		std::pow(p2[2] - p1[2], 2)
	);
}

//// Probe Mouse  ///

// compute the mouse position in window screen
POINT CalculateMousePosition(double pitch, double roll, int screenWidth, int screenHeight) {
	// limit the range between [];
	pitch *= -1.0; // reverse.
	pitch = std::max(-60.0, std::min(0.0, pitch));
	roll = std::max(-30.0, std::min(30.0, roll));

	// mapping 
	int x = static_cast<int>(((pitch + 60) / 60.0) * screenWidth);
	int y = static_cast<int>(((roll + 30) / 60.0) * screenHeight);

	// make sure the mouse in window screen  
	x = std::max(0, std::min(screenWidth - 1, x));
	y = std::max(0, std::min(screenHeight - 1, y));

	return { x, y };
}

// move the mouse
void MoveMouseToPosition(POINT position) {
	SetCursorPos(position.x, position.y);
}

// interpolation
double Lerp(double a, double b, double t) {
	return a + (b - a) * t;
}

// smoothing
void SmoothMoveMouseToPosition(POINT start, POINT end) {
	double t = 0.6;
	int x = static_cast<int>(Lerp(start.x, end.x, t));
	int y = static_cast<int>(Lerp(start.y, end.y, t));
	SetCursorPos(x, y);
}

//// Probe Mouse  ///


//// get Center and Normal from Surface
void ComputeCenterAndNormalFromSurface(mitk::Surface::Pointer surface, double center[3], double normal[3])
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


//// get Normal from three points in the plane/surface
void ComputeNormalFromThreePoints(double p1[3], double p2[3], double p3[3], double normal[3])
{
	double v1[3], v2[3];
	vtkMath::Subtract(p2, p1, v1); // v1 = p2 - p1
	vtkMath::Subtract(p3, p1, v2); // v2 = p3 - p1

	vtkMath::Cross(v1, v2, normal);
	vtkMath::Normalize(normal);
}


/// adjust the position of crosshair
void AdjustCrosshair(mitk::RenderingManager::Pointer renderingManager, double center[3], double normal[3], std::string tarPlane)
{
	/* 
	   stdmulti.widget0: Axial
	   stdmulti.widget1: Sagittal
	   stdmulti.widget2: Coronal
	   stdmulti.widget3: 3D view 
	*/

	std::string sameWithNormal = "stdmulti.widget0", tmpName = "stdmulti.widget2";
	if (tarPlane == "Coronal") {
		sameWithNormal = "stdmulti.widget2"; // set planning plane normal to "Coronal" view nomal. ex: femur anterior planning plane
		tmpName = "stdmulti.widget0";
	}

	mitk::Point3D newPosition;
	newPosition[0] = center[0]; newPosition[1] = center[1]; newPosition[2] = center[2];
	mitk::Vector3D newNormal, newNormal1;

	vtkMath::Normalize(normal);

	double v[3] = { 1.0, 0.0, 0.0 }; // x axis

	// y_axis = v x z_axis(normal)
	double y_axis[3];
	vtkMath::Cross(v, normal, y_axis);
	vtkMath::Normalize(y_axis);

	// 
	double x_axis[3];
	vtkMath::Cross(normal, y_axis, x_axis);
	vtkMath::Normalize(x_axis);
	auto allRenderWindows = renderingManager->GetAllRegisteredRenderWindows();
	for (auto renderWindow : allRenderWindows)
	{
		auto renderer = mitk::BaseRenderer::GetInstance(renderWindow);
		if (renderer)
		{
			mitk::SliceNavigationController* snc = renderer->GetSliceNavigationController();
			if (snc)
			{
				std::string windowName = renderer->GetName();
				if (windowName == sameWithNormal) // Axial OR Coronal
				{
					newNormal[0] = normal[0]; newNormal[1] = normal[1]; newNormal[2] = normal[2];
					snc->ReorientSlices(newPosition, newNormal);
				}
				else if (windowName == "stdmulti.widget1")// Sagittal
				{
					newNormal[0] = x_axis[0]; newNormal[1] = x_axis[1]; newNormal[2] = x_axis[2];
					snc->ReorientSlices(newPosition, newNormal);
				}
				else if (windowName == tmpName) // Coronal OR Axial
				{   
					/*newNormal[0] = normal[0]; newNormal[1] = normal[1]; newNormal[2] = normal[2];
					newNormal1[0] = sagittal_normal[0]; newNormal1[1] = sagittal_normal[1]; newNormal1[2] = sagittal_normal[2];
					snc->ReorientSlices(newPosition, newNormal, newNormal1);*/

					newNormal[0] = y_axis[0]; newNormal[1] = y_axis[1]; newNormal[2] = y_axis[2];
					snc->ReorientSlices(newPosition, newNormal);
				}
				snc->GetRenderer()->GetCameraController()->Fit();
				//MITK_INFO << windowName;

				//MITK_INFO << newNormal[0] << " " << newNormal[1] << " " << newNormal[1];
			}
		}
	}
	renderingManager->RequestUpdateAll();
}

// save tranform matrix in txt file
std::string saveTransMtxInTxtFile(std::string abspath, std::string folderName, double mtx[16])
{
	try {
		// Create directory if it does not exist
		std::string path = abspath + folderName;
		if (!std::filesystem::exists(path))
		{
			std::filesystem::create_directories(path);
		}

		// current time
		std::time_t now = std::time(nullptr);
		std::tm* localTime = std::localtime(&now);

		// define filename
		std::ostringstream filename;
		filename << path << "\\" << folderName << "_"
			<< std::put_time(localTime, "%Y%m%d_%H%M%S")
			<< ".txt";

		// Open file and write matrix data
		std::ofstream outFile(filename.str());
		if (outFile.is_open()) {
			for (int i = 0; i < 16; i++)
			{
				outFile << mtx[i];
				if (i != 15) outFile << ",";
			}
			outFile << std::endl;
			outFile.close();
			//std::cout << "Matrix saved to " << filename.str() << std::endl;
			return "Matrix saved to " + filename.str();
		}
		else {
			return "Error opening file for writing.";
			//std::cerr << "Error opening file for writing." << std::endl;
		}
	}
	catch (const std::exception & e)
	{
		std::cerr << "Error: " << e.what() << std::endl;
		return "Failed to save Transofrm Matrix result";
	}
}

// reload the transform matrix in txt file
std::string reloadTransMtxInTxtFile(std::string abspath, std::string folderName, double mtx[16])
{
	std::string path = abspath + folderName;
	if (!std::filesystem::exists(path))
	{
		return "Path does not exist";
	}

	QString filename = QFileDialog::getOpenFileName(nullptr, QObject::tr("Open Transform Matrix"), path.c_str(),
		QObject::tr("Text Files (*.txt)"));

	if (!filename.isEmpty()) {
		std::string file = filename.toStdString();
		std::ifstream inFile(file);
		std::string line;

		if (inFile.is_open()) {
			if (std::getline(inFile, line))
			{
				std::stringstream ss(line);
				std::string token;
				int index = 0;
				while (std::getline(ss, token, ','))
				{
					if (index < 16) mtx[index++] = std::stod(token);
					else break;
				}
			}
			inFile.close();

			return "Matrix loaded successfully from " + file;
		}
		else {
			//std::cerr << "Error opening file for reading." << std::endl;
			return "Error opening file for reading.";
		}
	}
	else {
		return "No file selected.";
	}
}