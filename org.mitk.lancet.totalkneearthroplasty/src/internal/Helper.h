#ifndef HELPER_H
#define HELPER_H

// mitk image
#include <mitkImage.h>
#include <mitkAffineTransform3D.h>
#include <mitkMatrixConvert.h>
#include <mitkPoint.h>
#include <mitkSurface.h>
#include <QmitkRenderWindow.h>
#include <mitkIRenderWindowPart.h>
#include "surfaceregistraion.h"

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

/// Data Type Conversion

// convert "nd data" to "vtkMatrix4x4"
vtkSmartPointer<vtkMatrix4x4> getVtkMatrix4x4(mitk::NavigationData::Pointer nd);

// Convert "Double" To "vtkMatrix4x4"
vtkSmartPointer<vtkMatrix4x4> DoubleToVtkMatrix(const double elements[16]);

// Coordinate Conversion, convert the nd data to nd_ref Coordinate.
mitk::NavigationData::Pointer GetNavigationDataInRef(mitk::NavigationData::Pointer nd,
	mitk::NavigationData::Pointer nd_ref);

// Coordinate Conversion, convert the matrix to matrixRF Coordinate. (vtxmatrix4x4)
vtkSmartPointer<vtkMatrix4x4> getVtkMatrix4x4InRef(vtkSmartPointer<vtkMatrix4x4> matrix, vtkSmartPointer<vtkMatrix4x4> matrixRF);

// Convert "Translation[3] & Roatation[3][3]" To "vtkMatrix4x4"
void CombineRotationTranslation(float Rto[3][3], float Tto[3], vtkSmartPointer<vtkMatrix4x4> resultMatrix);

// Compute point to point Euclidean Distance 
double EuclideanDistance(const mitk::Point3D& p1, const mitk::Point3D& p2);
double EuclideanDistance(const double p1[3], const double p2[3]);

// Mouse Controled by Euler Angles
void MoveMouseToPosition(POINT position);
POINT CalculateMousePosition(double pitch, double roll, int screenWidth, int screenHeight); 
double Lerp(double a, double b, double t); // interpolation
void SmoothMoveMouseToPosition(POINT start, POINT end);  // smooth

// get Center and Normal from Surface
void ComputeCenterAndNormalFromSurface(mitk::Surface::Pointer surface, double center[3], double normal[3]);

//// get Normal from three points in the plane/surface
void ComputeNormalFromThreePoints(double p1[3], double p2[3], double p3[3], double normal[3]);

/// adjust the position of crosshair
void AdjustCrosshair(mitk::RenderingManager::Pointer renderingManager, double newPosition[3], double newNormal[3], std::string tarPlane = "Axial");

// save Tranform matrix in txt file
std::string saveTransMtxInTxtFile(std::string abspath, std::string folderName, double mtx[16]); // return the status info.
// reload the tranform matix in txt file
std::string reloadTransMtxInTxtFile(std::string abspath, std::string folderName, double mtx[16]); // return the status info.
#endif // HELPER_H