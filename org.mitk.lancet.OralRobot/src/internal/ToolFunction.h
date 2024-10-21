#pragma once
#ifndef ToolFunction_h
#define ToolFunction_h


#include <QmitkAbstractView.h>

#include <iostream>
#include <cstring>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <QTextBrowser>

#include <QtConcurrent>

class ToolFunction :public QObject {
	Q_OBJECT
public:
	ToolFunction();
	~ToolFunction();
	void PrintMatrix(QTextBrowser* browser, std::string matrix_name, double matrix[16]);
	void PrintMatrix(QTextBrowser* browser, std::string matrix_name, vtkMatrix4x4* matrix);
	void PrintMatrix(QTextBrowser* browser, std::string matrix_name, vtkNew<vtkMatrix4x4> matrix);
	void PrintMatrix(QTextBrowser* browser, std::string matrix_name, vtkSmartPointer<vtkMatrix4x4> matrix);
	void PrintTCP(QTextBrowser* browser, int result, std::array<double, 6>& TCP);
	void PrintUCS(QTextBrowser* browser, int result, std::array<double, 6>& UCS);
	void PrintJoint(QTextBrowser* browser, int result, std::array<double, 6>& Joint);
	void PrintFreedom(QTextBrowser* browser, int result, std::array<double, 6>& Freedom);
	void PrintTCP(QTextBrowser* browser, int result, double dX, double dY, double dZ, double dRx, double dRy, double dRz);
	void PrintUCS(QTextBrowser* browser, int result, double dX, double dY, double dZ, double dRx, double dRy, double dRz);
	void PrintJoint(QTextBrowser* browser, int result, double dX, double dY, double dZ, double dRx, double dRy, double dRz);
	void PrintFreedom(QTextBrowser* browser, int result, double dX, double dY, double dZ, double dRx, double dRy, double dRz);

	void PrintError(QTextBrowser* browser, int nRet, std::string behavior);


	void SaveFiles(QTextBrowser* browser, std::string filecatalog, double matrix[16]);
	void SaveFiles(QTextBrowser* browser, std::string filecatalog, vtkMatrix4x4* matrix);
	void SaveFiles(QTextBrowser* browser, std::string filecatalog, vtkNew<vtkMatrix4x4> matrix);
	void SaveFiles(QTextBrowser* browser, std::string filecatalog, vtkSmartPointer<vtkMatrix4x4> matrix);
	
	void ReuseFiles(QTextBrowser* browser, std::string filecatalog, double matrix[16]);
	void ReuseFiles(QTextBrowser* browser, std::string filecatalog, vtkMatrix4x4* matrix);
	void ReuseFiles(QTextBrowser* browser, std::string filecatalog, vtkNew<vtkMatrix4x4> matrix);
	void ReuseFiles(QTextBrowser* browser, std::string filecatalog, vtkSmartPointer<vtkMatrix4x4> matrix);

	void MultiplyMatrices(vtkTransform* matrix, vtkTransform* matrix_1to2,
		vtkTransform* matrix_2to3, vtkTransform* matrix_3to4, vtkTransform* matrix_4to5,
		vtkTransform* matrix_5to6, vtkTransform* matrix_6to7, vtkTransform* matrix_7to8);
	void MultiplyMatrices(vtkTransform* matrix, vtkTransform* matrix_1to2,
		vtkTransform* matrix_2to3, vtkTransform* matrix_3to4, vtkTransform* matrix_4to5,
		vtkTransform* matrix_5to6, vtkTransform* matrix_6to7);
	void MultiplyMatrices(vtkTransform* matrix, vtkTransform* matrix_1to2,
		vtkTransform* matrix_2to3, vtkTransform* matrix_3to4, vtkTransform* matrix_4to5,
		vtkTransform* matrix_5to6);
	void MultiplyMatrices(vtkTransform* matrix, vtkTransform* matrix_1to2,
		vtkTransform* matrix_2to3, vtkTransform* matrix_3to4, vtkTransform* matrix_4to5);
	void MultiplyMatrices(vtkTransform* matrix, vtkTransform* matrix_1to2,
		vtkTransform* matrix_2to3, vtkTransform* matrix_3to4);
	void MultiplyMatrices(vtkTransform* matrix, vtkTransform* matrix_1to2, vtkTransform* matrix_2to3);

	void MultiplyMatrices(vtkTransform* matrix, vtkMatrix4x4* matrix_1to2,
		vtkMatrix4x4* matrix_2to3, vtkMatrix4x4* matrix_3to4, vtkMatrix4x4* matrix_4to5,
		vtkMatrix4x4* matrix_5to6, vtkMatrix4x4* matrix_6to7, vtkMatrix4x4* matrix_7to8);
	void MultiplyMatrices(vtkTransform* matrix, vtkMatrix4x4* matrix_1to2,
		vtkMatrix4x4* matrix_2to3, vtkMatrix4x4* matrix_3to4, vtkMatrix4x4* matrix_4to5,
		vtkMatrix4x4* matrix_5to6, vtkMatrix4x4* matrix_6to7);
	void MultiplyMatrices(vtkTransform* matrix, vtkMatrix4x4* matrix_1to2,
		vtkMatrix4x4* matrix_2to3, vtkMatrix4x4* matrix_3to4, vtkMatrix4x4* matrix_4to5,
		vtkMatrix4x4* matrix_5to6);
	void MultiplyMatrices(vtkTransform* matrix, vtkMatrix4x4* matrix_1to2,
		vtkMatrix4x4* matrix_2to3, vtkMatrix4x4* matrix_3to4, vtkMatrix4x4* matrix_4to5);
	void MultiplyMatrices(vtkTransform* matrix, vtkMatrix4x4* matrix_1to2,
		vtkMatrix4x4* matrix_2to3, vtkMatrix4x4* matrix_3to4);
	void MultiplyMatrices(vtkTransform* matrix, vtkMatrix4x4* matrix_1to2, vtkMatrix4x4* matrix_2to3);

	void CombineRotationTranslation(float Rto[3][3], float Tto[3], vtkMatrix4x4* resultMatrix);

	void Double4x4ToMatrix4x4(double matrix[16], vtkMatrix4x4* vtkmatrix);
	void Matrix4x4ToDouble4x4(double matrix[16], vtkMatrix4x4* vtkmatrix);

public slots:
	void Sleep(int msec);

};
#endif ToolFunction_h