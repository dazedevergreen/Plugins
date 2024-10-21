#pragma once
#ifndef ToolFunction_h
#define ToolFunction_h


#include <QmitkAbstractView.h>

#include <iostream>
#include <cstring>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <QTextBrowser>
#include <filesystem>

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
	void PrintXYZRXRYRZ(QTextBrowser* browser, int result, std::array<double, 6>& TCP, std::string dataname);

	void PrintXYZRXRYRZ(QTextBrowser* browser, int result, double dX, double dY, double dZ, double dRx, double dRy, double dRz, std::string dataname);


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

class TeeStreambuf : public std::streambuf {
public:
	TeeStreambuf(std::streambuf* buf1, std::streambuf* buf2)
		: stream1(buf1), stream2(buf2), is_new_line(true) {}

protected:
	// ��д overflow ��������ÿ���ַ�д��������������
	virtual int overflow(int c) override {
		if (c == EOF) {
			return EOF;
		}
		else {
			// ������µ�һ�У������ʱ���
			if (is_new_line && c != '\n') {
				writeTimestamp();
				is_new_line = false;  // ���ʱ����󽫱�־����Ϊ false
			}

			// д���ַ�������������
			if (stream1->sputc(c) == EOF || stream2->sputc(c) == EOF) {
				return EOF;
			}

			// ����ǻ��з���������һ����Ҫ���ʱ���
			if (c == '\n') {
				is_new_line = true;
			}

			return c;
		}
	}

	// ��д sync ������ȷ������������ͬ��
	virtual int sync() override {
		int result1 = stream1->pubsync();
		int result2 = stream2->pubsync();
		return (result1 == 0 && result2 == 0) ? 0 : -1;
	}

private:
	std::streambuf* stream1;
	std::streambuf* stream2;
	bool is_new_line;  // ���ڸ����Ƿ������еĿ�ͷ

	// д��ʱ�������־�ļ�
	void writeTimestamp() {
		// ��ȡ��ǰʱ��
		auto t = std::time(nullptr);
		auto tm = *std::localtime(&t);

		// ��ʽ��ʱ��Ϊ�ַ���
		std::ostringstream timestamp;
		timestamp << "[" << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << "] ";

		std::string ts = timestamp.str();
		for (char ch : ts) {
			stream2->sputc(ch);  // ֻд����־����������д���նˣ�
		}
	}
};
#endif ToolFunction_h