#include "DentalRobot.h"

#include <fstream>
#include <iostream>
#include <string>
#include <cstdlib>
#include <filesystem>


// mitk image

//#include "leastsquaresfit.h"
//#include "mitkGizmo.h"
#include "surfaceregistraion.h"
#include <vtkSphere.h>
#include <vtkLandmarkTransform.h>
#include "mitkNodePredicateAnd.h"
#include "mitkNodePredicateDataType.h"
#include "mitkNodePredicateNot.h"
#include "mitkNodePredicateOr.h"
#include "mitkNodePredicateProperty.h"
#include <mitkImageAccessByItk.h>
#include <vtkIterativeClosestPointTransform.h>
#include "PrintDataHelper.h"
#include <iomanip>
#include <mitkPoint.h>

#include <fstream>
#include <iostream>
#include <string>
#include <cstdlib>
#include <filesystem>


void DentalRobot::calibrateTCP()
{
	//选择点集
	if (GetDataStorage()->GetNamedNode("probe_head_tail_mandible") == nullptr ||
		GetDataStorage()->GetNamedNode("probe_head_tail_maxilla") == nullptr)
	{
		m_Controls.textBrowser->append("probe_head_tail_mandible or probe_head_tail_maxilla is missing!");
		return;
	}
	auto probe_head_tail_mandible = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("probe_head_tail_mandible")->GetData());
	auto probe_head_tail_maxilla = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("probe_head_tail_maxilla")->GetData());
	
	if (probe_head_tail_mandible->GetSize() != 2 || probe_head_tail_maxilla->GetSize() != 2)
	{
		m_Controls.textBrowser->append("probe_head_tail_mandible or probe_head_tail_maxilla is problematic!");
		return;
	}
	auto probe_head_tail = mitk::PointSet::New();
	
	if (m_Controls.radioButton_maxilla_2->isChecked())
	{
		probe_head_tail = probe_head_tail_maxilla;
	}
	else
	{
		probe_head_tail = probe_head_tail_mandible;
	}

	//计算探针方向向量并标准化处理
	auto probe_head = probe_head_tail->GetPoint(0);
	auto probe_tail = probe_head_tail->GetPoint(1);
	Eigen::Vector3d z_probeInCalibratorRF;
	z_probeInCalibratorRF[0] = probe_tail[0] - probe_head[0];
	z_probeInCalibratorRF[1] = probe_tail[1] - probe_head[1];
	z_probeInCalibratorRF[2] = probe_tail[2] - probe_head[2];
	z_probeInCalibratorRF.normalize();
	//定义标准向量，计算探针向量和标准向量的差异
	Eigen::Vector3d z_std{ 0, 0, 1 };
	Eigen::Vector3d rotAxis = z_std.cross(z_probeInCalibratorRF);//旋转轴：即垂直于z_std和z_probeInCalibratorRF的向量
	rotAxis.normalize();//标准化
	if (rotAxis.norm() < 0.00001)
	{
		rotAxis[0] = 1;
		rotAxis[1] = 0;
		rotAxis[2] = 0;
	}
	//求出所需要旋转的角度
	double rotAngle = 180 * acos(z_std.dot(z_probeInCalibratorRF)) / 3.141592654;//转化为弧度
	//进行角度的转化
	auto trans_calibratorRFtoTCP = vtkTransform::New();
	trans_calibratorRFtoTCP->Identity();
	trans_calibratorRFtoTCP->PostMultiply();
	trans_calibratorRFtoTCP->RotateWXYZ(rotAngle, rotAxis[0], rotAxis[1], rotAxis[2]);
	trans_calibratorRFtoTCP->Update();

	auto T_CalibratorRFtoTCP = trans_calibratorRFtoTCP->GetMatrix();
	T_CalibratorRFtoTCP->SetElement(0, 3, probe_head[0]);
	T_CalibratorRFtoTCP->SetElement(1, 3, probe_head[1]);
	T_CalibratorRFtoTCP->SetElement(2, 3, probe_head[2]);

	memcpy_s(m_T_calibratorRFToTCP, sizeof(double) * 16, T_CalibratorRFtoTCP->GetData(), sizeof(double) * 16);

	//计算转化矩阵
	//1.计算T_EndRFtoCalibratorRF(从相机拿数据)
	auto T_CameratoCalibratorRF = vtkMatrix4x4::New();
	T_CameratoCalibratorRF->DeepCopy(m_T_CamToCalibratorRF);

	auto T_EndRFtoCamera = vtkMatrix4x4::New();
	T_EndRFtoCamera->DeepCopy(m_T_CamToEndRF);
	T_EndRFtoCamera->Invert();

	auto trans_EndRFtoCalibratorRF = vtkTransform::New();
	trans_EndRFtoCalibratorRF->Identity();
	trans_EndRFtoCalibratorRF->PostMultiply();
	trans_EndRFtoCalibratorRF->SetMatrix(T_CameratoCalibratorRF);
	trans_EndRFtoCalibratorRF->Concatenate(T_EndRFtoCamera);
	trans_EndRFtoCalibratorRF->Update();
	auto T_EndRFtoCalibratorRF = trans_EndRFtoCalibratorRF->GetMatrix();
	memcpy_s(m_T_EndRFToCalibratorRF, sizeof(double) * 16, T_EndRFtoCalibratorRF->GetData(), sizeof(double) * 16);


	//2.计算法兰到TCP的转化矩阵T_FlangetoTCP
	auto T_FlangetoEndRF = vtkMatrix4x4::New();
	T_FlangetoEndRF->DeepCopy(m_T_FlangeToEdnRF);

	auto trans_FlangetoTCP = vtkTransform::New();
	trans_FlangetoTCP->Identity();
	trans_FlangetoTCP->PostMultiply();
	trans_FlangetoTCP->SetMatrix(T_CalibratorRFtoTCP);
	trans_FlangetoTCP->Concatenate(T_CameratoCalibratorRF);
	trans_FlangetoTCP->Concatenate(T_EndRFtoCamera);
	trans_FlangetoTCP->Concatenate(T_FlangetoEndRF);
	trans_FlangetoTCP->Update();
	auto T_FlangetoTCP = trans_FlangetoTCP->GetMatrix();
	memcpy_s(m_T_FlangeToTCP, sizeof(double) * 16, T_FlangetoTCP->GetData(), sizeof(double) * 16);

	//机械臂处理
	Eigen::Vector3d RX;
	Eigen::Vector3d RY;
	Eigen::Vector3d RZ;
	Eigen::Matrix3d Re;

	RX[0] = T_FlangetoTCP->GetElement(0, 0);
	RX[1] = T_FlangetoTCP->GetElement(1, 0);
	RX[2] = T_FlangetoTCP->GetElement(2, 0);

	RY[0] = T_FlangetoTCP->GetElement(0, 1);
	RY[1] = T_FlangetoTCP->GetElement(1, 1);
	RY[2] = T_FlangetoTCP->GetElement(2, 1);

	RZ[0] = T_FlangetoTCP->GetElement(0, 2);
	RZ[1] = T_FlangetoTCP->GetElement(1, 2);
	RZ[2] = T_FlangetoTCP->GetElement(2, 2);

	Re << RX[0], RY[0], RZ[0],
		RX[1], RY[1], RZ[1],
		RX[2], RY[2], RZ[2];

	//欧拉角顺序zyx
	Eigen::Vector3d eulerAngle = Re.eulerAngles(2, 1, 0);

	//提取X，Y，Z分量，并将欧拉角从弧度转化为度
	finaltcp[0] = T_FlangetoTCP->GetElement(0, 3);
	finaltcp[1] = T_FlangetoTCP->GetElement(1, 3);
	finaltcp[2] = T_FlangetoTCP->GetElement(2, 3);
	finaltcp[3] = vtkMath::DegreesFromRadians(eulerAngle(2));
	finaltcp[4] = vtkMath::DegreesFromRadians(eulerAngle(1));
	finaltcp[5] = vtkMath::DegreesFromRadians(eulerAngle(0));

	m_Controls.textBrowser->append("===== Tcp x,y,z =====");
	m_Controls.textBrowser->append(QString::number(finaltcp[0]) + "/" +
	QString::number(finaltcp[1]) + "/" + QString::number(finaltcp[2]));

	m_Controls.textBrowser->append("===== Tcp rx,ry,rz ====");
	m_Controls.textBrowser->append(QString::number(finaltcp[3]) + "/" +
		QString::number(finaltcp[4]) + "/" + QString::number(finaltcp[5]));

	m_Controls.textBrowser->append("==== Robot Calibration Succeeded! ====");

	PrintTCP_qt(m_Controls.textBrowser, finaltcp[0], finaltcp[1], finaltcp[2], finaltcp[3], finaltcp[4], finaltcp[5]);
	int nRet = HRIF_SetTCPByName(0, 0, sTcpName);
	int nRet1 = HRIF_SetTCP(0, 0, finaltcp[0], finaltcp[1], finaltcp[2], finaltcp[3], finaltcp[4], finaltcp[5]);
	PrintResult(m_Controls.textBrowser, nRet1, "TCP Set");
	if (nRet1 == 0) 
	{
		m_Controls.textBrowser->append("==== set TCP Succeeded! ====");
	}
	else
	{
		m_Controls.textBrowser->append("==== set TCP Failed! ====");
	}
}

void DentalRobot::PrintTCP_qt(QTextBrowser* browser, double x, double y, double z, double rx, double ry, double rz)
{
	browser->append("---------------------------------------------------");
	browser->append("dX_tcp=" + QString::number(x));
	browser->append("dY_tcp=" + QString::number(y));
	browser->append("dZ_tcp=" + QString::number(z));
	browser->append("dRx_tcp=" + QString::number(rx));
	browser->append("dRy_tcp=" + QString::number(ry));
	browser->append("dRz_tcp=" + QString::number(rz));
	browser->append("---------------------------------------------------");
}

void DentalRobot::saveTCPCalibrateMatrix()
{
	try
	{
		std::string desktoppath = std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\DIRobot\\TCPCalibrate.txt";
		std::filesystem::path dir = std::filesystem::path(desktoppath).parent_path();
		if (!std::filesystem::exists(dir))
		{
			std::filesystem::create_directories(dir);
		}
		std::ofstream robotMatrixFile(desktoppath);
		if (!robotMatrixFile.is_open())
		{
			throw std::ios_base::failure("Failed to open file for writing.");
		}
		for (int i = 0; i < 16; i++)
		{
			robotMatrixFile << m_T_patientRFtoImage[i];
			if (i != 15)
			{
				robotMatrixFile << ",";
			}
			else
			{
				robotMatrixFile << ";";
			}
		}
		robotMatrixFile << std::endl;
		robotMatrixFile.close();
		m_Controls.textBrowser->append("Registration result saved.");
	}
	catch (const std::exception & e)
	{
		std::cerr << "Error:" << e.what() << std::endl;
		m_Controls.textBrowser->append("Failed to save registration result.");
	}
}

void DentalRobot::reuseTCPCalibrateMatrix()
{
	try
	{
		std:string desktoppath = std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\DIRobot\\TCPCalibrate.txt";
		std::filesystem::path dir = std::filesystem::path(desktoppath).parent_path();
		if (!std::filesystem::exists(dir))
		{
			throw std::ios_base::failure("Directory does not exist.");
		}
		std::ifstream inputFile(desktoppath);
		if (!inputFile.is_open())
		{
			throw std::ios_base::failure("Failed to open file for reading.");
		}
		std::string line;
		if (std::getline(inputFile, line))
		{
			std::stringstream ss(line);
			std::string token;
			int index = 0;
			while (std::getline(ss, token, ','))
			{
				if (index < 16)
				{
					m_T_FlangeToTCP[index] = std::stod(token);
					index ++;
				}
				else
				{
					break;
				}
			}
		}
		inputFile.close();

		QString output;
		for (int i = 0; i < 16; i++)
		{
			output += "Regiatration Matrix:[" + QString::number(i) + "]" + QString::number(m_T_FlangeToTCP[i]) + " ";
		}
		m_Controls.textBrowser->append(output);
	}
	catch (const std::exception & e)
	{
		std::cerr << "Error:" << e.what() << std::endl;
		m_Controls.textBrowser->append("Failed to load registration result");
	}
	vtkSmartPointer <vtkMatrix4x4> T_FlangetoTCP;
	T_FlangetoTCP->DeepCopy(m_T_FlangeToTCP);

	Eigen::Vector3d RX;
	Eigen::Vector3d RY;
	Eigen::Vector3d RZ;
	Eigen::Vector3d Re;

	RX[0] = T_FlangetoTCP->GetElement(0, 0);
	RX[1] = T_FlangetoTCP->GetElement(1, 0);
	RX[2] = T_FlangetoTCP->GetElement(2, 0);

	RY[0] = T_FlangetoTCP->GetElement(0, 1);
	RY[1] = T_FlangetoTCP->GetElement(1, 1);
	RY[2] = T_FlangetoTCP->GetElement(2, 1);

	RZ[0] = T_FlangetoTCP->GetElement(0, 2);
	RZ[1] = T_FlangetoTCP->GetElement(1, 2);
	RZ[2] = T_FlangetoTCP->GetElement(2, 2);

	Re << RX[0], RY[0], RZ[0],
		RX[1], RY[1], RZ[1],
		RX[2], RY[2], RZ[2];
}

void DentalRobot::collectDitch() 
{
	if (m_probeDitchPset_rf == nullptr)
	{
		m_probeDitchPset_rf = mitk::PointSet::New();
	}
	m_IndexOfLandmark++;
	m_Controls.lineEdit_collectedLandmark->setText(QString::number(m_IndexOfLandmark));
	m_Controls.textBrowser->append(QString(" m_IndexOfLandmark: ") + QString::number(m_IndexOfLandmark));
	//获取T_patientToProbeRF矩阵
	auto vtkT_CameraToProbe = vtkMatrix4x4::New();
	auto vtkT_PatientRFToCamera = vtkMatrix4x4::New();
	vtkT_CameraToProbe->DeepCopy(m_T_CamToProbe);
	vtkT_PatientRFToCamera->DeepCopy(m_T_CamToPatientRF);
	vtkT_PatientRFToCamera->Invert();

	auto transform = vtkTransform::New();
	transform->Identity();
	transform->PostMultiply();
	transform->SetMatrix(vtkT_CameraToProbe);
	transform->Concatenate(vtkT_PatientRFToCamera);
	transform->Update();
	auto T_patientToProbeRF = transform->GetMatrix();

	//保存T_patientrfToProbeRF
	/*memcpy_s(T_PatientRFToProbe, sizeof(double) * 16, T_patientToProbeRF->GetData(), sizeof(double) * 16);*/

	//获取probe尖端的点在patinentRF坐标系下的坐标
	auto tmptrans = vtkTransform::New();
	tmptrans->Identity();
	tmptrans->PostMultiply();
	tmptrans->SetMatrix(vtkT_PatientRFToCamera);
	tmptrans->MultiplyPoint(ProbeTop, nd_tip_FpatientRF);
	vtkProbeTip_Oral->InsertNextPoint(nd_tip_FpatientRF[0], nd_tip_FpatientRF[1], nd_tip_FpatientRF[2]);

	m_Controls.textBrowser->append(QString("Probe Point Landmark: (") + QString::number(nd_tip_FpatientRF[0]) + ", " + QString::number(nd_tip_FpatientRF[1]) + ", "
		+ QString::number(nd_tip_FpatientRF[2]) + ")");
}

void DentalRobot::resetImageRegistration()
{
	m_IndexOfLandmark = 0;
	m_Controls.lineEdit_collectedLandmark->setText(QString::number(0));
	vtkProbeTip_Oral->Reset();
	vtkProbeTip_Oral_icp->Reset();
	m_probeDitchPset_rf = mitk::PointSet::New();
	m_Controls.lineEdit_maxError->setText("NaN");
	m_Controls.lineEdit_avgError->setText("NaN");

	if (vtkProbeTip_Oral->GetNumberOfPoints() == 0 && vtkProbeTip_Oral_icp->GetNumberOfPoints() == 0 && m_probeDitchPset_rf == nullptr)
	{
		m_Controls.textBrowser->append("Replace image configuration");
	}
	else
	{
		m_Controls.textBrowser->append("Replace image configuration failed");
	}
}

void DentalRobot::imageRegistration()
{
	if (m_steelBalls_cmm == nullptr || m_probeDitchPset_cmm == nullptr || GetDataStorage()->GetNamedNode("steelball_image") == nullptr)
	{
		m_Controls.textBrowser->append("Image steelBall extraction should be conducted first!");
		return;
	}

	if (m_steelBalls_cmm->GetSize() != dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("steelball_image")->GetData())->GetSize())
	{
		m_Controls.textBrowser->append("Image steelBall extraction is not complete!");
		return;
	}
	// Step 1: 金属球从硬件工装到提取出来的金属球的转换矩阵
	auto landmarkRegistrator = mitk::SurfaceRegistration::New();
	// m_steelBalls_cmm: splint的金属球在初始位置的点集
	landmarkRegistrator->SetLandmarksSrc(m_steelBalls_cmm);
	// 在CBCT中进行金属球提取后的点集
	landmarkRegistrator->SetLandmarksTarget(dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("steelball_image")->GetData()));

	landmarkRegistrator->ComputeLandMarkResult();

	auto tmpMatrix = landmarkRegistrator->GetResult();
	// Step 2: 凹槽点从硬件工装中拿出来
	auto probePoints_cmm = mitk::PointSet::New();

	auto probePoints_image = mitk::PointSet::New();
	// m_probeDitchPset_cmm：splint的凹槽点在初始位置的点集
	for (int i{ 0 }; i < m_probeDitchPset_cmm->GetSize(); i++)
	{
		probePoints_cmm->InsertPoint(m_probeDitchPset_cmm->GetPoint(i));
	}
	// 将凹槽点移动到与提取出的金属球点集匹配位置，记为probePoints_image
	probePoints_cmm->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(tmpMatrix);
	probePoints_cmm->GetGeometry()->Modified();

	for (int i{ 0 }; i < probePoints_cmm->GetSize(); i++)
	{
		probePoints_image->InsertPoint(probePoints_cmm->GetPoint(i));
	}

	//// Step 3: 把探针采集的点放进去
	//m_probeDitchPset_rf: 在patientRF下的探针采集的凹槽点
	ConvertVTKPointsToMITKPointSet(vtkProbeTip_Oral, m_probeDitchPset_rf);//将点集合的格式转换一下
	if (m_probeDitchPset_rf == nullptr)
	{
		m_Controls.textBrowser->append("No probe ditch point has been captured");
		return;
	}

	if (m_probeDitchPset_rf->GetSize() < 5)
	{
		m_Controls.textBrowser->append("At least 5 probe ditch points should be captured");
		return;
	}
	m_Controls.textBrowser->append("5");
	//// Step 4: 迭代配准
	int collectedPointNum = m_probeDitchPset_rf->GetSize();
	int totalPointNum = probePoints_image->GetSize();

	auto sorted_probeDitchPset_rf = mitk::PointSet::New();
	SortPointSetByDistance(m_probeDitchPset_rf, sorted_probeDitchPset_rf);

	// Generate the tmp landmark_src
	auto tmp_landmark_src = mitk::PointSet::New();
	auto sorted_landmark_src = mitk::PointSet::New();
	double maxError{ 1000 };
	double avgError{ 1000 };

	std::vector<std::vector<int>> combinations = GenerateAllCombinations(totalPointNum - 1, collectedPointNum);

	for (const auto& combination : combinations) {

		tmp_landmark_src->Clear();

		for (int value : combination) {
			tmp_landmark_src->InsertPoint(probePoints_image->GetPoint(value));
		}

		SortPointSetByDistance(tmp_landmark_src, sorted_landmark_src);

		auto tmpLandmarkRegistrator = mitk::SurfaceRegistration::New();
		tmpLandmarkRegistrator->SetLandmarksSrc(sorted_landmark_src);
		tmpLandmarkRegistrator->SetLandmarksTarget(sorted_probeDitchPset_rf);


		m_Controls.textBrowser->append("sorted_landmark_src Pnum: " + QString::number(sorted_landmark_src->GetSize()));

		m_Controls.textBrowser->append("sorted_probeDitchPset_rf Pnum: " + QString::number(sorted_probeDitchPset_rf->GetSize()));

		tmpLandmarkRegistrator->ComputeLandMarkResult();
		double tmpMaxError = tmpLandmarkRegistrator->GetmaxLandmarkError();
		double tmpAvgError = tmpLandmarkRegistrator->GetavgLandmarkError();

		if (tmpMaxError < maxError && tmpAvgError < avgError)
		{
			maxError = tmpMaxError;
			avgError = tmpAvgError;

			m_Controls.lineEdit_maxError->setText(QString::number(tmpMaxError));
			m_Controls.lineEdit_avgError->setText(QString::number(tmpAvgError));

			memcpy_s(m_T_PatientRFtoImage, sizeof(double) * 16, tmpLandmarkRegistrator->GetResult()->GetData(), sizeof(double) * 16);
		}

		if (maxError < 0.5)
		{
			break;
		}

	}

	if (maxError < 1.5 && avgError < 1.5)
	{
		m_Controls.textBrowser->append("Image registration succeeded");
		m_Stat_patientRFtoImage = true;

	}
	else
	{
		m_Controls.textBrowser->append("Image registration failed, please collect more points or reset!");
		m_Stat_patientRFtoImage = false;

		// Clear m_T_patientRFtoImage
		auto identityMatrix = vtkMatrix4x4::New();
		identityMatrix->Identity();
		memcpy_s(m_T_PatientRFtoImage, sizeof(double) * 16, identityMatrix->GetData(), sizeof(double) * 16);

	}
}

void DentalRobot::ConvertVTKPointsToMITKPointSet(vtkSmartPointer<vtkPoints> vtkPoints, mitk::PointSet::Pointer mitkPointSet)
{
	// Iterate through all points in vtkPoints
	for (vtkIdType i = 0; i < vtkPoints->GetNumberOfPoints(); ++i) {
		double p[3];
		vtkPoints->GetPoint(i, p);

		// Create a MITK Point and set its coordinates
		mitk::PointSet::PointType point;
		point[0] = p[0];
		point[1] = p[1];
		point[2] = p[2];

		// Insert the point into the MITK PointSet
		mitkPointSet->InsertPoint(i, point);
	}
}


void DentalRobot::saveImageregisResult()
{
	try
	{
		std::string desktopPath = std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\DIRobot\\ImageRegis.txt";

		std::filesystem::path dir = std::filesystem::path(desktopPath).parent_path();
		if (!std::filesystem::exists(dir))
		{
			std::filesystem::create_directories(dir);
		}

		std::ofstream robotMatrixFile(desktopPath);
		if (!robotMatrixFile.is_open())
		{
			throw std::ios_base::failure("Failed to open file for writing.");
		}

		for (int i = 0; i < 16; i++)
		{
			robotMatrixFile << m_T_PatientRFtoImage[i];
			if (i != 15)
			{
				robotMatrixFile << ",";
			}
			else
			{
				robotMatrixFile << ";";
			}
		}
		robotMatrixFile << std::endl;
		robotMatrixFile.close();

		m_Controls.textBrowser->append("registration result saved");
	}
	catch (const std::exception & e)
	{
		std::cerr << "Error: " << e.what() << std::endl;
		m_Controls.textBrowser->append("Failed to save registration result");
	}
}

void DentalRobot::reuseImageregisResult()
{
	try
	{

		std::string desktopPath = std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\DIRobot\\ImageRegis.txt";

		std::filesystem::path dir = std::filesystem::path(desktopPath).parent_path();
		if (!std::filesystem::exists(dir))
		{
			throw std::ios_base::failure("Directory does not exist.");
		}

		std::ifstream inputFile(desktopPath);
		if (!inputFile.is_open())
		{
			throw std::ios_base::failure("Failed to open file for reading.");
		}

		std::string line;
		if (std::getline(inputFile, line))
		{
			std::stringstream ss(line);
			std::string token;
			int index = 0;
			while (std::getline(ss, token, ','))
			{
				if (index < 16)
				{
					m_T_PatientRFtoImage[index] = std::stod(token);
					index++;
				}
				else
				{
					break;
				}
			}
		}
		inputFile.close();

		// 显示读取的矩阵数据
		QString output;
		for (int i = 0; i < 16; i++)
		{
			output += "Registration Matrix[" + QString::number(i) + "]: " + QString::number(m_T_PatientRFtoImage[i]) + " ";
		}
		m_Controls.textBrowser->append(output);
	}
	catch (const std::exception & e)
	{
		std::cerr << "Error: " << e.what() << std::endl;
		m_Controls.textBrowser->append("Failed to load registration result");
	}
}

void DentalRobot::startNavigation()
{
	qDebug() << "---- - navigation start------";
	m_Controls.textBrowser->append("-----navigation start------");

	disconnect(m_AimoeVisualizeTimer, &QTimer::timeout, this, &DentalRobot::UpdateDrillVisual);

	// m_NaviMode = 0;
	//根据输入长度和 m_T_handpieceRFtoInputDrill 修改钻头/探针表面的长度和 m_T_handpieceRFtoInputDrill
	double inputDrillLength = m_Controls.lineEdit_drillLength->text().toDouble();//钻头长度（实际）
	auto probe_head_tail_mandible = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("probe_head_tail_mandible")->GetData());
	double probeDrillLength = GetPointDistance(probe_head_tail_mandible->GetPoint(0), probe_head_tail_mandible->GetPoint(1));//探针长度（设定）
	double z_scale = inputDrillLength / probeDrillLength;//缩放系数

	auto T_probeDrilltoInputDrill = vtkMatrix4x4::New();
	T_probeDrilltoInputDrill->Identity();
	T_probeDrilltoInputDrill->SetElement(2, 2, z_scale);//设置第三行第三列z轴的值为缩放系数
	T_probeDrilltoInputDrill->SetElement(2, 3, probeDrillLength - inputDrillLength);//设置第三行第四列z轴的平移

	//QString matrixString = MatrixToString(T_probeDrilltoInputDrill);
	//m_Controls.textBrowser->append("T_probeDrilltoInputDrill:\n"+matrixString);
	//auto T_handpieceRFtoDrill_init = vtkMatrix4x4::New();

	auto T_calibratorRFtoTCP = vtkMatrix4x4::New();
	T_calibratorRFtoTCP->DeepCopy(m_T_calibratorRFToTCP);

	auto T_EndRFtoCalibratorRF = vtkMatrix4x4::New();
	T_EndRFtoCalibratorRF->DeepCopy(m_T_EndRFToCalibratorRF);

	//计算到实际drill的转化矩阵
	auto tmpTrans = vtkTransform::New();
	tmpTrans->PostMultiply();
	tmpTrans->SetMatrix(T_probeDrilltoInputDrill);//maybe problem
	tmpTrans->Concatenate(T_calibratorRFtoTCP);
	tmpTrans->Concatenate(T_EndRFtoCalibratorRF);
	tmpTrans->Update();

	//auto tmpTrans = vtkTransform::New();
	//tmpTrans->PostMultiply();
	//tmpTrans->SetMatrix(T_probeDrilltoInputDrill);
	//tmpTrans->Concatenate(T_handpieceRFtoDrill_init);
	//tmpTrans->Update();

	auto T_EndRFtoDrill_new = tmpTrans->GetMatrix();

	//memcpy_s(m_T_handpieceRFtoInputDrill, sizeof(double) * 16, T_EndRFtoDrill_new->GetData(), sizeof(double) * 16);
	memcpy_s(m_T_EndRFToInputTCP, sizeof(double) * 16, T_EndRFtoDrill_new->GetData(), sizeof(double) * 16);

	TurnOffAllNodesVisibility();
	GetDataStorage()->GetNamedNode("CBCT Bounding Shape_cropped")->SetVisibility(true);
	GetDataStorage()->GetNamedNode("drillSurface")->SetVisibility(true);

	GetDataStorage()->GetNamedNode("stdmulti.widget0.plane")->SetVisibility(false);
	GetDataStorage()->GetNamedNode("stdmulti.widget1.plane")->SetVisibility(false);
	GetDataStorage()->GetNamedNode("stdmulti.widget2.plane")->SetVisibility(false);

	connect(m_AimoeVisualizeTimer, &QTimer::timeout, this, &DentalRobot::UpdateDrillVisual);

}

void DentalRobot::UpdateDrillVisual()
{
	qDebug() << "---- - Get TCP from Camera------";
	m_Controls.textBrowser->append("----Get TCP from Camera------");

	if (GetDataStorage()->GetNamedNode("drillSurface") == nullptr)
	{
		m_Controls.textBrowser->append("drillSurface is missing");
		return;
	}

	//------TCP from Camera Begin-------

/*	auto T_calibratorRFtoTCP = vtkMatrix4x4::New();
	T_calibratorRFtoTCP->DeepCopy(T_calibratorRFToInputTCP);*///源头2

	auto T_EndRFtoInputTCP = vtkMatrix4x4::New();
	T_EndRFtoInputTCP->DeepCopy(m_T_EndRFToInputTCP);

	auto T_CamtoEndRF = vtkMatrix4x4::New();
	T_CamtoEndRF->DeepCopy(m_T_CamToEndRF);

	auto T_patientRFtoCamera = vtkMatrix4x4::New();
	T_patientRFtoCamera->DeepCopy(m_T_CamToPatientRF);
	T_patientRFtoCamera->Invert();

	auto T_imageToPatientRF = vtkMatrix4x4::New();
	T_imageToPatientRF->DeepCopy(m_T_PatientRFtoImage);
	T_imageToPatientRF->Invert();

	auto trans_imagetoTCP = vtkTransform::New();
	trans_imagetoTCP->Identity();
	trans_imagetoTCP->PostMultiply();
	trans_imagetoTCP->SetMatrix(m_T_EndRFToInputTCP);
	trans_imagetoTCP->Concatenate(T_CamtoEndRF);
	trans_imagetoTCP->Concatenate(T_patientRFtoCamera);
	trans_imagetoTCP->Concatenate(T_imageToPatientRF);
	trans_imagetoTCP->Update();
	auto T_imagetoTCP = trans_imagetoTCP->GetMatrix();

	memcpy_s(m_T_ImageToInputTCP, sizeof(double) * 16, T_imagetoTCP->GetData(), sizeof(double) * 16);


	//------TCP from Camera End-------



	//------TCP from Robotic arm Begin-------
	//auto T_FlangetoTCP = vtkMatrix4x4::New();
	//T_FlangetoTCP->DeepCopy(T_FlangeToTCP);

	//auto T_CamtoEndRF = vtkMatrix4x4::New();
	//T_CamtoEndRF->DeepCopy(T_CamToEndRF);

	//auto T_EndRFtoFlange = vtkMatrix4x4::New();
	//T_EndRFtoFlange->DeepCopy(T_FlangeToEdnRF);
	//T_EndRFtoFlange->Invert();


	//auto T_patientRFtoCamera = vtkMatrix4x4::New();
	//T_patientRFtoCamera->DeepCopy(T_CamToPatientRF);
	//T_patientRFtoCamera->Invert();

	//auto T_imageToPatientRF = vtkMatrix4x4::New();
	//T_imageToPatientRF->DeepCopy(T_PatientRFtoImage);
	//T_imageToPatientRF->Invert();

	//auto tmpTrans = vtkTransform::New();
	//tmpTrans->Identity();
	//tmpTrans->PostMultiply();
	//tmpTrans->SetMatrix(T_FlangetoTCP);
	//tmpTrans->Concatenate(T_EndRFtoFlange);
	//tmpTrans->Concatenate(T_CamtoEndRF);
	//tmpTrans->Concatenate(T_patientRFtoCamera);
	//tmpTrans->Concatenate(T_imageToPatientRF);
	//tmpTrans->Update();
	//auto T_imagetoTCP = tmpTrans->GetMatrix();

	//memcpy_s(T_ImageToTCP, sizeof(double) * 16, T_imagetoTCP->GetData(), sizeof(double) * 16);

	//------TCP from Robotic arm End-------

	GetDataStorage()->GetNamedNode("drillSurface")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(T_imagetoTCP);
	GetDataStorage()->GetNamedNode("drillSurface")->GetData()->GetGeometry()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();

	UpdateDeviation();
}

void DentalRobot::UpdateDeviation() //修改
{
	/*GetDataStorage()->GetNamedNode("probe")->SetColor(1, 0, 0);
	connect(m_AimoeVisualizeTimer, &QTimer::timeout, this, &ZYXtest::UpdateTCP);
	connect(m_AimoeVisualizeTimer, &QTimer::timeout, this, &ZYXtest::UpdateProbe);*/

	// planned entry point and apex point in image frame
	auto plan_tip_pts = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("plan_tip_pts")->GetData());
	//mitk::Point3D apex_plan = plan_tip_pts->GetPoint(0);//顶点
	//mitk::Point3D entry_plan = plan_tip_pts->GetPoint(1);//切入点
	mitk::Point3D apex_plan = plan_tip_pts->GetPoint(1);//顶点
	mitk::Point3D entry_plan = plan_tip_pts->GetPoint(0);//切入点

	Eigen::Vector3d axis_plan;
	axis_plan[0] = entry_plan[0] - apex_plan[0];
	axis_plan[1] = entry_plan[1] - apex_plan[1];
	axis_plan[2] = entry_plan[2] - apex_plan[2];
	axis_plan.normalize();//表示方向，不表示距离

	// T_imageToProbe->T_imagetoTCP
	auto T_calibratorRFtoTCP = vtkMatrix4x4::New();
	T_calibratorRFtoTCP->DeepCopy(m_T_calibratorRFToTCP);

	auto T_EndRFtoCalibratorRF = vtkMatrix4x4::New();
	T_EndRFtoCalibratorRF->DeepCopy(m_T_EndRFToCalibratorRF);

	auto T_CamtoEndRF = vtkMatrix4x4::New();
	T_CamtoEndRF->DeepCopy(m_T_CamToEndRF);

	auto T_patientRFtoCamera = vtkMatrix4x4::New();
	T_patientRFtoCamera->DeepCopy(m_T_CamToPatientRF);
	T_patientRFtoCamera->Invert();

	auto T_imageToPatientRF = vtkMatrix4x4::New();
	T_imageToPatientRF->DeepCopy(m_T_PatientRFtoImage);
	T_imageToPatientRF->Invert();

	auto trans_imagetoTCP = vtkTransform::New();
	trans_imagetoTCP->Identity();
	trans_imagetoTCP->PostMultiply();
	trans_imagetoTCP->SetMatrix(T_calibratorRFtoTCP);
	trans_imagetoTCP->Concatenate(T_EndRFtoCalibratorRF);
	trans_imagetoTCP->Concatenate(T_CamtoEndRF);
	trans_imagetoTCP->Concatenate(T_patientRFtoCamera);
	trans_imagetoTCP->Concatenate(T_imageToPatientRF);
	trans_imagetoTCP->Update();
	auto T_imagetoTCP = trans_imagetoTCP->GetMatrix();

	Eigen::Vector3d axis_probe;//钻头在z轴上的位姿
	axis_probe[0] = T_imagetoTCP->GetElement(0, 2);
	axis_probe[1] = T_imagetoTCP->GetElement(1, 2);
	axis_probe[2] = T_imagetoTCP->GetElement(2, 2);

	double angleDevi = 180 * acos(axis_plan.dot(axis_probe)) / 3.1415926; //计算角度偏差

	m_Controls.lineEdit_angleError->setText(QString::number(angleDevi));

	//******** m_NaviMode = 0 , i.e, Drilling mode, only update the Drill deviation *************
	if (m_NaviMode == 0)
	{
		m_Controls.lineEdit_entryTotalError->setText("NaN");
		m_Controls.lineEdit_entryVertError->setText("NaN");
		m_Controls.lineEdit_entryHoriError->setText("NaN");
		m_Controls.lineEdit_apexTotalError->setText("NaN");
		m_Controls.lineEdit_apexVertError->setText("NaN");
		m_Controls.lineEdit_apexHoriError->setText("NaN");

		// Tip to planned apex
		mitk::Point3D drillTip;
		//drillTip[0] = m_T_imageToInputDrill[3];//源头1
		//drillTip[1] = m_T_imageToInputDrill[7];
		//drillTip[2] = m_T_imageToInputDrill[11];
		drillTip[0] = m_T_ImageToInputTCP[3];//源头1
		drillTip[1] = m_T_ImageToInputTCP[7];
		drillTip[2] = m_T_ImageToInputTCP[11];
		double tipToPlannedApex = GetPointDistance(drillTip, apex_plan);
		m_Controls.lineEdit_drillTipTotalError->setText(QString::number(tipToPlannedApex));

		// Vertical deviation
		Eigen::Vector3d plannedApexToDrillTip;
		plannedApexToDrillTip[0] = drillTip[0] - apex_plan[0];
		plannedApexToDrillTip[1] = drillTip[1] - apex_plan[1];
		plannedApexToDrillTip[2] = drillTip[2] - apex_plan[2];

		double verticalDevi = plannedApexToDrillTip.dot(axis_plan);
		m_Controls.lineEdit_drillTipVertError->setText(QString::number(verticalDevi));

		// Horizontal deviation
		double horizontalDevi = plannedApexToDrillTip.cross(axis_plan).norm();
		m_Controls.lineEdit_drillTipHoriError->setText(QString::number(horizontalDevi));
	}

	//******** m_NaviMode = 1 , i.e, Implantation mode, only update the implant deviation and angle deviation *************
	if (m_NaviMode == 1)
	{
		m_Controls.lineEdit_drillTipTotalError->setText("NaN");
		m_Controls.lineEdit_drillTipVertError->setText("NaN");
		m_Controls.lineEdit_drillTipHoriError->setText("NaN");

		//******* real implant entry ************
		// Total distance to  planned entry
		mitk::Point3D entry_real = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("implant_tip_pts")->GetData())->GetPoint(1);
		double totalEntryDis = GetPointDistance(entry_real, entry_plan);
		m_Controls.lineEdit_entryTotalError->setText(QString::number(totalEntryDis));

		// Vertical deviation
		Eigen::Vector3d entry_planToReal;
		entry_planToReal[0] = entry_real[0] - entry_plan[0];
		entry_planToReal[1] = entry_real[1] - entry_plan[1];
		entry_planToReal[2] = entry_real[2] - entry_plan[2];

		double verticalDevi_entry = entry_planToReal.dot(axis_plan);
		m_Controls.lineEdit_entryVertError->setText(QString::number(verticalDevi_entry));

		// Horizontal deviation
		double horizontalDevi_entry = entry_planToReal.cross(axis_plan).norm();
		m_Controls.lineEdit_entryHoriError->setText(QString::number(horizontalDevi_entry));


		//******* real implant apex ************
		// Total distance to  planned apex
		mitk::Point3D apex_real = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("implant_tip_pts")->GetData())->GetPoint(0);
		double totalApexDis = GetPointDistance(apex_real, apex_plan);
		m_Controls.lineEdit_apexTotalError->setText(QString::number(totalApexDis));

		// Vertical deviation
		Eigen::Vector3d apex_planToReal;
		apex_planToReal[0] = apex_real[0] - apex_plan[0];
		apex_planToReal[1] = apex_real[1] - apex_plan[1];
		apex_planToReal[2] = apex_real[2] - apex_plan[2];

		double verticalDevi_apex = apex_planToReal.dot(axis_plan);
		m_Controls.lineEdit_apexVertError->setText(QString::number(verticalDevi_apex));

		// Horizontal deviation
		double horizontalDevi_apex = apex_planToReal.cross(axis_plan).norm();
		m_Controls.lineEdit_apexHoriError->setText(QString::number(horizontalDevi_apex));

	}
}
	