// Qt
// Qt
#include <Qtimer>
#include <QTextBrowser>
#include "ToolFunction.h"
#include "vtkMath.h"

ToolFunction::ToolFunction() {

}

ToolFunction::~ToolFunction() {

}

void ToolFunction::PrintMatrix(QTextBrowser* browser, std::string matrix_name, double matrix[16]) {
	browser->append("---------------------------------------------------");
	std::cout << "---------------------------------------------------" << std::endl;
	std::cout << matrix_name << ":" << std::endl;
	QString Matrix = QString::fromStdString(matrix_name);
	Matrix += "\n";

	for (int i = 0; i < 16; i++) {
		std::cout << matrix[i] << "	";
		Matrix += QString::number(matrix[i]);
		Matrix += QString::fromStdString("	");
		if ((i + 1) % 4 == 0) {
			std::cout << std::endl;
			Matrix += "\n";
		}
	}
	std::cout << "---------------------------------------------------" << std::endl;
	browser->append(Matrix);
	browser->append("---------------------------------------------------");
}

void ToolFunction::PrintMatrix(QTextBrowser* browser, std::string matrix_name, vtkMatrix4x4* matrix) {
	browser->append("---------------------------------------------------");
	std::cout << "---------------------------------------------------" << std::endl;
	browser->append(QString::fromStdString(matrix_name + ":" + "\n"));
	std::cout << matrix_name << ":" << std::endl;
	/*std::cout << matrixName + ": " << std::endl;*/
	for (int i = 0; i < 4; ++i)
	{

		std::string row;
		for (int j = 0; j < 4; ++j)
		{
			std::cout << matrix->GetElement(i, j) << "		";
			row += std::to_string(matrix->GetElement(i, j)) + "	";
		}
		browser->append(QString::fromStdString(row) + "\n");
	}
	browser->append("---------------------------------------------------");
	std::cout << "---------------------------------------------------" << std::endl;

}

void ToolFunction::PrintMatrix(QTextBrowser* browser, std::string matrix_name, vtkNew<vtkMatrix4x4> matrix) {
	browser->append("---------------------------------------------------");
	std::cout << "---------------------------------------------------" << std::endl;
	browser->append(QString::fromStdString(matrix_name + ":" + "\n"));
	std::cout << matrix_name << ":" << std::endl;
	/*std::cout << matrixName + ": " << std::endl;*/
	for (int i = 0; i < 4; ++i)
	{

		std::string row;
		for (int j = 0; j < 4; ++j)
		{
			std::cout << matrix->GetElement(i, j) << "		";
			row += std::to_string(matrix->GetElement(i, j)) + "	";
		}
		browser->append(QString::fromStdString(row) + "\n");
	}
	std::cout << "---------------------------------------------------" << std::endl;
	browser->append("---------------------------------------------------");
}

void ToolFunction::PrintMatrix(QTextBrowser* browser, std::string matrix_name, vtkSmartPointer<vtkMatrix4x4> matrix) {
	browser->append("---------------------------------------------------");
	std::cout << "---------------------------------------------------" << std::endl;
	browser->append(QString::fromStdString(matrix_name + ":" + "\n"));
	std::cout << matrix_name << ":" << std::endl;

	/*std::cout << matrixName + ": " << std::endl;*/
	for (int i = 0; i < 4; ++i)
	{

		std::string row;
		for (int j = 0; j < 4; ++j)
		{
			std::cout << matrix->GetElement(i, j) << "		";
			row += std::to_string(matrix->GetElement(i, j)) + "	";
		}
		browser->append(QString::fromStdString(row) + "\n");
	}
	std::cout << "---------------------------------------------------" << std::endl;
	browser->append("---------------------------------------------------");
}

void ToolFunction::PrintXYZRXRYRZ(QTextBrowser* browser, int result, std::array<double, 6>& TCP, std::string dataname) {
	if (result) std::cout << result << std::endl;
	if (!result) {
		std::cout << "---------------------------------------------------" << std::endl;
		browser->append("---------------------------------------------------");
		browser->append(QString::fromStdString(dataname) + ":  \nX:" + QString::number(TCP[0]) + "   Y:" + QString::number(TCP[1]) + "   Z:" + QString::number(TCP[2]) + "\nRx" +
			QString::number(TCP[3]) + "   Ry:" + QString::number(TCP[4]) + "   Rz:" + QString::number(TCP[5]) + "   \n");

		std::cout << dataname << ":" << std::endl;
		std::cout << "Position - X: " << TCP[0] << ", Y: " << TCP[1] << ", Z: " << TCP[2] << std::endl;
		std::cout << "Orientation - Rx: " << TCP[3] << ", Ry: " << TCP[4] << ", Rz: " << TCP[5] << std::endl;
		std::cout << "---------------------------------------------------" << std::endl;
		browser->append("---------------------------------------------------");
	}
}

void ToolFunction::PrintXYZRXRYRZ(QTextBrowser* browser, int result, double dX, double dY, double dZ, double dRx, double dRy, double dRz, std::string dataname) {
	if (result) std::cout << result << std::endl;
	if (!result) {
		std::cout << "---------------------------------------------------" << std::endl;
		browser->append("---------------------------------------------------");
		browser->append(QString::fromStdString(dataname)+":  \nX:" + QString::number(dX) + "   Y:" + QString::number(dY) + "   Z:" + QString::number(dZ) + "\nRx" +
			QString::number(dRx) + "   Ry:" + QString::number(dRy) + "   Rz:" + QString::number(dRz) + "   \n");

		std::cout << dataname <<":" << std::endl;
		std::cout << "Position - X: " << dX << ", Y: " << dY << ", Z: " << dZ << std::endl;
		std::cout << "Orientation - Rx: " << dRx << ", Ry: " << dRy << ", Rz: " << dRz << std::endl;
		std::cout << "---------------------------------------------------" << std::endl;
		browser->append("---------------------------------------------------");
	}
}

void ToolFunction::PrintError(QTextBrowser* browser, int nRet, std::string behavior) {
	if (nRet == 0)
	{
		browser->append("---------------------------------------------------");
		browser->append(QString::fromStdString(behavior) + " Succeed" +  "\n");
		std::cout << behavior << " Succeed" << std::endl;
		std::cout << "---------------------------------------------------" << std::endl;
	}
	else
	{
		browser->append(QString::fromStdString(behavior) + " Failed" + "\n");
		browser->append(QString::fromStdString("ErrorCode: ") + QString::number(nRet) + "\n");
		std::cout << behavior << " Failed" << std::endl;
		std::cout << "ErrorCode: " << nRet << std::endl;

		//针对一些常见错误做的一些解释
		if (nRet == 39500)
		{
			std::cout << "Please check the connection of the robot arm" << std::endl;
			browser->append(QString::fromStdString("Please check the connection of the robot arm") + "\n");

		}
		if (nRet == 40025)
		{
			std::cout << "The robot is not in a state of readiness" << std::endl;
			browser->append(QString::fromStdString("The robot is not in a state of readiness")  + "\n");
		}

		if (nRet >= 40000 && nRet <= 40500)
		{
			std::cout << "CDS executed command with error: " << nRet << std::endl;
			browser->append(QString::fromStdString("CDS executed command with error: ")  + "\n");
		}
		else if (nRet >= 10000 && nRet <= 10015)
		{
			std::cout << "Robot servo drive reported fault code: " << nRet << std::endl;
			browser->append(QString::fromStdString("Robot servo drive reported fault code: ") + "\n");
		}
		else if (nRet >= 10016 && nRet <= 11000)
		{
			std::cout << "Robot collaboration algorithm detected fault: " << nRet << std::endl;
			browser->append(QString::fromStdString("Robot collaboration algorithm detected fault: ")  + "\n");
		}
		else if (nRet >= 15000 && nRet <= 16000)
		{
			std::cout << "Robot control module detected fault: " << nRet << std::endl;
			browser->append(QString::fromStdString("Robot control module detected fault: ")  + "\n");
		}
		else if (nRet >= 30001 && nRet <= 30016)
		{
			std::cout << "Modbus module error during command execution: " << nRet << std::endl;
			browser->append(QString::fromStdString("Modbus module error during command execution: ")  + "\n");
		}
		else if (nRet >= 20000 && nRet <= 39999)
		{
			if (nRet == 20031)
				std::cout << "the target cant arrived" << std::endl;
			std::cout << "CPS executed command with error: " << nRet << std::endl;
			browser->append(QString::fromStdString("the target cant arrived")  + "\n");
		}

		std::cout << "---------------------------------------------------" << std::endl;
		browser->append(QString::fromStdString("---------------------------------------------------")  + "\n");
	}
}

void ToolFunction::SaveFiles(QTextBrowser* browser, std::string filecatalog, double matrix[16]) {
	std::filesystem::path dir = std::filesystem::path(std::string(getenv("USERPROFILE")) + filecatalog).parent_path();

	if (!std::filesystem::exists(dir)) {
		if (!std::filesystem::create_directories(dir)) {
			std::cerr << "Failed to create directory: " << dir << std::endl;
		}
	}

	std::ofstream robotMatrixFile(std::string(getenv("USERPROFILE")) + filecatalog);
	for (int i = 0; i < 16; i++) {
		robotMatrixFile << matrix[i];
		if (i != 15) {
			robotMatrixFile << ",";
		}
		else {
			robotMatrixFile << ";";
		}
	}
	robotMatrixFile << std::endl;
	robotMatrixFile.close();
	std::cout << "------------------------SaveMatrix Succeed---------------------------" << std::endl;
	browser->append("--------------------------SaveMatrix Succeed-------------------------");
}

void ToolFunction::SaveFiles(QTextBrowser* browser, std::string filecatalog, vtkMatrix4x4* matrix) {

	std::filesystem::path dir = std::filesystem::path(std::string(getenv("USERPROFILE")) + filecatalog).parent_path();

	if (!std::filesystem::exists(dir)) {
		if (!std::filesystem::create_directories(dir)) {
			std::cerr << "Failed to create directory: " << dir << std::endl;
		}
	}

	std::ofstream robotMatrixFile(std::string(getenv("USERPROFILE")) + filecatalog);

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			robotMatrixFile << matrix->GetElement(i, j);
			if (i == 3 && j == 3) {
				robotMatrixFile << ";";
			}
			else {
				robotMatrixFile << ",";
			}
		}

	}
	robotMatrixFile << std::endl;
	robotMatrixFile.close();
	std::cout << "------------------------SaveMatrix Succeed---------------------------" << std::endl;
	browser->append("--------------------------SaveMatrix Succeed-------------------------");

}

void ToolFunction::SaveFiles(QTextBrowser* browser, std::string filecatalog, vtkNew<vtkMatrix4x4> matrix) {
	std::filesystem::path dir = std::filesystem::path(std::string(getenv("USERPROFILE")) + filecatalog).parent_path();

	if (!std::filesystem::exists(dir)) {
		if (!std::filesystem::create_directories(dir)) {
			std::cerr << "Failed to create directory: " << dir << std::endl;
		}
	}

	std::ofstream robotMatrixFile(std::string(getenv("USERPROFILE")) + filecatalog);


	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			robotMatrixFile << matrix->GetElement(i, j);
			if (i == 3 && j == 3) {
				robotMatrixFile << ";";
			}
			else {
				robotMatrixFile << ",";
			}
		}

	}
	robotMatrixFile << std::endl;
	robotMatrixFile.close();
	std::cout << "------------------------SaveMatrix Succeed---------------------------" << std::endl;
	browser->append("--------------------------SaveMatrix Succeed-------------------------");

}

void ToolFunction::SaveFiles(QTextBrowser* browser, std::string filecatalog, vtkSmartPointer<vtkMatrix4x4> matrix) {
	std::filesystem::path dir = std::filesystem::path(std::string(getenv("USERPROFILE")) + filecatalog).parent_path();

	if (!std::filesystem::exists(dir)) {
		if (!std::filesystem::create_directories(dir)) {
			std::cerr << "Failed to create directory: " << dir << std::endl;
		}
	}

	std::ofstream robotMatrixFile(std::string(getenv("USERPROFILE")) + filecatalog);


	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			robotMatrixFile << matrix->GetElement(i, j);
			if (i == 3 && j == 3) {
				robotMatrixFile << ";";
			}
			else {
				robotMatrixFile << ",";
			}
		}

	}
	robotMatrixFile << std::endl;
	robotMatrixFile.close();
	std::cout << "------------------------SaveMatrix Succeed---------------------------" << std::endl;
	browser->append("--------------------------SaveMatrix Succeed-------------------------");

}

void ToolFunction::ReuseFiles(QTextBrowser* browser, std::string filecatalog, double matrix[16]) {
	std::ifstream inputFile2(std::string(getenv("USERPROFILE")) + filecatalog);
	if (inputFile2.is_open()) {
		std::string line2;
		if (std::getline(inputFile2, line2)) {
			std::stringstream ss2(line2);
			std::string token2;
			int index2 = 0;
			while (std::getline(ss2, token2, ',')) {
				matrix[index2] = std::stod(token2);
				index2++;
			}
		}
		inputFile2.close();
		std::cout << "------------------------ReuseMatrix Succeed---------------------------" << std::endl;
		browser->append("--------------------------ReuseMatrix Succeed-------------------------");
		PrintMatrix(browser, filecatalog, matrix);
	}
	else {
		browser->append("--------------------------ReuseMatrix Failed-------------------------");
		PrintMatrix(browser, filecatalog, matrix);
	}
}

void ToolFunction::ReuseFiles(QTextBrowser* browser, std::string filecatalog, vtkMatrix4x4* matrix) {
	std::ifstream inputFile2(std::string(getenv("USERPROFILE")) + filecatalog);
	if (inputFile2.is_open()) {
		std::string line2;
		if (std::getline(inputFile2, line2)) {
			std::stringstream ss2(line2);
			std::string token2;
			int i = 0;
			int j = 0;
			while (std::getline(ss2, token2, ',')) {
				if (j == 4) {
					i++;
					j = 0;
				}
				matrix->SetElement(i, j, std::stod(token2));
				j++;
			}
		}
		inputFile2.close();
		std::cout << "------------------------ReuseMatrix Succeed---------------------------" << std::endl;
		browser->append("--------------------------ReuseMatrix Succeed-------------------------");
		PrintMatrix(browser, filecatalog, matrix);
	}
	else {
		browser->append("--------------------------ReuseMatrix Failed-------------------------");
	}

}

void ToolFunction::ReuseFiles(QTextBrowser* browser, std::string filecatalog, vtkNew<vtkMatrix4x4> matrix) {
	std::ifstream inputFile2(std::string(getenv("USERPROFILE")) + filecatalog);
	if (inputFile2.is_open()) {
		std::string line2;
		if (std::getline(inputFile2, line2)) {
			std::stringstream ss2(line2);
			std::string token2;
			int i = 0;
			int j = 0;
			while (std::getline(ss2, token2, ',')) {
				if (j == 4) {
					i++;
					j = 0;
				}
				matrix->SetElement(i, j, std::stod(token2));
				j++;
			}
		}
		inputFile2.close();
		std::cout << "------------------------ReuseMatrix Succeed---------------------------" << std::endl;
		browser->append("--------------------------ReuseMatrix Succeed-------------------------");
		PrintMatrix(browser, filecatalog, matrix->GetData());
	}
	else {
		browser->append("--------------------------ReuseMatrix Failed-------------------------");
	}
}

void ToolFunction::ReuseFiles(QTextBrowser* browser, std::string filecatalog, vtkSmartPointer<vtkMatrix4x4> matrix) {
	std::ifstream inputFile2(std::string(getenv("USERPROFILE")) + filecatalog);
	if (inputFile2.is_open()) {
		std::string line2;
		if (std::getline(inputFile2, line2)) {
			std::stringstream ss2(line2);
			std::string token2;
			int i = 0;
			int j = 0;
			while (std::getline(ss2, token2, ',')) {
				if (j == 4) {
					i++;
					j = 0;
				}
				matrix->SetElement(i, j, std::stod(token2));
				j++;
			}
		}
		inputFile2.close();
		std::cout << "------------------------ReuseMatrix Failed---------------------------" << std::endl;
		browser->append("--------------------------ReuseMatrix Failed-------------------------");
		PrintMatrix(browser, filecatalog, matrix);
	}
	else {
		browser->append("--------------------------ReuseMatrix Failed-------------------------");
	}
}

void ToolFunction::CombineRotationTranslation(float Rto[3][3], float Tto[3], vtkMatrix4x4* resultMatrix) {
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
	for (int j = 0; j < 3; ++j) {
		resultMatrix->SetElement(3, j, 0.0);
	}
	resultMatrix->SetElement(3, 3, 1);
}


void ToolFunction::MultiplyMatrices(vtkTransform* matrix, vtkTransform* matrix_1to2,
	vtkTransform* matrix_2to3, vtkTransform* matrix_3to4, vtkTransform* matrix_4to5,
	vtkTransform* matrix_5to6, vtkTransform* matrix_6to7, vtkTransform* matrix_7to8) {

	vtkNew<vtkMatrix4x4> I;
	I->Identity();

	matrix->PreMultiply();
	matrix->SetMatrix(I);
	matrix->Concatenate(matrix_1to2);
	matrix->Concatenate(matrix_2to3);
	matrix->Concatenate(matrix_3to4);
	matrix->Concatenate(matrix_4to5);
	matrix->Concatenate(matrix_5to6);
	matrix->Concatenate(matrix_6to7);
	matrix->Concatenate(matrix_7to8);
	matrix->Update();
}

void ToolFunction::MultiplyMatrices(vtkTransform* matrix, vtkTransform* matrix_1to2,
	vtkTransform* matrix_2to3, vtkTransform* matrix_3to4, vtkTransform* matrix_4to5,
	vtkTransform* matrix_5to6, vtkTransform* matrix_6to7) {

	vtkNew<vtkMatrix4x4> I;
	I->Identity();

	matrix->PreMultiply();
	matrix->SetMatrix(I);
	matrix->Concatenate(matrix_1to2);
	matrix->Concatenate(matrix_2to3);
	matrix->Concatenate(matrix_3to4);
	matrix->Concatenate(matrix_4to5);
	matrix->Concatenate(matrix_5to6);
	matrix->Concatenate(matrix_6to7);
	matrix->Update();
}

void ToolFunction::MultiplyMatrices(vtkTransform* matrix, vtkTransform* matrix_1to2,
	vtkTransform* matrix_2to3, vtkTransform* matrix_3to4, vtkTransform* matrix_4to5,
	vtkTransform* matrix_5to6) {

	vtkNew<vtkMatrix4x4> I;
	I->Identity();

	matrix->PreMultiply();
	matrix->SetMatrix(I);
	matrix->Concatenate(matrix_1to2);
	matrix->Concatenate(matrix_2to3);
	matrix->Concatenate(matrix_3to4);
	matrix->Concatenate(matrix_4to5);
	matrix->Concatenate(matrix_5to6);
	matrix->Update();
}

void ToolFunction::MultiplyMatrices(vtkTransform* matrix, vtkTransform* matrix_1to2,
	vtkTransform* matrix_2to3, vtkTransform* matrix_3to4, vtkTransform* matrix_4to5) {

	vtkNew<vtkMatrix4x4> I;
	I->Identity();

	matrix->PreMultiply();
	matrix->SetMatrix(I);
	matrix->Concatenate(matrix_1to2);
	matrix->Concatenate(matrix_2to3);
	matrix->Concatenate(matrix_3to4);
	matrix->Concatenate(matrix_4to5);
	matrix->Update();
}

void ToolFunction::MultiplyMatrices(vtkTransform* matrix, vtkTransform* matrix_1to2,
	vtkTransform* matrix_2to3, vtkTransform* matrix_3to4) {

	vtkNew<vtkMatrix4x4> I;
	I->Identity();

	matrix->PreMultiply();
	matrix->SetMatrix(I);
	matrix->Concatenate(matrix_1to2);
	matrix->Concatenate(matrix_2to3);
	matrix->Concatenate(matrix_3to4);
	matrix->Update();
}

void ToolFunction::MultiplyMatrices(vtkTransform* matrix, vtkTransform* matrix_1to2,
	vtkTransform* matrix_2to3) {

	vtkNew<vtkMatrix4x4> I;
	I->Identity();

	matrix->PreMultiply();
	matrix->SetMatrix(I);
	matrix->Concatenate(matrix_1to2);
	matrix->Concatenate(matrix_2to3);
	matrix->Update();
}

void ToolFunction::MultiplyMatrices(vtkTransform* matrix, vtkMatrix4x4* matrix_1to2,
	vtkMatrix4x4* matrix_2to3, vtkMatrix4x4* matrix_3to4, vtkMatrix4x4* matrix_4to5,
	vtkMatrix4x4* matrix_5to6, vtkMatrix4x4* matrix_6to7, vtkMatrix4x4* matrix_7to8) {

	vtkNew<vtkMatrix4x4> I;
	I->Identity();

	matrix->PreMultiply();
	matrix->SetMatrix(I);
	matrix->Concatenate(matrix_1to2);
	matrix->Concatenate(matrix_2to3);
	matrix->Concatenate(matrix_3to4);
	matrix->Concatenate(matrix_4to5);
	matrix->Concatenate(matrix_5to6);
	matrix->Concatenate(matrix_6to7);
	matrix->Concatenate(matrix_7to8);
	matrix->Update();
}

void ToolFunction::MultiplyMatrices(vtkTransform* matrix, vtkMatrix4x4* matrix_1to2,
	vtkMatrix4x4* matrix_2to3, vtkMatrix4x4* matrix_3to4, vtkMatrix4x4* matrix_4to5,
	vtkMatrix4x4* matrix_5to6, vtkMatrix4x4* matrix_6to7) {

	vtkNew<vtkMatrix4x4> I;
	I->Identity();

	matrix->PreMultiply();
	matrix->SetMatrix(I);
	matrix->Concatenate(matrix_1to2);
	matrix->Concatenate(matrix_2to3);
	matrix->Concatenate(matrix_3to4);
	matrix->Concatenate(matrix_4to5);
	matrix->Concatenate(matrix_5to6);
	matrix->Concatenate(matrix_6to7);
	matrix->Update();
}

void ToolFunction::MultiplyMatrices(vtkTransform* matrix, vtkMatrix4x4* matrix_1to2,
	vtkMatrix4x4* matrix_2to3, vtkMatrix4x4* matrix_3to4, vtkMatrix4x4* matrix_4to5,
	vtkMatrix4x4* matrix_5to6) {

	vtkNew<vtkMatrix4x4> I;
	I->Identity();

	matrix->PreMultiply();
	matrix->SetMatrix(I);
	matrix->Concatenate(matrix_1to2);
	matrix->Concatenate(matrix_2to3);
	matrix->Concatenate(matrix_3to4);
	matrix->Concatenate(matrix_4to5);
	matrix->Concatenate(matrix_5to6);
	matrix->Update();
}

void ToolFunction::MultiplyMatrices(vtkTransform* matrix, vtkMatrix4x4* matrix_1to2,
	vtkMatrix4x4* matrix_2to3, vtkMatrix4x4* matrix_3to4, vtkMatrix4x4* matrix_4to5) {

	vtkNew<vtkMatrix4x4> I;
	I->Identity();

	matrix->PreMultiply();
	matrix->SetMatrix(I);
	matrix->Concatenate(matrix_1to2);
	matrix->Concatenate(matrix_2to3);
	matrix->Concatenate(matrix_3to4);
	matrix->Concatenate(matrix_4to5);
	matrix->Update();
}

void ToolFunction::MultiplyMatrices(vtkTransform* matrix, vtkMatrix4x4* matrix_1to2,
	vtkMatrix4x4* matrix_2to3, vtkMatrix4x4* matrix_3to4) {

	vtkNew<vtkMatrix4x4> I;
	I->Identity();

	matrix->PreMultiply();
	matrix->SetMatrix(I);
	matrix->Concatenate(matrix_1to2);
	matrix->Concatenate(matrix_2to3);
	matrix->Concatenate(matrix_3to4);
	matrix->Update();
}

void ToolFunction::MultiplyMatrices(vtkTransform* matrix, vtkMatrix4x4* matrix_1to2, vtkMatrix4x4* matrix_2to3) {

	vtkNew<vtkMatrix4x4> I;
	I->Identity();

	matrix->PreMultiply();
	matrix->SetMatrix(I);
	matrix->Concatenate(matrix_1to2);
	matrix->Concatenate(matrix_2to3);
	matrix->Update();
}

void ToolFunction::Sleep(int msec)
{
	QTime dieTime = QTime::currentTime().addMSecs(msec);

	while (QTime::currentTime() < dieTime)

		QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

void ToolFunction::Double4x4ToMatrix4x4(double matrix[16], vtkMatrix4x4* vtkmatrix) {
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			vtkmatrix->SetElement(i, j, matrix[j + i * 4]);
		}
	}
}

void ToolFunction::Matrix4x4ToDouble4x4(double matrix[16], vtkMatrix4x4* vtkmatrix) {
	memcpy_s(matrix, sizeof(double) * 16, vtkmatrix->GetData(), sizeof(double) * 16);
}

