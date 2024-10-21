#include "DentalRobot.h"
//DaZu Robot
#include "HR_Pro.h"

//Aimooe Camera
#include "AimPositionAPI.h"
#include "AimPositionDef.h"

//Qt
#include <QFileDialog>
#include <Qtimer>

//AimHandle aimHandle = NULL;


void DentalRobot::connectArm() 
{
	m_LancetHansRobot->Connect();
}

void DentalRobot::powerOn() 
{
	m_LancetHansRobot->PowerOn();
}

void DentalRobot::powerOff()
{
	int nGrpDisable = HRIF_GrpDisable(0, 0);
	PrintResult(m_Controls.textBrowser, nGrpDisable, "powerOff");
}

void DentalRobot::connectCamera()
{
	T_AIMPOS_DATAPARA mPosDataPara;
	Aim_API_Initial(aimHandle);
	Aim_SetEthernetConnectIP(aimHandle, 192, 168, 31, 10);
	rlt = Aim_ConnectDevice(aimHandle, I_ETHERNET, mPosDataPara);
	if (rlt == AIMOOE_OK)
	{
		m_Controls.textBrowser->append("===== Aimooe Connect Success ====");
		std::cout << "Aimooe connect success";
	}
	else {
		m_Controls.textBrowser->append("===== Aimooe Connect Failed ====");
		std::cout << "Aimooe connect failed";
	}
	QString filepath = "D:\\";
	QString filename = QFileDialog::getExistingDirectory(nullptr, "Aimooe Tools", filepath);
	if (filename.isNull()) return;
	filename.append("/");
	std::cout << "The selected folder address :" << filename;
	rlt = Aim_SetToolInfoFilePath(aimHandle, filename.toLatin1().data());
	if (rlt == AIMOOE_OK)
	{
		m_Controls.textBrowser->append("===== Set filename success ====");
		std::cout << "set filename success";
	}
	else {
		m_Controls.textBrowser->append("===== Set filename failed ====");
		std::cout << "set filename failed";
	}
	int size = 0;
	Aim_GetCountOfToolInfo(aimHandle, size);
	if (size != 0)
	{
		t_ToolBaseInfo* toolarr = new t_ToolBaseInfo[size];
		rlt = Aim_GetAllToolFilesBaseInfo(aimHandle, toolarr);
		if (rlt == AIMOOE_OK)
		{
			for (int i = 0; i < size; i++)
			{
				char* ptool = toolarr[i].name;
				QString toolInfo = QString(ptool);
				m_Controls.textBrowser->append(toolInfo);
			}
		}
		delete[] toolarr;
	}
	else {
		std::cout << "There are no tool identification files in the current directory:";
		m_Controls.textBrowser->append("There are no tool identification files in the current directory:");
	}
	std::cout << "End of connection";
	m_Controls.textBrowser->append("End of connection");

	rlt = AIMOOE_OK;
	m_Controls.textBrowser->append("-------------------------------------------------------------");
}

void DentalRobot::openCameraQtTimer()
{
	if (m_AimooeVisualizeTimer == nullptr)
	{
		m_AimooeVisualizeTimer = new QTimer(this);
	}
	connect(m_AimooeVisualizeTimer, SIGNAL(timeout()), this, SLOT(updateCameraData_Dental())); //口腔拿数据第二种方式
	m_AimooeVisualizeTimer->start(100);
}


void DentalRobot::updateCameraData_Dental()
{
	auto prlt = GetNewToolData();
	if (rlt == AIMOOE_OK)//判断是否采集成功
	{
		do
		{
			//更新数据
			UpdateCameraToToolMatrix(prlt, "Oral_RobotBaseRF", m_T_CamToBaseRF, m_Controls.DentalRobot_BaseRF_DataLabel);
			UpdateCameraToToolMatrix(prlt, "Oral_RobotEndRF", m_T_CamToEndRF, m_Controls.DentalRobot_EndRF_DataLabel);
			//UpdateCameraToToolMatrix(prlt, "Oral_handpiece", T_CamToEndRF, m_Controls.Spine_RobotEndRFDataLabel);
			UpdateCameraToToolMatrix(prlt, "Oral_PatientRF", m_T_CamToPatientRF, m_Controls.DentalRobot_PatientRF_DataLabel);
			UpdateCameraToToolMatrix(prlt, "Oral_Probe", m_T_CamToProbe, m_Controls.DentalRobot_Probe_DataLabel);
			//UpdateCameraToToolMatrix(prlt, "Oral_ProbeA", T_CamToProbe, m_Controls.Spine_ProbeDataLabel);
			//UpdateCameraToToolMatrix(prlt, "Oral_ProbeB", T_CamToProbe, m_Controls.Spine_ProbeDataLabel);
			UpdateCameraToToolMatrix(prlt, "Oral_TCPRF", m_T_CamToCalibratorRF, m_Controls.DentalRobot_MetalBallRF_DataLabel);
			
			//获取Probe数据
			if (strcmp(prlt->toolname, "Oral_Probe") == 0)
			{
				if (prlt->validflag)
				{
					m_T_CamToProbe[3] = prlt->tooltip[0];
					m_T_CamToProbe[7] = prlt->tooltip[1];
					m_T_CamToProbe[11] = prlt->tooltip[2];
					ProbeTop[0] = prlt->tooltip[0];
					ProbeTop[1] = prlt->tooltip[1];
					ProbeTop[2] = prlt->tooltip[2];
				}
			}
			T_AimToolDataResult* pnext = prlt->next;
			delete prlt;
			prlt = pnext;
		} while (prlt != NULL);
	}
	else
	{
		delete prlt;
	}

}


//---------------------------------------------------------------------------------------------------------------
/**
*@brief 打印机械臂函数是否执行成功，不成功返回错误码和错误类型
*@note  用法：PrintResult(m_Controls.textBrowser, n, "你想写的话");
*/
//---------------------------------------------------------------------------------------------------------------
void DentalRobot::PrintResult(QTextBrowser* browser, int nRet, const char* message)
{
	if (nRet == 0) {
		browser->append(QString(message) + " Succeed");
		std::cout << message << " Succeed" << std::endl;
		std::cout << "---------------------------------------------------" << std::endl;
		browser->append("---------------------------------------------------");
	}
	else {
		browser->append(QString(message) + " Failed");
		std::cout << message << " Failed" << std::endl;

		std::cout << "ErrorCode: " << nRet << std::endl;
		string tmpstr;
		int zyx = HRIF_GetErrorCodeStr(0, nRet, tmpstr);
		std::cout << nRet << std::endl;
		/*browser->append("ErrorCode：" + QString::fromStdString(tmpstr));*/
		browser->append("ErrorCode: " + QString::number(nRet));
		//针对一些常见错误做的一些解释
		if (nRet == 39500)
		{
			std::cout << "Please check the connection of the robot arm" << std::endl;
			browser->append("Please check the connection of the robot arm");

		}
		if (nRet == 40025)
		{
			browser->append("The robot is not in a state of readiness" + QString::number(nRet));
		}

		if (nRet >= 40000 && nRet <= 40500) {
			std::cout << "CDS executed command with error: " << nRet << std::endl;
			/*browser->append("CDS executed command with error: " + QString::number(nRet));*/
			browser->append("CDS执行命令时发生错误：" + QString::number(nRet));
		}
		else if (nRet >= 10000 && nRet <= 10015) {
			std::cout << "Robot servo drive reported fault code: " << nRet << std::endl;
			/*browser->append("Robot servo drive reported fault code: " + QString::number(nRet));*/
			browser->append("机器人伺服驱动报告故障代码：" + QString::number(nRet));
		}
		else if (nRet >= 10016 && nRet <= 11000) {
			std::cout << "Robot collaboration algorithm detected fault: " << nRet << std::endl;
			/*browser->append("Robot collaboration algorithm detected fault: " + QString::number(nRet));*/
			browser->append("检测到机器人协作算法故障：" + QString::number(nRet));
		}
		else if (nRet >= 15000 && nRet <= 16000) {
			std::cout << "Robot control module detected fault: " << nRet << std::endl;
			/*browser->append("Robot control module detected fault: " + QString::number(nRet));*/
			/*browser->append("检测到机器人控制模块故障：" + QString::number(nRet));*/
		}
		else if (nRet >= 30001 && nRet <= 30016) {
			std::cout << "Modbus module error during command execution: " << nRet << std::endl;
			/*browser->append("Modbus module error during command execution: " + QString::number(nRet));*/
			/*browser->append("Modbus模块在执行命令时发生错误：" + QString::number(nRet));*/
		}
		else if (nRet >= 20000 && nRet <= 39999) {
			std::cout << "CPS executed command with error: " << nRet << std::endl;
			/*browser->append("CPS executed command with error: " + QString::number(nRet));*/
			/*browser->append("CPS执行命令时发生错误：" + QString::number(nRet));*/
		}

		std::cout << "---------------------------------------------------" << std::endl;
		browser->append("---------------------------------------------------");
	}
}