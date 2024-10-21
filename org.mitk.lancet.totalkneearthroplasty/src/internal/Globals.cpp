#include "Globals.h"
#include "TKAN.h"
#include "ProbeInteractor.h"
#include <windows.h> 
#include <fstream>
#include <iostream>
#include <string>
#include <cstdlib>
#include <filesystem>

/// <summary>
/// Define all Global Variables
/// </summary>

TKAN Tkan;
ProbeInteractor PIController;

int probeMarkerCnt = 0;

int screenWidth = GetSystemMetrics(SM_CXSCREEN);
int screenHeight = GetSystemMetrics(SM_CYSCREEN);

// toolname in AimTools tool definition file
std::vector<std::string> toolids = {"Probe", "Probe3_trigger", "TibiaBoneGuideRF", "TibiaRF", "FemurRF", "BoneGuideRF", "PlaneValidator", "PrecisionBlock", "LancetCalibrator"}; // tools name

// tool status
std::unordered_map<std::string, bool> toolStatusMap = {
	{toolids[0], false},
	{toolids[1], false},
	{toolids[2], false},
	{toolids[3], false},
	{toolids[4], false},
	{toolids[5], false},
	{toolids[6], false},
	{toolids[7], false}
};

// data/object name in mitkworkbench data manager.
std::unordered_map<std::string, std::string> planeDataName = { // [key, value], "value" represents the real data name.
	// planning plane 
	{"TibiaPlanningPlane", "TibiaPlanningPlane"}, // tibia
	{"DistalFemurPlanningPlane", "DistalCutPlane"}, // previous: FemurPlanningPlane

	{"AnteriorFemurPlanningPlane", "AnteriorCutPlane"}, // previous: FourInOnePlanningPlane
	{"AnteriorChamferFemurPlanningPlane", "AnteriorChamferCutPlane"},
	{"PosteriorFemurPlanningPlane", "PosteriorCutPlane"},
	{"PosteriorChamferFemurPlanningPlane", "PosteriorChamferCutPlane"},

	// cutting plane 
	{"TibiaBoneGuideCuttingPlane", "TibiaBoneGuideCuttingPlane"}, // tibia 
	{"FemurBoneGuideCuttingPlane", "FemurBoneGuideCuttingPlane"}, // distal femur

	{"AnteriorFemurCuttingPlane", "AnteriorFemurCuttingPlane"}, // 4-in-1 anterior
	{"AnteriorChamferFemurCuttingPlane", "AnteriorChamferFemurCuttingPlane"}, // 4-in-1 anterior chamfer
	{"PosteriorFemurCuttingPlane", "PosteriorFemurCuttingPlane"}, // 4-in-1 posterior
	{"PosteriorChamferFemurCuttingPlane", "PosteriorChamferFemurCuttingPlane"}, // 4-in-1 posterior chamfer

	// for precision test
	// precision block planning plane
	{"ABF", "ABFPlanningPlane"},
	{"ACE", "ACEPlanningPlane"},
	{"BCD", "BCDPlanningPlane"}, 

	{"ABD", "ABDPlanningPlane"},
	{"ABE", "ABEPlanningPlane"},
	{"ACD", "ACDPlanningPlane"},
	{"ACF", "ACFPlanningPlane"},
	{"BCE", "BCEPlanningPlane"},
	{"BCF", "BCFPlanningPlane"},

	// precision bone guide cutting plane
	{"PrecisionBGCuttingPlane", "PrecisionBoneGuideCuttingPlane"} 
};

std::unordered_map<std::string, std::string> surfaceDataName = {
	{"TibiaSurface", "TibiaSurface"},
	{"FemurSurface", "FemurSurface"}
};

std::unordered_map<std::string, std::string> boneGuideName = {
	{"TibiaBoneGuide", "TibiaBoneGuideSurface"},
	{"FemurBoneGuide", "FemurBoneGuideSurface"},
	{"FourInOneBoneGuide", "FourInOneBoneGuideSurface"},
	{"PrecisionBoneGuide", "PrecisionBoneGuide"}  
};

mitk::DataNode::Pointer planningPlane, boneGuide, cuttingPlane; // find the corresponding dataNode;
mitk::PointSet::Pointer precPlanningPointSet = mitk::PointSet::New(); // used to save 3 points in planning plane. 
mitk::PointSet::Pointer precCuttingPointSet = mitk::PointSet::New(); // the 3 points of Precision Bone Guide cutting plane.
//std::vector<std::array<double, 3>> precPlanningPoints = { // used to save 3 points in planning plane. 
//		{ {-45.132, -94.276, -87.729} }, // default: ABF 
//		{ {-55.330, -90.073, -22.718} },
//		{ {35.278, -98.199, -107.565} }
//};
//
//std::vector<std::array<double, 3>> precCuttingPoints = {
//		{ {12.84, 6.16, -12.95} }, // the 3 points of Precision Bone Guide cutting plane.
//		{ {39, 22, -12.95} },
//		{ {64.39, 10.26, -12.95} }
//};

// navigation data from Camera (NDI/Aimooe)
double nd_CameraToProbe[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
double nd_CameraToProbeTip[3]{0,0,0};
double nd_CameraToProbeTrigger[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
//double nd_CameraToTibiaBoneGuideRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 }; // Corresponding to the old version of the tibia bone guide RF, abandon
double nd_CameraToTibiaRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 }; // tibiaRF
double nd_CameraToFemurRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 }; // FemurRF

double nd_CameraToBoneGuideRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 }; // all Bone Guides use SAME Marker/RF
double nd_CameraToPlaneValidator[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 }; // PlaneValidator/PlaneProbe

double nd_CameraToPrecisionBlockRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 }; // precision block RF

//double nd_CameraToLancetCalibratorRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 }; // Lancet Calibrator RF

double boneGuideNaviTransMtx[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 }; // record the bone guide navigation transform matrix when clicking the save navi btn

std::string desktopPath = std::string(getenv("USERPROFILE")) + "\\Desktop\\";
std::string TransMtxSavePath = "TKAN_TranformMatrix\\";

std::map<std::string, std::array<double, 3>> colorMap = {
		{"red", {1.0, 0.0, 0.0}},
		{"green", {0.0, 1.0, 0.0}},
		{"blue", {0.0, 0.0, 1.0}},
		{"yellow", {1.0, 1.0, 0.0}},
		{"orange", {1.0, 0.67, 0}},
};

// fake data for test & debug
double m_T_tibiaRFtoImage_fake[16] = { // tibia
	-0.236856, 0.74366, -0.625195, 192.064,
	-0.931245, 0.00963623, 0.364265, 27.2691,
	0.276914, 0.668489, 0.690247, 85.364,
	0, 0, 0, 1
};

double m_T_FemurRFtoImage_fake[16] = { // femur
	-0.791292, 0.353987, 0.498548, -225.346,
	0.529511, -0.0109781, 0.848232, 109.079,
	0.305737, 0.935186, -0.178753, 246.72,
	0, 0, 0, 1
};

double m_T_TBGRFtoTBGImage_fake[16] = { // tibia BG
	-0.438467, 0.155873, -0.885127, 12.9553,
	-0.399967, 0.848104, 0.347486, -73.9436,
	0.804843, 0.506383, -0.309522, 134.676,
	0, 0, 0, 1
};

double m_T_FBGRFtoFBGImage_fake[16] = { // femur BG

};

double m_T_FIOBGRFtoFIOBGImage_fake[16] = { // 4-in-1 BG
	-0.0350101, 0.999368, 0.0061111, 26.2609,
	0.99356, 0.0354647, -0.107617, 162.645,
	-0.107766, 0.00230407, -0.994174, 27.8988,
	0, 0, 0, 1
};

double m_T_PVRFtoPVImage_fake[16] = { // Plane Validator
	-0.050262, 0.998163, -0.0338236, 5.02773,
	-0.0139677, 0.0331606, 0.999352, -5.37911,
	0.998638, 0.0507019, 0.0122753, 134.957,
	0, 0, 0, 1
									  
	/*0.99461, 0.0894286, -0.0524643, 124.452,
	-0.0866298, 0.994809, 0.0533972, 42.8621,
	0.0569671, -0.0485645, 0.997194, 8.78174,
	0, 0, 0, 1*/
};

double m_T_PBRFtoPBImage_fake[16] = { // precision block 
	0.490131, -0.856187, 0.163447, -28.46,
	0.422639, 0.397434, 0.814508, 0.612057,
	-0.762331, -0.330136, 0.556652, -184.992,
	0, 0, 0, 1
};

double m_T_PBGRFtoPBGImage_fake[16] = { // precision bone guide 
	-0.479722, 0.166521, -0.861474, 3.83759,
	-0.366332, 0.854146, 0.369101, -69.4175,
	0.797287, 0.492652, -0.348751, 135.902,
	0, 0, 0, 1
};


