// Globals.h
#ifndef GLOBALS_H
#define GLOBALS_H

#include "TKAN.h"
#include "ProbeInteractor.h"

/// <summary>
/// Declare all Global Variables, and Define in "Globals.cpp"
/// </summary>

class TKAN; // Forward Declaration
extern TKAN Tkan; // declaration a global variable

class ProbeInteractor;
extern ProbeInteractor PIController;

extern int probeMarkerCnt;

extern int screenWidth; // window size
extern int screenHeight;

// tool ids, for Aimooe
extern std::vector<std::string> toolids; // tools name

// tool status
extern std::unordered_map<std::string, bool> toolStatusMap;
//extern bool ProbeValid; 
//extern bool ProbeTriggerValid;
//extern bool TibiaBoneGuideRFValid;
//extern bool TibiaRFValid;


// data/object name in mitkworkbench data manager.
extern std::unordered_map<std::string, std::string> planeDataName;
extern std::unordered_map<std::string, std::string> surfaceDataName;
extern std::unordered_map<std::string, std::string> boneGuideName;

extern mitk::DataNode::Pointer planningPlane, boneGuide, cuttingPlane; 
extern mitk::PointSet::Pointer precPlanningPointSet;
extern mitk::PointSet::Pointer precCuttingPointSet;
//extern std::vector<std::array<double, 3>> precPlanningPoints;
//extern std::vector<std::array<double, 3>> precCuttingPoints;

// navigation data from Camera (NDI/Aimooe)
extern double nd_CameraToProbe[16]; // probe navigation data under Camera Coordinate
extern double nd_CameraToProbeTip[3]; // probe tip position (3D) under Camera Coordinate
extern double nd_CameraToProbeTrigger[16];
//extern double nd_CameraToTibiaBoneGuideRF[16];
extern double nd_CameraToTibiaRF[16];

extern double nd_CameraToBoneGuideRF[16];// all Bone Guides use only ONE Marker/RF
extern double nd_CameraToFemurRF[16]; // FemurRF
extern double nd_CameraToPlaneValidator[16]; // PlaneValidator/PlaneProbe

extern double nd_CameraToPrecisionBlockRF[16]; // precision block RF

//extern double nd_CameraToLancetCalibratorRF[16]; // Lancet Calibrator RF

extern double boneGuideNaviTransMtx[16]; // record the bone guide navigation transform matrix when clicking the save navi btn

extern std::string desktopPath;
extern std::string TransMtxSavePath;
extern std::map<std::string, std::array<double, 3>> colorMap;

extern double m_T_tibiaRFtoImage_fake[16];
extern double m_T_FemurRFtoImage_fake[16];
extern double m_T_TBGRFtoTBGImage_fake[16];
extern double m_T_FBGRFtoFBGImage_fake[16];
extern double m_T_FIOBGRFtoFIOBGImage_fake[16];
extern double m_T_PVRFtoPVImage_fake[16];
extern double m_T_PBRFtoPBImage_fake[16];
extern double m_T_PBGRFtoPBGImage_fake[16];

#endif // GLOBALS_H