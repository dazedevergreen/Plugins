#include "ProbeInteractor.h"

ProbeInteractor::ProbeInteractor() {
	// init
	consecFrame = 0; 
	consecFrameThreshold = 10; // threshold
	collectPermit = true;
}

ProbeInteractor::ProbeInteractor(int threshold) {
	// init
	consecFrame = 0;
	consecFrameThreshold = threshold; // threshold
	collectPermit = true;
}

bool ProbeInteractor::CanCollectData(int markerCnt) {
	if (markerCnt == 3 && collectPermit) {
		consecFrame++;
	}
	else if (markerCnt == 4) {
		// reset/init
		consecFrame = 0;
		collectPermit = true;
	}

	if (markerCnt == 3 && collectPermit && consecFrame >= consecFrameThreshold) {
		collectPermit = false; // frozen, change the status after collect data.
		return true; // condition satisified, do collect.
	}
	return false; 
}

bool ProbeInteractor::GetCollectPermit() {
	return collectPermit;
}