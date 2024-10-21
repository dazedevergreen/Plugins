#ifndef PROBE_INTERACTOR_H
#define PROBE_INTERACTOR_H

class ProbeInteractor {
public:
	ProbeInteractor();
	ProbeInteractor(int threshold); // define the Consecutive Frame Threshold
	
	bool CanCollectData(int markerCnt); // markerCnt is 3 OR 4 to determine the permission to collect data.

	bool GetCollectPermit();

private:
	int consecFrame = 0; // count the consecutive frames
	int consecFrameThreshold; // threshold
	bool collectPermit = true;
};

#endif // PROBE_INTERACTOR_H