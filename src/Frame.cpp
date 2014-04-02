#include <ollieRosTools/Frame.hpp>

/// initialise static members
CameraATAN Frame::cameraModel = CameraATAN();
Detector   Frame::detector    = Detector();
PreProc    Frame::preproc     = PreProc();
int Frame::idCounter   = -1;
int Frame::kfIdCounter   = -1;
float Frame::averageQuality = 0.8;

