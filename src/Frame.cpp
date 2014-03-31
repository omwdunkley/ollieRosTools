#include <ollieRosTools/Frame.hpp>

/// initialise static members
CameraATAN Frame::cameraModel = CameraATAN();
Detector   Frame::detector    = Detector();
PreProc    Frame::preproc     = PreProc();
long unsigned int Frame::idCounter   = 0;
long unsigned int Frame::kfIdCounter   = 0;
float Frame::averageQuality = 0.8;

