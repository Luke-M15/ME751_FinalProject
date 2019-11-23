//===================================================================
//Author: Luke Mottley
//ME 751 Final Project 
//
//Rolling tire on SCM deformable soil

//===================================================================


#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono_irrlicht/ChIrrApp.h"

// Use the namespace of Chrono

using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht

using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

int main(int argc, char* argv[]) {
// Set path to Chrono data directory
SetChronoDataPath(CHRONO_DATA_DIR);

// Create a Chrono physical system
ChSystemNSC mphysicalSystem;

// Create the Irrlicht visualization (open the Irrlicht device,
// bind a simple user interface, etc. etc.)
ChIrrApp application(&mphysicalSystem, L"A simple project template", core::dimension2d<u32>(800, 600),
	false);  // screen dimensions

// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
application.AddTypicalLogo();
application.AddTypicalSky();
application.AddTypicalLights();
application.AddTypicalCamera(core::vector3df(2, 2, -5),
							core::vector3df(0, 1, 0));  // to change the position of camera