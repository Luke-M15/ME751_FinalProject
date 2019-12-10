//===================================================================
//Author: Luke Mottley
//ME 751 Final Project 
//
//Rigid tire rolling on deformable soil (SCM), based significantly on demo_VEH_DeformableSoil.
//Rotational speed applied with the rotational motor. Record and plot
//  - terrain forces 
//	- motor torque 
//===================================================================


#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkMotorRotation.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"
#include "chrono_postprocess/ChGnuPlot.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::irrlicht;

using namespace irr;

const std::string out_dir = "Rolling_tire";

int main(int argc, char* argv[]) {
	GetLog() << "ME 751 Final Project Rolling Tire Simulation\n";
	// Set path to Chrono data directory
	SetChronoDataPath(CHRONO_DATA_DIR);

	// Global parameter for tire:
	double tire_rad = 0.8;
	double tire_vel_z0 = -3;
	ChVector<> tire_center(0, 0.02 + tire_rad, 0);

	double tire_w0 = tire_vel_z0 / tire_rad;

	// Create a Chrono::Engine physical system
	ChSystemSMC my_system;

	// Create the Irrlicht visualization (open the Irrlicht device,
	// bind a simple user interface, etc. etc.)
	ChIrrApp application(&my_system, L"Deformable soil", core::dimension2d<u32>(1280, 720), false, true);

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(1.0f, 1.4f, -1.2f), core::vector3df(0, (f32)tire_rad, 0));
	application.AddLightWithShadow(core::vector3df(1.5f, 5.5f, -2.5f), core::vector3df(0, 0, 0), 3, 2.2, 7.2, 40, 512,
		video::SColorf(0.8f, 0.8f, 1.0f));

	std::shared_ptr<ChBody> mtruss(new ChBody);
	mtruss->SetBodyFixed(true);
	my_system.Add(mtruss);

	// Initialize output
	
	if (!filesystem::create_directory(filesystem::path(out_dir))) {
		std::cout << "Error creating directory " << out_dir << std::endl;
		return 1;
	}
	
	utils::CSV_writer outFile(" ");

	//
	// CREATE A RIGID BODY WITH A MESH
	//

	std::shared_ptr<ChBody> rigidTire(new ChBody);
	my_system.Add(rigidTire);
	rigidTire->SetMass(500);
	rigidTire->SetInertiaXX(ChVector<>(20, 20, 20));
	rigidTire->SetPos(tire_center + ChVector<>(0, 0.3, 0));

	auto trimesh = std::make_shared<geometry::ChTriangleMeshConnected>();
	trimesh->LoadWavefrontMesh(GetChronoDataFile("tractor_wheel.obj"));

	std::shared_ptr<ChTriangleMeshShape> mrigidmesh(new ChTriangleMeshShape);
	mrigidmesh->SetMesh(trimesh);
	rigidTire->AddAsset(mrigidmesh);

	rigidTire->GetCollisionModel()->ClearModel();
	rigidTire->GetCollisionModel()->AddTriangleMesh(trimesh, false, false, VNULL, ChMatrix33<>(1), 0.01);
	rigidTire->GetCollisionModel()->BuildModel();
	rigidTire->SetCollide(true);

	std::shared_ptr<ChColorAsset> mcol(new ChColorAsset);
	mcol->SetColor(ChColor(0.3f, 0.3f, 0.3f));
	rigidTire->AddAsset(mcol);
	
	std::shared_ptr<ChLinkMotorRotationSpeed> myMotor(new ChLinkMotorRotationSpeed);
	myMotor->Initialize(rigidTire, mtruss, ChFrame<>(tire_center, Q_from_AngAxis(CH_C_PI_2, VECT_Y)));
	my_system.Add(myMotor);
	// create speed function and use it to set the motor speed
	auto mSpeed = std::make_shared<ChFunction_Ramp>(0.0,CH_C_PI / 4.0);
	auto mSpeed_const = std::make_shared<ChFunction_Const>(0.1);
	myMotor->SetSpeedFunction(mSpeed);
	myMotor->SetSpindleConstraint(ChLinkMotorRotation::SpindleConstraint::OLDHAM);

	//
	// THE DEFORMABLE TERRAIN
	//

	// Create the 'deformable terrain' object
	vehicle::SCMDeformableTerrain mterrain(&my_system);

	// Optionally, displace/tilt/rotate the terrain reference plane:
	mterrain.SetPlane(ChCoordsys<>(ChVector<>(0, 0, 0.5)));

	// Initialize the geometry of the soil: use either a regular grid:
	mterrain.Initialize(0.2, 1.5, 5, 20, 60);

	// Set the soil terramechanical parameters:
	mterrain.SetSoilParametersSCM(0.2e6,  // Bekker Kphi
		0,      // Bekker Kc
		1.1,    // Bekker n exponent
		0,      // Mohr cohesive limit (Pa)
		30,     // Mohr friction limit (degrees)
		0.01,   // Janosi shear coefficient (m)
		4e7,    // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
		3e4     // Damping (Pa s/m), proportional to negative vertical speed (optional)
	);

	// LETE sand parameters
	////mterrain.SetSoilParametersSCM(5301e3,  // Bekker Kphi
	////                              102e3,   // Bekker Kc
	////                              0.793,   // Bekker n exponent
	////                              1.3e3,   // Mohr cohesive limit (Pa)
	////                              31.1,    // Mohr friction limit (degrees)
	////                              1.2e-2,  // Janosi shear coefficient (m)
	////                              4e8,     // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
	////                              3e4      // Damping (Pa s/m), proportional to negative vertical speed (optional)
	////);

	mterrain.SetBulldozingFlow(true);  // inflate soil at the border of the rut
	mterrain.SetBulldozingParameters(
		55,   // angle of friction for erosion of displaced material at the border of the rut
		1,    // displaced material vs downward pressed material.
		5,    // number of erosion refinements per timestep
		10);  // number of concentric vertex selections subject to erosion
	// Turn on the automatic level of detail refinement, so a coarse terrain mesh
	// is automatically improved by adding more points under the wheel contact patch:
	mterrain.SetAutomaticRefinement(true);
	mterrain.SetAutomaticRefinementResolution(0.04);

	// Set some visualization parameters: either with a texture, or with falsecolor plot, etc.
	////mterrain.SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16, 16);
	mterrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_PRESSURE, 0, 30000.2);
	////mterrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_PRESSURE_YELD, 0, 30000.2);
	////mterrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_SINKAGE, 0, 0.15);
	////mterrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_SINKAGE_PLASTIC, 0, 0.15);
	////mterrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_SINKAGE_ELASTIC, 0, 0.05);
	////mterrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_STEP_PLASTIC_FLOW, 0, 0.0001);
	////mterrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_ISLAND_ID, 0, 8);
	////mterrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_IS_TOUCHED, 0, 8);
	mterrain.GetMesh()->SetWireframe(true);

	// ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
	application.AssetBindAll();

	// ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
	application.AssetUpdateAll();

	// Use shadows in realtime view
	application.AddShadowAll();

	// ==IMPORTANT!== Mark completion of system construction
	my_system.SetupInitial();

	application.SetTimestep(0.002);

	// add column headers for the output csv
	//outFile << "Time X_force Y_force Z_force X_moment Y_moment Z_moment X_pnt Y_pnt Z_pnt motorTrq" << std::endl;
	double t_end = 2;
	while (application.GetDevice()->run()) {
		double time = my_system.GetChTime();

		if (time >= t_end)
			break;

		vehicle::TerrainForce frc = mterrain.GetContactForce(rigidTire);
		double torq = myMotor->GetMotorTorque();

		outFile << time << frc.force << frc.moment << frc.point << torq << std::endl;
	

		application.BeginScene();

		application.DrawAll();

		application.DoStep();

		ChIrrTools::drawColorbar(0, 30000, "Pressure yield [Pa]", application.GetDevice(), 1180);

		application.EndScene();
	}

	outFile.write_to_file(out_dir + "/output.dat");
	// creating plots
	std::string plot1 = out_dir + "/terrain_y_force.gpl";
	postprocess::ChGnuPlot mplot1(plot1.c_str());
	//std::string plot1_png = out_dir + "/terrain_y_force.png";
	//mplot1.OutputPNG(plot1_png.c_str());
	mplot1.SetGrid();
	mplot1.SetLabelX("Time (sec)");
	mplot1.SetLabelY("Force (N)");
	mplot1.Plot("output.dat", 1, 3, "Terrain Y Force");
	
	std::string plot2 = out_dir + "/terrain_z_force.gpl";
	postprocess::ChGnuPlot mplot2(plot2.c_str());
	mplot2.SetGrid();
	mplot2.SetLabelX("Time (sec)");
	mplot2.SetLabelY("Force (N)");
	mplot2.Plot("output.dat", 1, 4, "Terrain Z Force");

	std::string plot3 = out_dir + "/motor_torque.gpl";
	postprocess::ChGnuPlot mplot3(plot3.c_str());
	mplot3.SetGrid();
	mplot3.SetLabelX("Time (sec)");
	mplot3.SetLabelY("Torque (N-m)");
	mplot3.Plot("output.dat", 1, 11, "Motor Torque");
	

	return 0;
}