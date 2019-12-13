//===================================================================
//Author: Luke Mottley
//ME 751 Final Project
// Full Vehicle model accelerating on deformable terrain, based on the HMMWV 
// deformable soil demo
//====================================================================
#include <cstdio>
#include <cmath>
#include <vector>

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_postprocess/ChGnuPlot.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================

// -----------------------------------------------------------------------------
// Terrain parameters
// -----------------------------------------------------------------------------

// Dimensions
double terrainHeight = 0;
double terrainLength = 16.0;  // size in X direction
double terrainWidth = 8.0;    // size in Y direction

// Divisions (X and Y)
int divLength = 128;//1024;
int divWidth = 64;//512;

// -----------------------------------------------------------------------------
// Vehicle parameters
// -----------------------------------------------------------------------------

// Type of wheel/tire (controls both contact and visualization)
enum WheelType { CYLINDRICAL, LUGGED };
WheelType wheel_type = LUGGED;

// Type of terrain
enum TerrainType { DEFORMABLE_SOIL, RIGID_SOIL };
TerrainType terrain_type = DEFORMABLE_SOIL;

// Type of powertrain model (SHAFTS, SIMPLE)
PowertrainModelType powertrain_model = PowertrainModelType::SIMPLE;

// Drive type (FWD, RWD, or AWD)
DrivelineType drive_type = DrivelineType::FWD;

// Chassis visualization (MESH, PRIMITIVES, NONE)
VisualizationType chassis_vis = VisualizationType::NONE;

// Initial vehicle position and orientation
ChVector<> initLoc(-5, -2, 0.6);
ChQuaternion<> initRot(1, 0, 0, 0);

// Contact material properties
float Y_t = 1.0e6f;
float cr_t = 0.1f;
float mu_t = 0.8f;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Simulation Length
double t_end = 5;

// Simulation step size
double step_size = 3e-3;

// Time interval between two render frames (1/FPS)
double render_step_size = 1.0 / 100;

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Output directories
const std::string out_dir = "HMMWV_FWD_straight_accel";
const std::string img_dir = out_dir + "/IMG";

// Visualization output
bool img_output = false;

// =============================================================================

class MyDriver : public ChDriver {
public:
	MyDriver(ChVehicle& vehicle, double delay) : ChDriver(vehicle), m_delay(delay) {}
	~MyDriver() {}

	virtual void Synchronize(double time) override {
		m_throttle = 0;
		m_steering = 0;
		m_braking = 0;

		double eff_time = time - m_delay;

		// Do not generate any driver inputs for a duration equal to m_delay.
		if (eff_time < 0)
			return;

		if (eff_time > 0.2)
			m_throttle = 0.9;
		else
			m_throttle = 1.5 * eff_time;
	}

private:
	double m_delay;
};

// =============================================================================

void CreateLuggedGeometry(std::shared_ptr<ChBody> wheelBody) {

	std::string lugged_file("vehicle/hmmwv/lugged_wheel_section.obj");
	geometry::ChTriangleMeshConnected lugged_mesh;
	ChConvexDecompositionHACDv2 lugged_convex;
	utils::LoadConvexMesh(GetChronoDataFile(lugged_file), lugged_mesh, lugged_convex);
	int num_hulls = lugged_convex.GetHullCount();

	auto coll_model = wheelBody->GetCollisionModel();
	coll_model->ClearModel();

	// Assemble the tire contact from 15 segments, properly offset.
	// Each segment is further decomposed in convex hulls.
	for (int iseg = 0; iseg < 15; iseg++) {
		ChQuaternion<> rot = Q_from_AngAxis(iseg * 24 * CH_C_DEG_TO_RAD, VECT_Y);
		for (int ihull = 0; ihull < num_hulls; ihull++) {
			std::vector<ChVector<> > convexhull;
			lugged_convex.GetConvexHullResult(ihull, convexhull);
			coll_model->AddConvexHull(convexhull, VNULL, rot);
		}
	}

	// Add a cylinder to represent the wheel hub.
	coll_model->AddCylinder(0.223, 0.223, 0.126);
	coll_model->BuildModel();

	// Visualization
	auto trimesh = std::make_shared<geometry::ChTriangleMeshConnected>();
	trimesh->LoadWavefrontMesh(GetChronoDataFile("vehicle/hmmwv/lugged_wheel.obj"), false, false);

	auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
	trimesh_shape->SetMesh(trimesh);
	trimesh_shape->SetName("lugged_wheel");
	wheelBody->AddAsset(trimesh_shape);

	auto mcolor = std::make_shared<ChColorAsset>(0.3f, 0.3f, 0.3f);
	wheelBody->AddAsset(mcolor);
}

// =============================================================================

int main(int argc, char* argv[]) {
	GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
	// Set path to Chrono data directory
	SetChronoDataPath(CHRONO_DATA_DIR);

	// --------------------
	// Create HMMWV vehicle
	// --------------------
	HMMWV_Full my_hmmwv;
	my_hmmwv.SetContactMethod(ChMaterialSurface::SMC);
	my_hmmwv.SetChassisFixed(false);
	my_hmmwv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
	my_hmmwv.SetPowertrainType(powertrain_model);
	my_hmmwv.SetDriveType(drive_type);
	my_hmmwv.SetTireType(TireModelType::RIGID);
	my_hmmwv.SetVehicleStepSize(step_size);
	my_hmmwv.Initialize();

	VisualizationType wheel_vis = (wheel_type == CYLINDRICAL) ? VisualizationType::MESH : VisualizationType::NONE;
	my_hmmwv.SetChassisVisualizationType(chassis_vis);
	my_hmmwv.SetWheelVisualizationType(wheel_vis);

	ChSystem* system = my_hmmwv.GetSystem();

	// --------------------------------------------------------
	// Set wheel contact material.
	// If needed, modify wheel contact and visualization models
	// --------------------------------------------------------
	for (int i = 0; i < 4; i++) {
		auto wheelBody = my_hmmwv.GetVehicle().GetWheelBody(i);
		wheelBody->GetMaterialSurfaceSMC()->SetFriction(mu_t);
		wheelBody->GetMaterialSurfaceSMC()->SetYoungModulus(Y_t);
		wheelBody->GetMaterialSurfaceSMC()->SetRestitution(cr_t);

		CreateLuggedGeometry(wheelBody);
	}

	// --------------------
	// Create driver system
	// --------------------
	MyDriver driver(my_hmmwv.GetVehicle(), 0.25);
	driver.Initialize();

	// ------------------
	// Create the terrain
	// ------------------
	ChTerrain* terrain;

	
	SCMDeformableTerrain* terrainD = new SCMDeformableTerrain(system);
	terrainD->SetPlane(ChCoordsys<>(VNULL, Q_from_AngX(CH_C_PI_2)));
	terrainD->SetSoilParametersSCM(2e6,   // Bekker Kphi
		0,     // Bekker Kc
		1.1,   // Bekker n exponent
		0,     // Mohr cohesive limit (Pa)
		30,    // Mohr friction limit (degrees)
		0.01,  // Janosi shear coefficient (m)
		2e8,   // Elastic stiffness (Pa/m), before plastic yield
		3e4    // Damping (Pa s/m), proportional to negative vertical speed (optional)
	);
	/*
	terrainD->SetBulldozingFlow(true);    // inflate soil at the border of the rut
	terrainD->SetBulldozingParameters(55, // angle of friction for erosion of displaced material at the border of the rut
									0.8, // displaced material vs downward pressed material.
									5,   // number of erosion refinements per timestep
									10); // number of concentric vertex selections subject to erosion
	*/
	// Turn on the automatic level of detail refinement, so a coarse terrain mesh
	// is automatically improved by adding more points under the wheel contact patch:
	terrainD->SetAutomaticRefinement(true);
	terrainD->SetAutomaticRefinementResolution(0.04);

	////terrainD->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 80, 16);
	////terrainD->SetPlotType(vehicle::SCMDeformableTerrain::PLOT_PRESSURE_YELD, 0, 30000.2);
	terrainD->SetPlotType(vehicle::SCMDeformableTerrain::PLOT_SINKAGE, 0, 0.1);

	terrainD->Initialize(terrainHeight, terrainLength, terrainWidth, divLength, divWidth);

	terrain = terrainD;


	// Complete system construction
	system->SetupInitial();

	// ---------------------------------------
	// Create the vehicle Irrlicht application
	// ---------------------------------------
	ChWheeledVehicleIrrApp app(&my_hmmwv.GetVehicle(), &my_hmmwv.GetPowertrain(), L"HMMWV FWD Deformable Soil Straight Accel");
	app.SetSkyBox();
	app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
	app.SetChaseCamera(trackPoint, 6.0, 0.5);
	app.SetTimestep(step_size);
	app.AssetBindAll();
	app.AssetUpdateAll();

	// -----------------
	// Initialize output
	// -----------------
	my_hmmwv.GetVehicle().SetSuspensionOutput(0, true);
	my_hmmwv.GetVehicle().SetSteeringOutput(0, true);
	my_hmmwv.GetVehicle().SetOutput(ChVehicleOutput::ASCII, out_dir, "component_output", 0.1);

	// Generate JSON information with available output channels
	my_hmmwv.GetVehicle().ExportComponentList(out_dir + "/component_list.json");

	if (!filesystem::create_directory(filesystem::path(out_dir))) {
		std::cout << "Error creating directory " << out_dir << std::endl;
		return 1;
	}
	if (img_output) {
		if (!filesystem::create_directory(filesystem::path(img_dir))) {
			std::cout << "Error creating directory " << img_dir << std::endl;
			return 1;
		}
	}
	
	utils::CSV_writer outFile(" ");
	// ---------------
	// Simulation loop
	// ---------------

	std::cout << "Total vehicle mass: " << my_hmmwv.GetTotalMass() << std::endl;

	// Solver settings.
	////system->SetSolverType(ChSolver::Type::MINRES);
	system->SetMaxItersSolverSpeed(50);
	system->SetMaxItersSolverStab(50);
	////system->SetTol(0);
	////system->SetMaxPenetrationRecoverySpeed(1.5);
	////system->SetMinBounceSpeed(2.0);
	////system->SetSolverOverrelaxationParam(0.8);
	////system->SetSolverSharpnessParam(1.0);

	// Number of simulation steps between two 3D view render frames
	int render_steps = (int)std::ceil(render_step_size / step_size);

	// Initialize simulation frame counter
	int step_number = 0;
	int render_frame = 0;
	ChRealtimeStepTimer realtime_timer;

	
	while (app.GetDevice()->run()) {
		double time = system->GetChTime();

		if (time >= t_end)
			break;
		auto frcL = static_cast<ChRigidTire*>(my_hmmwv.GetTire(0))->ReportTireForce(terrain);
		double longSL = my_hmmwv.GetTire(0)->GetLongitudinalSlip();
		double slipL = my_hmmwv.GetTire(0)->GetSlipAngle();
		auto frcR = static_cast<ChRigidTire*>(my_hmmwv.GetTire(1))->ReportTireForce(terrain);
		double longSR = my_hmmwv.GetTire(1)->GetLongitudinalSlip();
		double slipR = my_hmmwv.GetTire(1)->GetSlipAngle();
		auto drawL = my_hmmwv.GetVehicle().GetSuspension(0)->GetRevolute(LEFT)->Get_react_force();
		auto drawR = my_hmmwv.GetVehicle().GetSuspension(0)->GetRevolute(RIGHT)->Get_react_force();
		if (time > .75) {
			outFile << time << frcL.force.x() << frcL.force.y() << frcL.force.z() << longSL << slipL << drawL;
			outFile << frcR.force.x() << frcR.force.y() << frcR.force.z() << longSR << slipR << drawR << std::endl;
		}
		// Render scene
		app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
		app.DrawAll();
		ChIrrTools::drawColorbar(0, 0.1, "Sinkage", app.GetDevice(), 30);

		if (img_output && step_number % render_steps == 0) {
			char filename[100];
			sprintf(filename, "%s/img_%03d.jpg", img_dir.c_str(), render_frame + 1);
			app.WriteImageToFile(filename);
			render_frame++;
		}

		double throttle_input = driver.GetThrottle();
		double steering_input = driver.GetSteering();
		double braking_input = driver.GetBraking();

		// Update modules
		driver.Synchronize(time);
		terrain->Synchronize(time);
		my_hmmwv.Synchronize(time, steering_input, braking_input, throttle_input, *terrain);
		app.Synchronize("", steering_input, throttle_input, braking_input);

		// Advance dynamics
		double step = realtime_timer.SuggestSimulationStep(step_size);
		system->DoStepDynamics(step);
		app.Advance(step);

		// Increment frame number
		step_number++;

		app.EndScene();
	}
	outFile.write_to_file(out_dir + "/output.dat");
	// creating plots
	std::string plot1 = out_dir + "/left_tire_y_force.gpl";
	postprocess::ChGnuPlot mplot1(plot1.c_str());
	mplot1.SetGrid();
	mplot1.SetLabelX("Time (sec)");
	mplot1.SetLabelY("Force (N)");
	mplot1.Plot("output.dat", 1, 3, "Left Tire Y Force");

	std::string plot2 = out_dir + "/left_tire_z_force.gpl";
	postprocess::ChGnuPlot mplot2(plot2.c_str());
	mplot2.SetGrid();
	mplot2.SetLabelX("Time (sec)");
	mplot2.SetLabelY("Force (N)");
	mplot2.Plot("output.dat", 1, 4, "Left Tire Z Force");

	std::string plot3 = out_dir + "/right_tire_y_force.gpl";
	postprocess::ChGnuPlot mplot3(plot3.c_str());
	mplot3.SetGrid();
	mplot3.SetLabelX("Time (sec)");
	mplot3.SetLabelY("Force (N)");
	mplot3.Plot("output.dat", 1, 11, "Right Tire Y Force");

	std::string plot4 = out_dir + "/right_tire_z_force.gpl";
	postprocess::ChGnuPlot mplot4(plot4.c_str());
	mplot4.SetGrid();
	mplot4.SetLabelX("Time (sec)");
	mplot4.SetLabelY("Force (N)");
	mplot4.Plot("output.dat", 1, 12, "Right Tire Z Force");

	std::string plot5 = out_dir + "/left_tire_long_slip.gpl";
	postprocess::ChGnuPlot mplot5(plot5.c_str());
	mplot5.SetGrid();
	mplot5.SetLabelX("Time (sec)");
	mplot5.SetLabelY("Longitudinal Slip");
	mplot5.Plot("output.dat", 1, 5, "Left Tire Longitudinal Slip");

	std::string plot6 = out_dir + "/right_tire_long_slip.gpl";
	postprocess::ChGnuPlot mplot6(plot6.c_str());
	mplot6.SetGrid();
	mplot6.SetLabelX("Time (sec)");
	mplot6.SetLabelY("Longitudinal Slip");
	mplot6.Plot("output.dat", 1, 13, "Right Tire Longitudinal Slip");

	std::string plot7 = out_dir + "/left_tire_slip_angle.gpl";
	postprocess::ChGnuPlot mplot7(plot7.c_str());
	mplot7.SetGrid();
	mplot7.SetLabelX("Time (sec)");
	mplot7.SetLabelY("Slip Angle (deg)");
	mplot7.Plot("output.dat", 1, 6, "Left Tire Slip Angle");

	std::string plot8 = out_dir + "/right_tire_slip_angle.gpl";
	postprocess::ChGnuPlot mplot8(plot8.c_str());
	mplot8.SetGrid();
	mplot8.SetLabelX("Time (sec)");
	mplot8.SetLabelY("Slip Angle (deg)");
	mplot8.Plot("output.dat", 1, 114, "Right Tire Slip Angle");

	std::string plot9 = out_dir + "/front_left_drawbar.gpl";
	postprocess::ChGnuPlot mplot9(plot9.c_str());
	mplot9.SetGrid();
	mplot9.SetLabelX("Longitudinal Slip");
	mplot9.SetLabelY("Force (N)");
	mplot9.Plot("output.dat", 5, 7, "Front Left Draw Pull");

	std::string plot10 = out_dir + "/front_right_drawbar.gpl";
	postprocess::ChGnuPlot mplot10(plot10.c_str());
	mplot10.SetGrid();
	mplot10.SetLabelX("Longitudinal Slip");
	mplot10.SetLabelY("Force (N)");
	mplot10.Plot("output.dat", 13, 15, "Front Right Drawbar Pull");

	// Cleanup
	delete terrain;

	return 0;
}
