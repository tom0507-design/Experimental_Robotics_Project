// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/mmp_panda.urdf";

#define JOINT_CONTROLLER      0
#define POSORI_CONTROLLER     1

//int state = JOINT_CONTROLLER;
int state = POSORI_CONTROLLER;

// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
// - write
std::string JOINT_TORQUES_COMMANDED_KEY;

// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

unsigned long long controller_counter = 0;

const bool inertia_regularization = true;

int main() {

	JOINT_ANGLES_KEY = "sai2::cs225a::project::sensors::q";
	JOINT_VELOCITIES_KEY = "sai2::cs225a::project::sensors::dq";
	JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::fgc";

	const std::string OBJ_JOINT_ANGLES_KEY  = "sai2::cs225a::object::cup::sensors::q";
	const std::string OBJ_JOINT_VELOCITIES_KEY = "sai2::cs225a::object::cup::sensors::dq";

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// pose task
	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0,0,0.07);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

#ifdef USING_OTG
	posori_task->_use_interpolation_flag = true;
#else
	posori_task->_use_velocity_saturation_flag = true;
#endif
	
	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_kp_pos = 200.0;
	posori_task->_kv_pos = 20.0;
	posori_task->_kp_ori = 200.0;
	posori_task->_kv_ori = 20.0;

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);

#ifdef USING_OTG
	joint_task->_use_interpolation_flag = true;
#else
	joint_task->_use_velocity_saturation_flag = true;
#endif

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 250.0;
	joint_task->_kv = 15.0;

	VectorXd q_init_desired = initial_q;
	q_init_desired << 0.0, 0.0, 0.0, -30.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
	q_init_desired *= M_PI/180.0;
	joint_task->_desired_position = q_init_desired;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

		// update model
		robot->updateModel();
	
		if(state == JOINT_CONTROLLER)
		{
			// update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques;


			if( (robot->_q - q_init_desired).norm() < 0.15 )
			{
				posori_task->reInitializeTask();
				posori_task->_desired_position += Vector3d(-0.1,0.1,0.1);
				posori_task->_desired_orientation = AngleAxisd(M_PI/6, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;

				joint_task->reInitializeTask();
				joint_task->_kp = 0;

				state = POSORI_CONTROLLER;
			}
		}

		else if(state == POSORI_CONTROLLER)
		{
			// update task model and set hierarchy
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			//Set Desired Position
			Vector3d des_pos_in_world = Vector3d(0.0, -1, 0.6); // To the Box
			//Vector3d des_pos_in_world = Vector3d(-0.3, 0.8, 0.6); // To the cup
			posori_task->_desired_position = des_pos_in_world ;
			//
			VectorXd q_desired = VectorXd::Zero(10);
			q_desired << 0.0, -0.5, 0.0, -30.0*M_PI/180.0, -15.0*M_PI/180.0, -15.0*M_PI/180.0, -105.0*M_PI/180.0, 0.0*M_PI/180.0, 90.0*M_PI/180.0, 45.0*M_PI/180.0;

			MatrixXd Jv = MatrixXd::Zero(3,dof);
			MatrixXd Lambda = MatrixXd::Zero(3,3);
			MatrixXd J_bar = MatrixXd::Zero(dof,3);
			MatrixXd N = MatrixXd::Zero(dof,dof);

			robot->Jv(Jv, control_link, control_point);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->dynConsistentInverseJacobian(J_bar, Jv);
			robot->nullspaceMatrix(N, Jv);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + N.transpose()*(-50*(robot->_q - q_desired) - 10* robot->_dq);

			Vector3d ee_pos ;
			robot->position(ee_pos, control_link, control_point);
			cout << "current XYZ : " << '\n' << ee_pos << endl;

		}

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
