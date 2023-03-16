#include "lqr_att_controller.hpp"

static bool show_height;
static float height_setpoint;
static float x_setpoint;
static float y_setpoint;
static float yaw_setpoint;

static std::uint64_t previous_time, start_time;

/* ModuleBase */
LQRattControl::LQRattControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1),
	_loop_perf(perf_alloc(PC_ELAPSED, "lqr_att_control")) {

		set_firmware_dir();
		_equilibrium_state = get_ekf_state();
		read_K();
		read_A_L_C();
		read_L_C_reduced();
		read_B();

		show_height = false;
		x_setpoint = _equilibrium_state(1, 0);
		y_setpoint = _equilibrium_state(3, 0);
		height_setpoint = _equilibrium_state(5, 0);
		yaw_setpoint = _equilibrium_state(11, 0);
		memset(&_actuators, 0, sizeof(_actuators));

		previous_time = hrt_absolute_time();
		start_time = hrt_absolute_time();
		_x_hat = complementary_filter();
		// _y = get_ekf_state();
		_y = complementary_filter();

		for (int i = 0; i < 12; i++) {
			_current_state(i, 0) = 0;
		}
}

LQRattControl::~LQRattControl() {
	perf_free(_loop_perf);
}

bool LQRattControl::init() {
	ScheduleOnInterval(1000_us);
	return true;
}

int LQRattControl::task_spawn(int argc, char *argv[]) {
	LQRattControl *instance = new LQRattControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int LQRattControl::custom_command(int argc, char *argv[]) {
	static int ret; 
	static bool command_present = false;
	
	if (argc == 1) {
		if (!strncmp(argv[0], "show_data", 9)) {
			command_present = true;
			show_height = true;
		}
	}

	if (argc == 2) {
		if (!strncmp(argv[0], "set_height", 10)) {
			command_present = true;
			static std::string::size_type sz;
			height_setpoint = std::stof (argv[1], &sz); 
			PX4_INFO("Setting height to: %f", double(height_setpoint));
		}
	}

	if (argc == 2) {
		if (!strncmp(argv[0], "set_x", 5)) {
			command_present = true;
			static std::string::size_type sz;
			x_setpoint = std::stof (argv[1], &sz); 
			PX4_INFO("Setting X to: %f", double(x_setpoint));
		}
	}

	if (argc == 2) {
		if (!strncmp(argv[0], "set_y", 5)) {
			command_present = true;
			static std::string::size_type sz;
			y_setpoint = std::stof (argv[1], &sz); 
			PX4_INFO("Setting Y to: %f", double(y_setpoint));
		}
	}

	if (argc == 2) {
		if (!strncmp(argv[0], "set_yaw", 7)) {
			command_present = true;
			static std::string::size_type sz;
			yaw_setpoint = std::stof (argv[1], &sz); 
			PX4_INFO("Setting yaw to: %f", double(yaw_setpoint));
		}
	}

	if (command_present == false) {ret = print_usage("unknown command");}
	else {ret = 0;}

	return ret;
}

int LQRattControl::print_usage(const char *reason) {
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(R"DESCR_STR( LQRattControl)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("lqr_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "|Starts the controller");
	PRINT_MODULE_USAGE_COMMAND_DESCR("set_height <value>", "|Sets height setpoint of the vehicle");
	PRINT_MODULE_USAGE_ARG("Argument:<value>", "|(float) *UP direction is negative", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("show_height", "|Displays cuurent height of the vehicle");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}
int LQRattControl::print_status() {
	PX4_INFO("Running");
	perf_print_counter(_loop_perf);
	return 0;
}

void LQRattControl::Run() {
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	set_equilibrium_state();

	/* Set estimated x_hat_dot with linear or non linear predictor */
	_x_hat_dot = _A * _current_state + _B * (_u_control) +  _L * (_y -  _current_state);
	//_x_hat_dot = calc_nonlinear_dynamics(_current_state, _u_control) + _L * (_y -  _current_state);

	/* Make observer estimate positional velocities -- NOT YET STABLE! */ 
	//_x_hat_dot = _A * _current_state + _B * (_u_control) +  _L_reduced * (_y_reduced -  _C_reduced*_current_state);
	//_x_hat_dot = calc_nonlinear_dynamics(_current_state, _u_control) + _L_reduced * (_y_reduced -  _C_reduced*_current_state);

	/* Estimate current x_hat from x_hat_dot */
	std::uint64_t time_diff = hrt_absolute_time() - previous_time;
	if (time_diff > 0.0) {
		_current_state += (_x_hat_dot) * time_diff * 1e-6 ;
		previous_time = hrt_absolute_time();
	}


	/* Maneuver 1: Travels from origin to (5 ,5 ,5) */
	/*if (hrt_absolute_time() - start_time >= 5 * 1e6) {
		_equilibrium_state(5, 0) = -5;
		_equilibrium_state(1, 0) = 5;
		_equilibrium_state(3, 0) = 5;
		_equilibrium_state(11, 0) = yaw_setpoint;
	}*/
	/* Maneuver 1 end */

	/* Maneuver 2: Travels in a 3d circle */
	/*if (hrt_absolute_time() - start_time >= 5 * 1e6) {
		float r = 3.0f;
		float time = 50.0f;
		float factor = time / 6.28f;

		float theta = ((hrt_absolute_time() - start_time) * 1e-6) - 5.0;
		theta = (theta / factor) ;

		_equilibrium_state(1, 0) = r - (r * cos(theta));
		_equilibrium_state(3, 0) = r * sin(theta);
		_equilibrium_state(5, 0) =  -3 - fmod(theta, 6.28) / 3;
		_equilibrium_state(11, 0) = (fmod(theta, 6.28) * sign(cos(theta))) / 8;
	}*/
	/* Maneuver 2 end */

	/* Compute and publish control variables */
	compute();
	normalize();
	display();
	_actuators.control[0] = _u_control(1, 0);
	_actuators.control[1] = _u_control(2, 0);
	_actuators.control[2] = _u_control(3, 0);
	_actuators.control[3] = _u_control(0, 0);
	publish_acuator_controls();

	/* Update the observed state by EKF or complementary filter */
	//_y = get_ekf_state() + get_noise();
	_y = complementary_filter();
	_y_reduced = get_reduced_state();
	
	perf_end(_loop_perf);
}

int lqr_att_controller_main(int argc, char *argv[]){
	return LQRattControl::main(argc, argv);
}
///////////////////////////////////////////////////////////////////////////

/* Private functions */
/* Pub-Sub functions */
void LQRattControl::publish_acuator_controls() {
	_actuator_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
	_actuators.timestamp = hrt_absolute_time();
	static int response = orb_publish(ORB_ID(actuator_controls_0), _actuator_pub, &_actuators);
	memset(&_actuators, 0, sizeof(_actuators));

	if (response != 0)
		PX4_ERR("Actuator publish unsuccessful");
}
void LQRattControl::poll_vechicle_local_position() {
	_vehicle_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	orb_copy(ORB_ID(vehicle_local_position), _vehicle_pos_sub, &_vehicle_pos);
	orb_unsubscribe(_vehicle_pos_sub);
}

void LQRattControl::poll_vehicle_attitide() {
	_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_vehicle_attitude);
	orb_unsubscribe(_vehicle_attitude_sub);
}

void LQRattControl::poll_vehicle_angular_velocity() {
	_vehicle_angular_velocity_sub = orb_subscribe(ORB_ID(vehicle_angular_velocity));
	orb_copy(ORB_ID(vehicle_angular_velocity), _vehicle_angular_velocity_sub, &_vehicle_angular_velocity);
	orb_unsubscribe(_vehicle_angular_velocity_sub);
}

void LQRattControl::poll_sensor_combined() {
	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);
	orb_unsubscribe(_sensor_combined_sub);
}

void LQRattControl::poll_sensor_mag() {
	_sensor_mag_sub = orb_subscribe(ORB_ID(sensor_mag));
	orb_copy(ORB_ID(sensor_mag), _sensor_mag_sub, &_sensor_mag);
	orb_unsubscribe(_sensor_mag_sub);
}

/* File operation functions */
void LQRattControl::set_firmware_dir() {
	static std::__fs::filesystem::path current_path = std::__fs::filesystem::current_path();
	firmware_dir = std::string(current_path);
	static size_t firmware_pos = firmware_dir.find("Firmware");
	firmware_dir = firmware_dir.substr(0, firmware_pos);
}

void LQRattControl::read_K() {
	const unsigned ROW_SIZE = 4;
	const unsigned COL_SIZE = 12;

	static std::ifstream infile;
	static std::string current_path = firmware_dir + "Firmware/src/modules/lqr_att_controller/model_params/K.txt";
	infile.open(current_path);

	if (infile.is_open() == true) {
		for (unsigned row = 0; row < ROW_SIZE; row++) {
			std::string line;
			getline(infile, line);
			std::stringstream this_stream(line);
			for (unsigned column = 0 ; column < COL_SIZE; column++) {
				this_stream >> _K(row, column);
			}
		}
		infile.close();
	} else 
		PX4_ERR("K.txt not opened");
}

void LQRattControl::read_A_L_C() {
	const unsigned SIZE = 12;

	static std::ifstream infile_A, infile_L, infile_C;
	static std::string current_path_A = firmware_dir + "Firmware/src/modules/lqr_att_controller/model_params/A.txt";
	static std::string current_path_L = firmware_dir + "Firmware/src/modules/lqr_att_controller/model_params/L.txt";
	static std::string current_path_C = firmware_dir + "Firmware/src/modules/lqr_att_controller/model_params/C.txt";
	

	infile_A.open(current_path_A);
	infile_L.open(current_path_L);
	infile_C.open(current_path_C);

	if (infile_A.is_open() == true && infile_L.is_open() == true && infile_C.is_open() == true) {
		for (unsigned row = 0; row < SIZE; row++) {
			std::string line_A, line_L, line_C;
			getline(infile_A, line_A);
			getline(infile_L, line_L);
			getline(infile_C, line_C);

			std::stringstream this_stream_A(line_A);
			std::stringstream this_stream_L(line_L);
			std::stringstream this_stream_C(line_C);

			for (unsigned column = 0 ; column < SIZE; column++) {
				this_stream_A >> _A(row, column);
				this_stream_L >> _L(row, column);
				this_stream_C >> _C(row, column);
			}
		}
		infile_A.close();
		infile_L.close();
		infile_C.close();
	} else 
		PX4_ERR("A.txt or L.txt or C.txt not opened");	
}

void LQRattControl::read_L_C_reduced() {

	static std::ifstream infile_L, infile_C;
	static std::string current_path_L = firmware_dir + "Firmware/src/modules/lqr_att_controller/model_params/L_reduced.txt";
	static std::string current_path_C = firmware_dir + "Firmware/src/modules/lqr_att_controller/model_params/C_reduced.txt";

	infile_L.open(current_path_L);
	infile_C.open(current_path_C);

	if (infile_L.is_open() == true) {
		for (unsigned row = 0; row < 12; row++) {
			std::string line_L;
			getline(infile_L, line_L);

			std::stringstream this_stream_L(line_L);

			for (unsigned column = 0 ; column < 9; column++) {
				this_stream_L >> _L_reduced(row, column);
			}
		}
		infile_L.close();
	} else 
		PX4_ERR("L_reduced.txt not opened");

	if (infile_C.is_open() == true) {
		for (unsigned row = 0; row < 9; row++) {
			std::string line_C;
			getline(infile_C, line_C);

			std::stringstream this_stream_C(line_C);

			for (unsigned column = 0 ; column < 12; column++) {
				this_stream_C >> _C_reduced(row, column);
			}
		}
		infile_L.close();
	} else 
		PX4_ERR("C_reduced.txt not opened");	
}

void LQRattControl::read_B() {
	const unsigned ROW_SIZE = 12;
	const unsigned COL_SIZE = 4;

	static std::ifstream infile;
	static std::string current_path = firmware_dir + "Firmware/src/modules/lqr_att_controller/model_params/B.txt";
	infile.open(current_path);

	if (infile.is_open() == true) {
		for (unsigned row = 0; row < ROW_SIZE; row++) {
			std::string line;
			getline(infile, line);
			std::stringstream this_stream(line);
			for (unsigned column = 0 ; column < COL_SIZE; column++) {

				this_stream >> _B(row, column);
			}
		}
		infile.close();
	} else 
		PX4_ERR("B.txt not opened");	
}

void LQRattControl::write_state(Matrix <float, 12, 1> state, std::string filename) {
	const unsigned SIZE = 12;
	hrt_abstime now = hrt_absolute_time();

	static std::ofstream outfile;
	static std::string current_path = firmware_dir + "Firmware/src/modules/lqr_att_controller/outputs/" + filename + ".txt";
	outfile.open(current_path, std::ios::out | std::ios::app);

	outfile << now << "\t";
	for (unsigned count = 0; count < SIZE; count++) {
		if (count == 11) {
			outfile << state(count, 0) << "\t";
			outfile << calculate_rms(state) << "\n";
		}
		else
			outfile << state(count, 0) << "\t";
	}
	outfile.close();
}

/* State variable computation functions */
Matrix<float, 12, 1> LQRattControl::get_ekf_state() {
	poll_vechicle_local_position();
	poll_vehicle_attitide();
	poll_vehicle_angular_velocity();

	static Matrix<float, 12, 1> state;

	state(0, 0) = _vehicle_pos.vx;
	state(1, 0) = _vehicle_pos.x;
	state(2, 0) = _vehicle_pos.vy;
	state(3, 0) = _vehicle_pos.y;
	state(4, 0) = _vehicle_pos.vz;
	state(5, 0) = _vehicle_pos.z;

	state(6, 0) = _vehicle_angular_velocity.xyz[0];
	state(8, 0) = _vehicle_angular_velocity.xyz[1];
	state(10, 0) = _vehicle_angular_velocity.xyz[2];

	state(7, 0) =  Eulerf(Quatf(_vehicle_attitude.q)).phi();
	state(9, 0) =  Eulerf(Quatf(_vehicle_attitude.q)).theta();
	state(11, 0) =  Eulerf(Quatf(_vehicle_attitude.q)).psi();

	return state;
}

/* State variable computation functions */
Matrix<float, 9, 1> LQRattControl::get_reduced_state() {
	poll_vechicle_local_position();
	poll_vehicle_attitide();
	poll_vehicle_angular_velocity();

	static Matrix<float, 9, 1> state;

	state(1, 0) = _vehicle_pos.x;
	state(2, 0) = _vehicle_pos.y;
	state(3, 0) = _vehicle_pos.z;

	state(4, 0) = _vehicle_angular_velocity.xyz[0];
	state(5, 0) =  Eulerf(Quatf(_vehicle_attitude.q)).phi();
	state(6, 0) = _vehicle_angular_velocity.xyz[1];
	state(7, 0) =  Eulerf(Quatf(_vehicle_attitude.q)).theta();
	state(8, 0) = _vehicle_angular_velocity.xyz[2];
	state(9, 0) =  Eulerf(Quatf(_vehicle_attitude.q)).psi();

	return state;
}

Matrix<float, 12, 1> LQRattControl::complementary_filter() {
	poll_sensor_combined();
	poll_sensor_mag();
	poll_vechicle_local_position();
	poll_vehicle_angular_velocity();

	const float gyro_weight = 0.999f;
	const float accel_weight = 0.001f;
	const float mag_weight = 0.01f;

	static Matrix<float, 12, 1> estimated_state;
	const float us_to_s = 0.000001;
	float time_diff_gyro = _sensor_combined.gyro_integral_dt * us_to_s;

	estimated_state(0, 0) = _vehicle_pos.vx;
	estimated_state(1, 0) = _vehicle_pos.x;
	estimated_state(2, 0) = _vehicle_pos.vy;
	estimated_state(3, 0) = _vehicle_pos.y;
	estimated_state(4, 0) = _vehicle_pos.vz;
	estimated_state(5, 0) = _vehicle_pos.z;
	
	float roll_from_acc =  atan2(_sensor_combined.accelerometer_m_s2[1], sqrt(pow(_sensor_combined.accelerometer_m_s2[0], 2) + pow(_sensor_combined.accelerometer_m_s2[2], 2)));
	float pitch_from_acc = atan2(-_sensor_combined.accelerometer_m_s2[0], sqrt(pow(_sensor_combined.accelerometer_m_s2[1], 2) + pow(_sensor_combined.accelerometer_m_s2[2], 2)));
	
	float mag_length = sqrt(pow(_sensor_mag.x, 2) + pow(_sensor_mag.y, 2) + pow(_sensor_mag.z, 2));
	float mag_x = _sensor_mag.x / mag_length;
	float mag_y = _sensor_mag.y / mag_length;
	float mag_z = _sensor_mag.z / mag_length;

	float gyro_lpf_weight = 0.98f;
	float p_lpf_x = (1.0f-gyro_lpf_weight)*estimated_state(6,0) + gyro_lpf_weight * _sensor_combined.gyro_rad[0];
	float p_lpf_y = (1.0f-gyro_lpf_weight)*estimated_state(8,0) + gyro_lpf_weight * _sensor_combined.gyro_rad[1];
	float p_lpf_z = (1.0f-gyro_lpf_weight)*estimated_state(10,0) + gyro_lpf_weight * _sensor_combined.gyro_rad[2];

	estimated_state(6, 0) = p_lpf_x;
	estimated_state(7, 0) = gyro_weight * (estimated_state(7, 0) + estimated_state(6, 0) * time_diff_gyro) + accel_weight * roll_from_acc; 
	estimated_state(6, 0) = p_lpf_y;
	estimated_state(9, 0) = gyro_weight * (estimated_state(9, 0) + estimated_state(8, 0) * time_diff_gyro) + accel_weight * pitch_from_acc;

	float numerator = sin(estimated_state(7, 0))*mag_z - cos(estimated_state(7, 0))*mag_y;
	float denominator = cos(estimated_state(9, 0))*mag_x + sin(estimated_state(7, 0))*sin(estimated_state(9, 0))*mag_y + cos(estimated_state(9, 0))*sin(estimated_state(7, 0))*mag_z;
	float yaw_from_mag = atan2(numerator, denominator);

	estimated_state(6, 0) = p_lpf_z;
	estimated_state(11, 0) = (1.0f-mag_weight) * (estimated_state(11, 0) + estimated_state(10, 0) * time_diff_gyro) + mag_weight * yaw_from_mag;
	
	return estimated_state;
}

Matrix<float, 12, 1> LQRattControl::get_noise() {
	Matrix<float, 12, 1> noise;

	noise(0, 0) = linear_vel(generator);
	noise(2, 0) = linear_vel(generator);
	noise(4, 0) = linear_vel(generator);

	noise(1, 0) = linear_pos(generator);
	noise(3, 0) = linear_pos(generator);
	noise(5, 0) = linear_pos(generator);

	noise(6, 0) = angular_vel(generator);
	noise(8, 0) = angular_vel(generator);
	noise(10, 0) = angular_vel(generator);

	noise(7, 0) = angular_pos(generator);
	noise(9, 0) = angular_pos(generator);
	noise(11, 0) = angular_pos(generator);

	return noise;
}

void LQRattControl::set_equilibrium_state() {
	if (height_setpoint <= -10) {
		_equilibrium_state(5, 0) = -5;
		_equilibrium_state(1, 0) = 4;
		_equilibrium_state(3, 0) = 4;
		_equilibrium_state(11, 0) = yaw_setpoint;
	} else {
		_equilibrium_state(5, 0) = height_setpoint;
		_equilibrium_state(1, 0) = x_setpoint;
		_equilibrium_state(3, 0) = y_setpoint;
		_equilibrium_state(11, 0) = yaw_setpoint;	
	}
}

float LQRattControl::calculate_rms(Matrix<float,12, 1> vector) {
	float rms_value = 0;
	const unsigned SIZE = 12;

	for (unsigned i = 0; i < SIZE; i++) {
		rms_value += (vector(i, 0) * vector(i, 0));
	}
	rms_value = sqrt(rms_value);
	return rms_value;
}

Matrix<float, 12, 1> LQRattControl::calc_nonlinear_dynamics(Matrix<float, 12, 1> x, Matrix<float, 4, 1> u) {
	static Matrix<float, 12, 1> dx_dt;
	float m = 0.6;
    float Jx = 0.0092;
    float Jy = 0.0091;
    float Jz = 0.0101;
    float g = 9.81;

    float px_dot = x(0,0);
    float py_dot = x(0,2);
    float pz_dot = x(0,4);
    float p =      x(0,6);
    float phi =    x(0,7);
    float q =      x(0,8); 
    float theta =  x(0,9);
    float r =      x(0,10);

    float F = u(0,0);
    float tau_phi = u(1,0);
    float tau_theta = u(2,0);
    float tau_psi = u(3,0);
    float az = -F/m;

    dx_dt(0,0) = cos(phi)*sin(theta)*az;
    dx_dt(1,0) = px_dot;
    dx_dt(2,0) = -sin(phi)*az;
    dx_dt(3,0) = py_dot;
    dx_dt(4,0) = g + cos(phi)*cos(theta)*az;
    dx_dt(5,0) = pz_dot;  
    dx_dt(6,0) = 1/Jx*tau_phi;
    dx_dt(7,0) = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    dx_dt(8,0) = 1/Jy*tau_theta;
    dx_dt(9,0) = q*cos(phi) - r*sin(phi);
    dx_dt(10,0) = 1/Jz*tau_psi;
    dx_dt(11,0) = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);

	return dx_dt;
}

/* Control input computation functions */
void LQRattControl::compute() {
	static Matrix<float, 12, 1> delta_x;

	delta_x = _equilibrium_state - _current_state ;
	_u_control = _K * delta_x;
}

void LQRattControl::normalize() {
    _u_control(1,0) = fmin(fmax((_u_control(1,0)), -1.0f), 1.0f);  
    _u_control(2,0) = fmin(fmax((_u_control(2,0)),  -1.0f), 1.0f);
    _u_control(3,0) = fmin(fmax((_u_control(3,0)), -1.0f), 1.0f);
    _u_control(0,0) = fmin(fmax((_u_control(0,0)), 0.0f), 1.0f) + 0.20f;
    /*_u_control(1, 0) = fmin(fmax((_u_control(1, 0)) / (0.1080f * 4.0f), -1.0f), 1.0f) * 0.4f;
    _u_control(2, 0) = fmin(fmax((_u_control(2, 0)) / (0.1080f * 4.0f), -1.0f), 1.0f) * 0.4f;
    _u_control(3, 0) = fmin(fmax((_u_control(3, 0)) / (0.1f * 4.0f), -1.0f), 1.0f) * 0.4f;
    _u_control(0, 0) = fmin(fmax((_u_control(0, 0) + 16.0f) / (16.0f), 0.0f), 1.0f);*/
}

void LQRattControl::display() {
	if (show_height == true) {

		poll_vechicle_local_position();
		poll_vehicle_attitide();
		poll_vehicle_angular_velocity();

		PX4_INFO("height: %f x: %f y: %f yaw: %f", double(_vehicle_pos.z), double(_vehicle_pos.x), double(_vehicle_pos.y), double(Eulerf(Quatf(_vehicle_attitude.q)).psi()));
		show_height = false;
	}

}













