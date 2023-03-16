/* PX4 simulation interface libraries */
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

/* PX4 pub-sub libraries */
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

/* PX4 message interface libraries */
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_mag.h>

/* PX4 time interface libraries */
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

/* C++ utility libraries */
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>
#include <vector>
#include <numeric>
#include <random>

using namespace matrix;
using namespace time_literals;

extern "C" __EXPORT int lqr_att_controller_main(int argc, char *argv[]);

class LQRattControl : public ModuleBase<LQRattControl>, public ModuleParams, public px4::ScheduledWorkItem {

public:
	LQRattControl();
	virtual ~LQRattControl();

	bool init();

	/* Module Base */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;
	void Run() override;

private:
	/* Pub-Sub varibales*/
	orb_advert_t 	_actuator_pub{nullptr};
	int _vehicle_pos_sub{-1};
	int _vehicle_attitude_sub{-1};
	int _vehicle_angular_velocity_sub{-1};
	int _sensor_combined_sub{-1};
	int _sensor_mag_sub{-1};

	struct actuator_controls_s			_actuators {};
	struct vehicle_local_position_s		_vehicle_pos{};
	struct vehicle_attitude_s			_vehicle_attitude{};
	struct vehicle_angular_velocity_s	_vehicle_angular_velocity{};
	struct sensor_combined_s            _sensor_combined {};
	struct sensor_mag_s                 _sensor_mag {};

	/* System variables */
	Matrix<float, 12, 1> _equilibrium_state;
	Matrix<float, 12, 1> _current_state;
	Matrix<float, 4, 1> _u_control;
	Matrix<float, 12, 1> _x_hat;
	Matrix<float, 12, 1> _x_hat_dot;
	Matrix<float, 12, 1> _y;
	Matrix<float, 9, 1> _y_reduced; // y measurements if we do not use positional velocity 
	
	/* System constants */
	Matrix<float, 12, 12> _A;
	Matrix<float, 12, 4> _B;
	Matrix<float, 12, 12> _C;
	Matrix<float, 12, 12> _L;
	Matrix<float, 4, 12> _K;
	Matrix<float, 9, 12> _C_reduced; // C for if we do not use the velocity measurements
	Matrix<float, 12, 9> _L_reduced; // L for if we do not use the velocity measurements
	
	
	/* Noise injection parameters */
	std::default_random_engine generator;
  	std::normal_distribution<float> linear_vel{0.0, 0.1}; //{0.0, 0.1};//{0.0, 0.3};//{0.0f, 0.8};
  	std::normal_distribution<float> linear_pos{0.0, 0.2}; //{0.0, 0.2};//{0.0, 0.6};//{0.0f, 01.6};
  	std::normal_distribution<float> angular_vel{0.0, 0.01}; //{0.0, 0.01};//{0.0, 0.03};//{0.0f, 0.08};
  	std::normal_distribution<float> angular_pos{0.0, 0.02}; //{0.0, 0.02};//{0.0, 0.06};//0.0f, 0.16};

	/* Helper variables */
	std::string firmware_dir;
	perf_counter_t	_loop_perf;

	/* Pub-Sub functions */
	void publish_acuator_controls();
	void poll_vechicle_local_position();
	void poll_vehicle_attitide();
	void poll_vehicle_angular_velocity();
	void poll_sensor_combined();
	void poll_sensor_mag();

	/* File operation functions */
	void set_firmware_dir();
	void read_K();
	void read_A_L_C();
	void read_L_C_reduced();
	void read_B();
	void write_state(Matrix <float, 12, 1> state, std::string filename);

	
	/* State variable computation functions */
	Matrix<float, 12, 1> get_ekf_state();
	Matrix<float, 12, 1> complementary_filter();
	Matrix<float, 12, 1> get_noise();
	Matrix<float, 9, 1> get_reduced_state();
	void set_equilibrium_state();
	float calculate_rms(Matrix<float,12, 1> vector);
	Matrix<float, 12, 1> calc_nonlinear_dynamics(Matrix<float, 12, 1> x, Matrix<float, 4, 1> u);

	/* Control input computation functions */
	void compute();
	void normalize();
	void display();
};

