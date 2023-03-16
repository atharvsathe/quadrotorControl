#include "QuadrotorLQRControl.hpp"

#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

//#include <eigen3/Eigen/Dense> // ++ Math operations with Matrices ++

/* -------------------- added part ---------------------*/
#include <fstream>
#include <string>
#include <array>
#include <vector>
#include <iostream>
#include <cmath>
#include <memory>
#include <sstream>

using namespace matrix;
using namespace std;

QuadrotorLQRControl::QuadrotorLQRControl()
{

    for (int i=0;i<12;i++)
    {
       _current_state(i,0) = 0.0f;
       _eq_point(i,0) = 0.0f;
    }

    _eq_point(1,0) =  0.0f;
    _eq_point(3,0) =  0.0f;
    _eq_point(5,0) = -1.0f;

    u_control(0,0) = 0.0f;
    u_control(1,0) = 0.0f;
    u_control(2,0) = 0.0f;
    u_control(3,0) = 0.0f;

    _K = readMatrixK("/home/raffaele/PX4/Firmware/src/modules/mc_att_control/lqr_files/new_controller.txt");
    _P = readMatrixP("/home/raffaele/PX4/Firmware/src/modules/mc_att_control/lqr_files/new_pe.txt");

    ff_thrust = 5.886f; // [N]

    _auto_eq_point_flag = true;

    ofstream outfile1;
     outfile1.open("/home/raffaele/PX4/Firmware/src/modules/mc_att_control/output_files/control_input.txt", std::ios::out);
     outfile1.close();

        
     ofstream outfile3;
     outfile3.open("/home/raffaele/PX4/Firmware/src/modules/mc_att_control/output_files/state.txt", std::ios::out);
     outfile3.close();

     
     ofstream outfile5;
     outfile5.open("/home/raffaele/PX4/Firmware/src/modules/mc_att_control/output_files/lyapunov.txt", std::ios::out);
     outfile5.close();

ofstream outfile4;
     outfile4.open("/home/raffaele/PX4/Firmware/src/modules/mc_att_control/output_files/ekf.txt", std::ios::out);
     outfile4.close();

     _past_time = hrt_absolute_time() * 1e-6;

}



Matrix <float, 4, 12>  QuadrotorLQRControl::readMatrixK(const char *filename)
    {

    static Matrix <float, 4, 12> result;
    static int rows = 4;
    static int cols = 12;
    ifstream infile;
    infile.open(filename);
    if (infile.is_open()){
         for (int i=0; i<rows;i++){
    		string line;
    		getline(infile, line);
    		stringstream stream(line);
    		for (int j=0; j<cols; j++){
    			stream >> result(i,j);
    		}

    	}
    	infile.close();
    }else cout << "Unable to open file";
    return result;

 }

Matrix<float,4,1> QuadrotorLQRControl::LQRcontrol()
{
       
     
     //static Matrix<float,4,1> u_control;
     static Matrix<float,4,1> u_control_norm;
     static Matrix<float,12,1> delta_x;
     static Matrix<float, 1,12> v_b;
     static Matrix<float,1,12> delta_x_T;
     static Matrix<float,1,1> _lyap_fun;     
     const hrt_abstime now = hrt_absolute_time();

     float _current_time = now *1e-6;
    // float dt = _current_time-_past_time;
     
    _past_time = _current_time;

    delta_x   = _current_state - _eq_point;    
    u_control = - _K*(delta_x); 
 
    delta_x_T = delta_x.transpose();
    
    v_b = delta_x_T*_P;
    _lyap_fun = v_b*delta_x;
    //cout<< dt << "\t" << _P(0,0) << "\n";
   // !! IMPORTANT scale the control inputs.......


    u_control_norm(1,0) = fmin(fmax((u_control(1,0))/(0.1080f*4.0f), -1.0f), 1.0f);  
    u_control_norm(2,0) = fmin(fmax((u_control(2,0))/(0.1080f*4.0f),  -1.0f), 1.0f);
    u_control_norm(3,0) = fmin(fmax((u_control(3,0))/(0.1f*1.0f), -1.0f), 1.0f);
    u_control_norm(0,0) = fmin(fmax((u_control(0,0)+ff_thrust)/16.0f, 0.0f), 1.0f);

   // not normalized control inputs
     u_control(0,0) = u_control_norm(0,0)*16.0f;
     u_control(1,0) = u_control_norm(1,0)*4.0f;
     u_control(2,0) = u_control_norm(2,0)*4.0f;
     u_control(3,0) = u_control_norm(3,0)*0.05f;
     
    //"\t" <<  u_control(0,0)+ff_thrust << "\n";
         /* Save data*/
    writeStateOnFile("/home/raffaele/PX4/Firmware/src/modules/mc_att_control/output_files/state.txt", _current_state, now);
    writeInputOnFile("/home/raffaele/PX4/Firmware/src/modules/mc_att_control/output_files/control_input.txt", u_control_norm, now); 
    writeLyapunovOnFile("/home/raffaele/PX4/Firmware/src/modules/mc_att_control/output_files/lyapunov.txt", _lyap_fun(0,0), now); 
    writeStateOnFile("/home/raffaele/PX4/Firmware/src/modules/mc_att_control/output_files/ekf.txt", _current_state_ekf, now);
    
    return u_control_norm;    

}

Matrix<float,4,1> QuadrotorLQRControl::normalizationControlInputs(Matrix<float,4,1> _u)
{
   Matrix<float,4,1> _u_norm;
   _u_norm(0,0) = _u(0,0)*16.0f;
   _u_norm(1,0) = _u(1,0)*(0.1080f*4.0f);
   _u_norm(2,0) = _u(2,0)*(0.1080f*4.0f);
   _u_norm(3,0) = _u(3,0)*(0.1f*1.0f); 

return _u_norm;
}

Matrix<float,4,1> QuadrotorLQRControl::getLQRcontrols()
{

return u_control;

}

void QuadrotorLQRControl::setCurrentState(Matrix<float,12,1> _state_estimate)
{

      _current_state = _state_estimate;

}



void QuadrotorLQRControl::setCurrentState(struct vehicle_attitude_s _v_att, struct vehicle_local_position_s  _v_local_pos)
{

      _current_state(0,0) = _v_local_pos.vx;
      _current_state(1,0) = _v_local_pos.x;
      _current_state(2,0) = _v_local_pos.vy;
      _current_state(3,0) = _v_local_pos.y;
      _current_state(4,0) = _v_local_pos.vz;
      _current_state(5,0) = _v_local_pos.z;
    
      _current_state(6,0)  = _v_att.rollspeed;
      _current_state(7,0)  = Eulerf(Quatf(_v_att.q)).phi();
      _current_state(8,0)  = _v_att.pitchspeed;
      _current_state(9,0)  = Eulerf(Quatf(_v_att.q)).theta();
      _current_state(10,0) = _v_att.yawspeed;	
      _current_state(11,0) = Eulerf(Quatf(_v_att.q)).psi();

}

void QuadrotorLQRControl::setCurrentStateEkf(struct vehicle_attitude_s _v_att, struct vehicle_local_position_s  _v_local_pos)
{

      _current_state_ekf(0,0) = _v_local_pos.vx;
      _current_state_ekf(1,0) = _v_local_pos.x;
      _current_state_ekf(2,0) = _v_local_pos.vy;
      _current_state_ekf(3,0) = _v_local_pos.y;
      _current_state_ekf(4,0) = _v_local_pos.vz;
      _current_state_ekf(5,0) = _v_local_pos.z;
    
      _current_state_ekf(6,0)  = _v_att.rollspeed;
      _current_state_ekf(7,0)  = Eulerf(Quatf(_v_att.q)).phi();
      _current_state_ekf(8,0)  = _v_att.pitchspeed;
      _current_state_ekf(9,0)  = Eulerf(Quatf(_v_att.q)).theta();
      _current_state_ekf(10,0) = _v_att.yawspeed;	
      _current_state_ekf(11,0) = Eulerf(Quatf(_v_att.q)).psi();

}


void QuadrotorLQRControl::setAutoEqPoint(struct vehicle_attitude_s _v_att, struct vehicle_local_position_s  _v_local_pos)
{

      _eq_point(0,0) = _v_local_pos.vx;
      _eq_point(1,0) = _v_local_pos.x;
      _eq_point(2,0) = _v_local_pos.vy;
      _eq_point(3,0) = _v_local_pos.y;
      _eq_point(4,0) = _v_local_pos.vz;
      _eq_point(5,0) = _v_local_pos.z;
    
      _eq_point(6,0)  = _v_att.rollspeed;
      _eq_point(7,0)  = Eulerf(Quatf(_v_att.q)).phi();
      _eq_point(8,0)  = _v_att.pitchspeed;
      _eq_point(9,0)  = Eulerf(Quatf(_v_att.q)).theta();
      _eq_point(10,0) = _v_att.yawspeed;	
      _eq_point(11,0) = Eulerf(Quatf(_v_att.q)).psi();


}

void QuadrotorLQRControl::setEquilibriumPoint(Matrix<float,12,1> eqPoint)
{

      _eq_point(0,0) = eqPoint(0,0);
      _eq_point(1,0) = eqPoint(1,0);
      _eq_point(2,0) = eqPoint(2,0);
      _eq_point(3,0) = eqPoint(3,0);
      _eq_point(4,0) = eqPoint(4,0);
      _eq_point(5,0) = eqPoint(5,0);
    
      _eq_point(6,0)  = eqPoint(6,0);
      _eq_point(7,0)  = eqPoint(7,0);
      _eq_point(8,0)  = eqPoint(8,0);
      _eq_point(9,0)  = eqPoint(9,0);
      _eq_point(10,0) = eqPoint(10,0);	
      _eq_point(11,0) = eqPoint(11,0);

    
      

}

void QuadrotorLQRControl::setAutoEqPointFlag(bool flag)
{

   _auto_eq_point_flag = flag;
}

bool QuadrotorLQRControl::getAutoEqPointFlag()
{

   return _auto_eq_point_flag;
}

/* Save data on files */

void QuadrotorLQRControl::writeStateOnFile(const char *filename, Matrix <float, 12, 1> vect, hrt_abstime t) {

	ofstream outfile;
	outfile.open(filename, std::ios::out | std::ios::app);
        
        outfile << t << "\t";   // time
       
	for(int i=0;i<12;i++){
		if(i==11){
			outfile << vect(i,0) << "\n";
		}else{
	         outfile << vect(i,0) << "\t";
		}
	}
	outfile.close();
	return;
}


void QuadrotorLQRControl::writeInputOnFile(const char *filename, Matrix <float, 4, 1> vect, hrt_abstime t) {

	ofstream outfile;
	outfile.open(filename, std::ios::out | std::ios::app);
        
        outfile << t << "\t";   // time
        
	for(int i=0;i<4;i++){
		if(i==3){
			outfile << vect(i,0) << "\n";
		}else{
	         outfile << vect(i,0) << "\t";
		}
	}
	outfile.close();
	return;
}

void QuadrotorLQRControl::writeLyapunovOnFile(const char *filename, float value, hrt_abstime t) {

	ofstream outfile;
	outfile.open(filename, std::ios::out | std::ios::app);
        
        outfile << t << "\t" << value << "\n";   
	outfile.close();
	return;
}

Matrix <float, 12, 12>  QuadrotorLQRControl::readMatrixP(const char *filename)
    {

    static Matrix <float, 12, 12> result;
    static int rows = 12;
    static int cols = 12;
    ifstream infile;
    infile.open(filename);
    if (infile.is_open()){
         for (int i=0; i<rows;i++){
    		string line;
    		getline(infile, line);
    		stringstream stream(line);
    		for (int j=0; j<cols; j++){
    			stream >> result(i,j);
    		}

    	}
    	infile.close();
    }else cout << "Unable to open file";
    return result;

 }
