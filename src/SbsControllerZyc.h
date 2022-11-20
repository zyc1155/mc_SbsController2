#pragma once

#include <mc_control/mc_controller.h>
#include <mc_tasks/CoMTask.h>
#include<stdio.h>


#include "api.h"
#include "driver.h"
#include "test.h"

const double DEGTORAD=M_PI/180.0;
const double RADTODEG=180.0/M_PI;
const double HEIGHTREF=0.9;
const double GRAVITY = 9.8;
const double A_LIM=0.5;

const int NUMJOINTS = 12;

struct SbsControllerZyc_DLLAPI SbsControllerZyc : public mc_control::MCController
{
    SbsControllerZyc(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;

    void get_angles();
    void set_CtrlPos();
    void state_swiching();
    void set_Jacobian();
    void set_desiredVel();
    void cal_thetad();
    void set_angles();
    void output_data();

    Eigen::Vector3d sat_func(double lim, const Eigen::Vector3d& val);
    Eigen::Vector3d cal_Dangvel_InR(const Eigen::Matrix3d& RR_ref, const Eigen::Matrix3d& RR);
protected:
    void createGUI();

private:
    std::shared_ptr<mc_tasks::CoMTask> comTask;
    mc_rtc::Configuration config_;

    casadi_mem *cas_mem;
    //Falcon_Driver right_falcon, left_falcon;

    // Set 'true' to lift right foot
    bool rightFootLift_ = false, haha = false;
    FILE * fp;
    bool first;
    int ctrl_mode, ctrl_mode2;
    double timer_mode, leftFootRatio, ttime;
    double omega; 
    double q_range_max[NUMJOINTS], q_range_min[NUMJOINTS];
    std::chrono::_V2::system_clock::time_point start_time;
    std::vector<std::string> joint_name, sensornames;

    double fuck_G[3][12], fuck_B[3][12]; 

    Eigen::Vector3d posRA_, posRB_;
    Eigen::Vector3d posRA, posRB, posRAp, posRBp, vel_posRA, vel_posRB;
    Eigen::Vector3d Q_ref, Q_ep;

    Eigen::Vector3d A_p_QA, B_p_QB, W_Q_W;

    Eigen::Vector3d A_p_BA_ref, B_p_AB_ref, A_p_GA_ref, B_p_GB_ref;
    Eigen::Vector3d A_p_BA, B_p_AB, A_p_GA, B_p_GB, A_p_BAp, B_p_ABp, A_p_GAp, B_p_GBp;
    Eigen::Matrix3d A_R_B_ref, B_R_A_ref, A_R_H_ref, B_R_H_ref;
    Eigen::Matrix3d A_R_B, B_R_A, A_R_H, B_R_H;
    Eigen::Vector3d A_v_GAd, B_v_GBd, A_v_BAd, B_v_ABd;
    Eigen::Vector3d A_v_GA, B_v_GB, A_v_BA, B_v_AB;
    Eigen::Vector3d A_w_Bd, B_w_Ad, A_w_Hd, B_w_Hd;
    Eigen::Vector3d A_w_B, B_w_A, A_w_H, B_w_H;
    Eigen::Vector3d A_a_GAd, B_a_GBd;
    Eigen::Vector3d A_a_GA, B_a_GB;

    const double *arg_;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> A_R_B_, B_R_A_, A_R_H_, B_R_H_;
    Eigen::Matrix<double, 3, NUMJOINTS, Eigen::RowMajor> A_J_pBA_, B_J_pAB_, A_J_pGA_, B_J_pGB_;
    Eigen::Matrix<double, 3, NUMJOINTS, Eigen::RowMajor> A_J_wBA_, B_J_wAB_, A_J_wHA_, B_J_wHB_;

    Eigen::Vector3d W_p_GW;

    Eigen::Matrix3d COMShifter_Kp, COMShifter_Kd;

    Eigen::Matrix<double, NUMJOINTS, 1> theta,thetap,thetad, thetadp, theta_range_max, theta_range_min;
    Eigen::Matrix<double, NUMJOINTS, 1>theta_dot, theta_dotd;

    Eigen::Matrix<double, 3, NUMJOINTS> A_J_pBA, B_J_pAB, A_J_pGA, B_J_pGB;
    Eigen::Matrix<double, 3, NUMJOINTS> A_J_wBA, B_J_wAB, A_J_wHA, B_J_wHB;

    Eigen::Vector3d A_f_A, B_f_B, A_n_A, B_n_B;
    sva::ForceVecd left, right, world_wrench;

    Eigen::Matrix3d W_R_A;
    Eigen::Vector3d W_p_AW, real_COM, real_rfoot;
};