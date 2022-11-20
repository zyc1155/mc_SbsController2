#include "SbsControllerZyc.h"

SbsControllerZyc::SbsControllerZyc(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config)
    : mc_control::MCController(rm, dt) //, right_falcon(0), left_falcon(0)
{
    config_.load(config);
    solver().addConstraintSet(contactConstraint);
    solver().addConstraintSet(kinematicsConstraint);
    solver().addConstraintSet(selfCollisionConstraint);
    solver().addConstraintSet(*compoundJointConstraint);
    solver().addTask(postureTask);
    solver().setContacts({{}});

    postureTask->stiffness(100.0);
    postureTask->weight(1.0);

    joint_name = {"LCY", "LCR", "LCP", "LKP", "LAP", "LAR", "RCY", "RCR", "RCP", "RKP", "RAP", "RAR"};
    sensornames = {"LeftFootForceSensor", "RightFootForceSensor"};

    std::vector<tasks::qp::JointStiffness> stiffnesses;

    for (int i = 0; i < 12; i++)
    {
        stiffnesses.push_back({joint_name[i], 1.0});
    }
    postureTask->jointStiffness(solver(), stiffnesses);

    Eigen::VectorXd ww(59);
    ww.head(18) = Eigen::MatrixXd::Constant(18, 1, 0.001);
    ww.tail(41) = Eigen::MatrixXd::Constant(41, 1, 1000.0);

    // Eigen::VectorXd ww(59);
    // ww.head(6) = Eigen::MatrixXd::Constant(6, 1, 0.001);
    // ww.tail(53) = Eigen::MatrixXd::Constant(53, 1, 1000.0);

    postureTask->dimWeight(ww);

    comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0, 100.0, 1000.0);
    solver().addTask(comTask);

    addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
    addContact({robot().name(), "ground", "RightFoot", "AllGround"});

    first = true;

    ctrl_mode = 0;
    ctrl_mode2 = 0;

    omega = sqrt(GRAVITY / HEIGHTREF);
    COMShifter_Kp = Eigen::Matrix3d::Zero();
    COMShifter_Kd = Eigen::Matrix3d::Zero();

    for (int i = 0; i < 3; i++)
    {
        COMShifter_Kp(i, i) = 70.0;

        COMShifter_Kd(i, i) = COMShifter_Kp(i, i) / omega + omega;
    }

    A_v_GAd = Eigen::Vector3d::Zero();
    B_v_GBd = Eigen::Vector3d::Zero();

    posRA = Eigen::Vector3d::Zero();
    posRB = Eigen::Vector3d::Zero();
    thetad = Eigen::Matrix<double, NUMJOINTS, 1>::Zero();

    cas_mem = casadi_alloc(h_functions());

    arg_ = theta.data();
    *(cas_mem->arg) = arg_;

    cas_mem->res = new double *[cas_mem->sz_res];

    cas_mem->res[0] = A_R_H.data();
    cas_mem->res[1] = A_R_B.data();
    cas_mem->res[2] = A_p_BA.data();
    cas_mem->res[3] = A_p_GA.data();
    cas_mem->res[4] = A_J_wHA.data();
    cas_mem->res[5] = A_J_wBA.data();
    cas_mem->res[6] = A_J_pBA.data();//fuck_B[0];//A_J_pBA_.data();
    cas_mem->res[7] = A_J_pGA.data();//fuck_G[0];//A_J_pGA_.data();

    createGUI();

    gui()->addElement({"a"},
                      mc_rtc::gui::Point3D("A_p_GA", [this]()
                                           { return A_p_GA; }),
                      mc_rtc::gui::Point3D("COM", [this]()
                                           { 
                                            Eigen::Vector3d W_p_GW;
                                            W_p_GW =  W_p_AW + W_R_A * A_p_GA;
                                            return W_p_GW; }),
                      mc_rtc::gui::Point3D("COM real", [this]()
                                           { return real_COM; }),
                      mc_rtc::gui::Point3D("right foot", [this]()
                                           { 
                                            Eigen::Vector3d W_p_BW;
                                            W_p_BW =  W_p_AW + W_R_A * A_p_BA;
                                            return W_p_BW; }),
                      mc_rtc::gui::Point3D("right foot real", [this]()
                                           { return real_rfoot; }),
                      mc_rtc::gui::Rotation("body", [this]()
                                            { return sva::PTransformd{W_R_A * A_R_H, Eigen::Vector3d::Zero()}; }));

    fp = fopen("/home/zyc/data.csv", "w");

    mc_rtc::log::success("SbsControllerZyc init done ");
}

bool SbsControllerZyc::run()
{
    if (first)
    {
        start_time = std::chrono::high_resolution_clock::now();
    }
    ttime = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_time).count();

    if (rightFootLift_)
    {
        removeContact({robot().name(), "ground", "RightFoot", "AllGround"});
        removeContact({robot().name(), "ground", "LeftFoot", "AllGround"});
        solver().removeTask(comTask);

        std::vector<tasks::qp::JointStiffness> stiffnesses;

        for (int i = 0; i < 12; i++)
        {
            stiffnesses.push_back({joint_name[i], 100.0});
        }
        postureTask->jointStiffness(solver(), stiffnesses);

        Eigen::VectorXd ww(59);
        ww.head(6) = Eigen::MatrixXd::Constant(6, 1, 0.001);
        ww.tail(53) = Eigen::MatrixXd::Constant(53, 1, 1000.0);

        thetad = theta;

        rightFootLift_ = false;
        haha = true;
    }

    get_angles();
    set_CtrlPos();
    state_swiching();
    set_Jacobian();
    set_desiredVel();
    cal_thetad();
    set_angles();
    output_data();

    if (first)
        first = false;

    return mc_control::MCController::run();
}

void SbsControllerZyc::reset(const mc_control::ControllerResetData &reset_data)
{
    mc_control::MCController::reset(reset_data);
}

void SbsControllerZyc::get_angles()
{
    int Joint_Index;
    sva::PTransformd ZMP_frame(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    for (int i = 0; i < 12; i++)
    {
        Joint_Index = robot().jointIndexByName(joint_name[i]);
        theta(i) = realRobot().q()[Joint_Index][0];
    }

    left = realRobot().surfaceWrench("LeftFootCenter");
    right = realRobot().surfaceWrench("RightFootCenter");

    world_wrench = realRobot().netWrench(sensornames);

    if (world_wrench.force()[2] > 1.0)
        W_Q_W = realRobot().zmp(world_wrench, ZMP_frame);

    A_f_A = left.force();
    A_n_A = left.moment();

    B_f_B = right.force();
    B_n_B = right.moment();

    W_R_A = realRobot().surfacePose("LeftFootCenter").rotation();
    W_p_AW = realRobot().surfacePose("LeftFootCenter").translation();

    real_COM = realRobot().com();
    real_rfoot = realRobot().surfacePose("RightFootCenter").translation();
}

void SbsControllerZyc::set_CtrlPos()
{
    // posRB_ = right_falcon.Get_Pos();
    // posRA_ = left_falcon.Get_Pos();

    // posRB << -(posRB_(2) - 0.12), -posRB_(0), posRB_(1);
    // posRA << -(posRA_(2) - 0.12), -posRA_(0), posRA_(1);
    posRB << 0, 0, 0;
    posRA << 0, 0, 0;

    // if(ttime < 5.0)
    //   posRB << .0,.0,.0;
    // else
    //   posRB << .0,.0,0.01;

    // if (rightFootLift_)
    // {
    //     posRB << .0, .0, .01;
    // }

    if (first)
    {
        posRAp = posRA;
        posRBp = posRB;
    }

    vel_posRA = (posRA - posRAp) / timeStep;
    vel_posRB = (posRB - posRBp) / timeStep;

    posRAp = posRA;
    posRBp = posRB;
}

void SbsControllerZyc::state_swiching()
{
    if (ctrl_mode == 1)
    {
        ctrl_mode2 = 0;
        if (fabs(Q_ep(0)) < 0.08 && fabs(Q_ep(1)) < 0.04)
        {
            ctrl_mode = 2;
            timer_mode = 0.0;
            leftFootRatio = 1.0;
        }
    }
    else if (ctrl_mode == 2)
    {
        ctrl_mode2 = 0;
        timer_mode += timeStep;
        if ((timer_mode > 0.1) && B_f_B(2) > 1e-1)
        {
            ctrl_mode = 0;
            leftFootRatio = 0.5;
        }
    }
    else if (ctrl_mode == 5)
    {
        ctrl_mode2 = 1;
        if (fabs(Q_ep(0)) < 0.08 && fabs(Q_ep(1)) < 0.04)
        {
            ctrl_mode = 6;
            timer_mode = 0.0;
            leftFootRatio = 0.0;
        }
    }
    else if (ctrl_mode == 6)
    {
        ctrl_mode2 = 1;
        timer_mode += timeStep;
        if ((timer_mode > 0.1) && A_f_A(2) > 1e-1)
        {
            ctrl_mode = 0;
            leftFootRatio = 0.5;
        }
    }
    else if ((ctrl_mode == 0 && vel_posRB(2) > 0.1 && posRB(2) > 0.0))
    {
        if (ctrl_mode2 == 1)
        {
            A_v_GAd = A_R_B * B_v_GBd;
            A_p_GAp = A_R_B * (B_p_GBp - B_p_ABp);
            A_p_GA_ref = A_R_B * (B_p_GB_ref - B_p_AB);

            cas_mem->res[0] = A_R_H_.data();
            cas_mem->res[1] = A_R_B_.data();
            cas_mem->res[2] = A_p_BA.data();
            cas_mem->res[3] = A_p_GA.data();
            cas_mem->res[4] = A_J_wHA_.data();
            cas_mem->res[5] = A_J_wBA_.data();
            cas_mem->res[6] = A_J_pBA_.data();
            cas_mem->res[7] = A_J_pGA_.data();
        }

        ctrl_mode2 = 0;
        ctrl_mode = 1;
    }
    else if ((ctrl_mode == 0 && vel_posRA(2) > 0.1 && posRA(2) > 0.0))
    {
        if (ctrl_mode2 == 0)
        {
            B_v_GBd = B_R_A * A_v_GAd;
            B_p_GBp = B_R_A * (A_p_GAp - A_p_BAp);
            B_p_GB_ref = B_R_A * (A_p_GA_ref - A_p_BA);
        }

        ctrl_mode2 = 1;
        ctrl_mode = 5;
    }
}

void SbsControllerZyc::set_Jacobian()
{
    casadi_eval(cas_mem);
    if (ctrl_mode2 == 0)
    {
        // A_R_H = A_R_H_;
        // A_R_B = A_R_B_;
        // A_J_wHA = A_J_wHA_;
        // A_J_wBA = A_J_wBA_;
        //A_J_pBA = A_J_pBA_;
        //A_J_pGA = A_J_pGA_;

        // for(int i=0;i<3;i++)
        // {
        //     for(int j=0;j<12;j++)
        //     {
        //         A_J_pBA(i,j)=fuck_B[i][j];
        //         A_J_pGA(i,j)=fuck_G[i][j];
        //     }
        // }

    }
}

void SbsControllerZyc::set_desiredVel()
{
    if (ctrl_mode == 0)
    {
        if (ctrl_mode2 == 0)
        {
            Q_ref = A_p_BA / 2.0;
        }
        else
        {
            Q_ref = B_p_AB / 2.0;
        }
    }
    else
    {
        Q_ref = Eigen::Vector3d::Zero();
    }

    Q_ref(2) += HEIGHTREF;

    Q_ref << 0.0, -0.105, 0.9;

    if (first)
    {
        A_p_GA_ref = A_p_GA;
    }

    if (ctrl_mode2 == 0)
    {

        // A_a_GAd = sat_func(A_LIM, COMShifter_Kp * (Q_ref - A_p_GA_ref) - COMShifter_Kd * A_v_GAd);

        // A_v_GAd += A_a_GAd * timeStep;
        // A_p_GA_ref += A_v_GAd * timeStep;

        A_a_GAd = Eigen::Vector3d::Zero();
        A_v_GAd = sat_func(0.1, 1.0 * (Q_ref - A_p_GA));

        Q_ep = A_p_GA - A_a_GAd / (omega * omega);
        // Q_ep = A_p_GA ;
        Q_ep(2) = Q_ep(2) - HEIGHTREF;
    }
    else if (ctrl_mode2 == 1)
    {
        B_a_GBd = sat_func(A_LIM, COMShifter_Kp * (Q_ref - B_p_GB_ref) - COMShifter_Kd * B_v_GBd);

        B_v_GBd += B_a_GBd * timeStep;
        B_p_GB_ref += B_v_GBd * timeStep;

        Q_ep = B_p_GB - B_a_GBd / (omega * omega);
        Q_ep(2) = Q_ep(2) - HEIGHTREF;
    }

    if ((ctrl_mode == 0 || ctrl_mode == 1 || ctrl_mode == 5))
    {
        A_v_BAd = Eigen::Vector3d::Zero();
        B_v_ABd = Eigen::Vector3d::Zero();
        A_w_Bd = Eigen::Vector3d::Zero();
        B_w_Ad = Eigen::Vector3d::Zero();

        if (ctrl_mode2 == 0)
        {
            A_p_BA_ref(0) = posRB(0) * 3.0;
            A_p_BA_ref(1) = -0.11 + posRB(1) * 2.0;
            A_p_BA_ref(2) = posRB(2) * 1.7;

            A_R_H_ref = Eigen::Matrix3d::Identity();
            A_w_Hd = cal_Dangvel_InR(A_R_H_ref, A_R_H);
        }
        else
        {
            B_p_AB_ref(0) = posRA(0) * 3.0;
            B_p_AB_ref(1) = 0.11 + posRA(1) * 2.0;
            B_p_AB_ref(2) = posRA(2) * 1.7;

            B_R_H_ref == Eigen::Matrix3d::Identity();
            B_w_Hd = cal_Dangvel_InR(B_R_H_ref, B_R_H);
        }
    }
    else if (ctrl_mode2 == 0)
    {
        A_p_BA_ref(0) = posRB(0) * 3.0;
        A_p_BA_ref(1) = -0.21 + posRB(1) * 2.0;
        A_p_BA_ref(2) = posRB(2) * 1.7;

        double PRL = 5.0, DRL = 0.3;
        A_v_BAd = sat_func(0.15, PRL * (A_p_BA_ref - A_p_BA));

        A_w_Hd = Eigen::Vector3d::Zero();

        A_R_B_ref = Eigen::Matrix3d::Identity();
        A_w_Bd = cal_Dangvel_InR(A_R_B_ref, A_R_B);
    }
    else
    {
        B_p_AB_ref(0) = posRA(0) * 3.0;
        B_p_AB_ref(1) = 0.21 + posRA(1) * 2.0;
        B_p_AB_ref(2) = posRA(2) * 1.7;

        double PRL = 10.0, DRL = 0.3;
        B_v_ABd = sat_func(0.5, PRL * (B_p_AB_ref - B_p_AB));

        B_w_Hd = Eigen::Vector3d::Zero();

        B_R_A_ref = Eigen::Matrix3d::Identity();
        B_w_Ad = cal_Dangvel_InR(B_R_A_ref, B_R_A);
    }
}

void SbsControllerZyc::cal_thetad()
{
    Eigen::Matrix<double, 12, NUMJOINTS> JJ;
    Eigen::Matrix<double, 12, 1> vdd;

    JJ << A_J_pGA, A_J_pBA, A_J_wHA, A_J_wBA;
    vdd << A_v_GAd, A_v_BAd, A_w_Hd, A_w_Bd;
    //vdd << A_v_GAd, Eigen::Matrix<double, 9, 1>::Zero();

    theta_dotd = JJ.colPivHouseholderQr().solve(vdd);
    // theta_dotd=Matrix<double, 11, 1>::Zero();

    if (first)
    {
        thetad = theta;
    }

    thetadp = thetad;
    thetad += theta_dotd * timeStep;
}

void SbsControllerZyc::set_angles()
{
    int Joint_Index;
    for (int i = 0; i < 12; i++)
    {
        Joint_Index = robot().jointIndexByName(joint_name[i]);

        if (haha)
            postureTask->target({{joint_name[i], {thetad(i)}}});
        else
            postureTask->target({{joint_name[i], robot().q()[Joint_Index]}});
    }
}

void SbsControllerZyc::output_data()
{

    fprintf(fp, "%.3lf,", ttime);

    fprintf(fp, ",%d,%d,", ctrl_mode, ctrl_mode2);

    for (int i = 0; i < 3; i++)
        fprintf(fp, ",%.6lf", Q_ref(i));
   
    fprintf(fp, ",");
    for (int i = 0; i < 3; i++)
        fprintf(fp, ",%.6lf", A_p_GA(i));

    fprintf(fp, ",");
    for (int i = 0; i < 3; i++)
        fprintf(fp, ",%.6lf", A_p_BA(i));

    fprintf(fp, "\n");
    fflush(fp);
}

Eigen::Vector3d SbsControllerZyc::sat_func(double _lim, const Eigen::Vector3d &val)
{
    double lim = fabs(_lim);
    Eigen::Vector3d result;

    for (int i = 0; i < 3; i++)
    {
        if (val(i) > lim)
            result(i) = lim;
        else if (val(i) < -lim)
            result(i) = -lim;
        else
            result(i) = val(i);
    }

    return result;
}

Eigen::Vector3d SbsControllerZyc::cal_Dangvel_InR(const Eigen::Matrix3d &RR_ref, const Eigen::Matrix3d &RR)
{
    double k3_norm, v3, A3;
    Eigen::Vector3d k3, n3;
    Eigen::Matrix3d RR_c;
    RR_c = RR_ref * RR.transpose();

    k3(0) = (RR_c(2, 1) - RR_c(1, 2)) / 2.0;
    k3(1) = (RR_c(0, 2) - RR_c(2, 0)) / 2.0;
    k3(2) = (RR_c(1, 0) - RR_c(0, 1)) / 2.0;
    k3_norm = k3.norm();

    if (k3_norm < 0.001)
    {
        v3 = 0.0;
    }
    else
    {
        n3 = k3 / k3_norm;
        A3 = atan2(k3_norm, (RR_c(0, 0) + RR_c(1, 1) + RR_c(2, 2) - 1) / 2.0);
        v3 = 0.8 * A3;
        if (v3 > 0.4)
            v3 = 0.4;
    }

    return v3 * n3;
}

void SbsControllerZyc::createGUI()
{
    // auto & gui = *ctl.gui();
    gui()->addElement({"SbsController", "Task"}, mc_rtc::gui::Label("Lift right foot", [this]()
                                                                    { return rightFootLift_; }),
                      mc_rtc::gui::Checkbox(
                          "Activated", [this]()
                          { return rightFootLift_; },
                          [this]()
                          { rightFootLift_ = !rightFootLift_; }));
}

CONTROLLER_CONSTRUCTOR("SbsControllerZyc", SbsControllerZyc)
