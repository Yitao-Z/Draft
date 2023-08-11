#include <vector>
#include <time.h>

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <gazebo_msgs/LinkStates.h>
#include <qpOASES.hpp>
#include <cstdlib>

#include "c_load.h"
#include "z_planner.h"
#include "planner_z.h"
#include "planner_xy.h"
#include "matplotlibcpp.h"


namespace plt = matplotlibcpp;



// Flags setting
int block_detect = 0;           // The flag for the block detection, set 0 if there is no block
int param_recieve = 0;          // The flag for the rosparam input,  set 0 if we do not apply rosparam
double detect_distance=1;       // The distance where we detect the position of the block
int sample_time = 400;          // The number of data we saved in csv files for debugging

double PI = 3.14;
// Parameters setting of SLIP model
double g = 9.8;             
double Zc = 0.7;            //CoM height

double Ts = 0.025;          //sampling time in Horizontal MPC
double Ts_z = 0.025;        //sampling time in Vertical MPC

double k = 1208.52;             //The stiffness of the spring 
double m = 15.0;                //The mass of the robot
double w_z = 8.975971429;       //The parameter of Vertical dynamics, w_z=sqrt(k/m);



// Parameters setting of requirement and noise
double Upward_velocity = 0;     // pre-set upward velocity 

double forward_velocity = 0.4;    // pre-set forward velocity 
double lateral_velocity = 0.0;     // pre-set lateral velocity 

double forward_velocity_limit = 0.40;    // Max forward velocity 
double lateral_velocity_limit = 0.25;    // Max lateral velocity 

double slope_angle = 0/180.0*3.14159;    // The angle of the slope, set 0 if the robot is on hte ground
double step_time = 0.7;                  // The time cost in each footstep
double step_length = forward_velocity*step_time;     // The length of each footstep
double inter_feet_clearance = 0.28;                  // The width of two feet

double noise = 0.0;                        // The percentage of sensory noise


//Parameters of MPC planner
int N_steps = 4;                        // number of walking steps in the prediction horizon in the horizontal dirction
int N_steps_z = 1;                      // number of walking steps in the prediction horizon in the vertical dirction
int count_plt = 0;
MatrixXd Qx(2,2);                       // weighting matrix on the CoM state in x direction
MatrixXd Qy(2,2);                       // weighting matrix on the CoM state in y direction
MatrixXd Qz(2,2);                       // weighting matrix on the CoM state in z direction

MatrixXd Wx = 1000*MatrixXd::Identity(1,1);     // weighting matrix distance between different steps in x direction 
MatrixXd Wy = 1600*MatrixXd::Identity(1,1);     // weighting matrix distance between different steps in y direction 
MatrixXd Wz = 10*MatrixXd::Identity(1,1);       // weighting matrix distance between different steps in z direction 

MatrixXd Rx = 300*MatrixXd::Identity(1,1);     // weighting matix on the ZMP tracking
MatrixXd Ry = 350*MatrixXd::Identity(1,1);     // weighting matix on the ZMP tracking
MatrixXd Rz = 75*MatrixXd::Identity(1,1);      // weighting matix on the ZMP tracking

// Parameters subscribed from the slider_controller node #############################
MatrixXd x_hat(2,1); // estimated current CoM position and velocity in the x direction with noise
MatrixXd y_hat(2,1); // estimated current CoM position and velocity in the y direction with noise
MatrixXd z_hat(2,1); // estimated current CoM position and velocity in the z direction with noise

MatrixXd block_pos1(3,1); // estimated 1st block position 
MatrixXd block_pos2(3,1); // estimated 2ed block position  


double remaining_time = step_time;              // remaining time for the current step
double p0_x = 0.0;                              // current foot position in x direction
double p0_y =inter_feet_clearance/2;            // current foot position in y direction
double support_foot = -1;                       // right 1, left -1
const double LOOP_RATE = 500;                   // frequency of the communication


MatrixXd x_ori(2,1);    // estimated current CoM position and velocity in the x direction without noise
MatrixXd y_ori(2,1);    // estimated current CoM position and velocity in the y direction without noise
MatrixXd z_ori(2,1);    // estimated current CoM position and velocity in the z direction without noise





/***********************************************
* @brief Save the data as csv files for further debugging
************************************************/
void save_csv(const char* filenames,double arry[][14], int rows, int cols)
{
    cout<<"I am a test program"<<endl;
    ofstream outFile;
    outFile.open(filenames,ios::out);
    for(int i =0;i<rows;i++)
    {
        for(int j=0;j<cols;j++)
            outFile<<arry[i][j]<<',';
        outFile<<endl;
    }
    outFile.close();
}


/***********************************************
* @brief Obtaining the states from the Gazebo 
************************************************/
void callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    x_hat << msg->data[0],msg->data[1];
    y_hat << msg->data[2],msg->data[3];
    z_hat << msg->data[4],msg->data[5];

    x_ori = x_hat;
    y_ori = y_hat;
    z_ori = z_hat;

    //adding noise
    x_hat(1,0) = x_hat(1,0)+noise*((rand()%100)/100.0*2-1)*forward_velocity_limit;

    x_hat(0,0) = x_hat(0,0)+noise*((rand()%100)/100.0*2-1)*0.05;



    y_hat(1,0) = y_hat(1,0)+noise*((rand()%100)/100.0*2-1)*lateral_velocity_limit;
    y_hat(0,0) = y_hat(0,0)+noise*((rand()%100)/100.0*2-1)*0.05;


    z_hat(1,0) = z_hat(1,0)+noise*((rand()%100)/100.0*2-1)*0.05;
    z_hat(0,0) = z_hat(0,0)+noise*((rand()%100)/100.0*2-1)*0.05;

    remaining_time = msg->data[6];
    p0_x = msg->data[7]+noise*((rand()%100)/100.0*2-1)*0.05;
    p0_y = msg->data[8]+noise*((rand()%100)/100.0*2-1)*0.05;
    support_foot = msg->data[9];
}


/***********************************************
* @brief Obtaining the position of the block 
************************************************/
void linkCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
    block_pos1 << msg->pose[12].position.x, msg->pose[12].position.y, msg->pose[12].position.z;      
}


/***********************************************
* @brief Function to judge if robot meet the block 
************************************************/
int judge_block(double* x, double* y, int N)
{
    for(int i = 0;i<N;i++)
        if((pow(x[i]-block_pos1(0,0),2)+pow(y[i]-block_pos1(1,0),2)<=detect_distance*detect_distance)&&(pow(y[i]-block_pos1(1,0),2)<=0.1))
            return 1;
    return 0;
}


/***********************************************
* @brief Velocity planner to avoid the block
************************************************/
 void block_vel_modify(double *forward_velocity_tmp,double *lateral_velocity_tmp)
 {
    // slide window for the velocities from 1 to N-1
    double future_x=0;
    double future_y=0;
    for(int i =0;i<N_steps;i++)
    {
        forward_velocity_tmp[i]=forward_velocity_tmp[i+1];
        lateral_velocity_tmp[i]=lateral_velocity_tmp[i+1];
        future_x=future_x+step_time*forward_velocity_tmp[i];
        future_y=future_y+step_time*lateral_velocity_tmp[i];
    }

    // decide the velocity in N
    double dis_x= block_pos1(0,0)-(p0_x + future_x);
    double dis_y= block_pos1(1,0)-(p0_y + future_y-(support_foot<0?1:0)*inter_feet_clearance);
    double offset_x=(sqrt(pow(dis_x,2)+pow(dis_y,2))-(detect_distance-0.3))*10;
    forward_velocity_tmp[N_steps] = forward_velocity*((exp(offset_x)-1)/(exp(offset_x)+1));
    lateral_velocity_tmp[N_steps] = lateral_velocity_limit;
 }



/***********************************************
* @brief Velocity change when we are applying rosparam
************************************************/
void param_vel_modify(double *forward_velocity_tmp,double *lateral_velocity_tmp,double tmp_fv,double tmp_lv)
{
    double future_x=0;
    double future_y=0;
    for(int i =0;i<N_steps+1;i++)
    {
        forward_velocity_tmp[i]=tmp_fv;
        lateral_velocity_tmp[i]=tmp_lv;
    }
}

/***********************************************
* @brief normal velocity change 
************************************************/
 void normal_vel_modify(double *forward_velocity_tmp,double *lateral_velocity_tmp)
 {
    
    double future_x=0;
    double future_y=0;
    for(int i =0;i<N_steps;i++)
    {
        forward_velocity_tmp[i]=forward_velocity_tmp[i+1];
        lateral_velocity_tmp[i]=lateral_velocity_tmp[i+1];
    }

    forward_velocity_tmp[N_steps] = forward_velocity;
    lateral_velocity_tmp[N_steps] = lateral_velocity;
 }


/***********************************************
* @brief To initalize the parameter 
************************************************/
void main_parameter_initial()
{

    Qx << 1e-4,0,0,12;
    Qy << 1e-4,0,0,12;
    Qz << 1e-4,0,0,6;

    block_pos1 = -1000*MatrixXd::Ones(3,1);

    x_hat << 0,0;
    y_hat << 0,0;  
    z_hat << Zc,0;
}  








/***********************************************
* @brief The main control loop
************************************************/
int main(int argc, char ** argv)
{
    unsigned seed;
    srand((unsigned)time( NULL ));
    clock_t start_time,end_time;
    double running_time=0;    //ms


    // the setting of the ros nodes 
    ros::init(argc, argv, "ros_SLIDER_planner_node");
    ros::NodeHandle step_planner_node;

    // output of the MPC motion planner 
    ros::Publisher pub = step_planner_node.advertise<std_msgs::Float64MultiArray>("/slider_gazebo/zmp_foothold",1);

    // input of the MPC
    ros::Subscriber sub = step_planner_node.subscribe<std_msgs::Float64MultiArray>("/slider_gazebo/planner_input", 1, callback);
    
    // To get the position of the block
    ros::Subscriber sub_link = step_planner_node.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1, linkCallback);
    

    ros::Rate Loop_rate(LOOP_RATE);
    std_msgs::Float64MultiArray opt;
    main_parameter_initial();

    // MPC variables
    int current_steps = 0;
    int support_foot_pre = support_foot;
    int Ns,Nr,N,Ns_z,Nr_z,N_z;
    
    // flag of finding blocks
    int flag_block=0;

    // pos,vel and acce of CoM 
    double current_z_pva[3];
    double p0_z;

    // the data that we want to save
    double save_data[sample_time][14];

    // pre-set velocity series for special motion mission
    double vel_list[25];

    // Count for the special motion mission
    int count_vel=-1;


    // forward and lateral velocity applied in MPC
    double forward_velocity_tmp[5];
    double lateral_velocity_tmp[5];

    

    // ellipse motion test
    for(int i=0;i<25;i++)
    {
        if(i<5)
            vel_list[i]=3.14159;
        else
            vel_list[i]=3.14159-2*3.14159*(i-5)/19;
    }



    // initialize the velocity applied in MPC
    double fv_t = min(forward_velocity_limit,0.2+ros::Time::now().toSec()*0.02);
    for(int i=0;i<5;i++)
    {
        // forward_velocity_tmp[i]=forward_velocity;
        forward_velocity_tmp[i]=fv_t;
        lateral_velocity_tmp[i]=lateral_velocity;
    }

    // initialize the prediction horizon length
    Ns = (int)(step_time/Ts);
    N = N_steps*Ns;

    // initialize the state space representation of horizontal dynamics
    MatrixXd A(2,2),B(2,1);
    MatrixXd Gamma,Phi,Gamma_z,Phi_z;

    // initialize the robot parameter 
    Robot robot(g, k, m, Ts_z, Upward_velocity, w_z);

    // initialize the state space representation of vertical dynamics 
    robot.gen_offline_matrices();
    genLIPM(g,Zc,Ts,&A,&B);
    
    Current_state current_state(z_hat,0,remaining_time);

    // Arrays for the optimized variables
    double Uopt_x[(1+N_steps)*Ns+N_steps];
    double Uopt_y[(1+N_steps)*Ns+N_steps];
    double Uopt_z[(1+N_steps)*Ns+N_steps];


    // initialize the SLIP model in the horizontal direction
    SLIP Z(Zc,step_time);
    Z.update_state(z_hat(0,0), 0.0, z_hat(0,0)+0.12, z_hat(0,0)+0.12+step_length*tan(slope_angle)/2);
    double z_currentStep = z_hat(0,0);
    double z_nextStep = z_hat(0,0);

    start_time = clock();
    

    // parameters to ensure the correctness of control loop
    int pre_Nr=0;
    int pre_Nr_z=0;
    int pre_N=0;

    // control loop
    while (ros::ok())
	{
        // update the current step length
        step_length = forward_velocity_tmp[0]*step_time;

        // update the LIP model in horizontal direction
        genLIPM(g,Zc,Ts,&A,&B);  

        
        Ns = (int)(step_time/Ts);       //update the number of samples per step
        Nr = (int)(remaining_time/Ts);  //update the number of remaining samples for the current step
        N = Nr+N_steps*Ns;              //update the horizon length  


        Ns_z = (int)(step_time/robot.Ts);       //update the number of samples per step
        Nr_z = (int)(remaining_time/robot.Ts);    //update the number of remaining samples for the current step
        N_z = Nr_z+N_steps_z*Ns_z;                  //update the horizon length
        



        // obtain the prediction matrices in the horizontal direction
        Phi=MatrixXd::Zero(N*2,2);
        Gamma=MatrixXd::Zero(N*2,N);
        genPredictionMatrices(A,B,N,&Gamma,&Phi);



        // update the supporting foot
        if(support_foot_pre != support_foot)
        {

            //detect the block 
            if(block_detect)
                flag_block = judge_block(Uopt_x,Uopt_y,N);

            //update the counter for the velocity series 
            count_vel++;
            if (count_vel>=25)
                count_vel=0;


            // apply this when we want to avoid the block with the velocity planner 
            // if(flag_block)
            //     block_vel_modify(forward_velocity_tmp,lateral_velocity_tmp);
            // else
            // normal_vel_modify(forward_velocity_tmp,lateral_velocity_tmp);

            // update the velocity if we are applying the rosparam
            double tmp_fv,tmp_lv;
            if(param_recieve&&step_planner_node.getParam("/forward_speed",tmp_fv)&&step_planner_node.getParam("/lateral_speed",tmp_lv))
                param_vel_modify(forward_velocity_tmp,lateral_velocity_tmp,tmp_fv,tmp_lv);
            else
                normal_vel_modify(forward_velocity_tmp,lateral_velocity_tmp);
            //forward_velocity_tmp[N_steps]=cos(count_vel/25.0*2*PI)*forward_velocity;
            //lateral_velocity_tmp[N_steps]=sin(count_vel/25.0*2*PI)*lateral_velocity_limit;

            current_steps++;
            support_foot_pre = support_foot;

            // update the states in z direction
            Z.update_state(z_hat(0,0), 0.0, z_hat(0,0)+0.12, z_hat(0,0)+0.12+step_length*tan(slope_angle)/2);
            z_currentStep = z_hat(0,0);
            z_nextStep = z_hat(0,0)+step_length*tan(slope_angle);
        }
        

        // get the states in z direction
        Z.get_com_state(step_time - remaining_time,current_z_pva);
        p0_z = z_hat(0,0)+0.12-9.8/pow(robot.w,2);   

        current_state.z_hat = z_hat;
        current_state.p0_z = p0_z;
        current_state.remaining_time = remaining_time;


        // update the parameter in MPC in z direction
        MPCparameter mpc(N_steps_z,Qz,Rz,robot,current_state,step_time);
        N_z = mpc.Nr+N_steps_z*Ns_z;
        
        
        Phi_z = MatrixXd::Zero(N_z*2,2);
        Gamma_z = MatrixXd::Zero(N_z*2,N_z);
        genPredictionMatrices(robot.A,robot.B,N_z,&Gamma_z,&Phi_z);
        mpc.genPredictionMatrices(Gamma_z,Phi_z);
    
        
        cout<<"*********************************************"<<endl;
        cout<<"********************Q*********************"<<endl;
        
        

        // initalize the QP solver variables
        SQProblem* qp_z= new SQProblem(mpc.N+mpc.N_steps+1,mpc.N+mpc.N_steps+1);
        if(current_state.remaining_time<1e-6)
        {
            delete qp_z;
            SQProblem* qp_z= new SQProblem(mpc.N+mpc.N_steps,mpc.N+mpc.N_steps);
        } 
        SQProblem* qp_x= new SQProblem(N+N_steps,N+N_steps);
        SQProblem* qp_y= new SQProblem(N+N_steps,N+N_steps);

        if(pre_Nr_z!=mpc.Nr)
        {

            start_time = clock();
            // MPC motion planner for z diretion
            mpcPlanner_z(robot,current_state,mpc,Gamma_z,Phi_z,qp_z,Uopt_z);
            cout<<"***********mpcPlanner_z, finish!**************************"<<endl;
            end_time = clock();
            running_time = double(end_time-start_time)/CLOCKS_PER_SEC*1000; 

            cout<<"the time is:"<<running_time<<"ms"<<endl;
            pre_Nr_z = mpc.Nr;
        }
        
        if(pre_Nr!=Nr)
        { 
            
            if (count_vel<0)
                count_vel=0;
            
            // modifying this when we want to apply the velocity series
            // for(int i=0;i<N_steps+1;i++)
            // {
            //     int index = count_vel+i>=25?count_vel+i-25:count_vel+i;
            //     forward_velocity_tmp[i]=-forward_velocity *cos(vel_list[index]) ;
            //     lateral_velocity_tmp[i]=lateral_velocity *sin(vel_list[index]) ;
            // }
           




            cout<<"Now the forward speed is:"<<forward_velocity_tmp[0]<<endl;
            cout<<"Now the lateral speed is:"<<lateral_velocity_tmp[0]<<endl;


            start_time = clock();

             // MPC motion planner for xy diretion
            mpcPlanner(0,x_hat,p0_x,p0_y,support_foot,Ns,Nr,N_steps,N,inter_feet_clearance,forward_velocity_tmp,lateral_velocity_tmp,Qx,Rx,Wx,step_length,Gamma,Phi,flag_block,block_pos1,qp_x,Uopt_x);        
            mpcPlanner(1,y_hat,p0_x,p0_y,support_foot,Ns,Nr,N_steps,N,inter_feet_clearance,forward_velocity_tmp,lateral_velocity_tmp,Qy,Ry,Wy,step_length,Gamma,Phi,flag_block,block_pos1,qp_y,Uopt_y);
                

            end_time = clock();
            running_time = double(end_time-start_time)/CLOCKS_PER_SEC*1000;
            start_time = clock();
            cout<<"***********mpcPlanner_xy, finish!**************************"<<endl;
            cout<<"the time is:"<<running_time<<"ms"<<endl;


           
       
            pre_Nr = Nr;


            // save the data
            if(count_plt<sample_time)
            {
                // save_data[count_plt][0]=Uopt_x[0];
                // save_data[count_plt][1]=Uopt_y[0];
                // save_data[count_plt][2]=Uopt_z[0];

                save_data[count_plt][0]=x_hat(0,0);
                save_data[count_plt][1]=y_hat(0,0);
                save_data[count_plt][2]=x_hat(1,0);
                save_data[count_plt][3]=y_hat(1,0);
                save_data[count_plt][4]=Uopt_x[0];
                save_data[count_plt][5]=Uopt_y[0];
                //save_data[count_plt][6]=Uopt_x[N];
                //save_data[count_plt][7]=Uopt_y[N];
                save_data[count_plt][6]=p0_x;
                save_data[count_plt][7]=p0_y;


                save_data[count_plt][8]=x_ori(0,0);
                save_data[count_plt][9]=y_ori(0,0);
                save_data[count_plt][10]=z_ori(0,0);
                save_data[count_plt][11]=x_ori(1,0);
                save_data[count_plt][12]=y_ori(1,0);
                save_data[count_plt][13]=z_ori(1,0);

            }
            else if(count_plt==sample_time)
            {
                // change the saved file name
                //cout<<save_time<<endl;
                //save_csv("/home/rui/Documents/aSLIP_SLIDER_C/src/slider_controller/src/step_planner_handC/section3/normalFandL.csv",save_data, sample_time, 14);
                count_plt++;
                
            }
            count_plt++;
        }
        
        if(flag_block)
            cout<<"we find the block!!"<<endl;
        

        //update the pos, vel, acc of the z direction
        MatrixXd u_z = Map<MatrixXd>(Uopt_z,mpc.N,1);
        MatrixXd z_all = mpc.Phi*current_state.z_hat+mpc.Gamma*u_z;
        
        double z = z_all(0,0);
        double zd = z_all(1,0);
        double zdd = -pow(robot.w,2)*z_hat(0,0)+pow(robot.w,2)*u_z(0,0);


        // publish optimized data
        opt.data = {Uopt_x[0], Uopt_y[0], (p0_x+Uopt_x[N]), (p0_y+Uopt_y[N]), z, zd, zdd, z_currentStep, z_nextStep, (p0_x+Uopt_x[N]+Uopt_x[N+1]), (p0_y+Uopt_y[N]+Uopt_y[N+1]), (p0_x+Uopt_x[N]+Uopt_x[N+1]+Uopt_x[N+2]), (p0_y+Uopt_y[N]+Uopt_y[N+1]+Uopt_y[N+2]), (p0_x+Uopt_x[N]+Uopt_x[N+1]+Uopt_x[N+2]+Uopt_x[N+3]), (p0_y+Uopt_y[N]+Uopt_y[N+1]+Uopt_y[N+2]+Uopt_y[N+3])};
        pub.publish(opt);

        cout<<"the count_plt is:"<<count_plt<<endl;
       
        ros::spinOnce();
        Loop_rate.sleep();
        delete qp_z,qp_x,qp_y;
    }

    return 0;

}