#include "../include/air_right_arm_interface.hpp"

namespace air
{   

    RightArmIK::RightArmIK()
    {
        
    }
    
    double RightArmIK::toRadian(double degree)
    {
        return (degree / 180.0) * M_PI;
    }

    double RightArmIK::toDegree(double radian)
    {
        return (radian / M_PI) * 180.0;
    }

    DoublePair RightArmIK::wristEulerToLinearPosition(DoublePair desired_pos)
    {
        double r = desired_pos.first;
        double p = desired_pos.second;

        double C1 = std::cos(r);
        double C2 = std::cos(p);
        double S1 = std::sin(r);
        double S2 = std::sin(p);

        double pitch_command = 162.0 - sqrt(pow(X1-X1*C2,2) + pow((Y1-(X1*S1*S2+Y1*C1)),2) +
                              pow((Z-(Zz-X1*C1*S2+Y1*S1)),2));
 
        double roll_command = 162.0 - sqrt(pow(X2-X2*C2,2) + pow((Y2-(X2*S1*S2+Y2*C1)),2) +
                              pow((Z-(Zz-X2*C1*S2+Y2*S1)),2));

        return std::make_pair(roll_command, pitch_command);
    }

    DoublePair RightArmIK::wristEulerToLinearVelocity(DoublePair desired_pos, DoublePair desired_vel)
    {
        double th_roll  = desired_pos.first;
        double th_pitch = desired_pos.second;
        double th_dot_roll  = desired_vel.first;
        double th_dot_pitch = desired_vel.second;

        double C1 = std::cos(th_roll);
        double C2 = std::cos(th_pitch);
        double S1 = std::sin(th_roll);
        double S2 = std::sin(th_pitch);

        double pitchL = 162.0 - sqrt(pow(X1-X1*C2,2) + pow((Y1-(X1*S1*S2+Y1*C1)),2) +
                              pow((Z-(Zz-X1*C1*S2+Y1*S1)),2));
        double rollL = 162.0 - sqrt(pow(X2-X2*C2,2) + pow((Y2-(X2*S1*S2+Y2*C1)),2) +
                                    pow((Z-(Zz-X2*C1*S2+Y2*S1)),2));

        double pitch_vel_cmd = (2*X1*X1*S2*th_dot_pitch - 2*Y1*X1*C1*S2*th_dot_roll - 2*Y1*X1*S1*C2*th_dot_pitch +
                          2*Y1*Y1*S1*th_dot_roll -2*Z*X1*S1*S2*th_dot_roll + 2*Z*X1*C1*C2*th_dot_pitch - 2*Z*Y1*C1*th_dot_roll +
                          2*Zz*X1*S1*S2*th_dot_roll - 2*Zz*X1*C1*C2*th_dot_pitch + 2*Zz*Y1*C1*th_dot_roll)/(2*(pitchL - 162.0));


        double roll_vel_cmd = (2*(X2 - X2*C2)*X2*S2*th_dot_pitch + 2*(Y2 - (X2*S1*S2 + Y2*C1))*(Y2*S1*th_dot_roll -
                        X2*C1*S2*th_dot_roll - X2*S1*C2*th_dot_pitch) + 2*((Z - Zz) +
                        (X2*C1*S2 - Y2*S1))*(-X2*S1*S2*th_dot_roll + X2*C1*C2*th_dot_pitch - Y2*C1*th_dot_roll))/(2*(rollL - 162.0));


        return std::make_pair(roll_vel_cmd, pitch_vel_cmd);
    }

    DoublePair RightArmIK::wristLinearToRadianPosition(DoublePair roll_pitch_in_mm)
    {
        double rollxpiston = roll_pitch_in_mm.first;
        double pitchypiston = roll_pitch_in_mm.second;

        double p00 = 0.001665;
        double p10 = 1;
        double p01 = 1.023;
        double p20 = 0.001848;
        double p11 = -0.001393;
        double p02 = 0.0004754;
        double p30 = 3.407e-05;
        double p21 = 0.0001189;
        double p12 = 0.0001038;
        double p03 = 0.000226;
        double p40 = 5.659e-06;
        double p31 = 4.071e-06;
        double p22 = 6.077e-06;
        double p13 = 2.604e-06;
        double p04 = -2.154e-06;

        double r00 = -8.671e-06;
        double r10 = 1.019;
        double r01 = -1.007;
        double r20 = 2.939e-05;
        double r11 = -1.657e-05;
        double r02 = 1.088e-05;
        double r30 = 4.714e-05;
        double r21 = -0.0001397;
        double r12 = 0.0001287;
        double r03 = -3.563e-05;
        double r40 = -9.05e-08;
        double r31 = -4.625e-07;
        double r22 = 2.809e-06;
        double r13 = -3.96e-06;
        double r04 = 1.643e-06;

        double roll_rad = toRadian(r00 + r10*rollxpiston + r01*pitchypiston + r20*pow(rollxpiston,2) + r11*rollxpiston*pitchypiston + r02*pow(pitchypiston,2) + r30*pow(rollxpiston,3) + r21*pow(rollxpiston,2)*pitchypiston + r12*rollxpiston*pow(pitchypiston,2) + r03*pow(pitchypiston,3) + r40*pow(rollxpiston,4) + r31*pow(rollxpiston,3)*pitchypiston + r22*pow(rollxpiston,2)*pow(pitchypiston,2) + r13*rollxpiston*pow(pitchypiston,3) + r04*pow(pitchypiston,4));
        double pitch_rad = -toRadian(p00 + p10*rollxpiston + p01*pitchypiston + p20*pow(rollxpiston,2) + p11*rollxpiston*pitchypiston + p02*pow(pitchypiston,2) + p30*pow(rollxpiston,3) + p21*pow(rollxpiston,2)*pitchypiston + p12*rollxpiston*pow(pitchypiston,2) + p03*pow(pitchypiston,3) + p40*pow(rollxpiston,4) + p31*pow(rollxpiston,3)*pitchypiston + p22*pow(rollxpiston,2)*pow(pitchypiston,2) + p13*rollxpiston*pow(pitchypiston,3) + p04*pow(pitchypiston,4));

        return std::make_pair(roll_rad, pitch_rad);
    }

    DoublePair RightArmIK::wristLinearToRadianVelocity(DoublePair roll_pitch_vel_in_mm)
    {
        return roll_pitch_vel_in_mm;
    }

    double RightArmIK::elbowLinearToRadianPosition(double pos_in_mm)
    {
        constexpr double D    =150.57;
        constexpr double L    =180.50;
        constexpr double o1   =28.0;
        constexpr double o2   =33.0;
        constexpr double r    =54.95;
        constexpr double h    =226.0;
        constexpr double h1   {std::sqrt(h*h+o2*o2)};
        double m     =pos_in_mm;
        double Ly   {std::sqrt(o1*o1+(m+L)*(m+L))};
        double A    {std::acos((r*r+h1*h1-Ly*Ly)/(2.0*r*h1))};
        double B    {-360.0+90.0+D+toDegree(A+std::atan2(o2,h))};
        return      toRadian(B);
    }

    double RightArmIK::elbowLinearToRadianVelocity( double vel_in_mm)
    {
        constexpr double r    {54.95};
        return      vel_in_mm/r;
    }

    double RightArmIK::elbowRadianToLinearPosition(double pos_in_rad)
    {
        constexpr double D    =150.57;
        constexpr double L    =180.50;
        constexpr double o1   =28.0;
        constexpr double o2   =33.0;
        constexpr double r    =54.95;
        constexpr double h    =226.0;
        constexpr double h1   {std::sqrt(h*h+o2*o2)};
        double A    {pos_in_rad+2.0*M_PI-M_PI/2.0-toRadian(D)-std::atan2(o2,h)};
        return      sqrt((r*r+h1*h1-2.0*r*h1*cos(A))-(o1*o1))-L;
    }

    double RightArmIK::elbowRadianToLinearVelocity(double vel_in_rad)
    {
        constexpr double r    {54.95};
        return      r*vel_in_rad;
    }

    // ---------------------------------------------------------------------------------------------------------------------------------------------------------------

    RightArm::RightArm()
    {
        
        m_RightArmIK = RightArmIK();

        bhf::ads::SetLocalAddress({192, 168, 6, 101, 1, 1});
        
        m_Route = new AdsDevice(m_RemoteIpV4, m_RemoteNetID, AMSPORT_R0_PLC_TC3);
        
        m_AdsActTimestamp = new AdsVariable<uint32_t>(*m_Route, "GVL.fActTimeStamp");

        AdsVariable<uint32_t> AdsLastTimestamp {*m_Route, "GVL.fLastTimeStamp"};
        AdsVariable<bool> AdsHalt{*m_Route, "GVL.state_HALT"};
        AdsVariable<bool> AdsTimeout{*m_Route, "GVL.state_TIMEOUT"};

        AdsLastTimestamp = ros::Time::now().sec;
        *m_AdsActTimestamp = ros::Time::now().sec;

        AdsHalt = false;
        AdsTimeout = false;

    }

    void RightArm::setJointNames(const std::vector<std::string> joint_names)
    {
        m_JointNames = joint_names;
    }

    sensor_msgs::JointState RightArm::readJoint(int num_joints, std::array<double, 7>& raw_joint_positions)
    {
        sensor_msgs::JointState jointState;

        std::array<double, 7> positionArray;
        std::array<double, 7> velocityArray;

        try{

            AdsVariable<std::array<double, 7>> positionVar{*m_Route, "GVL.fActPose_Arm"};
            AdsVariable<std::array<double, 7>> velocityVar{*m_Route, "GVL.fActVel_Arm"};
            positionArray = positionVar;
            velocityArray = velocityVar;

        }catch(const AdsException& ex){

            std::cout << "Error Code: " << ex.errorCode << std::endl;
            std::cout << "AdsException message: " << ex.what() << std::endl;

        }catch(const std::runtime_error& ex){

            std::cout << ex.what() << std::endl;

        }

        // For s1, s2 and s3 Joints: 

        for(int i = 0; i < 3; i++)
        {
            jointState.position.push_back(RightArmIK::toRadian(positionArray[i]));
            jointState.velocity.push_back(RightArmIK::toRadian(velocityArray[i]));
        }

        // Right Arm Elbow Joint:
        jointState.position.push_back(m_RightArmIK.elbowLinearToRadianPosition(positionArray[3]));
        jointState.velocity.push_back(m_RightArmIK.elbowLinearToRadianVelocity(velocityArray[3]));

        // Roll, pitch Joints:

        DoublePair actual_RP_Euler_Position = m_RightArmIK.wristLinearToRadianPosition(DoublePair(positionArray[4], positionArray[5]));
        DoublePair actual_RP_Euler_Velocity = m_RightArmIK.wristLinearToRadianVelocity(DoublePair(velocityArray[4], velocityArray[5]));

        jointState.position.push_back(actual_RP_Euler_Position.first);
        jointState.velocity.push_back(actual_RP_Euler_Velocity.first);
        jointState.position.push_back(actual_RP_Euler_Position.second);
        jointState.velocity.push_back(actual_RP_Euler_Velocity.second);

        // For Yaw Joint:

        jointState.position.push_back(RightArmIK::toRadian(positionArray[6]));
        jointState.velocity.push_back(RightArmIK::toRadian(velocityArray[6]));

        return jointState;

    }

    bool RightArm::writeJoint(int num_joints, std::vector<double> act_pose, std::vector<double> pose, std::vector<double> vel, std::vector<double> acc)
    {
        try{

            AdsVariable<std::array<double, 7>> adsPosVar{*m_Route, "GVL.fGoalPos_Arm"};
            AdsVariable<std::array<double, 7>> adsVelVar{*m_Route, "GVL.fGoalVel_Arm"};
            //AdsVariable<std::array<double, 7>> adsAccVar{*m_Route, "GVL.fGoalAcc_Arm"};
            
            std::array<double, 7> posToWrite;
            std::array<double, 7> velToWrite;
            /*std::array<double, 7> accToWrite;
            
            for(int i =0; i<7;i++)
            {
                accToWrite[i]=acc.at(i);
            }*/
            // s1, s2, s3
            for(int i = 0; i < 3; i++)
            {
                posToWrite[i] = RightArmIK::toDegree(pose.at(i));
                velToWrite[i] = RightArmIK::toDegree(vel.at(i));
            }

            // elbow

            double beckhoff_urdf_error = M_PI / 2.0;
            double elbow_desired_position = pose.at(3);
            //elbow_desired_position += beckhoff_urdf_error;

            posToWrite[3] = m_RightArmIK.elbowRadianToLinearPosition(elbow_desired_position);
            velToWrite[3] = m_RightArmIK.elbowRadianToLinearVelocity(vel.at(3));

            // Roll | Pitch Joints

            DoublePair desired_rp_angular_position;
            DoublePair desired_rp_angular_velocity;
            DoublePair desired_rp_linear_position;
            DoublePair desired_rp_linear_vel;

            desired_rp_angular_position = std::make_pair(pose.at(4), pose.at(5));
            desired_rp_angular_velocity = std::make_pair(vel.at(4), vel.at(5));
            desired_rp_linear_position = m_RightArmIK.wristEulerToLinearPosition(desired_rp_angular_position);
            desired_rp_linear_vel = m_RightArmIK.wristEulerToLinearVelocity(desired_rp_angular_position, desired_rp_angular_velocity);
            
            posToWrite[4] = desired_rp_linear_position.first;
            velToWrite[4] = desired_rp_linear_vel.first;

            posToWrite[5] = desired_rp_linear_position.second;
            velToWrite[5] = desired_rp_linear_vel.second;

            // Yaw Joint

            posToWrite[6] = RightArmIK::toDegree(pose.at(6));
            velToWrite[6] = RightArmIK::toDegree(vel.at(6));

            adsPosVar = posToWrite;
            adsVelVar = velToWrite;
            //adsAccVar = accToWrite;

        } catch(const AdsException& ex){
            
            std::cout << "Error Code: " << ex.errorCode << std::endl;
            std::cout << "AdsException message: " << ex.what() << std::endl;

        } catch(const std::runtime_error& ex){

            std::cout << ex.what() << std::endl;

        }

        return true;
    }
    
}

