/*
 * Copyright (c) 2018 Agility Robotics
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef CASSIEPLUGIN
#define CASSIEPLUGIN

#include <array>
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <ignition/math.hh>

#include "cassie_core_sim.h"
#include "udp.h"

#define RECVLEN (PACKET_HEADER_LEN + CASSIE_USER_IN_T_PACKED_LEN)
#define SENDLEN (PACKET_HEADER_LEN + CASSIE_OUT_T_PACKED_LEN)

#define MOTOR_FILTER_NB 9
#define JOINT_FILTER_NB 4
#define JOINT_FILTER_NA 3


/**
 * @brief The CassiePlugin class contains vars and methods used by
 * Gazebo to load the plugin and communicate over UDP
 */
class CassiePlugin : public gazebo::ModelPlugin {

    // Event pointer
    gazebo::event::ConnectionPtr updateConnectionPtr_;

    // World Pointer
    gazebo::physics::WorldPtr worldPtr_;

    // Cassie core
    cassie_core_sim_t *corePtr_;

    // Motor Filter
    int motorFilterB_[MOTOR_FILTER_NB];
    typedef struct motor_filter {
        int x[MOTOR_FILTER_NB];
    } motor_filter_t;

    // Joint Filter
    double jointFilterB_[JOINT_FILTER_NB];
    double jointFilterA_[JOINT_FILTER_NA];
    typedef struct joint_filter {
        double x[JOINT_FILTER_NB];
        double y[JOINT_FILTER_NA];
    } joint_filter_t;

    // Motor properties
    std::array<gazebo::physics::JointPtr, 10> motor_;
    std::array<elmo_out_t*, 10> driveOut_;
    std::array<motor_filter_t, 10> motorFilter_;
    const std::array<int, 10> kMotorBits_;
    const std::array<double, 10> kGearRatio_;
    const std::array<double, 10> kMaxSpeed_;
    const std::array<double, 10> kMaxTorque_;
    const std::array<double, 10> kMotorOffset_;

    // Joint properties
    std::array<gazebo::physics::JointPtr, 6> joint_;
    std::array<cassie_joint_out_t*, 6> jointOut_;
    std::array<joint_filter_t, 6> jointFilter_;
    const std::array<int, 6> kJointBits_;
    const std::array<double, 10> kJointOffset_;

    // Pelvis pointer
    gazebo::physics::LinkPtr pelvisPtr_;

    // Latency buffer for cassie out
    cassie_out_t cassieOut_;

    // Update Rate
    double updateRate_;
    double updatePeriod_;
    gazebo::common::Time lastUpdateTime_;

    // UDP socket
    int sock_;

    // Send/receive buffers
    unsigned char recvBuf_[RECVLEN];
    unsigned char sendBuf_[SENDLEN];

    // Pointers to parts of buffers
    unsigned char *headerInPtr_;
    unsigned char *dataInPtr_;
    unsigned char *headerOutPtr_;
    unsigned char *dataOutPtr_;

    // Other UDP structures
    packet_header_info_t headerInfo_;
    cassie_user_in_t cassieUserIn_;
    struct sockaddr_storage srcAddr_;
    socklen_t addrLen_;
    gazebo::common::Time lastPacketTime_;
    gazebo::common::Time firstPacketTime_;

    // Sim flag
    bool runSim_;

    /**
     * @brief setMotorEncoder reads the drive position and sets motor encoder values
     * @param drive is the data struct with drive information
     * @param position is the drive position
     * @param filter is the motor filter pointer
     * @param bits number of bits for the drive encoder
     * @param ratio gear ratio
     */
    void setMotorEncoder(elmo_out_t *drive, double position,
                         motor_filter_t *filter,const int bits);

    /**
     * @brief setJointEncoder reads the joint position and sets joint encoder value
     * @param joint is the data struct with joint information
     * @param position is the joint position
     * @param filter is the joint filter pointer
     * @param bits number of bits for the joint encoder
     */
    void setJointEncoder(cassie_joint_out_t *joint, double position,
                         joint_filter_t *filter,const int bits);

    /**
     * @brief setMotor commands motor torques
     * @param outjoint is the joint pointer
     * @param u is calculated torque
     * @param sto STO switch
     * @param ratio is the gear ratio
     * @param tmax is the maximum safe torque
     * @param wmax is the max motor speed
     * @return
     */
    double setMotor(gazebo::physics::JointPtr outJoint, double u,
                    const bool sto, const double ratio,
                    const double tmax, const double wmax);

    /**
     * @brief initElmoOut sets the elmo drive params
     * @param elmoOut elmo pointer
     * @param torqueLimit
     * @param gearRatio
     */
    void initElmoOut(elmo_out_t *elmoOut, const double torqueLimit,
                     const double gearRatio);

    /**
     * @brief initCassieLegOut sets the Cassie Leg Out params
     * @param legOut cassie leg out pointer
     */
    void initCassieLegOut(cassie_leg_out_t *legOut);

    /**
     * @brief initCassieOut sets Cassie Out struct params
     * @param cassieOut pointer to Cassie Out
     */
    void initCassieOut(cassie_out_t *cassieOut);

    /**
     * @brief Load is called when the plugin is loaded in the sdf
     * @param _model pointer to the model
     * @param _sdf pointer to the SDF
     */
    virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /**
     * @brief onUpdate is called after every simulation update
     */
    void onUpdate();

    /**
     * @brief applyTorques is used to apply Torques to all the drives
     * @param cassieIn is the data struct which is fed to the robot
     */
    void applyTorques(const cassie_in_t* cassieIn);

    /**
     * @brief updateCassieOut updates the CassieOut data struct
     */
    void updateCassieOut();

    /**
     * @brief detachPelvis detaches the pelvis from the world
     */
    void detachPelvis();

public:
    /**
     * @brief CassiePlugin constructor for the plugin class
     */
    CassiePlugin();

    /**
     * @brief CassiePlugin destructor for the plugin class
     */
    ~CassiePlugin();


};

#endif // CASSIEPLUGIN

