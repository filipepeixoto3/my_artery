/*
 * Artery V2X Simulation Framework
 * Copyright 2019 Raphael Riebl et al.
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#ifndef ARTERY_CPSERVICE_H_V08YXH9S
#define ARTERY_CPSERVICE_H_V08YXH9S

#include "artery/application/ItsG5BaseService.h"
#include "artery/utility/Channel.h"
#include "artery/utility/Geometry.h"
#include <vanetza/asn1/cpm.hpp>
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp/simtime.h>

#include "artery/application/json.hpp"
#include "artery/application/VehicleKinematics.h"

#include "artery/traci/MobilityBase.h"
#include "artery/traci/Controller.h"
#include "artery/traci/VehicleController.h"

#include "artery/application/KalmanFilter.h"

#include <boost/geometry/index/rtree.hpp>
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>

#include <omnetpp/ccanvas.h>
#include <omnetpp/clistener.h>
#include <omnetpp/csimplemodule.h>
#include <veins/base/modules/BaseMobility.h>
#include <zmq.hpp>

#include <list>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <map>
#include <fstream>
#include <sstream>
#include <random>



using json = nlohmann::json;

using namespace traci;

namespace artery
{

class NetworkInterfaceTable;
class Timer;
class VehicleDataProvider;
class API;

typedef struct {
    double yaw;
    double x;
    double y;
    double z;
    double norm_sensor_angle;
    double norm_detection_angle;
    double sensor_dist;
    omnetpp::cFigure::Point sensor_pos;
} uss_setup;

typedef struct {
    double timestamp;
    uint16_t detection_delta_time;
    double depth;
    omnetpp::cFigure::Point detection_coord;
    int repeat_count;
    omnetpp::cLineFigure* line1;
    omnetpp::cLineFigure* line2;
    omnetpp::cArcFigure* arc;
}   uss_value;

typedef struct {
    double timestamp;
    uint16_t detection_delta_time;
    omnetpp::cFigure::Point detection_coord;
} object_info;

typedef struct {
    double timestamp;
    uint16_t detection_delta_time;
    omnetpp::cFigure::Point detection_coord;
    double speed;
    double direction;
    int sensor;
    int count_reset;
    omnetpp::cLineFigure* cross_line1;
    omnetpp::cLineFigure* cross_line2;
} track_info;

bool operator<(const std::string& one, const std::string& other);

class CpService : public ItsG5BaseService
{

public:
    CpService();
    virtual ~CpService();
 
protected:
    void initialize() override;
    void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
    void trigger() override;

private:
    void checkTriggeringConditions(const omnetpp::SimTime&);
    bool checkHeadingDelta() const;
    bool checkPositionDelta() const;
    bool checkSpeedDelta() const;
    void sendCpm(const omnetpp::SimTime&);
    omnetpp::SimTime genCpmDcc();


    ChannelNumber mPrimaryChannel = channel::CCH;
    const NetworkInterfaceTable* mNetworkInterfaceTable = nullptr;
    const VehicleDataProvider* mVehicleDataProvider = nullptr;
	const traci::VehicleController* mVehicleController = nullptr;
    const Timer* mTimer = nullptr;
    LocalDynamicMap* mLocalDynamicMap = nullptr;
    
    omnetpp::SimTime mGenCpmMin;
    omnetpp::SimTime mGenCpmMax;
    omnetpp::SimTime mGenCpm;
    unsigned mGenCpmLowDynamicsCounter;
    unsigned mGenCpmLowDynamicsLimit;
    Position mLastCpmPosition;
    vanetza::units::Velocity mLastCpmSpeed;
    vanetza::units::Angle mLastCpmHeading;
    omnetpp::SimTime mLastCpmTimestamp;
    vanetza::units::Angle mHeadingDelta;
    vanetza::units::Length mPositionDelta;
    vanetza::units::Velocity mSpeedDelta;
    bool mDccRestriction;
    bool mFixedRate;

    omnetpp::cCanvas* canvas = nullptr;
    omnetpp::cGroupFigure* detections = nullptr;
    omnetpp::cGroupFigure* rings = nullptr;
    omnetpp::cFigure::Point detection_pos;
    omnetpp::cFigure::Point sensor_pos;

    traci::Boundary m_boundary;

	std::map<int, uss_setup> uss_setups;
    std::map<int, uss_value> detections_map;
    std::map<int, object_info> objects_map;
    
    std::map<int, track_info> tracks_map;
    std::map<int, track_info> past_tracks_map;
    int tracks_count = 0;
    // std::vector<int> active_tracks;
    // std::vector<int> past_active_tracks;
    std::map<int, KalmanFilter> kf_map;

    std::string topic;
    double vehicle_length;
    double vehicle_width;
    double vehicle_angle;
    omnetpp::cFigure::Point cur_pos_center;

    omnetpp::cFigure::Point static_object[12];
    
    std::list<Identifier_t> active_tracks_ids;
    std::string protocol;
    std::string host;
    zmq::socket_t socket;
    zmq::context_t context;

    std::ofstream detection_file; 
    std::ofstream tracks_file; 
    std::ofstream cpms_file; 
    std::mt19937 gen;                           // Random number engine
    std::uniform_int_distribution<> distrib;    // Distribution for IDs
    // std::random_device rd;
    // std::mt19937 gen(rd());
    // std::uniform_int_distribution<> distrib(0, 255);


    void updateEveryTimestamp();
    void receiveFromCarla();
    void storeDetections(int role_name, double initial_timestamp, uint16_t detection_delta_time, double depth);
    //omnetpp::cFigure::Point findPosition(omnetpp::cFigure::Point cpc, omnetpp::cFigure::Point A, omnetpp::cFigure::Point B, double d_A, double d_B, omnetpp::cFigure::Point C, omnetpp::cFigure::Point D);
    void determineObjects();
    void multiObjectTracking();

    std::map<int, int> optimal_assignment(const std::vector<std::vector<double>>& cost_matrix, const std::vector<int>& track_ids);

    // std::vector<std::vector<double>> generate_cost_matrix(double distance_weight, double direction_weight);

    std::pair<std::vector<std::vector<double>>, std::vector<std::vector<double>>> generate_cost_matrices();

};

vanetza::asn1::Cpm createCollectivePerceptionMessage(const VehicleDataProvider& vdp, uint16_t genDeltaTime, const VehicleController& vc, std::map<int, uss_setup> us,std::map<int, uss_value> dm, std::map<int, track_info> tm, omnetpp::cGroupFigure& det, omnetpp::cFigure::Point cpc);

}  // namespace artery

#endif /* ARTERY_CpSERVICE_H_V08YXH9S */
