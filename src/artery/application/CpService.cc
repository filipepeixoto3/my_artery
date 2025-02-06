/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2019 Raphael Riebl et al.
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/application/CpService.h"

#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/CpObject.h"
#include "artery/application/KalmanFilter.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/VehicleKinematics.h"
#include "artery/traci/Controller.h"
#include "artery/traci/VehicleController.h"
#include "artery/utility/simtime_cast.h"
#include "veins/base/utils/Coord.h"

#include <omnetpp/cexception.h>
#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/transmission.hpp>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include <vanetza/facilities/cam_functions.hpp>

#include <chrono>
#include <cmath>
#include <random>
#include <thread>

// batota!
#include "artery/traci/VehicleController.h"
#include "artery/traci/Cast.h"

namespace artery
{

using namespace omnetpp;
class API;


static const auto microdegree = vanetza::units::degree * boost::units::si::micro;
static const auto decidegree = vanetza::units::degree * boost::units::si::deci;
static const auto dekadegree = vanetza::units::degree * boost::units::si::deka;

static const auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;
static const auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

static const simsignal_t scSignalCpmReceived = cComponent::registerSignal("CpmReceived");
static const simsignal_t scSignalCpmSent = cComponent::registerSignal("CpmSent");

bool operator<(const std::string& one, const std::string& other)  // necessario para o map (for some unknown reasone)
{
    return std::tie(one) < std::tie(other);
}

template <typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
    boost::units::quantity<U> v{q};
    return std::round(v.value());
}

static SpeedValue_t buildSpeedValue(const vanetza::units::Velocity& v)
{
    static const vanetza::units::Velocity lower{0.0 * boost::units::si::meter_per_second};
    static const vanetza::units::Velocity upper{163.82 * boost::units::si::meter_per_second};

    SpeedValue_t speed = SpeedValue_unavailable;
    if (v >= upper) {
        speed = 16382;  // see CDD A.74 (TS 102 894 v1.2.1)
    } else if (v >= lower) {
        speed = round(v, centimeter_per_second) * SpeedValue_oneCentimeterPerSec;
    }
    return speed;
}


Define_Module(CpService)

CpService::CpService() :
    mGenCpmMin{100, SIMTIME_MS}, mGenCpmMax{1000, SIMTIME_MS}, mGenCpm(mGenCpmMax), mGenCpmLowDynamicsCounter(0), mGenCpmLowDynamicsLimit(3)
{
}

CpService::~CpService()
{
    EV_INFO << "Closing socket..." << endl;
    this->socket.close();
    EV_INFO << "Closing context..." << endl;
    this->context.close();

    detections->removeFromParent();
    rings->removeFromParent();
}

void CpService::initialize()
{

    ItsG5BaseService::initialize();
    mNetworkInterfaceTable = &getFacilities().get_const<NetworkInterfaceTable>();
    mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
    mVehicleController = &getFacilities().get_const<traci::VehicleController>();
    mTimer = &getFacilities().get_const<Timer>();
    mLocalDynamicMap = &getFacilities().get_mutable<artery::LocalDynamicMap>();

    // vehicle dimensions
    vehicle_length = mVehicleController->getLength().value();
    vehicle_width = mVehicleController->getWidth().value();
    // EV << "Length: " << vehicle_length << endl << "Width: " << vehicle_width << endl;
    // setup dos sensores do veiculo
    uss_setups[0] = {
        -78,
        vehicle_length / 2 + 0.1,
        -5 * vehicle_width / 12,
        0.5,
        0,
        0,
        sqrt(pow(abs(vehicle_length / 2 + 0.1), 2) + pow(abs(-5 * vehicle_width / 12), 2)),
        omnetpp::cFigure::Point(0, 0)};
    uss_setups[1] = {
        -41,
        vehicle_length / 2 + 0.1,
        -vehicle_width / 4,
        0.5,
        0,
        0,
        sqrt(pow(abs(vehicle_length / 2 + 0.1), 2) + pow(abs(-vehicle_width / 4), 2)),
        omnetpp::cFigure::Point(0, 0)};
    uss_setups[2] = {
        -8,
        vehicle_length / 2 + 0.1,
        -vehicle_width / 12,
        0.5,
        0,
        0,
        sqrt(pow(abs(vehicle_length / 2 + 0.1), 2) + pow(abs(-vehicle_width / 12), 2)),
        omnetpp::cFigure::Point(0, 0)};
    uss_setups[3] = {
        +8,
        vehicle_length / 2 + 0.1,
        vehicle_width / 12,
        0.5,
        0,
        0,
        sqrt(pow(abs(vehicle_length / 2 + 0.1), 2) + pow(abs(vehicle_width / 12), 2)),
        omnetpp::cFigure::Point(0, 0)};
    uss_setups[4] = {
        +41,
        vehicle_length / 2 + 0.1,
        vehicle_width / 4,
        0.5,
        0,
        0,
        sqrt(pow(abs(vehicle_length / 2 + 0.1), 2) + pow(abs(vehicle_width / 4), 2)),
        omnetpp::cFigure::Point(0, 0)};
    uss_setups[5] = {
        +78,
        vehicle_length / 2 + 0.1,
        5 * vehicle_width / 12,
        0.5,
        0,
        0,
        sqrt(pow(abs(vehicle_length / 2 + 0.1), 2) + pow(abs(5 * vehicle_width / 12), 2)),
        omnetpp::cFigure::Point(0, 0)};
    uss_setups[6] = {
        180 - 78,
        -vehicle_length / 2 - 0.1,
        5 * vehicle_width / 12,
        0.5,
        0,
        0,
        sqrt(pow(abs(-vehicle_length / 2 - 0.1), 2) + pow(abs(5 * vehicle_width / 12), 2)),
        omnetpp::cFigure::Point(0, 0)};
    uss_setups[7] = {
        180 - 29,
        -vehicle_length / 2 - 0.1,
        vehicle_width / 4,
        0.5,
        0,
        0,
        sqrt(pow(abs(-vehicle_length / 2 - 0.1), 2) + pow(abs(vehicle_width / 4), 2)),
        omnetpp::cFigure::Point(0, 0)};
    uss_setups[8] = {
        180 - 6,
        -vehicle_length / 2 - 0.1,
        vehicle_width / 12,
        0.5,
        0,
        0,
        sqrt(pow(abs(-vehicle_length / 2 - 0.1), 2) + pow(abs(vehicle_width / 12), 2)),
        omnetpp::cFigure::Point(0, 0)};
    uss_setups[9] = {
        6 - 180,
        -vehicle_length / 2 - 0.1,
        -vehicle_width / 12,
        0.5,
        0,
        0,
        sqrt(pow(abs(-vehicle_length / 2 - 0.1), 2) + pow(abs(-vehicle_width / 12), 2)),
        omnetpp::cFigure::Point(0, 0)};
    uss_setups[10] = {
        29 - 180,
        -vehicle_length / 2 - 0.1,
        -vehicle_width / 4,
        0.5,
        0,
        0,
        sqrt(pow(abs(-vehicle_length / 2 - 0.1), 2) + pow(abs(-vehicle_width / 4), 2)),
        omnetpp::cFigure::Point(0, 0)};
    uss_setups[11] = {
        78 - 180,
        -vehicle_length / 2 - 0.1,
        -5 * vehicle_width / 12,
        0.5,
        0,
        0,
        sqrt(pow(abs(-vehicle_length / 2 - 0.1), 2) + pow(abs(-5 * vehicle_width / 12), 2)),
        omnetpp::cFigure::Point(0, 0)};


    // avoid unreasonable high elapsed time values for newly inserted vehicles
    mLastCpmTimestamp = simTime();

    // generation rate boundaries
    mGenCpmMin = par("minInterval");
    mGenCpmMax = par("maxInterval");
    mGenCpm = mGenCpmMax;

    // vehicle dynamics thresholds
    mHeadingDelta = vanetza::units::Angle{par("headingDelta").doubleValue() * vanetza::units::degree};
    mPositionDelta = par("positionDelta").doubleValue() * vanetza::units::si::meter;
    mSpeedDelta = par("speedDelta").doubleValue() * vanetza::units::si::meter_per_second;

    mDccRestriction = par("withDccRestriction");
    mFixedRate = par("fixedRate");

    // look up primary channel for CP
    mPrimaryChannel = getFacilities().get_const<MultiChannelPolicy>().primaryChannel(vanetza::aid::CP);

    // insert group of figures that will be drawn in the canvas
    canvas = getSimulation()->getModuleByPath("World")->getCanvas();
    detections = new cGroupFigure();
    rings = new cGroupFigure();
    canvas->addFigure(detections);
    canvas->addFigure(rings);

    topic = mVehicleController->getVehicleId();

    // Connect to ZMQ
    this->context = zmq::context_t{1};
    this->socket = zmq::socket_t{context, ZMQ_SUB};

    this->socket.setsockopt(ZMQ_RCVTIMEO, 4000);  // set timeout to 4 seconds
    this->socket.setsockopt(ZMQ_SNDTIMEO, 4000);  // set timeout to 4 seconds

    // EV << "Will subscribe to: " << topic << endl;
    zmq_setsockopt(this->socket, ZMQ_SUBSCRIBE, topic.c_str(), topic.length());

    std::string host = par("host").stringValue();
    int port = par("port").intValue();
    std::string addr = "tcp://" + host + ":" + std::to_string(port);
    // EV << "Trying connecting to: " << addr << endl;
    this->socket.connect(addr);

    EV_INFO << "Connect to ZMQ succesful!" << endl;

    // remove(boost::lexical_cast<std::string>(mVehicleDataProvider->getStationId()) + "_detection_pos.txt");
    std::ostringstream oss;
    oss << mVehicleController->getVehicleId() << "_detection_pos.txt";
    remove(oss.str().c_str());
    detection_file.open(oss.str().c_str(), std::ios_base::app);
    detection_file << "id,sensor,timestamp,x,y,active_objects" << endl;
    detection_file.close();
}

void CpService::trigger()
{
    Enter_Method("trigger");
    updateEveryTimestamp();
    receiveFromCarla();                     // checks if there is content in the socket connected to CARLA and if so the data is processed
    processDetections();                    // extract objects from the detections_map
    multiObjectTracking();                  // tracking of multiple objects and correcting their positions with kalman filter
    checkTriggeringConditions(simTime());   // checks if coditions to create and a send a CPM are meet


    //TESTING FASE
    // double ang1 = uss_setups[0].norm_detection_angle-(15*PI/180);
    // while (ang1 < -PI)
    //     ang1 += 2 * PI;
    // while (ang1 >= PI)
    //     ang1 -= 2 * PI;
    // double ang2 = uss_setups[0].norm_detection_angle+(15*PI/180);
    // while (ang2 < -PI)
    //     ang2 += 2 * PI;
    // while (ang2 >= PI)
    //     ang2 -= 2 * PI;
    //      double ang3 = uss_setups[5].norm_detection_angle-(15*PI/180);
    // while (ang3 < -PI)
    //     ang3 += 2 * PI;
    // while (ang3 >= PI)
    //     ang3 -= 2 * PI;
    // double ang4 = uss_setups[5].norm_detection_angle+(15*PI/180);
    // while (ang4 < -PI)
    //     ang4 += 2 * PI;
    // while (ang4 >= PI)
    //     ang4 -= 2 * PI;
    
    // const traci::Boundary boundary{mVehicleController->getTraCI()->simulation.getNetBoundary()};
    // artery::Position arteryPos;

    // omnetpp::cFigure::Point coord1 =
    //     omnetpp::cFigure::Point(-cos(ang1) * 5.5, sin(ang1) * 5.5)
    //     + cur_pos_center + uss_setups[0].sensor_pos;
    // arteryPos = Position(coord1.x, coord1.y);
    // traci::TraCIPosition traCIPos1 = position_cast(boundary,arteryPos);

    // omnetpp::cFigure::Point coord2 =
    //     omnetpp::cFigure::Point(-cos(ang2) * 5.5, sin(ang2) * 5.5)
    //     + cur_pos_center + uss_setups[0].sensor_pos;
    // arteryPos = Position(coord2.x, coord2.y);
    // traci::TraCIPosition traCIPos2 = position_cast(boundary,arteryPos);

    // omnetpp::cFigure::Point coord3 =
    //     omnetpp::cFigure::Point(-cos(ang3) * 5.5, sin(ang3) * 5.5)
    //     + cur_pos_center + uss_setups[5].sensor_pos;
    // arteryPos = Position(coord3.x, coord3.y);
    // traci::TraCIPosition traCIPos3 = position_cast(boundary,arteryPos);
    
    // omnetpp::cFigure::Point coord4 =
    //     omnetpp::cFigure::Point(-cos(ang4) * 5.5, sin(ang4) * 5.5)
    //     + cur_pos_center + uss_setups[5].sensor_pos;
    // arteryPos = Position(coord4.x, coord4.y);
    // traci::TraCIPosition traCIPos4 = position_cast(boundary,arteryPos);

    // std::vector<traci::TraCIPosition> coords;
    // coords.push_back(traCIPos1);
    // coords.push_back(traCIPos2);
    // coords.push_back(traCIPos3);
    // coords.push_back(traCIPos4);

    // omnetpp::cFigure::Point a = omnetpp::cFigure::Point(coord2.x + 0.1, coord2.y + 0.1);
    // omnetpp::cFigure::Point b = omnetpp::cFigure::Point(coord2.x + 0.1, coord2.y - 0.1);
    // omnetpp::cFigure::Point c = omnetpp::cFigure::Point(coord2.x - 0.1, coord2.y - 0.1);
    // omnetpp::cFigure::Point d = omnetpp::cFigure::Point(coord2.x - 0.1, coord2.y + 0.1);

    // auto cross_line1 = new cLineFigure();
    // cross_line1->setStart(a);
    // cross_line1->setEnd(c);
    // cross_line1->setLineColor("GREEN");
    // detections->addFigure(cross_line1);
    // auto cross_line2 = new cLineFigure();
    // cross_line2->setStart(b);
    // cross_line2->setEnd(d);
    // cross_line2->setLineColor("GREEN");
    // detections->addFigure(cross_line2);


    // for (size_t i = 0; i < coords.size(); ++i) {
    //     EV << "Point " << i << ": ("
    //               << coords[i].x << ", " << coords[i].y << ")\n";
    // }

    detections_map.clear();
    objects_map.clear();
}

void CpService::indicate(const vanetza::btp::DataIndication& ind, std::unique_ptr<vanetza::UpPacket> packet)  // when a CPM is received
{
    Enter_Method("indicate");

    Asn1PacketVisitor<vanetza::asn1::Cpm> visitor;
    const vanetza::asn1::Cpm* cpm = boost::apply_visitor(visitor, *packet);
    if (cpm && cpm->validate()) {
        CpObject obj = visitor.shared_wrapper;
        emit(scSignalCpmReceived, &obj);

        const vanetza::asn1::Cpm& msg = obj.asn1();
        using namespace traci;
        traci::API traci;

        TraCIGeoPosition tgp = {
            (double)msg->cpm.cpmParameters.managementContainer.referencePosition.longitude / (Longitude_oneMicrodegreeEast * 1000000),
            (double)msg->cpm.cpmParameters.managementContainer.referencePosition.latitude / (Latitude_oneMicrodegreeNorth * 1000000)};


        TraCIPosition tp;
        tp.x = mVehicleController->getTraCI()->convert2D(tgp).x;
        tp.y = mVehicleController->getTraCI()->convert2D(tgp).y;
        tp.z = 0;
        const traci::Boundary boundary{mVehicleController->getTraCI()->simulation.getNetBoundary()};
        Position pos = position_cast(boundary, tp);
        omnetpp::cFigure::Point cpc = omnetpp::cFigure::Point(
            pos.x.value() -
                cos(-((double)msg->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.heading.headingValue * PI / 1800) + PI / 2) *
                    ((double)msg->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.vehicleLength->vehicleLengthValue / 100) / 2,
            pos.y.value() +
                sin(-((double)msg->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.heading.headingValue * PI / 1800) + PI / 2) *
                    ((double)msg->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.vehicleLength->vehicleLengthValue / 100) / 2);

        // DRAW THE DETECTIONS RECEIVED IN THE CPM
        for (int i = 0; i < msg->cpm.cpmParameters.perceivedObjectContainer->list.count; i++) {
            PerceivedObject* object = msg->cpm.cpmParameters.perceivedObjectContainer->list.array[i];
            // EV << "xDistance:" << (double)object->xDistance.value / 100 << endl << "yDistance: " << (double)object->yDistance.value / 100 << endl;


            // ReferencePosition_t r = getPos(libsumo::VAR_POSITION, msg->header.stationID);

            active_obj_ids.push_back(object->objectID);
            // omnetpp::cFigure::Point final_pos = {((double)object->xDistance.value) / 100.00, ((double)object->yDistance.value) / 100.00};
            omnetpp::cFigure::Point final_pos = {cpc.x + ((double)object->xDistance.value / 100), cpc.y + ((double)object->yDistance.value / 100)};
            omnetpp::cFigure::Point a = omnetpp::cFigure::Point(final_pos.x + 0.1, final_pos.y + 0.1);
            omnetpp::cFigure::Point b = omnetpp::cFigure::Point(final_pos.x + 0.1, final_pos.y - 0.1);
            omnetpp::cFigure::Point c = omnetpp::cFigure::Point(final_pos.x - 0.1, final_pos.y - 0.1);
            omnetpp::cFigure::Point d = omnetpp::cFigure::Point(final_pos.x - 0.1, final_pos.y + 0.1);

            auto cross_line1 = new cLineFigure();
            cross_line1->setStart(a);
            cross_line1->setEnd(c);
            cross_line1->setLineColor("GREEN");
            // detections->addFigure(cross_line1);
            auto cross_line2 = new cLineFigure();
            cross_line2->setStart(b);
            cross_line2->setEnd(d);
            cross_line2->setLineColor("GREEN");
            // detections->addFigure(cross_line2);
        }

        // for (int i = 0; i < msg->cpm.cpmParameters.sensorInformationContainer->list.count; i++) {
        //     SensorInformation* si = msg->cpm.cpmParameters.sensorInformationContainer->list.array[i];
        //     EV << "sensorId: " << si->sensorID << endl << "type: " << si->type << endl;
        //     EV << "refPointId: " << si->detectionArea.choice.vehicleSensor.refPointId << endl;
        //     EV << "xSensorOffset: " << si->detectionArea.choice.vehicleSensor.xSensorOffset << endl;
        //     EV << "ySensorOffset: " << si->detectionArea.choice.vehicleSensor.ySensorOffset << endl;
        //     EV << "range: " << si->detectionArea.choice.vehicleSensor.vehicleSensorPropertyList.list.array[0]->range;
        //     EV << "horizontalOpeningAngleStart: " << si->detectionArea.choice.vehicleSensor.vehicleSensorPropertyList.list.array[0]->horizontalOpeningAngleStart;
        //     EV << "horizontalOpeningAngleEnd: " << si->detectionArea.choice.vehicleSensor.vehicleSensorPropertyList.list.array[0]->horizontalOpeningAngleEnd;
        // }

        mLocalDynamicMap->updatePerception(obj);
    }
}

void CpService::checkTriggeringConditions(const SimTime& T_now)
{
    SimTime& T_GenCpm = mGenCpm;
    const SimTime& T_GenCpmMin = mGenCpmMin;
    const SimTime& T_GenCpmMax = mGenCpmMax;
    const SimTime T_GenCpmDcc = mDccRestriction ? genCpmDcc() : T_GenCpmMin;
    const SimTime T_elapsed = T_now - mLastCpmTimestamp;
    if (T_elapsed >= T_GenCpmDcc && T_now >= 1) {
        if (mFixedRate) {
            sendCpm(T_now);
        } else if (checkHeadingDelta() || checkPositionDelta() || checkSpeedDelta()) {
            sendCpm(T_now);
            T_GenCpm = std::min(T_elapsed, T_GenCpmMax); /*< if middleware update interval is too long */
            mGenCpmLowDynamicsCounter = 0;
        } else if (T_elapsed >= T_GenCpm) {
            sendCpm(T_now);
            if (++mGenCpmLowDynamicsCounter >= mGenCpmLowDynamicsLimit) {
                T_GenCpm = T_GenCpmMax;
            }
        }
    }
}

bool CpService::checkHeadingDelta() const
{
    return !vanetza::facilities::similar_heading(mLastCpmHeading, mVehicleDataProvider->heading(), mHeadingDelta);
}

bool CpService::checkPositionDelta() const
{
    return (distance(mLastCpmPosition, mVehicleDataProvider->position()) > mPositionDelta);
}

bool CpService::checkSpeedDelta() const
{
    return abs(mLastCpmSpeed - mVehicleDataProvider->speed()) > mSpeedDelta;
}

void CpService::sendCpm(const SimTime& T_now)
{    
    if (tracks_map.size() == 0) {
        return;
    }
    uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
    auto cpm = createCollectivePerceptionMessage(
        *mVehicleDataProvider, genDeltaTimeMod, *mVehicleController, uss_setups, detections_map, tracks_map, *detections, cur_pos_center);
    mLastCpmPosition = mVehicleDataProvider->position();
    mLastCpmSpeed = mVehicleDataProvider->speed();
    mLastCpmHeading = mVehicleDataProvider->heading();
    mLastCpmTimestamp = T_now;

    using namespace vanetza;
    btp::DataRequestB request;
    request.destination_port = btp::ports::CPM;
    request.gn.its_aid = aid::CP;
    request.gn.transport_type = geonet::TransportType::SHB;
    request.gn.maximum_lifetime = geonet::Lifetime{geonet::Lifetime::Base::One_Second, 1};
    request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
    request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;
    CpObject obj(std::move(cpm));
    emit(scSignalCpmSent, &obj);
    using CpmByteBuffer = convertible::byte_buffer_impl<asn1::Cpm>;
    std::unique_ptr<geonet::DownPacket> payload{new geonet::DownPacket()};
    std::unique_ptr<convertible::byte_buffer> buffer{new CpmByteBuffer(obj.shared_ptr())};
    payload->layer(OsiLayer::Application) = std::move(buffer);
    this->request(request, std::move(payload));
}


SimTime CpService::genCpmDcc()
{
    // network interface may not be ready yet during initialization, so look it up at this later point
    auto netifc = mNetworkInterfaceTable->select(mPrimaryChannel);
    vanetza::dcc::TransmitRateThrottle* trc = netifc ? netifc->getDccEntity().getTransmitRateThrottle() : nullptr;
    if (!trc) {
        throw cRuntimeError("No DCC TRC found for CP's primary channel %i", mPrimaryChannel);
    }

    static const vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP2, 0);
    vanetza::Clock::duration interval = trc->interval(ca_tx);
    SimTime dcc{std::chrono::duration_cast<std::chrono::milliseconds>(interval).count(), SIMTIME_MS};
    return std::min(mGenCpmMax, std::max(mGenCpmMin, dcc));
}

vanetza::asn1::Cpm createCollectivePerceptionMessage(
    const VehicleDataProvider& vdp, uint16_t genDeltaTime, const VehicleController& vc, std::map<int, uss_setup> us, std::map<int, uss_value> dm, std::map<int, track_info> tm,
    omnetpp::cGroupFigure& det, omnetpp::cFigure::Point cpc)
{
    vanetza::asn1::Cpm message;

    ItsPduHeader_t& header = (*message).header;
    header.protocolVersion = 2;
    header.messageID = ItsPduHeader__messageID_cpm;
    header.stationID = vdp.station_id();

    CollectivePerceptionMessage_t& cpm = (*message).cpm;
    cpm.generationDeltaTime = genDeltaTime * GenerationDeltaTime_oneMilliSec;

    // CPM CONTAINER -> (Station Data | Cpm Management | Sensor Information |Perceived Object | Free Space Addendum)

    // Station Data Container (Done)
    cpm.cpmParameters.stationDataContainer = vanetza::asn1::allocate<StationDataContainer_t>();
    StationDataContainer_t* station = cpm.cpmParameters.stationDataContainer;
    station->present = StationDataContainer_PR_originatingVehicleContainer;
    OriginatingVehicleContainer_t& ovc = station->choice.originatingVehicleContainer;
    ovc.heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
    ovc.heading.headingValue = round(vdp.heading(), decidegree);
    ovc.speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3;
    ovc.speed.speedValue = buildSpeedValue(vdp.speed());
    ovc.vehicleLength = new VehicleLength_t();
    ovc.vehicleLength->vehicleLengthValue = vc.getLength().value() * 100;
    ovc.vehicleLength->vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_unavailable;

    // Cpm Management Container (Done)
    CpmManagementContainer_t& management = cpm.cpmParameters.managementContainer;
    management.stationType = StationType_passengerCar;
    management.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
    management.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;

    management.referencePosition.latitude = round(vdp.latitude(), microdegree) * Latitude_oneMicrodegreeNorth;
    management.referencePosition.longitude = round(vdp.longitude(), microdegree) * Longitude_oneMicrodegreeEast;


    management.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
    management.referencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
    management.referencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;

    // Sensor Information Container (Done)
    cpm.cpmParameters.sensorInformationContainer = vanetza::asn1::allocate<SensorInformationContainer_t>();
    SensorInformationContainer_t* sic = cpm.cpmParameters.sensorInformationContainer;
    int index = 1;
    auto current_time = simTime();
    for (auto pair : tm) {
        
        auto key = pair.first;
        auto value = pair.second;
        uss_setup desired_config = us[value.sensor];
        auto si = vanetza::asn1::allocate<SensorInformation>();
        si->sensorID = value.sensor;
        si->type = SensorType_ultrasonic;
        si->detectionArea.present = DetectionArea_PR_vehicleSensor;


        VehicleSensor_t& vs = si->detectionArea.choice.vehicleSensor;
        vs.xSensorOffset = (XSensorOffset_t)(desired_config.x * 100);
        vs.ySensorOffset = (XSensorOffset_t)(desired_config.y * 100);

        if (vs.xSensorOffset > 0) {  // dont know why ("Describes the mounting position of a sensor along the negative x-direction from Reference Point
                                        // indicated by the refPointID")
            vs.xSensorOffset = -vs.xSensorOffset;
        }
        if (vs.ySensorOffset > 0) {
            vs.ySensorOffset = -vs.ySensorOffset;
        }

        auto vsp = vanetza::asn1::allocate<VehicleSensorProperties>();
        vsp->range = 5.5 * 10;  // 5.5
        vsp->horizontalOpeningAngleStart = (long)((desired_config.yaw - 30) * 10);
        vsp->horizontalOpeningAngleEnd = ((long)(desired_config.yaw + 30) * 10);

        vsp->horizontalOpeningAngleStart = 0;
        vsp->horizontalOpeningAngleEnd = 0;

        ASN_SEQUENCE_ADD(&vs.vehicleSensorPropertyList, vsp);
        ASN_SEQUENCE_ADD(sic, si);
    }

    // Perceived Object Container
    cpm.cpmParameters.perceivedObjectContainer = vanetza::asn1::allocate<PerceivedObjectContainer_t>();
    NumberOfPerceivedObjects_t& perceivedObjects = cpm.cpmParameters.numberOfPerceivedObjects;
    perceivedObjects = (long)tm.size();
    // EV << "perceivedObjects: " << perceivedObjects << endl;


    for (auto pair : tm) {
        auto key = pair.first;
        auto value = pair.second;
        uss_setup desired_config = us[value.sensor];

        auto po = vanetza::asn1::allocate<PerceivedObject>();
        po->objectID = key;
        po->timeOfMeasurement = (long)(current_time.dbl() - value.timestamp); 
        // po->timeOfMeasurement = 0; 
        // EV << "timeOfMeasurement: " << po->timeOfMeasurement << endl;
        po->xDistance.value = (value.detection_coord.x - cpc.x) * 100;
        // EV << "xDistance: " << po->xDistance.value << endl;
        po->yDistance.value = (value.detection_coord.y - cpc.y) * 100;
        // EV << "yDistance: " << po->yDistance.value << endl;

        DistanceConfidence_t conf = std::round(std::sqrt(2 * std::pow(dm[value.sensor].depth, 2) * (1 - std::cos(PI / 12))) * 100);
        if (conf > 100) {
            po->xDistance.confidence = DistanceConfidence_outOfRange;
            po->yDistance.confidence = DistanceConfidence_outOfRange;
        } else {
            po->xDistance.confidence = conf;
            // EV << "xDistance.confidence: " << po->xDistance.confidence << endl;
            po->yDistance.confidence = conf;
            // EV << "yDistance.confidence: " << po->yDistance.confidence << endl;
        }

        // po->xSpeed.value = (long)(value.speed * std::cos(value.direction) * 100);
        po->xSpeed.value = 0;
        po->xSpeed.confidence = SpeedConfidence_unavailable;

        // po->ySpeed.value = (long)(value.speed * std::sin(value.direction) * 100);
        po->ySpeed.value = 0;
        po->ySpeed.confidence = SpeedConfidence_unavailable;

        // po->yawAngle->value = (long)((value.direction) * 1800/PI); //crasha não se bem porquê (usar round(?))
        // po->yawAngle->confidence = AngleConfidence_unavailable;

        EV << "Object " << po->objectID << ": " << endl <<
        "xDistance:" << po->xDistance.value << endl <<
        "yDistance:" << po->yDistance.value << endl <<
        "DistanceConfidence: " << conf << endl <<
        "xSpeed:" << po->xSpeed.value << endl <<
        "ySpeed:" << po->ySpeed.value << endl <<
        "direction: " << value.direction << endl <<
        "timeOfMeasurement: " << po->timeOfMeasurement << endl;

        ASN_SEQUENCE_ADD(cpm.cpmParameters.perceivedObjectContainer, po);
    }
    return message;
}

void CpService::receiveFromCarla()
{
    // set timeout of 400ms
    this->socket.setsockopt(ZMQ_RCVTIMEO, 400);

    zmq::message_t reply{};
    int sndhwm;
    size_t sndhwm_size = sizeof(sndhwm);
    int rc;
    std::string uk = "null";
    std::list<int> sensors_with_input;
    do {
        socket.recv(reply, zmq::recv_flags::none);
        rc = zmq_getsockopt(socket, ZMQ_RCVMORE, &sndhwm, &sndhwm_size);

        if (reply.to_string().empty()) {
            EV << "    msg empty! return!=" << endl;

            for (auto it = uss_setups.begin(); it != uss_setups.end(); ++it) {
                if ((std::find(sensors_with_input.begin(), sensors_with_input.end(), it->first) == sensors_with_input.end()) &&
                    detections_map.find(it->first) != detections_map.end()) {
                    cLineFigure* line1 = detections_map[it->first].line1;
                    cLineFigure* line2 = detections_map[it->first].line2;
                    cArcFigure* arc = detections_map[it->first].arc;
                    if (line1 != nullptr && line2 != nullptr) {
                        // detections->removeFigure(line1);
                        // detections->removeFigure(line2);
                        // detections->removeFigure(arc);
                    }
                    detections_map.erase(it->first);
                }
            }
            // return;
            break; 
            // quando lê tudo, acaba
        }
        // Check if the message starts with '{' indicating a JSON message
        if (reply.to_string()[0] == '{') {
            // Try to parse message
            json jsonResp = json::parse(reply.to_string());
            std::string message_type = jsonResp["message_type"].get<std::string>();
            std::string vehicle_id = jsonResp["vehicle_id"].get<std::string>();
            std::string sensor_id = jsonResp["sensor_id"].get<std::string>();
            double initial_timestamp = jsonResp["initial_timestamp"].get<double>();

            uint16_t detection_delta_time = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
            ;
            // Print to console, based on sensor type
            if (message_type == "USS_DATA") {
                double depth = jsonResp["depth"].get<double>();
                int role_name = std::stoi(jsonResp["role_name"].get<std::string>());
                sensors_with_input.push_back(role_name);

                EV << "    USS-" << sensor_id << "<" << role_name << ">" << "@" << vehicle_id << " (t=" << initial_timestamp << ")  Depth=" << depth << "m"
                   << endl;
                if (depth <= 5.5)
                    storeDetections(role_name, initial_timestamp, detection_delta_time, std::round(depth * 1000) / 1000);  // storing the detections in the "detections_map"
            } else {
                EV << "    Unknown message type..." << endl;
            }
        } else {
            EV << "STRING=" << reply.to_string() << endl;
            EV << "RCV_MORE=" << sndhwm << endl;
        }
    } while (true);


}

void CpService::updateEveryTimestamp()
{
    vehicle_angle = mVehicleController->getHeading().radian();
    // From front-center-bumper to center (sumo reference system).
    cur_pos_center = omnetpp::cFigure::Point(
        mVehicleController->getPosition().x.value() - cos(vehicle_angle) * vehicle_length / 2,
        mVehicleController->getPosition().y.value() + sin(vehicle_angle) * vehicle_length / 2);

    for (auto& pair : uss_setups) {
        pair.second.norm_sensor_angle = vehicle_angle - atan2(pair.second.y, pair.second.x) + PI;
        while (pair.second.norm_sensor_angle < -PI)
            pair.second.norm_sensor_angle += 2 * PI;
        while (pair.second.norm_sensor_angle >= PI)
            pair.second.norm_sensor_angle -= 2 * PI;

        // pair.second.sensor_dist = sqrt(pow(abs(pair.second.x), 2) + pow(abs(pair.second.y), 2));
        pair.second.sensor_pos = omnetpp::cFigure::Point(
            -cos(pair.second.norm_sensor_angle) * pair.second.sensor_dist, sin(pair.second.norm_sensor_angle) * pair.second.sensor_dist);

        pair.second.norm_detection_angle = vehicle_angle - (pair.second.yaw * (PI / 180)) + PI;
        while (pair.second.norm_detection_angle < -PI)
            pair.second.norm_detection_angle += 2 * PI;
        while (pair.second.norm_detection_angle >= PI)
            pair.second.norm_detection_angle -= 2 * PI;
    }
    for (int i = 0; i < rings->getNumFigures(); i++)
        rings->removeFigure(i);


    auto timestamp = simTime();

    // Remove tracks older than 2 seconds (100 timestamps ago)
    for (auto pair : tracks_map) {
        auto key = pair.first;
        auto value = pair.second;
        // Check if the track's timestamp is older than 1 seconds and the track is still in the active_tracks list
        if (timestamp - value.timestamp > 1 && std::find(active_tracks.begin(), active_tracks.end(), key) != active_tracks.end()) {
            EV << "ENTROU NO REMOVE OLD TRACKS!" << endl;
            // Remove the track from `active_tracks`
            active_tracks.erase(std::remove(active_tracks.begin(), active_tracks.end(), key), active_tracks.end());
            // Check if the track is also in `past_active_tracks`
            if (std::find(past_active_tracks.begin(), past_active_tracks.end(), key) != past_active_tracks.end()) {
                // If the track is found, remove it from `past_active_tracks` as well
                past_active_tracks.erase(std::remove(past_active_tracks.begin(), past_active_tracks.end(), key), past_active_tracks.end());
            }
        }
    }
}

void CpService::storeDetections(int role_name, double timestamp, uint16_t detection_delta_time, double depth)
{
    // pos of the detection in referance to the sensor
    omnetpp::cFigure::Point detection_pos =
        omnetpp::cFigure::Point(-cos(uss_setups[role_name].norm_detection_angle) * depth, sin(uss_setups[role_name].norm_detection_angle) * depth);
    // exact artery coord of the detection
    omnetpp::cFigure::Point coord = cur_pos_center + uss_setups[role_name].sensor_pos + detection_pos;
    
    uss_setup desired_config = uss_setups[role_name];
    
    detections_map[role_name].depth = depth;
    detections_map[role_name].timestamp = timestamp;
    detections_map[role_name].detection_coord = coord;
    detections_map[role_name].detection_delta_time = detection_delta_time;

    // DRAW DETECTION
    auto cross_line1 = new cLineFigure();
    cross_line1->setStart(omnetpp::cFigure::Point(coord.x + 0.1, coord.y + 0.1));
    cross_line1->setEnd(omnetpp::cFigure::Point(coord.x - 0.1, coord.y - 0.1));
    cross_line1->setLineColor("GREEN");
    cross_line1->setZIndex(3);
    //detections->addFigure(cross_line1);

    auto cross_line2 = new cLineFigure();
    cross_line2->setStart(omnetpp::cFigure::Point(coord.x + 0.1, coord.y - 0.1));
    cross_line2->setEnd(omnetpp::cFigure::Point(coord.x - 0.1, coord.y + 0.1));
    cross_line2->setLineColor("GREEN");
    cross_line2->setZIndex(3);
    //detections->addFigure(cross_line2);

    auto arc = new cArcFigure();
    arc->setBounds(cFigure::Rectangle(cur_pos_center.x + desired_config.sensor_pos.x, cur_pos_center.y + desired_config.sensor_pos.y, 2 * depth, 2 * depth));
    arc->setPosition(cur_pos_center + desired_config.sensor_pos, cFigure::ANCHOR_CENTER);
    arc->setStartAngle(desired_config.norm_detection_angle - 30 * PI / 180 + PI);
    arc->setEndAngle(desired_config.norm_detection_angle + 30 * PI / 180 + PI);
    // detections->addFigure(arc);

    detections_map[role_name].line1 = cross_line1;
    detections_map[role_name].line2 = cross_line2;
    detections_map[role_name].arc = arc;
}

void CpService::processDetections(){
    bool skip_next = false;

    std::ostringstream oss;
    const traci::Boundary boundary{mVehicleController->getTraCI()->simulation.getNetBoundary()};
    traci::TraCIPosition traCIPos;
    artery::Position arteryPos;

    for (auto pair : detections_map){
        if(skip_next){
            skip_next = false;
            continue;
        }
        auto key = pair.first;
        auto value = pair.second;
        object_info oi;
        DistanceConfidence_t conf = std::round(std::sqrt(2 * std::pow(value.depth, 2) * (1 - std::cos(PI / 12))) * 100);
        
        //if (conf <= 100){

        auto assign_to_map = [&](int target_key, const auto& source) {
            oi.detection_delta_time = source.detection_delta_time;
            oi.timestamp = source.timestamp;
            oi.detection_coord = source.detection_coord;
            objects_map[target_key] = oi;
        };

        if (key != 5 && detections_map.find(key + 1) != detections_map.end() && tracks_map.size() < detections_map.size()) {
            auto& next_detection = detections_map[key + 1];
            if(value.depth > next_detection.depth){
                assign_to_map(key + 1, next_detection);
            }
            skip_next = true; // Skip processing the next key
        }else{
            assign_to_map(key, value);
        }

            // WRITE INFO TO FILE
            oss.str("");            // Clear the content
            oss.clear();            // Reset any error flags
            oss << mVehicleController->getVehicleId() << "_detection_pos.txt";
            detection_file.open(oss.str().c_str(), std::ios_base::app);
            // arteryPos = Position(oi.detection_coord.x, oi.detection_coord.y);
            // traCIPos = position_cast(boundary,arteryPos);

            detection_file << mVehicleController->getVehicleId() << "," << oi.timestamp << "," << oi.detection_coord.x << ","
                            << oi.detection_coord.y << "," << tracks_count << endl;
                            
            if(detection_file.is_open()){
                EV << mVehicleController->getVehicleId() << "," << key  << "," << oi.timestamp << "," << oi.detection_coord.x << ","
                            << oi.detection_coord.y << "," <<  tracks_count << endl;
            }
            detection_file.close();
    }
    // DRAW DETECTION
    for (auto pair : objects_map){
        auto key = pair.first;
        auto value = pair.second;
        uss_setup desired_config = uss_setups[key];

        auto coord = value.detection_coord;
        auto cross_line1 = new cLineFigure();
        cross_line1->setStart(omnetpp::cFigure::Point(coord.x + 0.1, coord.y + 0.1));
        cross_line1->setEnd(omnetpp::cFigure::Point(coord.x - 0.1, coord.y - 0.1));
        cross_line1->setLineColor("GREEN");
        cross_line1->setZIndex(3);
        //detections->addFigure(cross_line1);

        auto cross_line2 = new cLineFigure();
        cross_line2->setStart(omnetpp::cFigure::Point(coord.x + 0.1, coord.y - 0.1));
        cross_line2->setEnd(omnetpp::cFigure::Point(coord.x - 0.1, coord.y + 0.1));
        cross_line2->setLineColor("GREEN");
        cross_line2->setZIndex(3);
        //detections->addFigure(cross_line2);

        // auto arc = new cArcFigure();
        // arc->setBounds(cFigure::Rectangle(cur_pos_center.x + desired_config.sensor_pos.x, cur_pos_center.y + desired_config.sensor_pos.y, 2 * depth, 2 * depth));
        // arc->setPosition(cur_pos_center + desired_config.sensor_pos, cFigure::ANCHOR_CENTER);
        // arc->setStartAngle(desired_config.norm_detection_angle - 30 * PI / 180 + PI);
        // arc->setEndAngle(desired_config.norm_detection_angle + 30 * PI / 180 + PI);
        // detections->addFigure(arc);
    }
}

void CpService::multiObjectTracking()
{
    if (active_tracks.empty()) { // if there are no active tracks, all the objects will become their own track 
        for (const auto& dm : objects_map) {
            Identifier_t id = 0;
            std::random_device rd;
            std::mt19937 gen(rd());
            do {
                std::uniform_int_distribution<> distrib(0, 255);
                id = distrib(gen);
            } while ((std::find(active_obj_ids.begin(), active_obj_ids.end(), id) != active_obj_ids.end()));

            active_obj_ids.push_back(id);  // objectIDs in use
            auto key = dm.first;
            auto value = dm.second;
            tracks_map[id].detection_coord = value.detection_coord;
            tracks_map[id].timestamp = value.timestamp;
            tracks_map[id].sensor = key;
            tracks_map[id].speed = 0;
            tracks_map[id].direction = 0;

            active_tracks.push_back(id);
            kf_map[id] = KalmanFilter(0.05, 0, 0, 0, 0.3, 0.3);
            tracks_count++;
        }
        EV << "ACTIVE TRACKS ESTÁ VAZIO SUPOSTAMENTE!" << endl;
    } else {
    
        auto cost_matrix = generate_cost_matrix(0.3, 0.5); 
                // cost matrix:   
        //               detections:
        //               0    1    2
        //         0  [[0.8  0.3  0.9],          
        // tracks: 1   [0.3  0.9  0.9],
        //         2   [0.9  0.8  0.3]]
        std::vector<int> min_cost_indices(cost_matrix.size());

        // Loop through each row to find the minimum index for each row
        // for (size_t i = 0; i < cost_matrix.size(); ++i) {
        //     // Initialize min_value to the maximum possible value
        //     double min_value = std::numeric_limits<double>::infinity();
        //     int min_index = -1;
        //     // Find the minimum element in the row
        //     for (size_t j = 0; j < cost_matrix[i].size(); ++j) {
        //         if (cost_matrix[i][j] < min_value) {
        //             min_value = cost_matrix[i][j];
        //             min_index = j;
        //         }
        //     }
        //     // Store the index of the minimum value for the current row
        //     min_cost_indices[i] = min_index;
        // }
        // min_cost_indices: [1  0  2]
        
        // track 0 -> detection 1
        // track 1 -> detection 0
        // track 2 -> detection 2
        

        EV << "cost_matrix:" << endl;
        for (size_t i = 0; i < cost_matrix.size(); ++i){
             for (size_t j = 0; j < cost_matrix[i].size(); ++j) {
                EV << cost_matrix[i][j] << " ";
             }
             EV << endl;
        }
        EV << "min_cost_indices:" << endl;
        for (size_t i = 0; i < cost_matrix.size(); ++i){
            EV << min_cost_indices[i]<< " ";
        }
        EV << endl;

        std::vector<int> keys;
        for(auto pair : objects_map){
            keys.push_back(pair.first);
        }
        
        min_cost_indices = optimal_assignment(cost_matrix);
        if (active_tracks.size() == objects_map.size()) { 
            
            EV << "HÁ TANTOS OBJECTOS COMO TRACKS!" << endl;
            // each track has a new observation
            // if (min_cost_indices.size() != std::set<int>(min_cost_indices.begin(), min_cost_indices.end()).size()) {
                // min_cost_indices = optimal_assignment(cost_matrix);
            // }

            for (size_t i = 0; i < active_tracks.size(); ++i) {
                if(min_cost_indices[i] != -1){
                    omnetpp::cFigure::Point pq = objects_map[keys[min_cost_indices[i]]].detection_coord - tracks_map[active_tracks[i]].detection_coord;
                    double magPq = std::sqrt(std::pow(pq.x, 2) + std::pow(pq.y, 2));
                    double angl = std::acos(pq.x / magPq);
                    if (pq.y > 0)
                        angl = angl + PI;

                    tracks_map[active_tracks[i]].direction = angl;
                    tracks_map[active_tracks[i]].speed = std::sqrt(std::pow(tracks_map[active_tracks[i]].detection_coord.x - objects_map[keys[min_cost_indices[i]]].detection_coord.x, 2)
                    + std::pow(tracks_map[active_tracks[i]].detection_coord.y - objects_map[keys[min_cost_indices[i]]].detection_coord.y, 2))
                    / (objects_map[keys[min_cost_indices[i]]].timestamp-tracks_map[active_tracks[i]].timestamp);
                    // EV << "x1: " << tracks_map[active_tracks[i]].detection_coord.x << endl <<
                    // "x2: " << objects_map[keys[min_cost_indices[i]]].detection_coord.x << endl <<
                    // "y1: " << tracks_map[active_tracks[i]].detection_coord.y << endl <<
                    // "y2: " << objects_map[keys[min_cost_indices[i]]].detection_coord.y << endl <<
                    // "SPEED CALCULATED: " << tracks_map[active_tracks[i]].speed << endl <<
                    // "ANGL CALCULATED: " << angl << endl;
                    tracks_map[active_tracks[i]].detection_coord = objects_map[keys[min_cost_indices[i]]].detection_coord;
                    tracks_map[active_tracks[i]].timestamp = objects_map[keys[min_cost_indices[i]]].timestamp;
                    tracks_map[active_tracks[i]].sensor = keys[min_cost_indices[i]];
                }
                
            }
        } else if (active_tracks.size() < objects_map.size()) { 
            // there are more observations than tracks so all the track have a new observation and some new tracks are created 
            EV << "HÁ MAIS OBJECTOS DO QUE TRACKS!" << endl;
            for (size_t i = 0; i < active_tracks.size(); ++i) {
                if(min_cost_indices[i] != -1){
                    omnetpp::cFigure::Point pq = objects_map[keys[min_cost_indices[i]]].detection_coord - tracks_map[active_tracks[i]].detection_coord;
                    double magPq = std::sqrt(std::pow(pq.x, 2) + std::pow(pq.y, 2));
                    double angl = std::acos(pq.x / magPq);
                    if (pq.y > 0)
                        angl = angl + PI;

                    tracks_map[active_tracks[i]].direction = angl;
                    tracks_map[active_tracks[i]].speed = std::sqrt(std::pow(tracks_map[active_tracks[i]].detection_coord.x - objects_map[keys[min_cost_indices[i]]].detection_coord.x, 2)
                    + std::pow(tracks_map[active_tracks[i]].detection_coord.y - objects_map[keys[min_cost_indices[i]]].detection_coord.y, 2))
                    / (objects_map[keys[min_cost_indices[i]]].timestamp-tracks_map[active_tracks[i]].timestamp);
                    // EV << "x1: " << tracks_map[active_tracks[i]].detection_coord.x << endl <<
                    // "x2: " << objects_map[keys[min_cost_indices[i]]].detection_coord.x << endl <<
                    // "y1: " << tracks_map[active_tracks[i]].detection_coord.y << endl <<
                    // "y2: " << objects_map[keys[min_cost_indices[i]]].detection_coord.y << endl <<
                    // "SPEED CALCULATED: " << tracks_map[active_tracks[i]].speed << endl <<
                    // "ANGL CALCULATED: " << angl << endl;
                    tracks_map[active_tracks[i]].detection_coord = objects_map[keys[min_cost_indices[i]]].detection_coord;
                    tracks_map[active_tracks[i]].timestamp = objects_map[keys[min_cost_indices[i]]].timestamp;
                    tracks_map[active_tracks[i]].sensor = keys[min_cost_indices[i]];
                    
                    keys.erase(std::remove(keys.begin(), keys.end(), min_cost_indices[i]), keys.end());
                }
                
            }

            for (size_t i = 0; i < objects_map.size() - active_tracks.size(); ++i) {

                Identifier_t id = 0;
                std::random_device rd;
                std::mt19937 gen(rd());
                do {
                    std::uniform_int_distribution<> distrib(0, 255);
                    id = distrib(gen);
                } while ((std::find(active_obj_ids.begin(), active_obj_ids.end(), id) != active_obj_ids.end()));

                active_obj_ids.push_back(id);  // objectIDs in use

                tracks_map[id].detection_coord = objects_map[keys[i]].detection_coord;
                tracks_map[id].timestamp = objects_map[keys[i]].timestamp;
                tracks_map[id].sensor = keys[i];
                tracks_map[id].speed = 0;
                tracks_map[id].direction = 0;

                active_tracks.push_back(id);
                kf_map[id] = KalmanFilter(0.05, 0, 0, 0, 0.3, 0.3);
                tracks_count++;
            }
        } else {
            // there are more tracks than observations so some tracks wont be "refreshed"
            EV << "HÁ MAIS TRACKS DO QUE OBJECTOS!" << endl;
            double min_cost = 0;
            int tracks_aux = -1;
            int pos_aux = -1;

            for (size_t j = 0; j < objects_map.size(); ++j) {
                min_cost = cost_matrix[0][j];
                pos_aux = j;
                tracks_aux = 0;
                for (size_t i = 0; i < active_tracks.size(); ++i) {
                    if (cost_matrix[i][j] < min_cost) {
                        min_cost = cost_matrix[i][j];
                        tracks_aux = i;
                    }
                }
                
                omnetpp::cFigure::Point pq = objects_map[keys[pos_aux]].detection_coord - tracks_map[active_tracks[tracks_aux]].detection_coord;
                double magPq = std::sqrt(std::pow(pq.x, 2) + std::pow(pq.y, 2));
                double angl = std::acos(pq.x / magPq);
                if (pq.y > 0)
                    angl = angl + PI;

                tracks_map[active_tracks[tracks_aux]].direction = angl;
                tracks_map[active_tracks[tracks_aux]].speed = std::sqrt(std::pow(tracks_map[active_tracks[tracks_aux]].detection_coord.x - objects_map[keys[pos_aux]].detection_coord.x, 2) 
                + std::pow(tracks_map[active_tracks[tracks_aux]].detection_coord.y - objects_map[keys[pos_aux]].detection_coord.y, 2))
                / ((objects_map[keys[pos_aux]].timestamp-tracks_map[active_tracks[tracks_aux]].timestamp));
                // EV << "x1: " << tracks_map[active_tracks[tracks_aux]].detection_coord.x << endl <<
                // "x2: " << objects_map[keys[pos_aux]].detection_coord.x << endl <<
                // "y1: " << tracks_map[active_tracks[tracks_aux]].detection_coord.y << endl <<
                // "y2: " << objects_map[keys[pos_aux]].detection_coord.y << endl <<
                // "SPEED CALCULATED: " << tracks_map[active_tracks[tracks_aux]].speed << endl <<
                // "ANGL CALCULATED: " << angl << endl;
                tracks_map[active_tracks[tracks_aux]].detection_coord = objects_map[keys[pos_aux]].detection_coord;
                tracks_map[active_tracks[tracks_aux]].timestamp = objects_map[keys[pos_aux]].timestamp;
                tracks_map[active_tracks[tracks_aux]].sensor = keys[pos_aux];
            }
        }
    }

    std::vector<int> remove_tracks;

    for (const auto& at : active_tracks) {
        auto prediction = kf_map[at].predict();
        if((std::sqrt(std::pow(prediction[0] - tracks_map[at].detection_coord.x, 2) + std::pow(prediction[1] - tracks_map[at].detection_coord.y, 2))) >= 1){
            tracks_map[at].count_reset = tracks_map[at].count_reset + 1;
        }
        else{
            tracks_map[at].count_reset = 0;
        }
        
        if(tracks_map[at].count_reset >= 7){ //  0.2 segundo / 0,05 T = 4 timestamps
            cLineFigure* line1 = tracks_map[at].cross_line1;
            cLineFigure* line2 = tracks_map[at].cross_line2;
            if (line1 != nullptr && line2 != nullptr) {
                detections->removeFigure(tracks_map[at].cross_line1);
                detections->removeFigure(tracks_map[at].cross_line2);
            }
            tracks_map.erase(at);
            kf_map.erase(at);
            remove_tracks.push_back(at);
        }else{
            kf_map[at].update({tracks_map[at].detection_coord.x, tracks_map[at].detection_coord.y});
            auto updated = kf_map[at].getPosition();
            tracks_map[at].detection_coord = omnetpp::cFigure::Point(updated(0), updated(1));

            cLineFigure* line1 = tracks_map[at].cross_line1;
            cLineFigure* line2 = tracks_map[at].cross_line2;
            if (line1 != nullptr && line2 != nullptr) {
                detections->removeFigure(tracks_map[at].cross_line1);
                detections->removeFigure(tracks_map[at].cross_line2);
            }
            // DRAW DETECTION
            auto coord = tracks_map[at].detection_coord;
            auto cross_line1 = new cLineFigure();
            auto cross_line2 = new cLineFigure();
            cross_line1->setStart(omnetpp::cFigure::Point(coord.x + 0.1, coord.y + 0.1));
            cross_line1->setEnd(omnetpp::cFigure::Point(coord.x - 0.1, coord.y - 0.1));
            cross_line2->setStart(omnetpp::cFigure::Point(coord.x + 0.1, coord.y - 0.1));
            cross_line2->setEnd(omnetpp::cFigure::Point(coord.x - 0.1, coord.y + 0.1));
            cross_line1->setZIndex(3);
            cross_line2->setZIndex(3);
            if(at % 2 == 0){
                cross_line1->setLineColor("RED");
                cross_line2->setLineColor("RED");
            }else{
                cross_line1->setLineColor("BLUE");
                cross_line2->setLineColor("BLUE");
            }
            detections->addFigure(cross_line1);
            detections->addFigure(cross_line2);
            
            tracks_map[at].cross_line1 = cross_line1;
            tracks_map[at].cross_line2 = cross_line2;
        }
    }
    for (auto at : remove_tracks) {
        EV << "REMOVEU A TRACK " << at << endl;
        active_tracks.erase(std::remove(active_tracks.begin(), active_tracks.end(), at), active_tracks.end());
        cLineFigure* line1 = tracks_map[at].cross_line1;
        cLineFigure* line2 = tracks_map[at].cross_line2;
        if (line1 != nullptr && line2 != nullptr) {
            detections->removeFigure(tracks_map[at].cross_line1);
            detections->removeFigure(tracks_map[at].cross_line2);
        }
    }
    past_tracks_map = tracks_map;
    past_active_tracks = active_tracks;
}

std::vector<std::vector<double>> CpService::generate_cost_matrix(double distance_weight, double direction_weight) {

    size_t n = active_tracks.size();
    size_t m = objects_map.size();
    std::vector<std::vector<double>> cost_matrix(n, std::vector<double>(m, 0.0));

    for (size_t i = 0; i < n; ++i) {
        int j = 0;
        for (auto pair : objects_map) {
            auto key = pair.first;
            auto value = pair.second;

            auto pred_track_pos = tracks_map[active_tracks[i]];
            auto detection_pos = value.detection_coord;
            auto past_tracks_map_pos = past_tracks_map[past_active_tracks[i]];

            double dx = pred_track_pos.detection_coord.x - detection_pos.x;
            double dy = pred_track_pos.detection_coord.y - detection_pos.y;

            double distance = std::sqrt(dx * dx + dy * dy);
            double ang_dif = 0.0;

            if (!past_active_tracks.empty()) {
                double delta_x = past_tracks_map_pos.detection_coord.x - pred_track_pos.detection_coord.x;
                double delta_y = past_tracks_map_pos.detection_coord.y - pred_track_pos.detection_coord.y;

                double angle1 = std::atan2(delta_y, delta_x);

                delta_x = pred_track_pos.detection_coord.x - detection_pos.x;
                delta_y = pred_track_pos.detection_coord.y - detection_pos.y;

                double angle2 = std::atan2(delta_y, delta_x);

                ang_dif = std::abs(angle1 - angle2);
            }

            cost_matrix[i][j] = std::round(distance * 100.0) / 100.0 * distance_weight + std::round(ang_dif * 100.0) / 100.0 * direction_weight;
            j++;
        }
    }
    return cost_matrix;
}

// std::vector<int> CpService::optimal_assignment(const std::vector<std::vector<double>>& cost_matrix) {
//     size_t num_tracks = cost_matrix.size();
//     std::vector<int> indices(num_tracks);
//     std::iota(indices.begin(), indices.end(), 0);
//     double best_cost = std::numeric_limits<double>::infinity();
//     std::vector<int> best_assignment;

//     do {
//         double total_cost = 0.0;
//         for (size_t i = 0; i < num_tracks; ++i) {
//             total_cost += cost_matrix[i][indices[i]];
//         }

//         if (total_cost < best_cost) {
//             best_cost = total_cost;
//             best_assignment = indices;
//         }
//     } while (std::next_permutation(indices.begin(), indices.end()));

//     return best_assignment;
// }

std::vector<int> CpService::optimal_assignment(const std::vector<std::vector<double>>& cost_matrix) {
    const double UNASSIGNED_COST = 1e6; // High cost for unassigned tracks

    size_t num_tracks = cost_matrix.size();
    size_t num_detections = cost_matrix[0].size();

    // Augment the cost matrix with dummy assignments for unassigned tracks
    std::vector<std::vector<double>> augmented_matrix = cost_matrix;
    for (auto& row : augmented_matrix) {
        row.push_back(UNASSIGNED_COST); // Add dummy column
    }

    // Adjust the size for permutations
    size_t num_assignments = augmented_matrix[0].size();
    std::vector<int> indices(num_assignments);
    std::iota(indices.begin(), indices.end(), 0);

    double best_cost = std::numeric_limits<double>::infinity();
    std::vector<int> best_assignment(num_tracks, -1);

    do {
        double total_cost = 0.0;
        bool valid = true;

        // Compute the total cost for the current permutation
        for (size_t i = 0; i < num_tracks; ++i) {
            int assignment = indices[i];
            if (assignment < num_detections) {
                total_cost += augmented_matrix[i][assignment];
            } else if (assignment == num_detections) {
                // Track is unassigned
                total_cost += UNASSIGNED_COST;
            } else {
                valid = false;
                break;
            }
        }

        if (valid && total_cost < best_cost) {
            best_cost = total_cost;
            best_assignment = indices;
        }
    } while (std::next_permutation(indices.begin(), indices.end()));

    // Map assignments back to the original detection set (-1 indicates unassigned)
    for (size_t i = 0; i < best_assignment.size(); ++i) {
        if (best_assignment[i] >= num_detections) {
            best_assignment[i] = -1; // Unassigned
        }
    }

    return best_assignment;
}

// int MyVeinsRSUApp::pointInPolygon (int nvert, Coord verts[], Coord point)
// {
//     int i, j, c = 0;

//     for (i = 0, j = nvert-1; i < nvert; j = i++) {
//         if ( ((verts[i].y>point.y) != (verts[j].y>point.y)) && (point.x < (verts[j].x-verts[i].x) * (point.y-verts[i].y) / (verts[j].y-verts[i].y) + verts[i].x) )
//            c = !c;
//     }
//     return c;
// }

}  // namespace artery
