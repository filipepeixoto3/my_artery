/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2019 Raphael Riebl et al.
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/application/CpService.h"

#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/CpObject.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/VehicleKinematics.h"
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
    EV << "Length: " << vehicle_length << endl << "Width: " << vehicle_width << endl;
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

    EV << "Entrou!" << endl;
    // Connect to ZMQ
    this->context = zmq::context_t{1};
    this->socket = zmq::socket_t{context, ZMQ_SUB};

    this->socket.setsockopt(ZMQ_RCVTIMEO, 4000);  // set timeout to 4 seconds
    this->socket.setsockopt(ZMQ_SNDTIMEO, 4000);  // set timeout to 4 seconds

    // EV << "Will subscribe to: " << topic << endl;
    zmq_setsockopt(this->socket, ZMQ_SUBSCRIBE, topic.c_str(), topic.length());

    int port = par("port").intValue();
    std::string addr = "tcp://localhost:" + std::to_string(port);
    // EV << "Trying connecting to: " << addr << endl;
    this->socket.connect(addr);

    EV_INFO << "Connect to ZMQ succesful!" << endl;

    // remove(boost::lexical_cast<std::string>(mVehicleDataProvider->getStationId()) + "_detection_pos.txt");
    std::ostringstream oss;
    oss << mVehicleController->getVehicleId() << "_detection_pos.txt";
    remove(oss.str().c_str());
    detection_file.open(oss.str().c_str(), std::ios_base::app);
    detection_file << "id,timestamp,x,y,confidence" << endl;
    detection_file.close();
}


void CpService::trigger()
{
    Enter_Method("trigger");
    updateEveryTimestamp();
    receiveFromCarla();                    // checks if there is content in the socket connected to CARLA and if so the data is processed
    checkTriggeringConditions(simTime());  // checks if coditions to create and a send a CPM are meet
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

        //     Position cur_pos_center = omnetpp::cFigure::Point(
        // pos.x - (cos((double)msg->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.heading/100) *
        // (((double)msg->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.vehicleLength/100) / 2), pos.y +
        // (sin((double)msg->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.heading/100) *
        // (((double)msg->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.vehicleLength/100) / 2));

        EV << "(indicate)" << endl;
        EV << "parachoques:" << endl << "x: " << pos.x.value() << endl << "y: " << pos.y.value() << endl;
        EV << "cur_pos_center:" << endl << "x: " << cpc.x << endl << "y: " << cpc.y << endl;
        EV << "Vehicle_length: "
           << ((double)msg->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.vehicleLength->vehicleLengthValue / 100) << endl;
        EV << "Vehicle_heading: " << ((double)msg->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.heading.headingValue * PI / 1800)
           << endl;


        // DRAW THE DETECTIONS RECEIVED IN THE CPM
        for (int i = 0; i < msg->cpm.cpmParameters.perceivedObjectContainer->list.count; i++) {
            PerceivedObject* object = msg->cpm.cpmParameters.perceivedObjectContainer->list.array[i];
            EV << "xDistance:" << (double)object->xDistance.value / 100 << endl << "yDistance: " << (double)object->yDistance.value / 100 << endl;


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
            detections->addFigure(cross_line1);
            auto cross_line2 = new cLineFigure();
            cross_line2->setStart(b);
            cross_line2->setEnd(d);
            cross_line2->setLineColor("GREEN");
            detections->addFigure(cross_line2);
        }

        for (int i = 0; i < msg->cpm.cpmParameters.sensorInformationContainer->list.count; i++) {
            SensorInformation* si = msg->cpm.cpmParameters.sensorInformationContainer->list.array[i];
            EV << "sensorId: " << si->sensorID << endl << "type: " << si->type << endl;
            EV << "refPointId: " << si->detectionArea.choice.vehicleSensor.refPointId << endl;
            EV << "xSensorOffset: " << si->detectionArea.choice.vehicleSensor.xSensorOffset << endl;
            EV << "ySensorOffset: " << si->detectionArea.choice.vehicleSensor.ySensorOffset << endl;
            EV << "range: " << si->detectionArea.choice.vehicleSensor.vehicleSensorPropertyList.list.array[0]->range;
            EV << "horizontalOpeningAngleStart: "
               << si->detectionArea.choice.vehicleSensor.vehicleSensorPropertyList.list.array[0]->horizontalOpeningAngleStart;
            EV << "horizontalOpeningAngleEnd: " << si->detectionArea.choice.vehicleSensor.vehicleSensorPropertyList.list.array[0]->horizontalOpeningAngleEnd;
        }

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
    std::list<object_info> objects;
    determineObjects(objects);
    if (objects.empty() && T_now > 1) {
        return;
    }
    uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
    auto cpm = createCollectivePerceptionMessage(
        *mVehicleDataProvider, genDeltaTimeMod, *mVehicleController, uss_setups, detections_map, objects, *detections, cur_pos_center);
    objects.clear();
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
    const VehicleDataProvider& vdp, uint16_t genDeltaTime, const VehicleController& vc, std::map<int, uss_setup> us, std::map<int, uss_value> dm,
    std::list<object_info>& objects, omnetpp::cGroupFigure& det, omnetpp::cFigure::Point cpc)
{
    vanetza::asn1::Cpm message;

    ItsPduHeader_t& header = (*message).header;
    header.protocolVersion = 2;
    header.messageID = ItsPduHeader__messageID_cpm;
    header.stationID = vdp.station_id();

    CollectivePerceptionMessage_t& cpm = (*message).cpm;
    cpm.generationDeltaTime = genDeltaTime * GenerationDeltaTime_oneMilliSec;

    // CPM CONTAINER -> (Station Data | Cpm Management | Sensor Information |Perceived Object | Free Space Addendum)
    EV << "genDeltaTIme: " << genDeltaTime << endl;
    // Station Data Container (Done)
    cpm.cpmParameters.stationDataContainer = vanetza::asn1::allocate<StationDataContainer_t>();
    StationDataContainer_t* station = cpm.cpmParameters.stationDataContainer;
    station->present = StationDataContainer_PR_originatingVehicleContainer;
    OriginatingVehicleContainer_t& ovc = station->choice.originatingVehicleContainer;
    ovc.heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
    ovc.heading.headingValue = round(vdp.heading(), decidegree);
    ovc.speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3;
    // ovc.speed.speedValue = vc.getSpeed().value();
    ovc.speed.speedValue = buildSpeedValue(vdp.speed());
    ovc.vehicleLength = new VehicleLength_t();
    ovc.vehicleLength->vehicleLengthValue = vc.getLength().value() * 100;
    ovc.vehicleLength->vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_unavailable;
    // ovc.vehicleWidth = (long)vc.getLength().value()*100;

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

    for (auto it = objects.begin(); it != objects.end(); ++it) {
        for (int sensor : it->sensors) {
            auto value = us[sensor];
            auto si = vanetza::asn1::allocate<SensorInformation>();
            si->sensorID = index;
            index += 1;
            si->type = SensorType_ultrasonic;
            si->detectionArea.present = DetectionArea_PR_vehicleSensor;


            VehicleSensor_t& vs = si->detectionArea.choice.vehicleSensor;
            vs.xSensorOffset = (XSensorOffset_t)(value.x * 100);
            vs.ySensorOffset = (XSensorOffset_t)(value.y * 100);

            if (vs.xSensorOffset > 0) {  // dont know why ("Describes the mounting position of a sensor along the negative x-direction from Reference Point
                                         // indicated by the refPointID")
                vs.xSensorOffset = -vs.xSensorOffset;
            }
            if (vs.ySensorOffset > 0) {
                vs.ySensorOffset = -vs.ySensorOffset;
            }


            // vs.xSensorOffset = 0;
            // vs.ySensorOffset = 0;
            EV << "xSensorOffset(create): " << (long)(value.x * 100) << endl;
            EV << "ySensorOffset(create): " << (long)(value.y * 100) << endl;
            auto vsp = vanetza::asn1::allocate<VehicleSensorProperties>();
            vsp->range = 5.5 * 10;  // 5.5
            vsp->horizontalOpeningAngleStart = (long)((value.yaw - 30) * 10);
            vsp->horizontalOpeningAngleEnd = ((long)(value.yaw + 30) * 10);

            vsp->horizontalOpeningAngleStart = 0;
            vsp->horizontalOpeningAngleEnd = 0;

            ASN_SEQUENCE_ADD(&vs.vehicleSensorPropertyList, vsp);
            ASN_SEQUENCE_ADD(sic, si);
        }
    }


    // Perceived Object Container
    cpm.cpmParameters.perceivedObjectContainer = vanetza::asn1::allocate<PerceivedObjectContainer_t>();
    NumberOfPerceivedObjects_t& perceivedObjects = cpm.cpmParameters.numberOfPerceivedObjects;
    perceivedObjects = (long)objects.size();


    for (auto it = objects.begin(); it != objects.end(); ++it) {
        auto po = vanetza::asn1::allocate<PerceivedObject>();
        po->objectID = it->objectID;
        po->timeOfMeasurement = (long)(genDeltaTime - it->detection_delta_time);
        EV << "timeOfMeasurement: " << po->timeOfMeasurement << endl;
        po->xDistance.value = (it->detection_coords.x - cpc.x) * 100;
        po->yDistance.value = (it->detection_coords.y - cpc.y) * 100;

        DistanceConfidence_t conf = std::round(std::sqrt(2 * std::pow(it->depth, 2) * (1 - std::cos(PI / 12))) * 100);
        if (conf > 100) {
            po->xDistance.confidence = DistanceConfidence_outOfRange;
            po->yDistance.confidence = DistanceConfidence_outOfRange;
        } else {
            po->xDistance.confidence = conf;
            po->yDistance.confidence = conf;
        }

        po->xSpeed.value = (long)(it->speed * std::cos(it->direction));
        po->xSpeed.confidence = SpeedConfidence_unavailable;
        po->ySpeed.value = (long)(it->speed * std::sin(it->direction));
        po->ySpeed.confidence = SpeedConfidence_unavailable;
        // po->yawAngle->value = (long)(it->direction) * 1800/PI; //crasha não se bem porquê (usar round(?))
        // po->yawAngle->confidence = AngleConfidence_unavailable;

        // EV << "Object " << po->objectID << ": " << endl <<
        // "xDistance:" << po->xDistance.value << endl <<
        // "yDistance:" << po->yDistance.value << endl <<
        // "DistanceConfidence: " << value << endl <<
        // "xSpeed:" << po->xSpeed.value << endl <<
        // "ySpeed:" << po->ySpeed.value << endl <<
        // "direction: " << it->direction << endl;

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
            return;
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

                processDetections(
                    role_name, initial_timestamp, detection_delta_time, std::round(depth * 1000) / 1000);  // processing the received data from the USS

                // } else if (message_type == "GNSS_DATA") {
                //     double altitude = jsonResp["altitude"].get<double>();
                //     double latitude = jsonResp["latitude"].get<double>();
                //     double longitude = jsonResp["longitude"].get<double>();

                //     EV << "    GNSS-" << sensor_id << "@" << vehicle_id << " (t=" << initial_timestamp << ")  Altitude=" << altitude << "m"
                //        << ",  Latitude=" << latitude << "º" << ",  Longitude=" << longitude << "º" << endl;
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
}

void CpService::processDetections(int role_name, double initial_timestamp, uint16_t detection_delta_time, double depth)
{

    omnetpp::cFigure::Point detection_pos =
        omnetpp::cFigure::Point(-cos(uss_setups[role_name].norm_detection_angle) * depth, sin(uss_setups[role_name].norm_detection_angle) * depth);
    omnetpp::cFigure::Point coord = cur_pos_center + uss_setups[role_name].sensor_pos + detection_pos;


    // calcular aqui sensor_angle detection_angle sensor_dist

    if (detections_map.find(role_name) == detections_map.end()) {
        // if there is no map entry it means that this detection is the first registered
        coord = drawDetections(role_name, depth);  // draws the detection on the canvas

        if ((role_name != 6 && detections_map.find(role_name - 1) != detections_map.end()) && detections_map[role_name - 1].speed != 0.0) {
            // if there is movement in the previous sensor that means that there is already an
            // object being tracked  so we atribuit that object's id to the registered detection
            detections_map[role_name].objectID = detections_map[role_name - 1].objectID;
            detections_map[role_name].speed = detections_map[role_name - 1].speed;
            detections_map[role_name].direction = detections_map[role_name - 1].direction;
        } else if ((role_name != 5 && detections_map.find(role_name + 1) != detections_map.end()) && detections_map[role_name + 1].speed != 0.0) {
            // if there is movement in the following sensor that means that there is already an
            // object being tracked  so we atribuit that object's id to the registered detection
            detections_map[role_name].objectID = detections_map[role_name + 1].objectID;
            detections_map[role_name].speed = detections_map[role_name + 1].speed;
            detections_map[role_name].direction = detections_map[role_name + 1].direction;
        } else {
            // if there is no detection in the adjacent sensors, that means its the
            // first time this object has been detected so we atribuit it a new id
            Identifier_t id = 0;
            std::random_device rd;
            std::mt19937 gen(rd());
            do {
                std::uniform_int_distribution<> distrib(0, 255);
                id = distrib(gen);
            } while ((std::find(active_obj_ids.begin(), active_obj_ids.end(), id) != active_obj_ids.end()));

            active_obj_ids.push_back(id);  // objectIDs in use
            detections_map[role_name].objectID = id;
            detections_map[role_name].speed = 0.0;
            detections_map[role_name].direction = 0.0;
        }
        detections_map[role_name].last_timestamp = detections_map[role_name].initial_timestamp;
        detections_map[role_name].initial_timestamp = initial_timestamp;
        detections_map[role_name].detection_delta_time = detection_delta_time;

    } else if (static_object[role_name].x == coord.x && static_object[role_name].y == coord.y && detections_map[role_name].speed != 0) {
        // in this scenario, and object exists the FOV of the sensor and the detecitons
        // go back to being from a stationary object so we atribuit it a new objectID
        Identifier_t id = 0;
        std::random_device rd;
        std::mt19937 gen(rd());
        do {
            std::uniform_int_distribution<> distrib(0, 255);
            id = distrib(gen);
        } while ((std::find(active_obj_ids.begin(), active_obj_ids.end(), id) != active_obj_ids.end()));
        active_obj_ids.push_back(id);
        detections_map[role_name].objectID = id;
        detections_map[role_name].last_timestamp = detections_map[role_name].initial_timestamp;
        detections_map[role_name].initial_timestamp = std::round(initial_timestamp);
        detections_map[role_name].detection_delta_time = detection_delta_time;
        detections_map[role_name].speed = 0.0;
        detections_map[role_name].direction = 0.0;

    } else if (depth != detections_map[role_name].depth && initial_timestamp != detections_map[role_name].last_timestamp) {
        //} else if (detections_map[role_name].detection_coords.x != coord.x || detections_map[role_name].detection_coords.y != coord.y ) {
        // if the depth differs it means that we are currently following
        // an object so we updtate only the necessary atribuits
        repeted_detections[role_name] = 0;  // used to verify if the detections are from a stationary object
        EV << "Object " << detections_map[role_name].objectID << " moving!" << endl;
        // distance between the previous coord and the present one
        auto dist = std::round(
                        std::sqrt(
                            std::pow((detections_map[role_name].detection_coords.x - coord.x), 2) +
                            std::pow((detections_map[role_name].detection_coords.y - coord.y), 2)) *
                        1000) /
                    1000;

        detections_map[role_name].speed = dist / (initial_timestamp - detections_map[role_name].last_timestamp);

        EV << "PrecessDetection " << detections_map[role_name].objectID << ": " << endl
           << "detections_map[role_name].detection_coords.x: " << detections_map[role_name].detection_coords.x << endl
           << "detections_map[role_name].detection_coords.y: " << detections_map[role_name].detection_coords.y << endl
           << "coord.x: " << coord.x << endl
           << "coord.y: " << coord.y << endl
           << "detections_map[role_name].depth: " << detections_map[role_name].depth << endl
           << "depth: " << depth << endl
           << "speed: " << detections_map[role_name].speed << endl
           << "dist: " << dist << endl
           << "time: " << initial_timestamp - detections_map[role_name].last_timestamp << endl
           << "timestamp: " << initial_timestamp << endl;

        // calculation of the direction of the object (https://www.youtube.com/watch?v=aUsOB24W3lk&t=303s)
        omnetpp::cFigure::Point pq = coord - detections_map[role_name].detection_coords;
        double magPq = std::sqrt(std::pow(pq.x, 2) + std::pow(pq.y, 2));
        double angl = std::acos(pq.x / magPq);
        if (pq.y > 0)
            angl = angl + PI;
        detections_map[role_name].direction = angl;
        EV << "Direction: " << angl << endl;

        // erase the drawings from the previous detection
        cLineFigure* line1 = detections_map[role_name].line1;
        cLineFigure* line2 = detections_map[role_name].line2;
        // cArcFigure* arc = detections_map[role_name].arc;
        if (line1 != nullptr && line2 != nullptr) {
            // detections->removeFigure(line1);
            // detections->removeFigure(line2);
            // detections->removeFigure(arc);
        }

        coord = drawDetections(role_name, depth);  // draw new detection
        EV << "role name: " << role_name << endl;
        detections_map[role_name].last_timestamp = detections_map[role_name].initial_timestamp;
        detections_map[role_name].initial_timestamp = initial_timestamp;
        detections_map[role_name].detection_delta_time = detection_delta_time;
    } else {
        // in this scenario the object is not moving it could mean that de sensor is detecting
        // a stationary object or the moving object has stoped in the middle of its tajectory


        detections_map[role_name].last_timestamp = detections_map[role_name].initial_timestamp;
        detections_map[role_name].initial_timestamp = initial_timestamp;
        detections_map[role_name].detection_delta_time = detection_delta_time;
        detections_map[role_name].speed = 0.0;
        detections_map[role_name].direction = 0.0;
        repeted_detections[role_name] = repeted_detections[role_name] + 1;
        if (repeted_detections[role_name] > 100) {
            // if the sensor detects the same depth for 5 seconds the
            // object being detected is considered a stationary object
            static_object[role_name] = detections_map[role_name].detection_coords;
        }
    }
    fullPathToFile();
}
void CpService::determineObjects(std::list<object_info>& objects)
{
    for (auto it = detections_map.begin(); it != detections_map.end(); ++it) {
        auto key = it->first;
        auto value = it->second;
        if (value.speed != 0.0) {
            // if there is movement in the sensor a moving object is present
            object_info oi;
            if (key != 5 && key != 11 && detections_map.find(key + 1) != detections_map.end() && detections_map[key + 1].speed != 0) {
                // if the following sensor is also detecting a moving object it is most
                // likely the same so we do the triangulation to get a more precise position

                /*********************************************DRAWING RINGS*********************************************/
                // auto ring1 = new cRingFigure();
                // ring1->setPosition(cur_pos_center + uss_setups[it->first].sensor_pos, cFigure::ANCHOR_CENTER);
                // ring1->setInnerRadius(value.depth);
                // rings->addFigure(ring1);

                // auto ring2 = new cRingFigure();
                // ring2->setPosition(cur_pos_center + uss_setups[key + 1].sensor_pos, cFigure::ANCHOR_CENTER);
                // ring2->setInnerRadius(detections_map[key + 1].depth);
                // rings->addFigure(ring2);
                /*******************************************************************************************************/

                // omnetpp::cFigure::Point possible_position = findPosition(
                //     cur_pos_center, cur_pos_center + uss_setups[it->first].sensor_pos, cur_pos_center + uss_setups[key + 1].sensor_pos, value.depth,
                //     detections_map[key + 1].depth, value.detection_coords,
                //     detections_map[key + 1].detection_coords);  // triangulation
                //      oi.detection_coords = possible_position;

                if (value.depth < detections_map[key + 1].depth) {
                    oi.detection_coords = value.detection_coords;
                } else {
                    oi.detection_coords = detections_map[key + 1].detection_coords;
                }
                oi.initial_timestamp = value.initial_timestamp;
                oi.depth = value.depth;
                oi.detection_delta_time = value.detection_delta_time;
                oi.objectID = value.objectID;

                if ((oi.detection_coords.x != value.detection_coords.x || oi.detection_coords.y != value.detection_coords.y) &&
                    (oi.detection_coords.x != detections_map[key + 1].detection_coords.x ||
                     oi.detection_coords.y != detections_map[key + 1].detection_coords.y)) {

                    // EV << oi.detection_coords.x << " != " << value.detection_coords.x << endl;
                    // EV << oi.detection_coords.y << " != " << value.detection_coords.y << endl;
                    // EV << oi.detection_coords.x << " != " << detections_map[key + 1].detection_coords.x << endl;
                    // EV << oi.detection_coords.y << " != " << detections_map[key + 1].detection_coords.y << endl;

                    omnetpp::cFigure::Point pq = value.detection_coords - value.last_detection_coords;
                    double magPq = std::sqrt(std::pow(pq.x, 2) + std::pow(pq.y, 2));
                    double angl = std::acos(pq.x / magPq);
                    if (pq.y > 0)
                        angl = angl + PI;

                    // distance between the previous coord and the present one
                    auto dist = std::sqrt(
                        std::pow((value.last_detection_coords.x - oi.detection_coords.x), 2) +
                        std::pow((value.last_detection_coords.y - oi.detection_coords.y), 2));

                    oi.direction = angl;
                    oi.speed = dist / (value.initial_timestamp - value.last_timestamp);
                    // oi.xDistance = -cos(uss_setups[key].norm_detection_angle) * value.depth;
                    // oi.yDistance = sin(uss_setups[key].norm_detection_angle) * value.depth;
                    oi.xDistance = (oi.detection_coords.x - cur_pos_center.x) * 100;
                    oi.yDistance = (oi.detection_coords.y - cur_pos_center.y) * 100;
                    oi.depth = value.depth;

                    // EV << "xDistance (send):" << oi.xDistance/100 << endl << "yDistance: " << oi.xDistance/100<< endl;
                    //  EV << "old speed: " << value.speed << endl;
                    //  EV << "old direction: " << value.direction << endl;
                    //  EV << "new speed: " << oi.speed << endl;
                    //  EV << "new direction: " << oi.speed << endl;

                } else {
                    oi.xDistance = (value.detection_coords.x - cur_pos_center.x) * 100;
                    oi.yDistance = (value.detection_coords.y - cur_pos_center.y) * 100;
                    oi.direction = value.direction;
                    oi.depth = value.depth;
                    oi.speed = value.speed;
                }


                oi.sensors.push_back(key);
                oi.sensors.push_back(key + 1);
                objects.push_back(oi);

                // // WRITE INFO TO FILE
                // std::ostringstream oss;
                // oss << mVehicleController->getVehicleId() << "_detection_pos.txt";
                // detection_file.open(oss.str().c_str(), std::ios_base::app);

                // const traci::Boundary boundary{mVehicleController->getTraCI()->simulation.getNetBoundary()};
                // artery::Position arteryPos = Position(value.detection_coords.x, value.detection_coords.y);
                // traci::TraCIPosition traCIPos = position_cast(boundary,arteryPos);

                // detection_file << mVehicleController->getVehicleId() << "," << oi.initial_timestamp << "," << traCIPos.x << ","
                //                << traCIPos.y << endl;
                // detection_file.close();
                ++it;
            } else {
                // if the following sensor isn't active, we just send the information gathered from de current sensor
                oi.detection_coords = value.detection_coords;
                oi.direction = value.direction;
                oi.initial_timestamp = value.initial_timestamp;
                oi.detection_delta_time = value.detection_delta_time;
                oi.objectID = value.objectID;
                oi.speed = value.speed;
                oi.depth = value.depth;

                oi.xDistance = (value.detection_coords.x - cur_pos_center.x) * 100;
                oi.yDistance = (value.detection_coords.y - cur_pos_center.y) * 100;
                oi.sensors.push_back(key);
                objects.push_back(oi);
                // // WRITE INFO TO FILE
                // std::ostringstream oss;
                // oss << mVehicleController->getVehicleId() << "_detection_pos.txt";
                // detection_file.open(oss.str().c_str(), std::ios_base::app);
                // const traci::Boundary boundary{mVehicleController->getTraCI()->simulation.getNetBoundary()};
                // artery::Position arteryPos = Position(value.detection_coords.x, value.detection_coords.y);
                // traci::TraCIPosition traCIPos = position_cast(boundary,arteryPos);

                // detection_file << mVehicleController->getVehicleId() << "," << oi.initial_timestamp << "," << traCIPos.x << ","
                //                << traCIPos.y << endl;
                detection_file.close();
            }
        }
    }
}

void CpService::fullPathToFile()
{
    for (auto it = detections_map.begin(); it != detections_map.end(); ++it) {
        auto key = it->first;
        auto value = it->second;
        if (value.speed != 0.0) {
            // if there is movement in the sensor a moving object is present
            object_info oi;
            if (key != 5 && key != 11 && detections_map.find(key + 1) != detections_map.end() && detections_map[key + 1].speed != 0) {


                if (value.depth < detections_map[key + 1].depth) {
                    oi.detection_coords = value.detection_coords;
                } else {
                    oi.detection_coords = detections_map[key + 1].detection_coords;
                }
                oi.initial_timestamp = value.initial_timestamp;
                oi.depth = value.depth;
                oi.detection_delta_time = value.detection_delta_time;
                oi.objectID = value.objectID;

                if ((oi.detection_coords.x != value.detection_coords.x || oi.detection_coords.y != value.detection_coords.y) &&
                    (oi.detection_coords.x != detections_map[key + 1].detection_coords.x ||
                     oi.detection_coords.y != detections_map[key + 1].detection_coords.y)) {

                    omnetpp::cFigure::Point pq = value.detection_coords - value.last_detection_coords;
                    double magPq = std::sqrt(std::pow(pq.x, 2) + std::pow(pq.y, 2));
                    double angl = std::acos(pq.x / magPq);
                    if (pq.y > 0)
                        angl = angl + PI;

                    // distance between the previous coord and the present one
                    auto dist = std::sqrt(
                        std::pow((value.last_detection_coords.x - oi.detection_coords.x), 2) +
                        std::pow((value.last_detection_coords.y - oi.detection_coords.y), 2));

                    oi.direction = angl;
                    oi.speed = dist / (value.initial_timestamp - value.last_timestamp);
                    // oi.xDistance = -cos(uss_setups[key].norm_detection_angle) * value.depth;
                    // oi.yDistance = sin(uss_setups[key].norm_detection_angle) * value.depth;
                    oi.xDistance = (oi.detection_coords.x - cur_pos_center.x) * 100;
                    oi.yDistance = (oi.detection_coords.y - cur_pos_center.y) * 100;
                    oi.depth = value.depth;

                    // EV << "xDistance (send):" << oi.xDistance/100 << endl << "yDistance: " << oi.xDistance/100<< endl;
                    //  EV << "old speed: " << value.speed << endl;
                    //  EV << "old direction: " << value.direction << endl;
                    //  EV << "new speed: " << oi.speed << endl;
                    //  EV << "new direction: " << oi.speed << endl;

                } else {
                    oi.xDistance = (value.detection_coords.x - cur_pos_center.x) * 100;
                    oi.yDistance = (value.detection_coords.y - cur_pos_center.y) * 100;
                    oi.direction = value.direction;
                    oi.depth = value.depth;
                    oi.speed = value.speed;
                }


                oi.sensors.push_back(key);
                oi.sensors.push_back(key + 1);

                // WRITE INFO TO FILE
                std::ostringstream oss;
                oss << mVehicleController->getVehicleId() << "_detection_pos.txt";
                detection_file.open(oss.str().c_str(), std::ios_base::app);

                const traci::Boundary boundary{mVehicleController->getTraCI()->simulation.getNetBoundary()};
                artery::Position arteryPos = Position(value.detection_coords.x, value.detection_coords.y);
                traci::TraCIPosition traCIPos = position_cast(boundary, arteryPos);

                DistanceConfidence_t conf = std::round(std::sqrt(2 * std::pow(oi.depth, 2) * (1 - std::cos(PI / 12))) * 100);

                detection_file << mVehicleController->getVehicleId() << "," << oi.initial_timestamp << "," 
                << traCIPos.x << "," << traCIPos.y << "," << conf << endl;
                detection_file.close();

                // DRAW DETECTION
                omnetpp::cFigure::Point final_pos = {value.detection_coords.x, value.detection_coords.y};
                omnetpp::cFigure::Point a = omnetpp::cFigure::Point(final_pos.x + 0.1, final_pos.y + 0.1);
                omnetpp::cFigure::Point b = omnetpp::cFigure::Point(final_pos.x + 0.1, final_pos.y - 0.1);
                omnetpp::cFigure::Point c = omnetpp::cFigure::Point(final_pos.x - 0.1, final_pos.y - 0.1);
                omnetpp::cFigure::Point d = omnetpp::cFigure::Point(final_pos.x - 0.1, final_pos.y + 0.1);

                auto cross_line1 = new cLineFigure();
                cross_line1->setStart(a);
                cross_line1->setEnd(c);
                cross_line1->setLineColor("RED");
                detections->addFigure(cross_line1);
                auto cross_line2 = new cLineFigure();
                cross_line2->setStart(b);
                cross_line2->setEnd(d);
                cross_line2->setLineColor("RED");
                detections->addFigure(cross_line2);

                ++it;
            } else {
                // if the following sensor isn't active, we just send the information gathered from de current sensor
                oi.detection_coords = value.detection_coords;
                oi.direction = value.direction;
                oi.initial_timestamp = value.initial_timestamp;
                oi.detection_delta_time = value.detection_delta_time;
                oi.objectID = value.objectID;
                oi.speed = value.speed;
                oi.depth = value.depth;

                oi.xDistance = (value.detection_coords.x - cur_pos_center.x) * 100;
                oi.yDistance = (value.detection_coords.y - cur_pos_center.y) * 100;
                oi.sensors.push_back(key);
                // WRITE INFO TO FILE
                std::ostringstream oss;
                oss << mVehicleController->getVehicleId() << "_detection_pos.txt";
                detection_file.open(oss.str().c_str(), std::ios_base::app);
                const traci::Boundary boundary{mVehicleController->getTraCI()->simulation.getNetBoundary()};
                artery::Position arteryPos = Position(value.detection_coords.x, value.detection_coords.y);
                traci::TraCIPosition traCIPos = position_cast(boundary, arteryPos);
                
                DistanceConfidence_t conf = std::round(std::sqrt(2 * std::pow(oi.depth, 2) * (1 - std::cos(PI / 12))) * 100);

                detection_file << mVehicleController->getVehicleId() << "," << oi.initial_timestamp << "," 
                << traCIPos.x << "," << traCIPos.y << "," << conf << endl;
                detection_file.close();

                // DRAW DETECTION
                omnetpp::cFigure::Point final_pos = {value.detection_coords.x, value.detection_coords.y};
                omnetpp::cFigure::Point a = omnetpp::cFigure::Point(final_pos.x + 0.1, final_pos.y + 0.1);
                omnetpp::cFigure::Point b = omnetpp::cFigure::Point(final_pos.x + 0.1, final_pos.y - 0.1);
                omnetpp::cFigure::Point c = omnetpp::cFigure::Point(final_pos.x - 0.1, final_pos.y - 0.1);
                omnetpp::cFigure::Point d = omnetpp::cFigure::Point(final_pos.x - 0.1, final_pos.y + 0.1);

                auto cross_line1 = new cLineFigure();
                cross_line1->setStart(a);
                cross_line1->setEnd(c);
                cross_line1->setLineColor("RED");
                detections->addFigure(cross_line1);
                auto cross_line2 = new cLineFigure();
                cross_line2->setStart(b);
                cross_line2->setEnd(d);
                cross_line2->setLineColor("RED");
                detections->addFigure(cross_line2);
            }
        }
    }
}

omnetpp::cFigure::Point CpService::findPosition(
    omnetpp::cFigure::Point cpc, omnetpp::cFigure::Point A, omnetpp::cFigure::Point B, double d_A, double d_B, omnetpp::cFigure::Point C,
    omnetpp::cFigure::Point D)
{  // does the triangulation
    double x1 = A.x, y1 = A.y;
    double x2 = B.x, y2 = B.y;
    double d = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));

    if (d > d_A + d_B || d < abs(d_A - d_B) || (d == 0 && d_A != d_B)) {
        // if there is no intersection (the circles do not touch)
        // we return the detection that is most close to the sensor
        if (d_A < d_B)
            return C;
        else
            return D;
    }

    double a = (d_A * d_A - d_B * d_B + d * d) / (2 * d);
    double h = sqrt(d_A * d_A - a * a);

    double x0 = x1 + a * (x2 - x1) / d;
    double y0 = y1 + a * (y2 - y1) / d;

    double rx = -(y2 - y1) * (h / d);
    double ry = (x2 - x1) * (h / d);

    omnetpp::cFigure::Point P1 = {x0 + rx, y0 + ry};
    omnetpp::cFigure::Point P2 = {x0 - rx, y0 - ry};

    if (std::sqrt(std::pow(A.x - cpc.x, 2) + std::pow(A.y - cpc.y, 2)) > std::sqrt(std::pow(B.x - cpc.x, 2) + std::pow(B.y - cpc.y, 2)))
        // if (P1.x < mVehicleDataProvider->position().x.value() && P1.x > mVehicleDataProvider->position().x.value() - vehicle_length)
        //  triangulation with only 2 detections gives us 2 possible coords
        //  we chose the coord that is in outside of the car
        return P1;
    else
        return P2;
}

omnetpp::cFigure::Point CpService::drawDetections(int role_name, double depth)
{
    // // From front-center-bumper to center (sumo reference system).

    uss_setup desired_config = uss_setups[role_name];
    omnetpp::cFigure::Point detection_pos =
        omnetpp::cFigure::Point(-cos(desired_config.norm_detection_angle) * depth, sin(desired_config.norm_detection_angle) * depth);

    omnetpp::cFigure::Point final_pos = cur_pos_center + desired_config.sensor_pos + detection_pos;

    auto cross_line1 = new cLineFigure();
    cross_line1->setStart(omnetpp::cFigure::Point(final_pos.x + 0.1, final_pos.y + 0.1));
    cross_line1->setEnd(omnetpp::cFigure::Point(final_pos.x - 0.1, final_pos.y - 0.1));
    cross_line1->setLineColor("BLACK");
    cross_line1->setZIndex(3);
    // detections->addFigure(cross_line1);

    auto cross_line2 = new cLineFigure();
    cross_line2->setStart(omnetpp::cFigure::Point(final_pos.x + 0.1, final_pos.y - 0.1));
    cross_line2->setEnd(omnetpp::cFigure::Point(final_pos.x - 0.1, final_pos.y + 0.1));
    cross_line2->setLineColor("BLACK");
    cross_line2->setZIndex(3);
    // detections->addFigure(cross_line2);

    auto arc = new cArcFigure();
    arc->setBounds(cFigure::Rectangle(cur_pos_center.x + desired_config.sensor_pos.x, cur_pos_center.y + desired_config.sensor_pos.y, 2 * depth, 2 * depth));
    arc->setPosition(cur_pos_center + desired_config.sensor_pos, cFigure::ANCHOR_CENTER);
    arc->setStartAngle(desired_config.norm_detection_angle - 30 * PI / 180 + PI);
    arc->setEndAngle(desired_config.norm_detection_angle + 30 * PI / 180 + PI);
    // detections->addFigure(arc);

    detections_map[role_name].depth = depth;
    detections_map[role_name].line1 = cross_line1;
    detections_map[role_name].line2 = cross_line2;
    detections_map[role_name].arc = arc;
    detections_map[role_name].last_timestamp = detections_map[role_name].initial_timestamp;
    detections_map[role_name].last_detection_coords = detections_map[role_name].detection_coords;
    detections_map[role_name].detection_coords = final_pos;

    return final_pos;
}

}  // namespace artery
