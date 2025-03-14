#include "traci/BasicNodeManager.h"

#include "traci/API.h"
#include "traci/CheckTimeSync.h"
#include "traci/Core.h"
#include "traci/ModuleMapper.h"
#include "traci/PersonSink.h"
#include "traci/VariableCache.h"
#include "traci/VehicleSink.h"
// #include "math.h"
#include <inet/common/ModuleAccess.h>

using namespace omnetpp;

namespace si = boost::units::si;

namespace traci
{
namespace
{
static const std::set<int> sPersonVariables{libsumo::VAR_POSITION, libsumo::VAR_SPEED, libsumo::VAR_ANGLE};
static const std::set<int> sVehicleVariables{libsumo::VAR_POSITION, libsumo::VAR_SPEED, libsumo::VAR_ANGLE};
static const std::set<int> sSimulationVariables{
    libsumo::VAR_DEPARTED_VEHICLES_IDS, libsumo::VAR_ARRIVED_VEHICLES_IDS, libsumo::VAR_TELEPORT_STARTING_VEHICLES_IDS, libsumo::VAR_TIME};

class VehicleObjectImpl : public BasicNodeManager::VehicleObject
{
public:
    VehicleObjectImpl(std::shared_ptr<VehicleCache> cache) : m_cache(cache) {}

    std::shared_ptr<VehicleCache> getCache() const override { return m_cache; }
    const TraCIPosition& getPosition() const override { return m_cache->get<libsumo::VAR_POSITION>(); }
    TraCIAngle getHeading() const override { return TraCIAngle{m_cache->get<libsumo::VAR_ANGLE>()}; }
    double getSpeed() const override { return m_cache->get<libsumo::VAR_SPEED>(); }

private:
    std::shared_ptr<VehicleCache> m_cache;
};

class PersonObjectImpl : public BasicNodeManager::PersonObject
{
public:
    PersonObjectImpl(std::shared_ptr<PersonCache> cache) : m_cache(cache) {}

    std::shared_ptr<PersonCache> getCache() const override { return m_cache; }
    const TraCIPosition& getPosition() const override { return m_cache->get<libsumo::VAR_POSITION>(); }
    TraCIAngle getHeading() const override { return TraCIAngle{m_cache->get<libsumo::VAR_ANGLE>()}; }
    double getSpeed() const override { return m_cache->get<libsumo::VAR_SPEED>(); }

private:
    std::shared_ptr<PersonCache> m_cache;
};

}  // namespace


Define_Module(BasicNodeManager)

const simsignal_t BasicNodeManager::addNodeSignal = cComponent::registerSignal("traci.node.add");
const simsignal_t BasicNodeManager::updateNodeSignal = cComponent::registerSignal("traci.node.update");
const simsignal_t BasicNodeManager::removeNodeSignal = cComponent::registerSignal("traci.node.remove");
const simsignal_t BasicNodeManager::addPersonSignal = cComponent::registerSignal("traci.person.add");
const simsignal_t BasicNodeManager::updatePersonSignal = cComponent::registerSignal("traci.person.update");
const simsignal_t BasicNodeManager::removePersonSignal = cComponent::registerSignal("traci.person.remove");
const simsignal_t BasicNodeManager::addVehicleSignal = cComponent::registerSignal("traci.vehicle.add");
const simsignal_t BasicNodeManager::updateVehicleSignal = cComponent::registerSignal("traci.vehicle.update");
const simsignal_t BasicNodeManager::removeVehicleSignal = cComponent::registerSignal("traci.vehicle.remove");

void BasicNodeManager::initialize()
{
    Core* core = inet::getModuleFromPar<Core>(par("coreModule"), this);
    subscribeTraCI(core);
    m_api = core->getAPI();
    m_mapper = inet::getModuleFromPar<ModuleMapper>(par("mapperModule"), this);
    m_nodeIndex = 0;
    m_person_sink_module = par("personSinkModule").stringValue();
    m_vehicle_sink_module = par("vehicleSinkModule").stringValue();
    m_subscriptions = inet::getModuleFromPar<SubscriptionManager>(par("subscriptionsModule"), this);
    m_destroy_vehicles_on_crash = par("destroyVehiclesOnCrash");
    m_ignore_persons = par("ignorePersons");
    // add by lip
    canvas = getSimulation()->getModuleByPath("World")->getCanvas();
    draw_figures = new cGroupFigure();
    canvas->addFigure(draw_figures);
    remove("persons_positions.csv");
    per_pos_file.open("persons_positions.csv" , std::ios_base::app);
    per_pos_file << "timestamp,person_id,x_gt,y_gt" << endl;
    per_pos_file.close();
    //car_pos_file.open("cars_positions.csv" , std::ios_base::app);
    //car_pos_file << "id,sensor,x0,y0,x1,y1,x2,y2" << endl;
    // car_pos_file.close();
    // end add by lip
}

void BasicNodeManager::finish()
{
    unsubscribeTraCI();
    cSimpleModule::finish();
    draw_figures->removeFromParent();
}

void BasicNodeManager::traciInit()
{
    m_boundary = Boundary{m_api->simulation.getNetBoundary()};
    m_subscriptions->subscribeSimulationVariables(sSimulationVariables);
    m_subscriptions->subscribeVehicleVariables(sVehicleVariables);

    // insert already running vehicles
    for (const std::string& id : m_api->vehicle.getIDList()) {
        addVehicle(id);
    }

    // initialize persons if enabled
    if (!m_ignore_persons) {
        m_subscriptions->subscribePersonVariables(sPersonVariables);

        // insert already running persons
        for (const std::string& id : m_api->person.getIDList()) {
            addPerson(id);
        }
    }

    // treat SUMO start time as constant offset
    m_offset = omnetpp::SimTime{m_api->simulation.getCurrentTime(), omnetpp::SIMTIME_MS};
    m_offset -= omnetpp::simTime();
}

void BasicNodeManager::traciStep()
{
    int x = draw_figures->getNumFigures();
    for (int i = x - 1; i >= 0; i--)
        draw_figures->removeFigure(i);
    processVehicles();
    if (!m_ignore_persons) {
        processPersons();
    }
    emit(updateNodeSignal, getNumberOfNodes());
}

void BasicNodeManager::traciClose()
{
    for (unsigned i = m_nodes.size(); i > 0; --i) {
        removeNodeModule(m_nodes.begin()->first);
    }
}

void BasicNodeManager::processVehicles()
{
    auto sim_cache = m_subscriptions->getSimulationCache();
    ASSERT(checkTimeSync(*sim_cache, omnetpp::simTime() + m_offset));

    const auto& departed = sim_cache->get<libsumo::VAR_DEPARTED_VEHICLES_IDS>();
    EV_DETAIL << "TraCI: " << departed.size() << " vehicles departed" << endl;
    for (const auto& id : departed) {
        addVehicle(id);
    }

    const auto& arrived = sim_cache->get<libsumo::VAR_ARRIVED_VEHICLES_IDS>();
    EV_DETAIL << "TraCI: " << arrived.size() << " vehicles arrived" << endl;
    for (const auto& id : arrived) {
        removeVehicle(id);
    }

    if (m_destroy_vehicles_on_crash) {
        const auto& teleport = sim_cache->get<libsumo::VAR_TELEPORT_STARTING_VEHICLES_IDS>();
        for (const auto& id : teleport) {
            EV_DETAIL << "TraCI: " << id << " got teleported and is removed!" << endl;
            removeVehicle(id);
        }
    }

    for (auto& vehicle : m_vehicles) {
        const std::string& id = vehicle.first;
        VehicleSink* sink = vehicle.second;
        updateVehicle(id, sink);
    }
}

void BasicNodeManager::addVehicle(const std::string& id)
{
    NodeInitializer init = [this, &id](cModule* module) {
        VehicleSink* vehicle = getVehicleSink(module);
        auto& traci = m_api->vehicle;
        vehicle->initializeSink(m_api, m_subscriptions->getVehicleCache(id), m_boundary);
        vehicle->initializeVehicle(traci.getPosition(id), TraCIAngle{traci.getAngle(id)}, traci.getSpeed(id));
        m_vehicles[id] = vehicle;
    };

    emit(addVehicleSignal, id.c_str());
    cModuleType* type = m_mapper->vehicle(*this, id);
    if (type != nullptr) {
        addNodeModule(id, type, init);
    } else {
        m_vehicles[id] = nullptr;
    }
    
    std::string file_name = id + "_sensor_positions.csv";
    remove(file_name.c_str());
}

void BasicNodeManager::removeVehicle(const std::string& id)
{
    emit(removeVehicleSignal, id.c_str());
    removeNodeModule(id);
    m_vehicles.erase(id);
}

void BasicNodeManager::updateVehicle(const std::string& id, VehicleSink* sink)
{
    auto vehicle = m_subscriptions->getVehicleCache(id);
    VehicleObjectImpl update(vehicle);
    emit(updateVehicleSignal, id.c_str(), &update);
    if (sink) {
        
        TraCIPosition traci_pos = vehicle->get<libsumo::VAR_POSITION>();
        sink->updateVehicle(traci_pos, TraCIAngle{vehicle->get<libsumo::VAR_ANGLE>()}, vehicle->get<libsumo::VAR_SPEED>());

        double vehicle_angle = angle_cast(TraCIAngle{vehicle->get<libsumo::VAR_ANGLE>()}).radian();
        double vehicle_length = (vehicle->get<libsumo::VAR_LENGTH>() * si::meter).value();
        double vehicle_width = (vehicle->get<libsumo::VAR_WIDTH>() * si::meter).value();
        double bbox_radius = sqrt(pow(vehicle_length / 2, 2) + pow(vehicle_width / 2, 2));
        auto position = traci::position_cast(m_boundary, vehicle->get<libsumo::VAR_POSITION>());

        // traci::API traCIAPI;
        // TraCIPosition p;
        // p.x = position.x.value();
        // p.y = position.y.value();
        // p.z = 0;


        //EV << "Pos do Popo " << id << " em coords: " << endl << "x: " << position.x.value() << endl << "y: " << position.y.value() <<endl;
        
        
        
        
        auto v_c = m_api->vehicle.getColor(id);
        omnetpp::cFigure::Color vehicle_color = omnetpp::cFigure::Color(v_c.r, v_c.g, v_c.b);
        omnetpp::cFigure::Point cur_pos_center =
            omnetpp::cFigure::Point(position.x.value() - cos(vehicle_angle) * vehicle_length / 2, position.y.value() + sin(vehicle_angle) * vehicle_length / 2);


        // DRAW VEHICLE
        auto rectangle = new cRectangleFigure();
        rectangle->setBounds(cFigure::Rectangle(cur_pos_center.x, cur_pos_center.y, vehicle_width, vehicle_length));
        rectangle->setPosition(cur_pos_center, cFigure::ANCHOR_CENTER);
        rectangle->rotate(-vehicle_angle + PI / 2, cur_pos_center);
        rectangle->setFilled(true);
        rectangle->setFillColor(vehicle_color);
        // rectangle->setFillColor("RED");
        rectangle->setZIndex(2.0);
        draw_figures->addFigure(rectangle);

        std::map<int, uss_setup> uss_setups;

            // DRAW SENSORS AND PIESLICES
            uss_setups[0] = {-78, vehicle_length / 2 + 0.1, -5 * vehicle_width / 12, 0.5, 0, 0, 0, omnetpp::cFigure::Point(0, 0)};
            uss_setups[1] = {-41, vehicle_length / 2 + 0.1, -vehicle_width / 4, 0.5, 0, 0, 0, omnetpp::cFigure::Point(0, 0)};
            uss_setups[2] = {-8, vehicle_length / 2 + 0.1, -vehicle_width / 12, 0.5, 0, 0, 0, omnetpp::cFigure::Point(0, 0)};
            uss_setups[3] = {+8, vehicle_length / 2 + 0.1, vehicle_width / 12, 0.5, 0, 0, 0, omnetpp::cFigure::Point(0, 0)};
            uss_setups[4] = {+41, vehicle_length / 2 + 0.1, vehicle_width / 4, 0.5, 0, 0, 0, omnetpp::cFigure::Point(0, 0)};
            uss_setups[5] = {+78, vehicle_length / 2 + 0.1, 5 * vehicle_width / 12, 0, 0.5}, 0, 0, omnetpp::cFigure::Point(0, 0);
            uss_setups[6] = {180 - 78, -vehicle_length / 2 - 0.1, 5 * vehicle_width / 12, 0, 0.5, 0, 0, omnetpp::cFigure::Point(0, 0)};
            uss_setups[7] = {180 - 29, -vehicle_length / 2 - 0.1, vehicle_width / 4, 0.5, 0, 0, 0, omnetpp::cFigure::Point(0, 0)};
            uss_setups[8] = {180 - 6, -vehicle_length / 2 - 0.1, vehicle_width / 12, 0.5, 0, 0, 0, omnetpp::cFigure::Point(0, 0)};
            uss_setups[9] = {6 - 180, -vehicle_length / 2 - 0.1, -vehicle_width / 12, 0.5, 0, 0, 0, omnetpp::cFigure::Point(0, 0)};
            uss_setups[10] = {29 - 180, -vehicle_length / 2 - 0.1, -vehicle_width / 4, 0.5, 0, 0, 0, omnetpp::cFigure::Point(0, 0)};
            uss_setups[11] = {78 - 180, -vehicle_length / 2 - 0.1, -5 * vehicle_width / 12, 0.5, 0, 0, 0, omnetpp::cFigure::Point(0, 0)};


            std::string file_name = id + "_sensor_positions.csv";
            remove(file_name.c_str());
            car_pos_file.open(file_name , std::ios_base::app);
            car_pos_file << "sensor_id,x0,y0,x1,y1,x2,y2" << endl;
            car_pos_file.close();


            for (auto it = uss_setups.begin(); it != uss_setups.end(); ++it) {
                auto key = it->first;
                auto value = it->second;
                value.norm_sensor_angle = vehicle_angle - atan2(value.y, value.x) + PI;
                while (value.norm_sensor_angle < -PI)
                    value.norm_sensor_angle += 2 * PI;
                while (value.norm_sensor_angle >= PI)
                    value.norm_sensor_angle -= 2 * PI;

                value.sensor_dist = sqrt(pow(abs(value.x), 2) + pow(abs(value.y), 2));
                value.sensor_pos = omnetpp::cFigure::Point(-cos(value.norm_sensor_angle) * value.sensor_dist, sin(value.norm_sensor_angle) * value.sensor_dist);

                value.norm_detection_angle = vehicle_angle - (value.yaw * (PI / 180)) + PI;
                while (value.norm_detection_angle < -PI)
                    value.norm_detection_angle += 2 * PI;
                while (value.norm_detection_angle >= PI)
                    value.norm_detection_angle -= 2 * PI;
                
                omnetpp::cFigure::Point cur_sensor_pos = cur_pos_center + value.sensor_pos;
                auto sensor_rectangle = new cRectangleFigure();
                sensor_rectangle->setBounds(cFigure::Rectangle(cur_sensor_pos.x, cur_sensor_pos.y, 0.2, 0.2));
                sensor_rectangle->setPosition(cur_sensor_pos, cFigure::ANCHOR_CENTER);
                sensor_rectangle->rotate(-vehicle_angle + PI / 2, cur_sensor_pos);
                sensor_rectangle->setFilled(true);
                sensor_rectangle->setFillColor("GRAY");
                draw_figures->addFigure(sensor_rectangle);

                auto cone = new cPieSliceFigure();
                cone->setBounds(cFigure::Rectangle(cur_sensor_pos.x, cur_sensor_pos.y, 11, 11));
                cone->setPosition(cur_sensor_pos, cFigure::ANCHOR_CENTER);
                cone->setStartAngle(value.norm_detection_angle + 15 * (PI / 180) + PI);
                cone->setEndAngle(value.norm_detection_angle - 15 * (PI / 180) + PI);
                cone->setFilled(true);
                cone->setFillOpacity(0.01);

                if (key % 2 == 0) {
                    cone->setLineColor("GREEN");
                    cone->setLineOpacity(0.3);
                    cone->setFillColor("GREEN");
                } else {
                    cone->setLineColor("BLUE");
                    cone->setLineOpacity(0.3);
                    cone->setFillColor("BLUE");
                }
                draw_figures->addFigure(cone);

                
                omnetpp::cFigure::Point corner1 =
                    omnetpp::cFigure::Point(-cos(value.norm_detection_angle - 15*(PI/180)) * 5.5, sin(value.norm_detection_angle - 15*(PI/180)) * 5.5);
                omnetpp::cFigure::Point coord_corner1 = cur_pos_center + value.sensor_pos + corner1;
                
                omnetpp::cFigure::Point corner2 =
                    omnetpp::cFigure::Point(-cos(value.norm_detection_angle + 15*(PI/180)) * 5.5, sin(value.norm_detection_angle + 15*(PI/180)) * 5.5);
                omnetpp::cFigure::Point coord_corner2 = cur_pos_center + value.sensor_pos + corner2;


                if (vehicle->get<libsumo::VAR_SPEED>() == 0){
                    
                    std::string file_name = id + "_sensor_positions.csv";
                    car_pos_file.open(file_name , std::ios_base::app);
                    car_pos_file
                    << key << ","
                    << cur_sensor_pos.x << "," 
                    << cur_sensor_pos.y << "," 
                    << coord_corner1.x << "," 
                    << coord_corner1.y << "," 
                    << coord_corner2.x << "," 
                    << coord_corner2.y << endl;
                    car_pos_file.close();
                }
                

                //DEBUG
                // auto corner_sensor_1 = omnetpp::cFigure::Point(x1, y1);
                // auto corner_sensor_2 = omnetpp::cFigure::Point(x2, y2);

                // auto debug1 = new cRectangleFigure();
                // sensor_rectangle->setBounds(cFigure::Rectangle(corner_sensor_1.x, corner_sensor_1.y, 0.2, 0.2));
                // sensor_rectangle->setPosition(corner_sensor_1, cFigure::ANCHOR_CENTER);
                // sensor_rectangle->setFilled(true);
                // sensor_rectangle->setFillColor("BLUE");
                // draw_figures->addFigure(debug1);

                // auto debug2 = new cRectangleFigure();
                // sensor_rectangle->setBounds(cFigure::Rectangle(corner_sensor_2.x, corner_sensor_2.y, 0.2, 0.2));
                // sensor_rectangle->setPosition(corner_sensor_2, cFigure::ANCHOR_CENTER);
                // sensor_rectangle->setFilled(true);
                // sensor_rectangle->setFillColor("GREEN");
                // draw_figures->addFigure(debug2);
            }
    }
}


void BasicNodeManager::processPersons()
{
    auto sim_cache = m_subscriptions->getSimulationCache();

    const auto& departed = sim_cache->get<libsumo::VAR_DEPARTED_PERSONS_IDS>();
    EV_DETAIL << "TraCI: " << departed.size() << " persons departed" << endl;
    for (const auto& id : departed) {
        addPerson(id);
    }

    const auto& arrived = sim_cache->get<libsumo::VAR_ARRIVED_PERSONS_IDS>();
    EV_DETAIL << "TraCI: " << arrived.size() << " persons arrived" << endl;
    for (const auto& id : arrived) {
        removePerson(id);
    }

    for (auto& person : m_persons) {
        const std::string& id = person.first;
        PersonSink* sink = person.second;
        updatePerson(id, sink);
    }
}

void BasicNodeManager::addPerson(const std::string& id)
{
    NodeInitializer init = [this, &id](cModule* module) {
        PersonSink* person = getPersonSink(module);
        auto& traci = m_api->person;
        person->initializeSink(m_api, m_subscriptions->getPersonCache(id), m_boundary);
        person->initializePerson(traci.getPosition(id), TraCIAngle{traci.getAngle(id)}, traci.getSpeed(id));
        m_persons[id] = person;
    };

    emit(addPersonSignal, id.c_str());
    cModuleType* type = m_mapper->person(*this, id);
    if (type != nullptr) {
        addNodeModule(id, type, init);
    } else {
        m_persons[id] = nullptr;
    }
}

void BasicNodeManager::removePerson(const std::string& id)
{
    emit(removePersonSignal, id.c_str());
    removeNodeModule(id);
    m_persons.erase(id);
}

void BasicNodeManager::updatePerson(const std::string& id, PersonSink* sink)
{
    auto person = m_subscriptions->getPersonCache(id);
    PersonObjectImpl update(person);
    emit(updatePersonSignal, id.c_str(), &update);
    if (sink) {
        TraCIPosition traci_pos = person->get<libsumo::VAR_POSITION>();
        sink->updatePerson(traci_pos, TraCIAngle{person->get<libsumo::VAR_ANGLE>()}, person->get<libsumo::VAR_SPEED>());


        // Draw person bounding box START

        double person_angle = angle_cast(TraCIAngle{person->get<libsumo::VAR_ANGLE>()}).radian();
        double person_length = (person->get<libsumo::VAR_LENGTH>() * si::meter).value();
        // EV << "person_length: " << person_length << endl;
        double person_width = (person->get<libsumo::VAR_WIDTH>() * si::meter).value();
        // EV << "person_width: " << person_width << endl;
        double bbox_radius = sqrt(pow(person_length / 2, 2) + pow(person_width / 2, 2));

        auto position = traci::position_cast(m_boundary, traci_pos);

        // From front-center-bumper to center (sumo reference system).
        omnetpp::cFigure::Point cur_pos_center =
            omnetpp::cFigure::Point(position.x.value() - cos(person_angle) * person_length / 2, position.y.value() + sin(person_angle) * person_length / 2);

        auto rectangle = new cRectangleFigure();
        rectangle->setBounds(cFigure::Rectangle(cur_pos_center.x, cur_pos_center.y, person_width, person_length));
        rectangle->setPosition(cur_pos_center, cFigure::ANCHOR_CENTER);
        rectangle->rotate(-person_angle + PI / 2, cur_pos_center);
        rectangle->setFilled(true);
        rectangle->setFillOpacity(0.05);
        rectangle->setFillColor("GRAY");
        rectangle->setZIndex(2.0);
        draw_figures->addFigure(rectangle);

        // auto currTime = m_api->simulation.getCurrentTime();
        auto currTime = omnetpp::simTime();       
        std::vector<omnetpp::cFigure::Point> verts;
        verts.push_back(omnetpp::cFigure::Point(269.397, 169.085)); // 230.355
        verts.push_back(omnetpp::cFigure::Point(266.612, 168.493)); // 230.947
        verts.push_back(omnetpp::cFigure::Point(266.612, 181.187)); // 218.253
        verts.push_back(omnetpp::cFigure::Point(269.397, 180.595)); // 218.845
        int i, j, c = 0;
        int nvert = 4;

        // for (i = 0, j = nvert-1; i < nvert; j = i++) {
        //     if ( ((verts[i].y>position.y.value()) != (verts[j].y>position.y.value())) && (position.x.value() < (verts[j].x-verts[i].x) * (position.y.value()-verts[i].y) / (verts[j].y-verts[i].y) + verts[i].x) )
        //         c = !c;
        // }
        // if(c){
        //     per_pos_file.open("persons_positions.csv" , std::ios_base::app);
        //     per_pos_file << id << "," << currTime << "," << position.x.value() << "," << position.y.value() << endl;
        //     per_pos_file.close();
        // }
        per_pos_file.open("persons_positions.csv" , std::ios_base::app);
        per_pos_file << currTime << "," << id << ","  << position.x.value() << "," << position.y.value() << endl;
        per_pos_file.close();
    }
}

cModule* BasicNodeManager::createModule(const std::string&, cModuleType* type)
{
    cModule* module = type->create("node", getSystemModule(), m_nodeIndex, m_nodeIndex);
    ++m_nodeIndex;
    return module;
}

cModule* BasicNodeManager::addNodeModule(const std::string& id, cModuleType* type, NodeInitializer& init)
{
    cModule* module = createModule(id, type);
    module->finalizeParameters();
    module->buildInside();
    m_nodes[id] = module;
    init(module);
    module->scheduleStart(simTime());
    module->callInitialize();
    emit(addNodeSignal, id.c_str(), module);

    return module;
}

void BasicNodeManager::removeNodeModule(const std::string& id)
{
    cModule* module = getNodeModule(id);
    if (module) {
        emit(removeNodeSignal, id.c_str(), module);
        module->callFinish();
        module->deleteModule();
        m_nodes.erase(id);
    } else {
        EV_DEBUG << "Node with id " << id << " does not exist, no removal\n";
    }
}

cModule* BasicNodeManager::getNodeModule(const std::string& id)
{
    auto found = m_nodes.find(id);
    return found != m_nodes.end() ? found->second : nullptr;
}

std::size_t BasicNodeManager::getNumberOfNodes() const
{
    return m_nodes.size();
}

VehicleSink* BasicNodeManager::getVehicleSink(cModule* node)
{
    ASSERT(node);
    cModule* module = node->getModuleByPath(m_vehicle_sink_module.c_str());
    if (!module) {
        throw cRuntimeError("No module found at %s relative to %s", m_vehicle_sink_module.c_str(), node->getFullPath().c_str());
    }

    auto* mobility = dynamic_cast<VehicleSink*>(module);
    if (!mobility) {
        throw cRuntimeError("Module %s is not a VehicleSink", module->getFullPath().c_str());
    }

    return mobility;
}

VehicleSink* BasicNodeManager::getVehicleSink(const std::string& id)
{
    auto found = m_vehicles.find(id);
    return found != m_vehicles.end() ? found->second : nullptr;
}

PersonSink* BasicNodeManager::getPersonSink(cModule* node)
{
    ASSERT(node);
    cModule* module = node->getModuleByPath(m_person_sink_module.c_str());
    if (!module) {
        throw cRuntimeError("No module found at %s relative to %s", m_person_sink_module.c_str(), node->getFullPath().c_str());
    }

    auto* mobility = dynamic_cast<PersonSink*>(module);
    if (!mobility) {
        throw cRuntimeError("Module %s is not a PersonSink", module->getFullPath().c_str());
    }

    return mobility;
}

PersonSink* BasicNodeManager::getPersonSink(const std::string& id)
{
    auto found = m_persons.find(id);
    return found != m_persons.end() ? found->second : nullptr;
}
}  // namespace traci
