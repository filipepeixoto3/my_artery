/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

//#include "artery/envmod/EnvironmentModelObject.h"
#include "artery/visualizer/RoadsCanvasVisualizer.h"
#include "artery/envmod/Geometry.h"
#include "artery/traci/Cast.h"
#include "traci/Core.h"
//add by lip
#include "traci/API.h"
//end add by lip
#include <boost/geometry/geometries/register/linestring.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <inet/common/ModuleAccess.h>
#include <algorithm>
#include <array>

using namespace omnetpp;


namespace artery
{

Define_Module(RoadsCanvasVisualizer)

namespace {
//const simsignal_t refreshSignal = cComponent::registerSignal("EnvironmentModel.refresh");
const simsignal_t traciInitSignal = cComponent::registerSignal("traci.init");
const simsignal_t traciCloseSignal = cComponent::registerSignal("traci.close");
//const simsignal_t traciNodeAddSignal = cComponent::registerSignal("traci.node.add");
//const simsignal_t traciNodeRemoveSignal = cComponent::registerSignal("traci.node.remove");
//const simsignal_t traciNodeUpdateSignal = cComponent::registerSignal("traci.node.update");

template<typename RT>
typename RT::const_query_iterator
query_intersections(RT& rtree, const std::vector<Position>& area)
{
#if BOOST_VERSION >= 106000 && BOOST_VERSION < 106200
    // Boost versions 1.60 and 1.61 do not compile without copy
    geometry::Polygon area_copy;
    boost::geometry::convert(area, area_copy);
    auto predicate = boost::geometry::index::intersects(area_copy);
#else
    auto predicate = boost::geometry::index::intersects(area);
#endif
    return rtree.qbegin(predicate);
}

} // namespace

RoadsCanvasVisualizer::RoadsCanvasVisualizer()
{
}

RoadsCanvasVisualizer::~RoadsCanvasVisualizer()
{
    clear();
}

//void RoadsCanvasVisualizer::refresh()
//{
//    for (auto& object_kv : mObjects) {
//        object_kv.second->update();
//    }

//    buildObjectRtree();

//    if (mDrawVehicles) {
//        int numObjects = mObjects.size();
//        int numFigures = mDrawVehicles->getNumFigures();

        // add missing polygon figures
//        while (numFigures < numObjects) {
//            auto polygon = new cPolygonFigure();
//            polygon->setFillColor(cFigure::BLUE);
//            polygon->setFilled(true);
//            mDrawVehicles->addFigure(polygon);
//            ++numFigures;
//        }

        // remove excessive polygon figures
//        while (numFigures > numObjects) {
//            --numFigures;
//            delete mDrawVehicles->removeFigure(numFigures);
//        }

        // update figures with current outlines
//        int figureIndex = 0;
//        for (const auto& object_kv : mObjects) {
//            // we add only polygon figures, thus static_cast should be safe
//            auto polygon = static_cast<cPolygonFigure*>(mDrawVehicles->getFigure(figureIndex));
//            std::vector<cFigure::Point> points;
//            for (const Position& pos : object_kv.second->getOutline()) {
//                points.push_back(cFigure::Point { pos.x.value(), pos.y.value() });
//            }
//            polygon->setPoints(points);
//            ++figureIndex;
//        }
//    }

//    emit(refreshSignal, this);
//}

//bool RoadsCanvasVisualizer::addVehicle(traci::VehicleController* vehicle)
//{
//    uint32_t id = 0;
//    if (mIdentityRegistry) {
//        auto identity = mIdentityRegistry->lookup<IdentityRegistry::traci>(vehicle->getVehicleId());
//        if (identity) {
//            id = identity->application;
//        }
//    }

//    auto object = std::make_shared<EnvironmentModelObject>(vehicle, id);
//    auto insertion = mObjects.emplace(object->getExternalId(), object);
//    if (insertion.second) {
//        auto box = boost::geometry::return_envelope<geometry::Box>(object->getOutline());
//        mObjectRtree.insert(ObjectRtreeValue { std::move(box), object });
//    }
//    ASSERT(mObjects.size() == mObjectRtree.size());
//    return insertion.second;
//}

bool RoadsCanvasVisualizer::addObstacle(const std::string& id, std::vector<Position> outline)
{
    boost::geometry::correct(outline);
    std::string invalid;
    if (!boost::geometry::is_valid(outline, invalid)) {
        // skip self-intersecting geometry
        EV_ERROR << "skip invalid obstacle polygon " << id << ": " << invalid << " \n";
        return false;
    }
    auto insertion = mObstacles.emplace(id, std::make_shared<EnvironmentModelObstacle>(id, outline));

    if (mDrawObstacles) {
        auto polygon = new cPolygonFigure();
        polygon->setFilled(true);
        polygon->setFillColor(cFigure::RED);
        polygon->setFillOpacity(0.7);
        polygon->setZIndex(1.0);
        for (const Position& pos : outline) {
            polygon->addPoint(cFigure::Point { pos.x.value(), pos.y.value() });
        }
        mDrawObstacles->addFigure(polygon);
    }

    return insertion.second;
}

bool RoadsCanvasVisualizer::addLane(const std::string& id, std::vector<Position> outline)
{

    auto insertion = mObstacles.emplace(id, std::make_shared<EnvironmentModelObstacle>(id, outline));

    if (mDrawObstacles) {

        auto line = new cPolylineFigure();
        double width = par("lineWidth");
        bool zoom = par("lineWidthZoom");
        line->setLineWidth(width);
        line->setZoomLineWidth(zoom);
        line->setZIndex(0.0);

        for (const Position& pos : outline) {
           line->addPoint(cFigure::Point { pos.x.value(), pos.y.value() });
        }
        mDrawObstacles->addFigure(line);
    }

    return insertion.second;
}


void RoadsCanvasVisualizer::buildObstacleRtree()
{
    struct envelope_maker
    {
        inline ObstacleRtreeValue operator()(const ObstacleDB::value_type& item) const
        {
            const auto& obstacle = item.second;
            auto box = boost::geometry::return_envelope<geometry::Box>(obstacle->getOutline());
            return ObstacleRtreeValue { std::move(box), obstacle };
        }
    };

    // bulk loading of obstacles for efficient packing
    mObstacleRtree = ObstacleRtree { mObstacles | boost::adaptors::transformed(envelope_maker()) };
}


void RoadsCanvasVisualizer::clear()
{
    mObstacles.clear();
    mObstacleRtree.clear();
}

void RoadsCanvasVisualizer::initialize()
{
    cModule* traci = getModuleByPath(par("traciModule"));
    cCanvas* canvas_parent = getParentModule()->getCanvas();
    if (traci) {
        traci->subscribe(traciInitSignal, this);
        traci->subscribe(traciCloseSignal, this);
    } else {
        throw cRuntimeError("No TraCI module found for signal subscription");
    }

    if (par("drawObstacles")) {
        mDrawObstacles = new omnetpp::cGroupFigure("obstacles");
        canvas_parent->addFigure(mDrawObstacles);
    }

    std::string obstacleTypes = par("obstacleTypes");
    boost::split(mObstacleTypes, obstacleTypes, boost::is_any_of(" "));
}

void RoadsCanvasVisualizer::finish()
{
}

void RoadsCanvasVisualizer::receiveSignal(cComponent* source, simsignal_t signal, const SimTime&, cObject*)
{
    if (signal == traciInitSignal) {
        auto core = check_and_cast<traci::Core*>(source);
        fetchObstacles(*core->getAPI());
	fetchLanes(*core->getAPI());
    } else if (signal == traciCloseSignal) {
        clear();
    }
}

void RoadsCanvasVisualizer::fetchObstacles(const traci::API& traci)
{
    auto& polygons = traci.polygon;
    const traci::Boundary boundary { traci.simulation.getNetBoundary() };
    for (const std::string& id : polygons.getIDList()) {
        if (!mObstacleTypes.empty()) {
            std::string type = polygons.getType(id);
            if (mObstacleTypes.find(type) == mObstacleTypes.end()) {
                // skip polygon because its type is not in our filter set
                EV_DEBUG << "ignore polygon " << id << " of type " << type << "\n";
                continue;
            }
        }

        std::vector<Position> shape;
        for (const traci::TraCIPosition& traci_point : polygons.getShape(id).value) {
            shape.push_back(traci::position_cast(boundary, traci_point));
        }
        if (shape.size() >= 3) {
            addObstacle(id, shape);
        } else {
            EV_WARN << "skip obstacle polygon " << id << " because its shape is degraded\n";
        }
    }

    buildObstacleRtree();
}

void RoadsCanvasVisualizer::fetchLanes(const traci::API& traci)
{
    auto& lanes = traci.lane;
    const traci::Boundary boundary { traci.simulation.getNetBoundary() };

    for (const std::string& id : lanes.getIDList()) {
        std::vector<Position> shape;

        for (const traci::TraCIPosition& traci_point : lanes.getShape(id).value) {
            shape.push_back(traci::position_cast(boundary, traci_point));
        }
        addLane(id, shape);



    }
    buildObstacleRtree();
}

std::shared_ptr<EnvironmentModelObstacle> RoadsCanvasVisualizer::getObstacle(const std::string& obsId)
{
    auto found = mObstacles.find(obsId);
    return found != mObstacles.end() ? found->second : nullptr;
}

std::vector<std::shared_ptr<EnvironmentModelObstacle>>
RoadsCanvasVisualizer::preselectObstacles(const std::vector<Position>& area)
{
    boost::geometry::validity_failure_type failure;
    if (!boost::geometry::is_valid(area, failure)) {
        std::string error_msg =  boost::geometry::validity_failure_type_message(failure);
        throw omnetpp::cRuntimeError("preselection polygon is invalid: %s", error_msg.c_str());
    }

    std::vector<std::shared_ptr<EnvironmentModelObstacle>> obstacles;
    ObstacleRtree::const_query_iterator it = query_intersections(mObstacleRtree, area);
    for (; it != mObstacleRtree.qend(); ++it) {
        obstacles.push_back(it->second);
    }
    return obstacles;
}

cPolylineFigure* RoadsCanvasVisualizer::createLine(const std::vector<artery::Position>& coords, cFigure::Color color, double width, bool zoom)
{


    std::vector<cFigure::Point> points;
    for (auto coord : coords) {
        points.push_back(cFigure::Point(coord.x.value(), coord.y.value()));
    }
    auto* line = new cPolylineFigure();
    line->setPoints(points);
    line->setLineColor(color);
    line->setLineWidth(width);
    line->setZoomLineWidth(zoom);


    return line;
}

} // namespace artery
