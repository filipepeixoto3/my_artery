/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#ifndef ROADSCANVASVISUALIZER_H_
#define ROADSCANVASVISUALIZER_H_

#include "artery/envmod/Geometry.h"
#include "artery/envmod/EnvironmentModelObstacle.h"
#include "artery/utility/Geometry.h"
#include <omnetpp/ccanvas.h>
#include <omnetpp/clistener.h>
#include <omnetpp/csimplemodule.h>
#include <boost/geometry/index/rtree.hpp>
#include <unordered_map>
#include <memory>
#include <string>

#include "veins/base/utils/Coord.h"


namespace traci {
    class API;
}

namespace artery
{

class EnvironmentModelObstacle;

class RoadsCanvasVisualizer : public omnetpp::cSimpleModule, public omnetpp::cListener
{
public:
    RoadsCanvasVisualizer();
    virtual ~RoadsCanvasVisualizer();

    // cSimpleModule life-cycle
    void initialize() override;
    void finish() override;

    // cListener handlers
    void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, const omnetpp::SimTime&, omnetpp::cObject*) override;
    
    std::shared_ptr<EnvironmentModelObstacle> getObstacle(const std::string& obsId);

    std::vector<std::shared_ptr<EnvironmentModelObstacle>>
    preselectObstacles(const std::vector<Position>& area);

private:
    
    bool addObstacle(const std::string& id, std::vector<Position> outline);

    bool addLane(const std::string& id, std::vector<Position> outline);
    
    void buildObstacleRtree();

    void clear();

    void fetchObstacles(const traci::API& api);

    void fetchLanes(const traci::API& api);

    using ObstacleDB = std::unordered_map<std::string, std::shared_ptr<EnvironmentModelObstacle>>;
    using ObstacleRtreeValue = std::pair<geometry::Box, std::shared_ptr<EnvironmentModelObstacle>>;
    using ObstacleRtree = boost::geometry::index::rtree<ObstacleRtreeValue, boost::geometry::index::rstar<16>>;

    ObstacleDB mObstacles;
    ObstacleRtree mObstacleRtree;
    omnetpp::cGroupFigure* mDrawObstacles = nullptr;
    std::set<std::string> mObstacleTypes; 
    cPolylineFigure* createLine(const std::vector<artery::Position>& coords, cFigure::Color color, double width, bool zoom);
};

} // namespace artery

#endif /* ROADSCANVASVISUALIZER_H_ */
