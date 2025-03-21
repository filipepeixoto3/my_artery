//
// Artery V2X Simulation Framework
// Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
// Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
//

package artery.visualizer;

simple RoadsCanvasVisualizer
{
    parameters:

        @display("i=misc/globe;is=s");

        string traciModule = default("traci");
        bool drawObstacles = default(true);
        string obstacleTypes = default("building shop");
        double lineWidth = default(1);  // line width of roads
        bool lineWidthZoom = default(false);  // whether zooming should affect line width
}
