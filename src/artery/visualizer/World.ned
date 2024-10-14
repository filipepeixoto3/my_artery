//
// Artery V2X Simulation Framework
// Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
// Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
//

package artery.visualizer;

import artery.visualizer.RoadsCanvasVisualizer;
import artery.inet.World;

network World extends artery.inet.World
{
    parameters:
        //**.drawEnvironmentModule = default("environmentModel");
	double playgroundSizeX @unit(m); // x size of the area the nodes are in (in meters)
        double playgroundSizeY @unit(m); // y size of the area the nodes are in (in meters)
        double playgroundSizeZ @unit(m); // z size of the area the nodes are in (in meters)	
	@display("bgb=$playgroundSizeX,$playgroundSizeY");	

    submodules:
        visualizer: RoadsCanvasVisualizer {
           parameters:
		@display("p=140,120"); 
                traciModule = default("traci");
        }
}
