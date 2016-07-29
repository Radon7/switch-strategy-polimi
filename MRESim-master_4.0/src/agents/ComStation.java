/*
 *     Copyright 2010, 2014 Julian de Hoog (julian@dehoog.ca), Victor Spirin (victor.spirin@cs.ox.ac.uk)
 *
 *     This file is part of MRESim 2.2, a simulator for testing the behaviour
 *     of multiple robots exploring unknown environments.
 *
 *     If you use MRESim, I would appreciate an acknowledgement and/or a citation
 *     of our paper:
 *
 *     @inproceedings{deHoog2009,
 *         title = "Role-Based Autonomous Multi-Robot Exploration",
 *         author = "Julian de Hoog, Stephen Cameron and Arnoud Visser",
 *         year = "2009",
 *         booktitle = "International Conference on Advanced Cognitive Technologies and Applications (COGNITIVE)",
 *         location = "Athens, Greece",
 *         month = "November",
 *     }
 *
 *     MRESim is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     MRESim is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License along with MRESim.
 *     If not, see <http://www.gnu.org/licenses/>.
 */
package agents;

import config.*;

import java.awt.Point;
import java.util.ArrayList;
import java.util.LinkedList;

import environment.CommunicationGrid;
import environment.Environment;
import environment.Location;
import environment.OccupancyGrid;
import environment.Environment.Status;
import environment.WeightGrid;


/**
 *
 * @author julh
 */
public class ComStation extends RealAgent implements Agent {

// <editor-fold defaultstate="collapsed" desc="Class variables and Constructors">


	
	    
	
    public ComStation(int envWidth, int envHeight, RobotConfig robot) {
        super(envWidth, envHeight, robot);   
        
        
    
    }
// </editor-fold>   


    
    
   
    @Override
    public void writeStep(Point nextLoc, double[] sensorData, Environment env) {
        batteryPower--;
    }
    
    
    
   
    
}
