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
package exploration;

import agents.RealAgent;
import config.Constants;
import config.SimulatorConfig;
import java.util.*;
import java.awt.Point;

/**
 *
 * @author julh
 */

public class RandomWalk {
    
    public static Random generator = new Random(SimulatorConfig.randomSeed);
    
    public static Point takeStep(RealAgent agent) {
        int maxcounter = 100;
        int ranVar = 0, newX = agent.getX(), newY = agent.getY(), counter = 0;
        
        boolean found = false;
        
        int acceptableDistanceToWall = Constants.WALL_DISTANCE;
        
        while (!found && counter < maxcounter) {
            // if after 50 iterations, we couldn't find new location, relax the condition about min distance to nearest wall
            if (counter > (maxcounter / 2)) acceptableDistanceToWall = 1; 
            ranVar = generator.nextInt(26);

            if(ranVar == 0)
                agent.setHeading(agent.getHeading() - Math.PI / 4);
            else if(ranVar < 3)
                agent.setHeading(agent.getHeading() - Math.PI / 8);
            else if(ranVar < 23)
                agent.setHeading(agent.getHeading() + 0);
            else if(ranVar < 25)
                agent.setHeading(agent.getHeading() + Math.PI / 8);
            else
                agent.setHeading(agent.getHeading() + Math.PI / 4);

            if(agent.getHeading() >= Math.PI) agent.setHeading(agent.getHeading() - 2*Math.PI);
            if(agent.getHeading() < -1*Math.PI) agent.setHeading(agent.getHeading() + 2*Math.PI);

            newX = agent.getX() + Math.round((float)(Constants.STEP_SIZE * Math.cos(agent.getHeading())));
            newY = agent.getY() + Math.round((float)(Constants.STEP_SIZE * Math.sin(agent.getHeading())));

            if(agent.getOccupancyGrid().locationExists(newX, newY) &&
               agent.getOccupancyGrid().directLinePossible(agent.getX(), agent.getY(), newX, newY) &&
               !agent.getOccupancyGrid().obstacleWithinDistance(newX, newY, acceptableDistanceToWall)) 
                    found = true;
            
            counter++;
        }
        // couldn't get a new location, try again but relax the condition about being next to a wall.
        while (!found && counter < 50) {
            ranVar = generator.nextInt(26);

            if(ranVar == 0)
                agent.setHeading(agent.getHeading() - Math.PI / 4);
            else if(ranVar < 3)
                agent.setHeading(agent.getHeading() - Math.PI / 8);
            else if(ranVar < 23)
                agent.setHeading(agent.getHeading() + 0);
            else if(ranVar < 25)
                agent.setHeading(agent.getHeading() + Math.PI / 8);
            else
                agent.setHeading(agent.getHeading() + Math.PI / 4);

            if(agent.getHeading() >= Math.PI) agent.setHeading(agent.getHeading() - 2*Math.PI);
            if(agent.getHeading() < -1*Math.PI) agent.setHeading(agent.getHeading() + 2*Math.PI);

            newX = agent.getX() + Math.round((float)(Constants.STEP_SIZE * Math.cos(agent.getHeading())));
            newY = agent.getY() + Math.round((float)(Constants.STEP_SIZE * Math.sin(agent.getHeading())));

            if(agent.getOccupancyGrid().locationExists(newX, newY) &&
               agent.getOccupancyGrid().directLinePossible(agent.getX(), agent.getY(), newX, newY) &&
               !agent.getOccupancyGrid().obstacleWithinDistance(newX, newY, 1)) 
                    found = true;
            
            counter++;
        }
        
        return(new Point(newX, newY));
    }
}
