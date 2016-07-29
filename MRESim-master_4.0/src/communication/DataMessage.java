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
package communication;

import agents.BasicAgent.ExploreState;
import agents.RealAgent;
import environment.CommunicationGrid;
import environment.Frontier;
import environment.Location;
import environment.OccupancyGrid;
import environment.WeightGrid;
import exploration.RVLocation;

import java.awt.Point;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;

/**
 *
 * @author julh
 */
public class DataMessage {
    public int ID;
    public int x;
    public int y;
    public OccupancyGrid occGrid;
    public int timeLastCentralCommand;
    public int lastContactAreaKnown;
    public double pathLength;
    public RVLocation parentRendezvous;
    public RVLocation childRendezvous;
    public RVLocation parentBackupRendezvous;
    public RVLocation childBackupRendezvous;
    public boolean missionComplete;
    public boolean directComm;
    public ExploreState state;
    public double distToBase;
    public double speed;
    public int relayID;
    public double maxRateOfInfoGatheringBelief;
    public Point frontierCentre;
    
    public ArrayList<Location> list; //AGGIUNTO
    public WeightGrid weightGrid; //AGGIUNTO
    public CommunicationGrid commGrid; //AGGIUNTO
    
    public LinkedList<Frontier> frontierList; //AGGIUNTO
    
    public LinkedList<Point> intendedPath;
    
    public boolean completedRotation;
    
    public HashMap<Integer, Point> m_opt;
    
    public DataMessage(RealAgent agent, int direct) {
        ID = agent.getID();
        x = agent.getX();
        y = agent.getY();
        if (agent.getOccupancyGrid() != null)
            occGrid = agent.getOccupancyGrid().copy();
        timeLastCentralCommand = agent.getTimeLastCentralCommand();
        lastContactAreaKnown = agent.getLastContactAreaKnown();
        if(agent.getPath() != null)
            pathLength = agent.getPath().getLength();
        else
            pathLength = 0;
        if (agent.getParentRendezvous() != null)
            parentRendezvous = agent.getParentRendezvous().copy();
        if (agent.getChildRendezvous() != null)
            childRendezvous = agent.getChildRendezvous().copy();
        if (agent.getParentBackupRendezvous() != null)
            parentBackupRendezvous = agent.getParentBackupRendezvous().copy();
        if (agent.getChildBackupRendezvous() != null)
            childBackupRendezvous = agent.getChildBackupRendezvous().copy();
        missionComplete = agent.isMissionComplete();
        state = agent.getState();
        if(direct==1) directComm = true;
        else directComm = false;
        distToBase = agent.distanceToBase();
        speed = agent.getSpeed();
        relayID = agent.getID();
        maxRateOfInfoGatheringBelief = agent.getMaxRateOfInfoGatheringBelief();
        
        if(agent.getLastFrontier() != null)
            frontierCentre = new Point(agent.getLastFrontier().getClosestPoint(agent.getLocation(), agent.getOccupancyGrid()));
        else
            frontierCentre = new Point(agent.getLocation());
        
        list = agent.getList(); //AGGIUNTO
        weightGrid = agent.getWeightGrid(); //AGGIUNTO
        commGrid = agent.getCommGrid(); //AGGIUNTO
        
        frontierList = agent.getFrontiers(); //AGGIUNTO
        
        m_opt = agent.getM_opt_p();
        
        //oldLocations = agent.getOldLocations();
        
        
        this.intendedPath = new LinkedList<Point>();
        
        if(agent.getPath().getPoints() != null){
            for(Point p: agent.getPath().getPoints())
            this.intendedPath.add(new Point(p));
        }
        
        completedRotation = agent.getCompletedRotation();
    }

}
