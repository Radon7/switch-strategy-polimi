package environment;

import java.awt.Point;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.HashMap;
import java.util.Map;

import agents.RealAgent;
import environment.Environment.*;
import config.Constants.*;
import java.util.Iterator;
import config.SimulatorConfig;

/**
 * Each node of the graph communicates with the nodes linked to it. It's
 * represented like an adjacency matrix. Each pixel in position ij can be 0 or
 * 1: 1 if i and j can communicate, 0 if they can't communicate or if that pixel
 * is not been discovered yet.
 *
 * @author Mattia
 *
 */
public class CommunicationGrid {

    // Values easy to change depending on model, see e.g.
    // Bahl & Padmanabhan:  RADAR: An In-Building RF-based User Location and Tracking System
    private static final double REF_SIGNAL = -92; //-82;
    private static final int PATHLOSS_FACTOR = 1;
    //private static final double REF_DISTANCE = 1;
    private static final double WALL_ATTENUATION = 5;
    private static final int MAX_WALLS = 4;
    private static final double CUTOFF = -92;
    
    private int[][] commMatrix;
    private int size = 0;

    public CommunicationGrid(RealAgent agent) {
        this.size = 0;
    }
    /*public CommunicationGrid(RealAgent agent) {
     ArrayList<Location> listOfLocations = agent.getList();
     int commRange = agent.getCommRange();
     OccupancyGrid occ = agent.getOccupancyGrid();

     if (listOfLocations != null) {

     commMatrix = new int[listOfLocations.size()][listOfLocations.size()];

     for (Location loc : listOfLocations) {
     for (Location loc2 : listOfLocations) {
     if (occ.directLinePossible2((int) loc.getPosition().getX(), (int) loc.getPosition().getY(), (int) loc2.getPosition().getX(), (int) loc2.getPosition().getY())
     && loc.getPosition().distance(loc2.getPosition()) < commRange * 2) {
     commMatrix[listOfLocations.indexOf(loc)][listOfLocations.indexOf(loc2)] = 1;
     } else {
     commMatrix[listOfLocations.indexOf(loc)][listOfLocations.indexOf(loc2)] = 0;
     }
     }
     }
     }
     }*/

    public void update(RealAgent agent, SimulatorConfig simConfig) {

        HashMap<Integer, Location> locations = (HashMap) agent.getLocationIDs();
        if (locations.isEmpty()) {
            return;
        }

        int commRange = agent.getCommRange();
        OccupancyGrid occ = agent.getOccupancyGrid();

        int numLoc = agent.getLocNumber();

        int[][] oldCommMatrix = commMatrix;
        commMatrix = new int[numLoc][numLoc];

        if (size != 0) {
            //copy the old part of the matrix
            for (int i = 0; i < size; i++) {
                System.arraycopy(oldCommMatrix[i], 0, commMatrix[i], 0, size);
            }
        }
        
        for (int i = 0; i < numLoc; i++) {
            for (int j = 0; j < numLoc; j++) {
                if (i >= size || j >= size) {
                    commMatrix[i][j] = 0;
                }
            }
        }

        //for (Map.Entry<Integer, Location> entry : locations.entrySet()) {
        for (int id1 = 0; id1 < numLoc; id1 ++) {
            //Integer id1 = entry.getKey();
            //Location loc1 = entry.getValue();
            Location loc1 = locations.get(id1);
            for (int id2 = id1; id2 < numLoc; id2++) {
                //can be optimized
                /*if (id1 < size && id2 < size) {
                    continue;
                }*/
                if(commMatrix[id1][id2] == 1) continue;
                
                Location loc2 = locations.get(id2);
                
                switch (simConfig.getCommModel()) {
                    case DirectLine: {
                        if (occ.directLinePossible3((int) loc1.getPosition().getX(), (int) loc1.getPosition().getY(), (int) loc2.getPosition().getX(), (int) loc2.getPosition().getY()) &&
                            occ.directLinePossible3((int) loc2.getPosition().getX(), (int) loc2.getPosition().getY(), (int) loc1.getPosition().getX(), (int) loc1.getPosition().getY()) && loc1.getPosition().distance(loc2.getPosition()) < commRange * 2) { //TOL in movement
                            commMatrix[id1][id2] = 1;
                            commMatrix[id2][id1] = 1;
                        } else {
                            commMatrix[id1][id2] = 0;
                            commMatrix[id2][id1] = 0;
                        }
                        break;
                    }
                    case StaticCircle: {
                        if (loc1.getPosition().distance(loc2.getPosition()) < commRange * 2) { //TOL in movement
                            commMatrix[id1][id2] = 1;
                            commMatrix[id2][id1] = 1;
                        } else {
                            commMatrix[id1][id2] = 0;
                            commMatrix[id2][id1] = 0;
                        }
                        break;
                    }
                    case PropModel1:{
                        if(signalStrength(commRange, occ, loc1.getPosition(), loc2.getPosition()) > CUTOFF
                           && signalStrength(commRange, occ, loc2.getPosition(), loc1.getPosition()) > CUTOFF){
                            commMatrix[id1][id2] = 1;
                            commMatrix[id2][id1] = 1;
                        } else {
                            commMatrix[id1][id2] = 0;
                            commMatrix[id2][id1] = 0;
                        }
                        break;
                    } 
     
                }

            }
        }

        size = numLoc;
    }

    public void setCommMatrix(int[][] commMatrix) {
        this.commMatrix = commMatrix;
    }

    public int[][] getCommMatrix() {
        return commMatrix;
    }

    public boolean locationExists(int x, int y) {
        return (x < config.Constants.MAX_COLS && x >= 0 && y < config.Constants.MAX_ROWS && y >= 0);
    }
    
    private static double signalStrength(int agentRange, OccupancyGrid occ, Point p1, Point p2) {
        int numWalls = Math.min(MAX_WALLS, occ.numObstaclesOnLine(p1.x, p1.y, p2.x, p2.y));
        double distance = p1.distance(p2);
        
        return (REF_SIGNAL - 10 * PATHLOSS_FACTOR * Math.log(distance / /*REF_DISTANCE*/agentRange) - numWalls * WALL_ATTENUATION);
    }
}
