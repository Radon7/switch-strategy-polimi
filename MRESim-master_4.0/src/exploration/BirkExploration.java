package exploration;

import agents.*;
import config.Constants;
import config.SimulatorConfig;
import config.SimulatorConfig.frontiertype;
import config.RobotConfig;
import environment.*;

import java.util.*;
import java.awt.*;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;

import path.Path;

/**
 * Classe che viene utilizzata per l'esplorazione dell'ambiente da parte dei
 * robot.
 *
 * @author Alberto
 *
 */
public class BirkExploration {

    // Returns new X, Y of RealAgent
    public static Point takeStep(RealAgent agent, int timeElapsed,
            SimulatorConfig.frontiertype frontierExpType, SimulatorConfig simConfig) throws FileNotFoundException {

        if (agent.getTimeToRecalculate() != 0 && !agent.isMissionComplete() && timeElapsed != Constants.INIT_CYCLES) {

            long realtimeStartAgentCycle = System.currentTimeMillis();

            if (agent.getM_opt() != new Point(0, 0) && (timeElapsed == Constants.INIT_CYCLES || agent.getTimeToRecalculate() == 1)) {
                Point goal = new Point(agent.getM_opt().x, agent.getM_opt().y);
                agent.setCurrentGoal(goal);
            }

            Point nextStep = new Point(agent.getX(), agent.getY());

            //<editor-fold defaultstate="collapsed" desc="If base station is in range, update timeLastDirectContactCS and lastContactAreaKnown">
            if (agent.getTeammate(Constants.BASE_STATION_ID).isInRange()) {
                agent.timeLastDirectContactCS = 1;
                agent.setLastContactAreaKnown(agent.getAreaKnown());
            } else {
                agent.timeLastDirectContactCS++;
            }

			//</editor-fold>
            //<editor-fold defaultstate="collapsed" desc="CHECK 0: Take a couple of random steps to start (just to gather some sensor data).">
            /*if(timeElapsed < Constants.INIT_CYCLES) {
             nextStep = RandomWalk.takeStep(agent);
             agent.setTimeSinceLastPlan(0);
             }*/
            //</editor-fold>
            //<editor-fold defaultstate="collapsed" desc="CHECK 1: if agent hasn't moved, then he may be stuck in front of a wall.  Taking a random step might help.">
            //else 
            if (agent.getEnvError()) {
                nextStep = RandomWalk.takeStep(agent);
                System.out.println(agent.toString() + "Error, random step");
                //agent.setTimeSinceLastPlan(0);
                agent.setTimeSinceLastPlan(Constants.REPLAN_INTERVAL);
                agent.setEnvError(false);
            } else if (agent.timeLastContactBS > Constants.MAX_TIME_NOT_COMM_BS) { // && agent.getLocation().distance(agent.getCurrentGoal()) <= 3
                agent.setRecovery(true);
                agent.setCurrentGoal(agent.getTeammate(1).getLocation());
                Path path = agent.calculatePath(agent.getLocation(), agent.getTeammate(1).getLocation());
                agent.addDirtyCells(agent.getPath().getAllPathPixels());
                agent.setPath(path);
                nextStep = agent.getNextPathPoint();
            } else if (agent.timeLastContactBS == 1 && agent.getRecovery() == true) {
                agent.setRecovery(false);
                agent.setCurrentGoal(agent.getLocation());
                Path path = agent.calculatePath(agent.getLocation(), agent.getLocation());
                agent.addDirtyCells(agent.getPath().getAllPathPixels());
                agent.setPath(path);
                nextStep = agent.getLocation();
            } else if (agent.getLocation().equals(agent.getCurrentGoal())) {
                nextStep = new Point(agent.getLocation());
                agent.setTimeToRecalculate(0);
            } /*else if(agent.getLocation().distance(agent.getCurrentGoal()) <= 2) {
             nextStep = new Point(agent.getCurrentGoal());
             agent.setTimeToRecalculate(0);
             }*/ //</editor-fold>
            //<editor-fold defaultstate="collapsed" desc="CHECK 2: Agent isn't stuck, not yet time to replan. Continue if we have points left in the previously planned path.">
            else if ((agent.timeSinceLastPlan < Constants.REPLAN_INTERVAL)
                    && agent.getPath().found && agent.getPath().getPoints().size() >= 1) {

                //First, check that I am not in the middle of a previous step.
                /*if (agent.getTempStep() != null) {

                    if (agent.getCurrentGoal().equals(agent.getLocation())) {
                        System.out.println(agent.toString() + " is already at goal.");
                        nextStep.x = agent.getX();
                        nextStep.y = agent.getY();
                    } else {

                        nextStep.x = agent.getTempStep().x;
                        nextStep.y = agent.getTempStep().y;
                        System.out.println(agent.toString() + " completing step from " + agent.getLocation() + " to " + nextStep.toString());

                        if (!agent.getOccupancyGrid().directLinePossible(agent.getX(), agent.getY(), nextStep.x, nextStep.y)) {
                            nextStep.x = agent.getX();
                            nextStep.y = agent.getY();
                            agent.setEnvError(true);
                            System.out.println(agent.toString() + " tried to cross a wall in completing step!");
                        }
                    }

                } else {*/
                    nextStep = agent.getNextPathPoint();
                    System.out.println("Agent " + agent + " continuing on path.");
                //}

            } //</editor-fold>
            //<editor-fold defaultstate="collapsed" desc="CHECK 3: Agent isn't stuck. Is it time to replan? Have we moved on to next time cycle?">
            else if (agent.getTimeSinceLastPlan() >= Constants.REPLAN_INTERVAL) {

                System.out.println(agent.toString() + "Recalculate Path");

                boolean isIn = false;
                for (Frontier f : agent.getFrontiers()) {
                    if (f.getCentre().equals(agent.getCurrentGoal())) {
                        agent.setLastFrontier(f);
                        isIn = true;
                        break;
                    }
                }
                if (!isIn) {
                    agent.setLastFrontier(null);
                }

                Path path = agent.calculatePath(agent.getLocation(), agent.getCurrentGoal());

                /*if (path.getPoints() == null || path.getPoints().size() == 0) {
                    path = agent.calculatePath(agent.getLocation(), agent.getCurrentGoal(), true);
                }*/

                agent.addDirtyCells(agent.getPath().getAllPathPixels());
                agent.setPath(path);
                nextStep = agent.getNextPathPoint();
                agent.timeSinceLastPlan = 0;

                if (nextStep == null || agent.getPath().getAllPathPixels() == null
                        || agent.getPath().getAllPathPixels().size() < 1
                        || agent.getPath() == null || agent.getPath().getPoints() == null
                        || agent.getPath().getPoints().size() < 1) {

                    for (int i = -3; i <= 3; i++) {
                        for (int j = -3; j <= 3; j++) {
                            if(agent.getOccupancyGrid().obstacleAt(i, j) || ! agent.getOccupancyGrid().freeSpaceAt(i, j)) continue;
                            
                            Point mod = new Point((int) agent.getCurrentGoal().getX() + i, (int) agent.getCurrentGoal().getY() + j);

                            path = agent.calculatePath(agent.getLocation(), mod);

                            /*if (path.getPoints() == null || path.getPoints().size() == 0) {
                                path = agent.calculatePath(agent.getLocation(), mod, true);
                            }*/

                            if(!path.found) continue;
                            agent.addDirtyCells(agent.getPath().getAllPathPixels());
                            agent.setPath(path);
                            nextStep = agent.getNextPathPoint();
                            agent.timeSinceLastPlan = 0;

                            if (nextStep != null) {
                                break;
                            }
                        }
                        if (nextStep != null) {
                            break;
                        }
                    }
                }
                if (nextStep != null && nextStep.equals(agent.getLocation()) && !agent.getCurrentGoal().equals(agent.getLocation())) {
                    nextStep = agent.getNextPathPoint();
                }
            } //</editor-fold>
            //<editor-fold defaultstate="collapsed" desc="CHECK 4: Agent isn't stuck, not yet time to replan, but we have no points left">
            else if (agent.getPath().getAllPathPixels() == null
                    || agent.getPath().getAllPathPixels().size() < 1
                    || agent.getPath() == null || agent.getPath().getPoints() == null
                    || agent.getPath().getPoints().size() < 1) {

                boolean isIn = false;
                for (Frontier f : agent.getFrontiers()) {
                    if (f.getCentre().equals(agent.getCurrentGoal())) {
                        agent.setLastFrontier(f);
                        isIn = true;
                        break;
                    }
                }
                if (!isIn) {
                    agent.setLastFrontier(null);
                }

                System.out.println(agent.toString() + "Building path");

                Path path = agent.calculatePath(agent.getLocation(), agent.getCurrentGoal());

                /*if (path.getPoints() == null || path.getPoints().size() == 0) {
                    path = agent.calculatePath(agent.getLocation(), agent.getCurrentGoal(), true);
                }*/

                agent.addDirtyCells(agent.getPath().getAllPathPixels());
                agent.setPath(path);
                nextStep = agent.getNextPathPoint();

                if (nextStep == null || agent.getPath().getAllPathPixels() == null
                        || agent.getPath().getAllPathPixels().size() < 1
                        || agent.getPath() == null || agent.getPath().getPoints() == null
                        || agent.getPath().getPoints().size() < 1) {

                    for (int i = -3; i <= 3; i++) {
                        for (int j = -3; j <= 3; j++) {
                            if(agent.getOccupancyGrid().obstacleAt(i, j) || ! agent.getOccupancyGrid().freeSpaceAt(i, j)) continue;
                            
                            Point mod = new Point((int) agent.getCurrentGoal().getX() + i, (int) agent.getCurrentGoal().getY() + j);

                            path = agent.calculatePath(agent.getLocation(), mod, true);

                            /*if (path.getPoints() == null || path.getPoints().size() == 0) {
                                path = agent.calculatePath(agent.getLocation(), mod, true);
                            }*/
                            
                            if(!path.found) continue;

                            agent.addDirtyCells(agent.getPath().getAllPathPixels());
                            agent.setPath(path);
                            nextStep = agent.getNextPathPoint();
                            agent.timeSinceLastPlan = 0;

                            if (nextStep != null) {
                                break;
                            }
                        }
                        if (nextStep != null) {
                            break;
                        }
                    }
                }
                if (nextStep != null && nextStep.equals(agent.getLocation()) && !agent.getCurrentGoal().equals(agent.getLocation())) {
                    nextStep = agent.getNextPathPoint();
                }

                //</editor-fold>
                //If something goes wrong: random step
            } else {
                nextStep = RandomWalk.takeStep(agent);
                System.out.println(agent.toString() + "Random step");
            }

            //agent.setTempStep(null);

            agent.timeSinceLastPlan++;

            agent.timeLastContactBS++;

            agent.setTimeToRecalculate(agent.getTimeToRecalculate() + 1);

            agent.setLastTotalKnowledgeBelief(agent.getCurrentTotalKnowledgeBelief());
            agent.setLastBaseKnowledgeBelief(agent.getCurrentBaseKnowledgeBelief());
            agent.setLastNewInfo(agent.getNewInfo());

            System.out.println(agent.toString() + "takeStep took " + (System.currentTimeMillis() - realtimeStartAgentCycle) + "ms.");

            if (nextStep == null) {
                return null;
            }

            return new Point(nextStep);
            // If mission is complete set goal to BS
        } else if (agent.isMissionComplete()) {
            agent.setPathToBaseStation();
            Point nextStep = new Point(0, 0);
            //First, check that I am not in the middle of a previous step.
            /*if (agent.getTempStep() != null) {

                if (agent.getCurrentGoal().equals(agent.getLocation())) {
                    System.out.println(agent.toString() + " is already at goal.");
                    nextStep.x = agent.getX();
                    nextStep.y = agent.getY();
                } else {

                    nextStep.x = agent.getTempStep().x;
                    nextStep.y = agent.getTempStep().y;
                    System.out.println(agent.toString() + " completing step from " + agent.getLocation() + " to " + nextStep.toString());

                    if (!agent.getOccupancyGrid().directLinePossible(agent.getX(), agent.getY(), nextStep.x, nextStep.y)) {
                        nextStep.x = agent.getX();
                        nextStep.y = agent.getY();
                        agent.setEnvError(true);
                        System.out.println(agent.toString() + " tried to cross a wall in completing step!");
                    }
                }

            } else {*/
                nextStep = agent.getNextPathPoint();
                System.out.println("Agent " + agent + " continuing on path to BS.");
            //}

            //agent.setTempStep(null);
            agent.setCurrentGoal(agent.getTeammate(1).getLocation());
            return new Point(nextStep);

            //Se i nuovi goal sono appena stati calcolati dalla BS, rimani fermo (deve attendere lo scambio di informazioni con la BS)	
        } else {
            agent.setTimeToRecalculate(1);
            System.out.println(agent.toString() + "Wait...");
            return new Point(agent.getLocation());
        }
    }

    // Calcola le frontiere, sceglie quelle interessanti e aggiorna le informazioni della BS
    public static void updateFrontiers(RealAgent agent, SimulatorConfig.frontiertype frontierExpType) {

        calculateFrontiers(agent, frontierExpType);

        LinkedList<Frontier> frontiers = frontiersOfInterest(agent, agent.getLastFrontier(), agent.getFrontiers(), agent.getOccupancyGrid());

        //Exclude those that were already in the list of locations
        /*LinkedList<Frontier> newFrontiers = new LinkedList<>();
         for (Frontier f : frontiers) {
         boolean found = false;
         for (Location loc : agent.getList()) {
         if (f.getCentre() == loc.getPosition() && !(agent.getFrontiers().contains(f))) {
         found = true;
         break;
         }
         }
         if (!found) {
         newFrontiers.add(f);
         }
         }
         */
        agent.setFrontiers(frontiers);
        agent.updateInfo(agent.getFrontiers());

    }

    // Aggiorna le frontiere e calcola i nuovi goal in cui spostare i robot
    public static HashMap<Integer, Point> replan(RealAgent agent, SimulatorConfig.frontiertype frontierExpType, int timeElapsed, SimulatorConfig simConfig) throws FileNotFoundException {

        int needNewGoal = 0;

        for (TeammateAgent t : agent.getAllTeammates().values()) {
            if (agent.getM_opt_p() != null && (timeElapsed == Constants.INIT_CYCLES || (t.getLocation().distance(agent.getM_opt_p().get(t.getID() - 1)) <= 0.5))) {
                needNewGoal++;
            }

        }

        if (needNewGoal == agent.getAllTeammates().values().size()) {

            agent.setTimeToRecalculate(0);

            updateFrontiers(agent, frontiertype.ReturnWhenComplete);
            /*for(Frontier f: agent.getFrontiers()) {
             for(Point p : agent.getOldLocations()) {
             if(f.getCentre() == p) {
             int index = agent.getFrontiers().indexOf(f);
             agent.getFrontiers().remove(index);
             break;
             }
             }
             }*/

            System.out.println(agent.toString() + "Frontiers Updated");

            long realtimeStart = System.currentTimeMillis();

            HashMap<Integer, Point> m_opt = chooseFrontier(agent, true);

            long replanTime = System.currentTimeMillis() - realtimeStart;
            agent.setReplanTime(agent.getReplanTime() + replanTime);
            updateReplanTime((int) replanTime, timeElapsed);

            System.out.println(agent.toString() + "replan took " + (System.currentTimeMillis() - realtimeStart) + "ms.");

            if (m_opt == null) {
                return null;
            }

            agent.setTimeToRecalculate(0);

            return new HashMap<Integer, Point>(m_opt);

            // Calcola le nuove frontiere e aggiorna tutte le variabili da passare a Stump
        } else {
            System.out.println(agent.toString() + "No Replan!");
            agent.setTimeToRecalculate(agent.getTimeToRecalculate() + 1);
            return new HashMap<Integer, Point>(agent.getM_opt_p());
        }
    }

// <editor-fold defaultstate="collapsed" desc="Choose best frontier">
    private static LinkedList<Frontier> frontiersOfInterest(RealAgent agent, Frontier lastFrontier, LinkedList<Frontier> frontiers, OccupancyGrid grid) {
        LinkedList<Frontier> copy = new LinkedList<>();
        LinkedList<Frontier> list = new LinkedList<>();

        for (Frontier f : frontiers) {
            copy.add(f.copy());
        }

        Frontier currFrontier;
        int counter = 0;
        for (int i = copy.size(); i > 0; i--) {
            currFrontier = copy.poll();

            // To avoid oscillation, add last frontier to list (just in case it
            // still is the best, but is not one of the closest)
            if (currFrontier == lastFrontier) {
                list.add(currFrontier);
                counter++;
                if (counter > Constants.MAX_NUM_FRONTIERS * 2) {
                    break;
                }
            } else if (currFrontier.getArea() >= Constants.MIN_FRONTIER_SIZE
                    && currFrontier.hasUnknownBoundary(grid)
                    && counter < Constants.MAX_NUM_FRONTIERS * 2) {
                //ignore the small frontiers inside walls
                Path pathToFrontier = agent.calculatePath(agent.getLocation(), 
                        currFrontier.getCentre());
                if (!pathToFrontier.found || //(currFrontier.getArea() < Constants.MIN_FRONTIER_SIZE * 4) && 
                       //(grid.obstacleWithinDistance(currFrontier.getCentre().x, currFrontier.getCentre().y, 1)))
                        grid.obstacleWithinDistance(currFrontier.getCentre().x, currFrontier.getCentre().y, 1))
                
                { } else {
                    list.add(currFrontier);
                    counter++;
                }
                // no need to break here, as we still want to add last frontier
                // if we haven't iterated through it yet.
            }

        }

        return new LinkedList<>(list);
    }

    private static double utilityEstimate(Point agentLoc, Frontier frontier) {
        //return agentLoc.distance(frontier.getCentre()); // Just distance
        return ((frontier.getArea() * 100000000) / Math.pow(agentLoc.distance(frontier.getCentre()), 4));
    }

    private static double utilityEstimate(Point agentLoc, Frontier frontier, RealAgent agent) {
        //long startTime = System.currentTimeMillis();
        //System.out.println("P1:" + (System.currentTimeMillis() - startTime));
        //startTime = System.currentTimeMillis();
        Path p = agent.calculatePath(agentLoc, frontier.getCentre(), true);
        //System.out.println("P2:" + (System.currentTimeMillis() - startTime));

        if (p.found) {
            //System.out.println("PATH FOUND L=" +  p.getLength());
            //System.out.println("PATH2 FOUND L=" +  p2.getLength());
            if(p.getLength() > 0)
                return ((frontier.getArea() * 100000000) / Math.pow(p.getLength(), 4));
            else
                return negBigValue;
            //return -p.getLength();
            //return (frontier.getArea() * 100000000);
        } else {
            System.out.println("PATH NOT FOUND from " + agentLoc + " to " + frontier + p.getLength());
            return negBigValue;
        }
    }

    // Calculates Euclidean distance from all known teammates and self to frontiers of interest
    private static PriorityQueue initializeUtilities(RealAgent agent, LinkedList<Frontier> frontiers, boolean considerOtherAgents) {
        PriorityQueue<Utility> utilities = new PriorityQueue<>();

        int lastCommLimit = 30;
        if (!considerOtherAgents) {
            lastCommLimit = 1;
        }

        // For each frontier of interest
        for (Frontier frontier : frontiers) {
            // Add teammates' utilities
            for (TeammateAgent teammate : agent.getAllTeammates().values()) {
                if (teammate.getID() != 1
                        && teammate.getID() != agent.getID()
                        && teammate.isExplorer()) {
                    utilities.add(new Utility(teammate.getID(),
                            teammate.getLocation(),
                            frontier,
                            utilityEstimate(teammate.getLocation(), frontier),
                            null));
                }
            }
        }

        return new PriorityQueue<>(utilities);
    }

    public static class Utility implements Comparable<Utility> {

        public int ID;
        public Point agentLocation;
        public Frontier frontier;
        public double utility;
        public Path path;

        public Utility(int id, Point al, Frontier f, double u, Path p) {
            ID = id;
            agentLocation = al;
            frontier = f;
            utility = u;
            path = p;
        }

        public int compareTo(Utility other) {
            if (other.utility > this.utility) {
                return 1;
            } else {
                return -1;
            }
        }
    }

    // Configurations of the robots at the previous step (and their utility)
    private static HashMap<Integer, Point> oldCfg_t = null;
    // Frontiers followed at the previous step
    private static ArrayList<Frontier> oldFrontiers = null;

    // timeout to remove a frontier
    private static int timeoutFrontiers = 0;
    // Blacklisted frontiers
    private static final ArrayList<Frontier> blacklist = new ArrayList<>();

    // max and min displacement
    private static final int maxDisp = 1;
    private static final int minDisp = -1;
    private static final int max_random = (maxDisp - minDisp) + 1;
    // Random generator
    public static Random generator = new Random(SimulatorConfig.randomSeed);

    // Negative big value for not feasible locations
    private static final double negBigValue = -1000000000.0;

    // Choose the next adjacent cells according to the "best" frontier (run by the BS)
    public static HashMap<Integer, Point> chooseFrontier(RealAgent agent, boolean considerOtherAgents) throws FileNotFoundException {
        // temporary variables for displacements on X and Y axes
        int randomDispX, randomDispY, displacement;
        int tmpX, tmpY;

        // Data structures for current configuration
        HashMap<Integer, Point> currentCfg_t = null;
        ArrayList<Frontier> currentFrontiers = null; // frontiers followed by robots
        ArrayList<Double> currentFrontiersValue = null;
        double currentUtility = 0.0; // Current Utility of the cfg chosen

        // Data structures for storing temporary configuration
        HashMap<Integer, Point> tmpCfg_t = new HashMap<>();
        double tmpUtility;
        ArrayList<Frontier> tmpFrontiers = new ArrayList<>(agent.getAllTeammates().values().size() + 1);
        ArrayList<Double> tmpFrontiersValue = new ArrayList<>(agent.getAllTeammates().values().size() + 1);

        // Hash maps location-utility value for optimization purpose
        HashMap<Point, Double> utilityValues = new HashMap<>();

        // Data structures to find the best frontier for a given location
        double bestFrontierValue;
        double tmpBestFrontierValue;
        Frontier bestFrontier;

        // Initialization of structures
        for (int i = 0; i < agent.getAllTeammates().size() + 1; i++) {
            tmpFrontiers.add(i, null);
            tmpFrontiersValue.add(i, negBigValue);
        }

        long startTime = System.currentTimeMillis();

        // Generation of population
        
        currentUtility = 1;
        currentCfg_t = new HashMap<>();
        
        for (int j = 2; j <= agent.getAllTeammates().size() + 1; j++) {
            TeammateAgent teammate = agent.getAllTeammates().get(j);
            currentCfg_t.put(teammate.getID() - 1, teammate.getLocation());
        }
        
        for (int i = 0; i < Constants.MAX_NUM_CFG; i++) {
            //long startTime2 = System.currentTimeMillis();
            // Clearing the temporary data structure
            tmpCfg_t.clear();

            // Compute utility of a population i
            tmpUtility = 0.0;

            // Setting the position of the BS
            tmpCfg_t.put(0, agent.getLocation());

            // Setting the position of the other agents
            // Note that their IDs start from 2 (1 is the BS) to the number of agents
            for (int j = 2; j <= agent.getAllTeammates().size() + 1; j++) {
                TeammateAgent teammate = agent.getAllTeammates().get(j);

                // Generation of random move (8-adjacency) according to default speed
                // Generating the 8-adjacent direction 
                randomDispX = (generator.nextInt(max_random) + minDisp);
                randomDispY = (generator.nextInt(max_random) + minDisp);

                // Compute the correct displacement so that the distance on the diagonal is less than or equal to the DEFAULT_SPEED
                displacement = (int) ((Math.abs(randomDispX) == Math.abs(randomDispY)) ? Math.floor(Constants.DEFAULT_SPEED * Math.cos(Math.PI / 4)) : Constants.DEFAULT_SPEED);
                // Compute the new position of the agent
                tmpX = (int) teammate.getLocation().getX() + randomDispX * displacement;
                tmpY = (int) teammate.getLocation().getY() + randomDispY * displacement;

                // Check whether the generated new position is on an obstacle or an obstacle is traversed during travel
                if (!agent.getOccupancyGrid().directLinePossible3(tmpX, tmpY, (int) teammate.getLocation().getX(), (int)teammate.getLocation().getY()) || !agent.getOccupancyGrid().directLinePossible3((int) teammate.getLocation().getX(), (int)teammate.getLocation().getY(), tmpX, tmpY) || !agent.getOccupancyGrid().freeSpaceAt(tmpX, tmpY) || agent.getOccupancyGrid().obstacleAt(tmpX, tmpY)) {
                    tmpX = (int) teammate.getLocation().getX();
                    tmpY = (int) teammate.getLocation().getY();
                }

                tmpCfg_t.put(teammate.getID() - 1, new Point(tmpX, tmpY));
            }

            // Compute the communication table
            int[][] commTable = agent.getOccupancyGrid().detectMultiHopCommunication(tmpCfg_t, agent.getCommRange());

            // Compute the utility for each agent
            for (int j = 2; j <= agent.getAllTeammates().size() + 1; j++) {
                TeammateAgent teammate = agent.getAllTeammates().get(j);
                // Check if destination cell not connected to base station
                if (commTable[0][teammate.getID() - 1] == 0) {
                    tmpUtility += negBigValue;
                    tmpFrontiers.set(teammate.getID() - 1, null);
                    tmpFrontiersValue.set(teammate.getID() - 1, negBigValue);
                } // Compute the utility of the best frontier
                else {
                    if (utilityValues.containsKey(tmpCfg_t.get(teammate.getID() - 1))) {
                        tmpUtility += utilityValues.get(tmpCfg_t.get(teammate.getID() - 1));
                    } else {
                        //System.out.println("CHECK FRONTIER");

                        //long startTime3 = System.currentTimeMillis();
                        bestFrontierValue = negBigValue;
                        bestFrontier = null;
                        // Find the "best" frontier and compute the distance
                        for (Frontier frontier : agent.getFrontiers()) {
                            // If frontier blacklisted
                            if (blacklist.contains(frontier)) {
                                tmpBestFrontierValue = negBigValue;
                                //System.out.println("BLACKLISTED FRONTIER " + String.valueOf(frontier));
                            } else {
                                tmpBestFrontierValue = utilityEstimate(tmpCfg_t.get(teammate.getID() - 1), frontier, agent);
                                //System.out.println("agent " + String.valueOf(j) + " point " +  tmpCfg_t.get(teammate.getID() - 1) + ", frontier " + frontier + String.valueOf(",d="+tmpBestFrontierValue));
                            }

                            //System.out.println("1_2_1:" + (System.currentTimeMillis() - startTime3));
                            //startTime3 = System.currentTimeMillis();
                            // If first assignment or the checked frontier is better
                            if (bestFrontier == null || bestFrontierValue < tmpBestFrontierValue) {
                                bestFrontierValue = tmpBestFrontierValue;
                                bestFrontier = frontier;
                            }

                            //System.out.println("1_2_2:" + (System.currentTimeMillis() - startTime3));
                            //startTime3 = System.currentTimeMillis();
                        }

                        // Add to the list of frontiers the best frontier found
                        tmpUtility += bestFrontierValue;
                        tmpFrontiers.set(teammate.getID() - 1, bestFrontier);
                        tmpFrontiersValue.set(teammate.getID() - 1, bestFrontierValue);
                        // Add the computed utility values to the hash map for optimization purpose
                        utilityValues.put(tmpCfg_t.get(teammate.getID() - 1), bestFrontierValue);
                    }
                }
            }
            //System.out.println("1_2:" + (System.currentTimeMillis() - startTime2));
            //startTime2 = System.currentTimeMillis();

            // Check if best configuration 
            if (tmpUtility > currentUtility) {
                currentUtility = tmpUtility;
                currentFrontiers = new ArrayList<>(tmpFrontiers);
                currentFrontiersValue = new ArrayList<>(tmpFrontiersValue);
                currentCfg_t = new HashMap<>(tmpCfg_t);
            }
            //System.out.println("1_3:" + (System.currentTimeMillis() - startTime2));
            //startTime2 = System.currentTimeMillis();
        }
        System.out.println("1:" + (System.currentTimeMillis() - startTime));
        startTime = System.currentTimeMillis();
        //System.out.println(String.valueOf("F="+agent.getFrontiers()));

        System.out.println("OLD_CFG=" + String.valueOf(oldCfg_t));
        System.out.println("OLD_FRONTIERS=" + String.valueOf(oldFrontiers));

        // Deadlock detection
        // According to position
        boolean timeoutIncreased = false;
        if (oldCfg_t != null) {
            int counterSamePosition = 1;
            // Check if robots are in the same area of the old location
            for (int i = 1; i < oldCfg_t.size(); i++) {
                if (agent.getOccupancyGrid().isNearPoint(oldCfg_t.get(i).x, oldCfg_t.get(i).y, currentCfg_t.get(i).x, currentCfg_t.get(i).y, (int) Constants.DEFAULT_SPEED)) {
                    counterSamePosition++;
                }
            }
            if (counterSamePosition == oldCfg_t.size()) {
                timeoutFrontiers++;
                timeoutIncreased = true;
                System.out.println("timeoutFrontiers=" + String.valueOf(timeoutFrontiers));
            } else {
                oldCfg_t = currentCfg_t;
            }
        } else {
            oldCfg_t = currentCfg_t;
        }

        // Deadlock detection according to frontier
        if (oldFrontiers != null) {
            if (!timeoutIncreased) {
                int counterSimilarFrontiers = 1;
                for (int i = 1; i < oldFrontiers.size(); i++) {
                    if ((oldFrontiers.get(i) != null && currentFrontiers.get(i) != null) && agent.getOccupancyGrid().isNearPoint(oldFrontiers.get(i).getCentre().x, oldFrontiers.get(i).getCentre().y, currentFrontiers.get(i).getCentre().x, currentFrontiers.get(i).getCentre().y, (int) Constants.DEFAULT_SPEED)) {
                        counterSimilarFrontiers++;
                    }
                }

                if (counterSimilarFrontiers == oldFrontiers.size()) {
                    timeoutFrontiers++;
                    System.out.println("timeoutFrontiers=" + String.valueOf(timeoutFrontiers));
                } else {
                    timeoutFrontiers = 0;
                    oldFrontiers = currentFrontiers;
                }
            } else {
                oldFrontiers = currentFrontiers;
            }
        } else {
            oldFrontiers = currentFrontiers;
        }

        // In case of deadlock, blacklist the "worst" frontier among those "followed" by the robots
        if (timeoutFrontiers > Constants.TIMEOUT_FRONTIERS) {
            try {
                tmpFrontiersValue = new ArrayList(currentFrontiersValue);
                Collections.sort(tmpFrontiersValue);
                int index = -1;
                for (Double v : tmpFrontiersValue) {
                    if (v >= 0) {
                        index = currentFrontiersValue.indexOf(v);
                        break;
                    }
                }

                if (index >= 0) {
                    blacklist.add(currentFrontiers.get(index));
                    System.out.println("Blacklist= " + String.valueOf(blacklist));
                } else {
                    blacklist.clear();
                    System.out.println("BLACKLIST CLEARED");
                }

            } catch (Exception e) {
                blacklist.clear();
                System.out.println("BLACKLIST CLEARED");
            }
            timeoutFrontiers = 0;
        }

        System.out.println("4:" + (System.currentTimeMillis() - startTime));
        startTime = System.currentTimeMillis();

        System.out.println("FRONTIERS=" + String.valueOf(currentFrontiers));
        System.out.println("CFG=" + String.valueOf(currentCfg_t) + "u=" + String.valueOf(currentUtility));

        agent.setM_opt_p(currentCfg_t);

        return currentCfg_t;
    }
// </editor-fold>     

// <editor-fold defaultstate="collapsed" desc="Calculate Frontiers">
    public static void calculateFrontiers(RealAgent agent, SimulatorConfig.frontiertype frontierExpType) {
        long realtimeStart = System.currentTimeMillis();
        //System.out.print(agent.toString() + "Calculating frontiers. ");

        // If recalculating frontiers, must set old frontiers dirty for image rendering
        for (Frontier f : agent.getFrontiers()) {
            agent.addDirtyCells(f.getPolygonOutline());
        }

        LinkedList<LinkedList> contours = ContourTracer.findAllContours(agent.getOccupancyGrid());

        LinkedList<Frontier> frontiers = new LinkedList<>();
        Frontier currFrontier;

        int contourCounter = 0;
        for (LinkedList<Point> currContour : contours) {
            currFrontier = new Frontier(agent.getX(), agent.getY(), currContour);
            // only consider frontiers that are big enough and reachable
            if (currFrontier.getArea() >= Constants.MIN_FRONTIER_SIZE && (agent.getOccupancyGrid().freeSpaceAt(currFrontier.getCentre().x, currFrontier.getCentre().y))) {
                if (!agent.getOccupancyGrid().safeSpaceAt(currFrontier.getCentre().x, currFrontier.getCentre().y)
                        && !agent.getOccupancyGrid().obstacleAt(currFrontier.getCentre().x, currFrontier.getCentre().y)) {
                    frontiers.add(currFrontier);
                    contourCounter++;
                }
            } else {

            }
        }

        LinkedList<Frontier> toRemove = new LinkedList<>();

        for (Frontier f1 : frontiers) {
            for (Frontier f2 : frontiers) {
                if (f1 != f2 && f1.getCentre().equals(f2.getCentre())) {
                    toRemove.add(f2);
                }
            }
        }
        for (Frontier f : toRemove) {
            frontiers.remove(f);
        }

        agent.setFrontiers(frontiers);

    }
// </editor-fold>

    public static void updateReplanTime(int replanTime, int timeSteps) {
        String userPath = System.getProperty("user.dir") + "/log_file/";

        File repTime = new File(userPath + "replanTimeSingle.txt");

        if (!repTime.exists()) {
            BufferedWriter writer = null;
            try {
                writer = new BufferedWriter(new FileWriter(repTime, false));
                String s = Integer.toString(replanTime) + " " + Integer.toString(timeSteps) + "\n";
                writer.write(s);
                writer.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else {
            BufferedWriter writer = null;
            try {
                writer = new BufferedWriter(new FileWriter(repTime, true));
                String s = Integer.toString(replanTime) + " " + Integer.toString(timeSteps) + "\n";
                writer.write(s);
                writer.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
