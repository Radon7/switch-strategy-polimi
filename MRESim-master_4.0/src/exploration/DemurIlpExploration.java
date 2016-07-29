package exploration;

import agents.*;
import config.Constants;
import config.SimulatorConfig;
import config.SimulatorConfig.frontiertype;
import environment.*;

import java.util.*;
import java.awt.*;
import java.io.FileNotFoundException;

import decisore.DemurIlp;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import path.Path;

/**
 * Classe che viene utilizzata per l'esplorazione dell'ambiente da parte dei
 * robot.
 *
 * @author Mattia
 *
 */
public class DemurIlpExploration {

    // Returns new X, Y of RealAgent
    public static Point takeStep(RealAgent agent, int timeElapsed,
            SimulatorConfig.frontiertype frontierExpType, SimulatorConfig simConfig) throws FileNotFoundException {

        if (agent.getTimeToRecalculate() != 0 && !agent.isMissionComplete() && timeElapsed != Constants.INIT_CYCLES) {

            long realtimeStartAgentCycle = System.currentTimeMillis();

            if (agent.getM_opt() != new Point(0, 0) && (timeElapsed == Constants.INIT_CYCLES || agent.getTimeToRecalculate() == 1)) {
                Point goal = new Point(agent.getM_opt().x, agent.getM_opt().y);
                System.out.println("Setting goal " + goal);
                agent.setCurrentGoal(goal);
            }

            System.out.println("The goal is " + agent.getCurrentGoal());
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
            if (timeElapsed < Constants.INIT_CYCLES) {
                nextStep = RandomWalk.takeStep(agent);
                agent.setTimeSinceLastPlan(0);
            } //</editor-fold>
            //<editor-fold defaultstate="collapsed" desc="CHECK 1: if agent hasn't moved, then he may be stuck in front of a wall.  Taking a random step might help.">
            /*else if(agent.getLocation().distance(agent.getCurrentGoal()) > 0 
             && agent.getLocation().distance(agent.getCurrentGoal()) <= 5
             && agent.getPrevX() == agent.getX() && agent.getPrevY() == agent.getY()){
                            
             // mi sono bloccato in un pixel sporco
             agent.setCurrentGoal(agent.getTeammate(1).getLocation());
             Path path = agent.calculatePath(agent.getLocation(), agent.getTeammate(1).getLocation());
             agent.addDirtyCells(agent.getPath().getAllPathPixels());
             agent.setPath(path);
             agent.setBackToBS(true);
             nextStep = agent.getNextPathPoint();
             }*/ else if (agent.getEnvError()) {
                nextStep = RandomWalk.takeStep(agent);
                System.out.println(agent.toString() + "Error, random step");
                //agent.setTimeSinceLastPlan(0);
                agent.setTimeSinceLastPlan(Constants.REPLAN_INTERVAL);
                agent.setEnvError(false);
                /*} else if (agent.timeLastContactBS > Constants.MAX_TIME_NOT_COMM_BS && agent.getLocation().distance(agent.getCurrentGoal()) == 0) {
                 //Sono arrivato, ma non riesco a comunicare. Meglio aspettare muovendomi tipo a quadrato o roba simile
                 agent.setRecovery(true);
                 agent.setCurrentGoal(agent.getTeammate(1).getLocation());
                 Path path = agent.calculatePath(agent.getLocation(), agent.getTeammate(1).getLocation());
                 agent.addDirtyCells(agent.getPath().getAllPathPixels());
                 agent.setPath(path);
                 nextStep = agent.getNextPathPoint();*/
                /*} else if (agent.timeLastContactBS == 1 && agent.getRecovery() == true) {
                 agent.setRecovery(false);
                 agent.setCurrentGoal(agent.getLocation());
                 Path path = agent.calculatePath(agent.getLocation(), agent.getLocation());
                 agent.addDirtyCells(agent.getPath().getAllPathPixels());
                 agent.setPath(path);
                 nextStep = agent.getLocation();*/
            } else if (agent.getLocation().equals(agent.getCurrentGoal())) {
                System.out.println(agent.toString() + " Staying at goal.");
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
                            //if(agent.getOccupancyGrid().obstacleAt(i, j) || ! agent.getOccupancyGrid().freeSpaceAt(i, j)) continue;
                            Point mod = new Point((int) agent.getCurrentGoal().getX() + i, (int) agent.getCurrentGoal().getY() + j);

                            path = agent.calculatePath(agent.getLocation(), mod);

                            /*if (path.getPoints() == null || path.getPoints().size() == 0) {
                                path = agent.calculatePath(agent.getLocation(), mod, true);
                            }*/

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
                            //if(agent.getOccupancyGrid().obstacleAt(i, j) || ! agent.getOccupancyGrid().freeSpaceAt(i, j)) continue;
                            
                            Point mod = new Point((int) agent.getCurrentGoal().getX() + i, (int) agent.getCurrentGoal().getY() + j);

                            path = agent.calculatePath(agent.getLocation(), mod);

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
            
            //In any case (a new plan is computed, or a new step is obtained, this value is set to null
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
            System.out.println(agent.toString() + " is going back to the BS.");
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

        LinkedList<Frontier> frontiers = frontiersOfInterest2(agent);

        //Exclude those that were already in the list of locations
        /*LinkedList<Frontier> newFrontiers = new LinkedList<Frontier>();
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
         }*/
        agent.setFrontiers(frontiers);
        agent.updateInfo(agent.getFrontiers());

    }

    // Aggiorna le frontiere e calcola i nuovi goal in cui spostare i robot
    public static HashMap<Integer, Point> replan(RealAgent agent, SimulatorConfig.frontiertype frontierExpType, int timeElapsed, SimulatorConfig simConfig) throws FileNotFoundException {

        int needNewGoal = 0;

        for (TeammateAgent t : agent.getAllTeammates().values()) {
            if (agent.getM_opt_p() != null && (timeElapsed == Constants.INIT_CYCLES || (t.isInRange() && t.getLocation().distance(agent.getM_opt_p().get(t.getID() - 1)) <= 4.25))) {
                needNewGoal++;
            } else {
                System.out.println("CURR LOCATION");
                System.out.println(t.getLocation());

                System.out.println("GOAL");
                System.out.println(agent.getM_opt_p().get(t.getID() - 1));
            }

        }

        if (needNewGoal >= agent.getAllTeammates().values().size()) {

            updateFrontiers(agent, frontiertype.ReturnWhenComplete);

            System.out.println(agent.toString() + "Frontiers Updated");

            long realtimeStart = System.currentTimeMillis();

            HashMap<Integer, Point> m_opt = chooseFrontier(agent, true, simConfig);

            System.out.println(agent.toString() + "replan took " + (System.currentTimeMillis() - realtimeStart) + "ms.");

            long replanTime = System.currentTimeMillis() - realtimeStart;
            agent.setReplanTime(agent.getReplanTime() + replanTime);
            updateReplanTime((int) replanTime, timeElapsed);

            agent.setTimeToRecalculate(0);

            if (m_opt == null) {
                return null;
            }

            //agent.setTimeLastUp(timeElapsed);
            //return new HashMap<Integer, Point>(m_opt);

        } else {
            System.out.println(agent.toString() + "No Replan!");
            agent.setTimeToRecalculate(agent.getTimeToRecalculate() + 1);
            //return new HashMap<Integer, Point>(agent.getM_opt_p());
        }
        // Calcola le nuove frontiere e aggiorna tutte le variabili da passare al decisore

        return new HashMap<Integer, Point>(agent.getM_opt_p());

    }

// <editor-fold defaultstate="collapsed" desc="Choose best frontier">
    private static LinkedList<Frontier> frontiersOfInterest(Frontier lastFrontier, LinkedList<Frontier> frontiers, OccupancyGrid grid) {

        LinkedList<Frontier> copy = new LinkedList<Frontier>();
        LinkedList<Frontier> partList = new LinkedList<Frontier>();

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
                partList.add(currFrontier);
                counter++;
                if (counter > Constants.MAX_NUM_FRONTIERS) {
                    break;
                }
            } else if (currFrontier.getArea() >= Constants.MIN_FRONTIER_SIZE
                    && currFrontier.hasUnknownBoundary(grid)
                    && counter < Constants.MAX_NUM_FRONTIERS) {
                //ignore the small frontiers inside walls
                if (((currFrontier.getArea() < Constants.MIN_FRONTIER_SIZE * 4) && (grid.obstacleWithinDistance(currFrontier.getCentre().x, currFrontier.getCentre().y, 3))
                        || grid.obstacleAt(currFrontier.getCentre().x, currFrontier.getCentre().y))
                        || !grid.isKnownAtBase(currFrontier.getCentre().x, currFrontier.getCentre().y)
                        || grid.obstacleWithinDistance(currFrontier.getCentre().x, currFrontier.getCentre().y, 4)) {

                } else {
                    partList.add(currFrontier);
                    counter++;
                }
                // no need to break here, as we still want to add last frontier
                // if we haven't iterated through it yet.
            }

        }

        return new LinkedList<Frontier>(partList);
    }

    private static LinkedList<Frontier> frontiersOfInterest2(RealAgent agent) {
        Frontier lastFrontier = agent.getLastFrontier();
        LinkedList<Frontier> frontiers = agent.getFrontiers();
        OccupancyGrid grid = agent.getOccupancyGrid();
        ArrayList<Location> curLocs = agent.getList();
        int commRange = agent.getCommRange();

        LinkedList<Frontier> copy = new LinkedList<Frontier>();
        LinkedList<Frontier> partList = new LinkedList<Frontier>();
        LinkedList<Frontier> finalList = new LinkedList<Frontier>();

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
                partList.add(currFrontier);
                counter++;
                if (counter > Constants.MAX_NUM_FRONTIERS) {
                    break;
                }
            } else if (currFrontier.getArea() >= Constants.MIN_FRONTIER_SIZE
                    && currFrontier.hasUnknownBoundary(grid)
                    && counter < Constants.MAX_NUM_FRONTIERS) {
                //ignore the small frontiers inside walls
                Path pathToFrontier = agent.calculatePath(agent.getLocation(), 
                        currFrontier.getCentre());
                if (!pathToFrontier.found  || //currFrontier.getArea() < Constants.MIN_FRONTIER_SIZE * 4) &&
                       //(grid.obstacleWithinDistance(currFrontier.getCentre().x, currFrontier.getCentre().y, 1)))
                        grid.obstacleWithinDistancePath(currFrontier.getCentre().x, currFrontier.getCentre().y, 1))
                { } else {
                    partList.add(currFrontier);
                    counter++;
                }
                // no need to break here, as we still want to add last frontier
                // if we haven't iterated through it yet.
            }

        }

        //if (curLocs.size() == 0)
        return new LinkedList<Frontier>(partList);

        /*copy = new LinkedList<Frontier>();
        
         for(Frontier f : partList)
         copy.add(f.copy());
        
         for(int i=copy.size(); i>0; i--) {
         currFrontier = copy.poll();

         boolean found = false;
         for (Location curLoc : curLocs) {
         if (grid.directLinePossible2((int) currFrontier.getCentre().x, currFrontier.getCentre().y, (int) curLoc.getPosition().getX(), (int) curLoc.getPosition().getY()) &&
         curLoc.getPosition().distance(currFrontier.getCentre()) < commRange * 2) {
         finalList.add(currFrontier);
         found = true;
         break;
         }
         }
         if (found) break;
            
         for(Frontier f2 : partList){
         if (grid.directLinePossible2((int) currFrontier.getCentre().x, currFrontier.getCentre().y, (int) f2.getCentre().x, (int) f2.getCentre().y) &&
         f2.getCentre().distance(currFrontier.getCentre()) < commRange * 2) {
         finalList.add(currFrontier);
         break;
         }
         }
         }
        
         return new LinkedList<Frontier>(finalList);*/
    }

    private static double utilityEstimate(Point agentLoc, Frontier frontier) {
        return ((frontier.getArea() * 100000000) / Math.pow(agentLoc.distance(frontier.getCentre()), 4));
    }

    // Calculates Euclidean distance from all known teammates and self to frontiers of interest
    private static PriorityQueue initializeUtilities(RealAgent agent, LinkedList<Frontier> frontiers, boolean considerOtherAgents) {
        PriorityQueue<Utility> utilities = new PriorityQueue<Utility>();

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

        return new PriorityQueue<Utility>(utilities);

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

    // Calcola l'utility di ogni frontiere e associa una frontiera ad ogni robot, 
// aggiorna i parametri necessari al decisore e utilizza modello ILP per il calcolo delle posizioni finali dei robot
    public static HashMap<Integer, Point> chooseFrontier(RealAgent agent, boolean considerOtherAgents, SimulatorConfig simConfig) throws FileNotFoundException {
        // Step 1:  Create list of frontiers of interest (closest ones)

        agent.updateD();

        agent.updateC_V();

        agent.updateF_FA();

        agent.updateG(simConfig);

        agent.updateM_1(); //only ready agents will be considered!

        agent.updateW_p(agent.getCommGrid().getCommMatrix());

        agent.updateR_mu();

        //agent.writeNotReadyButComm();
        DemurIlp s = new DemurIlp();
        HashMap<Integer, Point> m_opt = s.callDemurIlp(agent, simConfig);

        if (m_opt == null) {
            return null;
        }

        return m_opt;
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

        LinkedList<Frontier> frontiers = new LinkedList<Frontier>();
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

        LinkedList<Frontier> toRemove = new LinkedList<Frontier>();

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
