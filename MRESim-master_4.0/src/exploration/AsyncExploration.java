package exploration;

import agents.*;
import config.Constants;
import config.SimulatorConfig;
import config.SimulatorConfig.frontiertype;
import environment.*;

import java.util.*;
import java.awt.*;
import java.io.FileNotFoundException;

import decisore.Async;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import org.jgrapht.alg.DijkstraShortestPath;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;
import path.Path;

/**
 * Classe che viene utilizzata per l'esplorazione dell'ambiente da parte dei
 * robot.
 *
 * @author Jacopo
 *
 */
public class AsyncExploration {

    // Returns new X, Y of RealAgent
    public static Point takeStep(RealAgent agent, int timeElapsed,
            SimulatorConfig.frontiertype frontierExpType, SimulatorConfig simConfig) throws FileNotFoundException {

        if (!agent.isMissionComplete()) {

            long realtimeStartAgentCycle = System.currentTimeMillis();

            /*if (agent.getM_opt() != new Point(0, 0) && (timeElapsed == Constants.INIT_CYCLES || agent.getTimeToRecalculate() == 1)) {
             Point goal = new Point(agent.getM_opt().x, agent.getM_opt().y);
             agent.setCurrentGoal(goal);
             }*/
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
                //agent.setTimeSinceLastPlan(0);
            } //</editor-fold>
            //<editor-fold defaultstate="collapsed" desc="CHECK 1: if agent hasn't moved, then he may be stuck in front of a wall.  Taking a random step might help.">
            else if (agent.getEnvError()) {
                nextStep = RandomWalk.takeStep(agent);
                System.out.println(agent.toString() + "Error, random step");
                //agent.setTimeSinceLastPlan(Constants.REPLAN_INTERVAL);
                agent.setEnvError(false);
            } else if (agent.getLocation().equals(agent.getCurrentGoal())) {
                if (!agent.getAlreadyRotated()) {
                    agent.setAlreadyRotated(true);
                    //Complete the path
                    LinkedList<Point> rotation = Path.getCellsAround(agent.getOccupancyGrid(), agent.getLocation());
                    rotation.add(agent.getLocation());
                    agent.getPath().getPoints().clear();
                    agent.getPath().getPoints().addAll(rotation);
                    nextStep = agent.getNextPathPoint();
                } else {
                    agent.setCompletedRotation(true);
                    nextStep = new Point(agent.getLocation());
                }

            } else if (agent.getPath().found && agent.getPath().getPoints().size() >= 1) {
                nextStep = agent.getNextPathPoint();
                System.out.println("Agent " + agent + " continuing on path.");

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

                Path path = agent.calculatePath(agent.getLocation(), agent.getCurrentGoal(), true);

                agent.addDirtyCells(agent.getPath().getAllPathPixels());
                agent.setPath(path);
                nextStep = agent.getNextPathPoint();

                if (nextStep != null && nextStep.equals(agent.getLocation()) && !agent.getCurrentGoal().equals(agent.getLocation())) {
                    nextStep = agent.getNextPathPoint();
                } else {
                    //System.out.println(agent.toString() + ": ERROR IN BUILD PATH!");
                    System.out.println("RANDOMMMMMMM");
                    nextStep = RandomWalk.takeStep(agent);
                }

                //</editor-fold>
                //If something goes wrong: random step
            } else {
                nextStep = RandomWalk.takeStep(agent);
                System.out.println(agent.toString() + "Random step");
            }

            agent.timeSinceLastPlan++;

            agent.timeLastContactBS++;

            agent.setLastTotalKnowledgeBelief(agent.getCurrentTotalKnowledgeBelief());
            agent.setLastBaseKnowledgeBelief(agent.getCurrentBaseKnowledgeBelief());
            agent.setLastNewInfo(agent.getNewInfo());

            System.out.println(agent.toString() + "takeStep took " + (System.currentTimeMillis() - realtimeStartAgentCycle) + "ms.");

            if (nextStep == null) {
                return null;
            }

            return new Point(nextStep);
            // If mission is complete set goal to BS
        } else {

            agent.setPathToBaseStation();
            Point nextStep = new Point(0, 0);
            nextStep = agent.getNextPathPoint();
            System.out.println("Agent " + agent + " continuing on path to BS.");

            agent.setCurrentGoal(agent.getTeammate(1).getLocation());
            return new Point(nextStep);

        }
    }

    // Calcola le frontiere, sceglie quelle interessanti e aggiorna le informazioni della BS
    public static void updateFrontiers(RealAgent agent, SimulatorConfig.frontiertype frontierExpType) {

        calculateFrontiers2(agent, frontierExpType);

        LinkedList<Frontier> previousFrontiers = agent.getFrontiers();

        LinkedList<Frontier> frontiers = frontiersOfInterest2(agent, previousFrontiers);

        agent.setFrontiers(frontiers);
        agent.updateInfo(agent.getFrontiers());

    }

    public static void updatePositionEstimate(TeammateAgent t) {
        if (t.getIntendetPath().size() == 0) {
            t.setPositionEstimate(t.getLocation());
            return;
        }

        double distance_left = t.getSpeed();
        Point nextStep;

        while (distance_left > 0 && t.getIntendetPath().size() > 0) {
            System.out.println("Teammate " + Integer.toString(t.getID()) + " SUPPOSED location is " + t.getPositionEstimate());

            nextStep = t.getIntendetPath().removeFirst();

            System.out.println("Extracted " + nextStep.toString());

            if (nextStep == null) {
                System.out.println("PANICO!");
            }

            double dist = t.getPositionEstimate().distance(nextStep);
            if (dist < 0.1) {
                break;
            }

            System.out.println("Teammate " + Integer.toString(t.getID()) + " SUPPOSED distance to next path point: " + dist);
            if (dist > distance_left) {
                Point tempDest = new Point();
                tempDest.x = nextStep.x;
                tempDest.y = nextStep.y;

                double deltaY = nextStep.y - t.getPositionEstimate().y;
                double deltaX = nextStep.x - t.getPositionEstimate().x;

                double alpha = Math.atan2(deltaY, deltaX);
                nextStep.x = (int) Math.round((double) t.getPositionEstimate().x + (Math.cos(alpha) * distance_left));
                nextStep.y = (int) Math.round((double) t.getPositionEstimate().y + (Math.sin(alpha) * distance_left));

                System.out.println("Step corrected. Now is " + nextStep.toString());

                //Here I reput the supposed step in the list, since I did not 
                //reach it, but only if it is different from the nextStep (due to approx)
                if (!tempDest.equals(nextStep)) {
                    System.out.println("Reputting in list not completed step " + tempDest.toString());
                    t.getIntendetPath().addFirst(tempDest);
                }
                distance_left = 0;
            } else {
                distance_left = distance_left - dist;
            }

            System.out.println("Position estimate: " + nextStep.toString());
            t.setPositionEstimate(nextStep);

        }

    }

    // Aggiorna le frontiere e calcola i nuovi goal in cui spostare i robot
    public static HashMap<Integer, Point> replan(RealAgent agent, SimulatorConfig.frontiertype frontierExpType, int timeElapsed, SimulatorConfig simConfig) throws FileNotFoundException {

        agent.setTimeLastUpdate(agent.getTimeLastUpdate() + 1);
        //keep up to date an estimate of where unreachable robots are
        for (TeammateAgent t : agent.getAllTeammates().values()) {
            if (t.isInRange()) {
                t.setPositionEstimate(t.getLocation());
                System.out.println(Integer.toString(t.getID()) + " in range ");
            } else {
                System.out.println(Integer.toString(t.getID()) + " not in range ");
                updatePositionEstimate(t);
            }
        }

        //check if some frontier robot has arrived to destination
        for (int frontier : agent.getArrivalTimes().keySet()) {
            System.out.println("Examining frontier " + Integer.toString(frontier));
            if (agent.getArrivalTimes().get(frontier) == -1) {
                int robot = agent.getRobotAtFrontier().get(frontier);
                System.out.println("Examining robot " + Integer.toString(robot));
                TeammateAgent t = agent.getAllTeammates().get(robot + 1);
                if (t.getCompletedRotation() && t.isInRange() && t.getLocation().distance(agent.getM_opt_p().get(robot)) <= 0.1) {
                    agent.getArrivalTimes().put(frontier, timeElapsed);
                }
            }
        }

        //additional check to avoid continuous replanning
        if (agent.getAllEqual()) {
            agent.incrementEqualCounter();
        }

        if (agent.getAllEqual() && agent.getEqualCounter() < 5) {
            System.out.println("Not replanning, time not sufficient");
            return new HashMap<Integer, Point>(agent.getM_opt_p());
        }

        if (agent.getAllEqual()) {
            agent.setAllEqual(false);
        }

        int numberReady;
        boolean replanned = false;

        if (timeElapsed == Constants.INIT_CYCLES) {
            numberReady = agent.getAllTeammates().values().size();
        } else {
            //Replan happens if the number of ready robots exceeds the predefined threshold.
            //A robot becomes ready as soon as each link BS-frontier in which it takes part is completed.        

            HashMap<Integer, ArrayList<Integer>> newWaitingList = new HashMap<Integer, ArrayList<Integer>>(agent.getWaitingLists());

            for (Map.Entry<Integer, ArrayList<Integer>> entry : agent.getWaitingLists().entrySet()) {
                int frontier = entry.getKey();
                ArrayList<Integer> robots = entry.getValue();

                int completed = 0;
                for (int robot : robots) {

                    TeammateAgent t = agent.getAllTeammates().get(robot + 1);
                    //RealAgent a = agents[robot];
                    if (t.getCompletedRotation() && t.isInRange() && t.getLocation().distance(agent.getM_opt_p().get(t.getID() - 1)) <= 0.01) {
                        completed++;
                    }
                }
                if (completed == robots.size()) {
                    newWaitingList.remove(frontier);
                    System.out.println("Removing from waiting list frontier " + Integer.toString(frontier));
                    System.out.println("Logging this delay");
                    if (agent.getTrueFrontiers().contains(frontier) && !agent.getAlreadyLoggedFrontiers().contains(frontier)) {
                        logDelayFrontier(frontier, agent.getArrivalTimes().get(frontier), timeElapsed);
                        agent.getAlreadyLoggedFrontiers().add(frontier);
                    }
                    agent.getRobotAtFrontier().remove(frontier);
                    agent.getArrivalTimes().remove(frontier);
                }

            }

            agent.setWaitingLists(newWaitingList);

            //Check if some non-ready robot becomes ready.
            for (Map.Entry<Integer, TeammateAgent> entryR : agent.getAllTeammates().entrySet()) {
                int id = entryR.getKey() - 1;
                if (id <= 0) { //BS
                    continue;
                }
                TeammateAgent teammate = entryR.getValue();
                if (teammate.isReady()) {
                    continue;
                }

                boolean present = false;
                //Check the ID presence in the waiting list.
                for (Map.Entry<Integer, ArrayList<Integer>> entry : agent.getWaitingLists().entrySet()) {
                    ArrayList<Integer> waitingRobots = entry.getValue();
                    if (waitingRobots.contains(id)) {
                        present = true;
                    }
                }

                if (!present && teammate.getCompletedRotation() && teammate.isInRange() && teammate.getLocation().distance(agent.getM_opt_p().get(teammate.getID() - 1)) <= 0.01) {
                    agent.getAllTeammates().get(entryR.getKey()).setReady(true);
                }

            }

            numberReady = 0;
            //count the number of ready robots
            for (TeammateAgent t : agent.getAllTeammates().values()) {
                if (t.isReady()) {
                    numberReady++;
                }
            }
        }

        //update wasReady (for displaying purpose)
        for (TeammateAgent t : agent.getAllTeammates().values()) {
            t.setWasReady(t.isReady());
        }

        if (numberReady >= simConfig.repl_threshold) {
            //System.out.println("aaa " + Integer.toString(timeElapsed));
            if (timeElapsed == Constants.INIT_CYCLES || agent.getTimeLastUpdate() >= 10) {
                //System.out.println("calling update frontiers");
                updateFrontiers(agent, frontiertype.ReturnWhenComplete);
                agent.setTimeLastUpdate(0);
            }

            boolean at_least_one_f = false;

            for (Frontier f : agent.getFrontiers()) {
                boolean taken = false;
                for (Map.Entry<Integer, Point> entry : agent.getM_opt_p().entrySet()) {
                    Point dest = entry.getValue();
                    if (dest.equals(f.getCentre())) {
                        taken = true;
                        System.out.println("Frontier " + f.getCentre().toString() + " is taken by robot " + Integer.toString(entry.getKey()));
                        break;
                    }
                }
                if (!taken) {
                    at_least_one_f = true;
                    break;
                }
            }

            if (at_least_one_f) {
                System.out.println(agent.toString() + "Frontiers Updated");

                HashMap<Integer, Point> m_opt = chooseFrontier(timeElapsed, agent, true, simConfig);

                //agent.setTimeToRecalculate(0);
                if (m_opt == null) {
                    return null;
                }

                replanned = true;

            } else {
                System.out.println(agent.toString() + "No Replan, no new frontier available!");
                //agent.setTimeToRecalculate(agent.getTimeToRecalculate() + 1);
                //return new HashMap<Integer, Point>(agent.getM_opt_p());
            }
            // Calcola le nuove frontiere e aggiorna tutte le variabili da passare al decisore
        } else {
            System.out.println(agent.toString() + "No Replan!");
            //agent.setTimeToRecalculate(agent.getTimeToRecalculate() + 1);
            //return new HashMap<Integer, Point>(agent.getM_opt_p());
        }
        //if(replanned){
        //    computeReallocation(agent);
        //}
        /*if(!replanned){
         //check for the event to compute a better allocation
         //reallocation happens if - at least one robot is now communicating (before it was not)
         //and a new plan has been computed since its last communication.
         boolean reallocate = false;
         for(Map.Entry<Integer, TeammateAgent> entry : agent.getAllTeammates().entrySet()){
         if(entry.getValue().isInRange() && !(entry.getValue().isPrevInRange()) 
         && (entry.getValue().getStepLastComm() < agent.getTimeLastPlan())){
         reallocate = true;
         break;
         }
         }
            
         if(reallocate){
         computeReallocation(agent);
         }
        
         }*/
        //set the last communication times
        for (Map.Entry<Integer, TeammateAgent> entry : agent.getAllTeammates().entrySet()) {
            if (entry.getValue().isInRange()) {
                entry.getValue().setStepLastComm(timeElapsed);
            }
        }

        return new HashMap<Integer, Point>(agent.getM_opt_p());

    }

    static void computeReallocation(RealAgent agent) {
        //a new allocation is computed on the whole set of non-completed frontiers.
        //currently non communicating robots will be constrained to remain at their
        //current position, while the others are free to change location.

        ArrayList< TeammateAgent> commAgents = new ArrayList<TeammateAgent>();
        ArrayList< TeammateAgent> notCommAgents = new ArrayList<TeammateAgent>();
        HashMap<Integer, Integer> notCommDest = new HashMap<Integer, Integer>();

        for (TeammateAgent t : agent.getAllTeammates().values()) {
            if (t.isReady()) {
                continue; //will be used for a new plan
            }
            if (t.isInRange()) {
                commAgents.add(t);
            } else {
                notCommAgents.add(t);
                Point dest = agent.getM_opt_p().get(t.getID() - 1);
                for (Map.Entry<Integer, Location> entry : agent.getLocationIDs().entrySet()) {
                    if (entry.getValue().getPosition().equals(dest)) {
                        notCommDest.put(t.getID(), entry.getKey());
                    }
                }
            }
        }

        ArrayList<Integer> allIds = new ArrayList<Integer>();
        HashMap<Integer, ArrayList<Integer>> waitingListsInt = new HashMap<Integer, ArrayList<Integer>>();
        for (Map.Entry<Integer, ArrayList<Integer>> entry : agent.getWaitingLists().entrySet()) {
            ArrayList<Integer> relayIds = new ArrayList<Integer>();
            int frontier = entry.getKey() - 1;

            if (!allIds.contains(frontier)) {
                allIds.add(frontier);
            }

            ArrayList<Integer> robots = entry.getValue();
            //for each robot, get its destination in terms of ID
            for (int robot : robots) {
                Point dest = agent.getM_opt_p().get(robot);
                for (Map.Entry<Integer, Location> entry2 : agent.getLocationIDs().entrySet()) {
                    if (entry2.getValue().getPosition().equals(dest)) {
                        //and there will be for sure

                        relayIds.add(entry2.getKey());
                        if (!allIds.contains(entry2.getKey())) {
                            allIds.add(entry2.getKey());
                        }

                        break;
                    }
                }
            }

            waitingListsInt.put(frontier, relayIds);

        }

        writeFileAlloc(commAgents, notCommAgents, waitingListsInt, allIds, notCommDest, agent);

        Async s = new Async();
        s.reallocate(agent);
    }

    static void writeFileAlloc(ArrayList<TeammateAgent> commAgents, ArrayList<TeammateAgent> notCommAgents,
            HashMap<Integer, ArrayList<Integer>> waitingList, ArrayList<Integer> allIds,
            HashMap<Integer, Integer> notCommDest, RealAgent bs) {

        BufferedWriter writer = null;
        try {
            writer = new BufferedWriter(new FileWriter("reallocation.txt"));

            //write the paths to allocate
            for (Map.Entry<Integer, ArrayList<Integer>> entry : waitingList.entrySet()) {
                writer.write("F " + Integer.toString(entry.getKey()));
                for (int relay : entry.getValue()) {
                    writer.write(" " + Integer.toString(relay));
                }
                writer.write("\n");
            }

            for (TeammateAgent t : commAgents) {
                String id = Integer.toString(t.getID() - 1);
                for (int node : allIds) {
                    String dist = Integer.toString((int) t.getLocation().distance(bs.getLocationIDs().get(node).getPosition()));
                    writer.write("C " + id + " " + Integer.toString(node) + " " + dist + "\n");
                }
            }

            for (TeammateAgent t : notCommAgents) {
                String id = Integer.toString(t.getID() - 1);
                for (int node : allIds) {
                    String dist = Integer.toString((int) t.getLocation().distance(bs.getLocationIDs().get(node).getPosition()));
                    writer.write("NC " + id + " " + Integer.toString(node) + " " + dist + "\n");
                }
                writer.write("NCD " + id + " " + Integer.toString(notCommDest.get(t.getID())) + "\n");
            }

            writer.close();

        } catch (IOException e) {
            e.printStackTrace();
        }
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

    private static LinkedList<Frontier> frontiersOfInterest2(RealAgent agent, LinkedList<Frontier> prevFrontiers) {
        Frontier lastFrontier = agent.getLastFrontier();
        LinkedList<Frontier> frontiers = agent.getFrontiers();
        OccupancyGrid grid = agent.getOccupancyGrid();
        ArrayList<Location> curLocs = agent.getList();

        LinkedList<Frontier> copy = new LinkedList<Frontier>();
        LinkedList<Frontier> partList = new LinkedList<Frontier>();

        for (Frontier f : frontiers) {
            copy.add(f.copy());
        }

        Frontier currFrontier;
        int counter = 0;
        for (int i = copy.size(); i > 0; i--) {
            //System.out.println("ciao");
            currFrontier = copy.poll();

            // To avoid oscillation, add last frontier to list (just in case it
            // still is the best, but is not one of the closest)
            if (currFrontier.getArea() >= Constants.MIN_FRONTIER_SIZE) {

                Path pathToFrontier = agent.calculatePath(agent.getLocation(),
                        currFrontier.getCentre());
                if (!pathToFrontier.found || //(currFrontier.getArea() < Constants.MIN_FRONTIER_SIZE * 4) &&
                        //(grid.obstacleWithinDistance(currFrontier.getCentre().x, currFrontier.getCentre().y, 1)))
                        grid.obstacleWithinDistancePath(currFrontier.getCentre().x, currFrontier.getCentre().y, 3)) {
                } else {
                    //check that it is not too close to a previous frontier
                    /*boolean tooClose = false;
                     for (Frontier f : prevFrontiers) {
                     //System.out.println("Prev frontier " + f.getCentre());
                     if (f.getCentre().distance(currFrontier.getCentre()) <= 15) {
                     tooClose = true;
                     //leave the same frontier (this reduces the size of the graph)
                     partList.add(new Frontier(f.getPolygonOutline()));
                     break;
                     }
                     }
                     if (!tooClose) {*/
                    partList.add(currFrontier);
                    counter++;
                    //}
                }
                // no need to break here, as we still want to add last frontier
                // if we haven't iterated through it yet.
            }

        }

        LinkedList<Frontier> toRemove = new LinkedList<Frontier>();

        for (Frontier f1 : partList) {
            for (Frontier f2 : partList) {
                if (f1 != f2 && f1.getCentre().equals(f2.getCentre())) {
                    toRemove.add(f2);
                }
            }
        }
        for (Frontier f : toRemove) {
            partList.remove(f);
        }

        //if (curLocs.size() == 0)
        return new LinkedList<Frontier>(partList);
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
    public static HashMap<Integer, Point> chooseFrontier(int timeElapsed, RealAgent agent, boolean considerOtherAgents, SimulatorConfig simConfig) throws FileNotFoundException {
        // Step 1:  Create list of frontiers of interest (closest ones)

        //agent.updateD();
        long startAll = System.currentTimeMillis();

        agent.updateC_V();

        agent.updateF_FA();

        agent.updateG(simConfig);

        agent.updateM_1(); //only ready agents will be considered!

        agent.updateW_p(agent.getCommGrid().getCommMatrix());

        agent.updateCurrWaitingList();

        agent.updateApprox(simConfig);
        //agent.updateR_mu();

        //additional check: the shortest path to one frontier in comm grid from BS or existing
        //non ready vertex must be shorter or equal than the number of ready robots
        /*double min_dist = -1;
         if (timeElapsed != Constants.INIT_CYCLES) {
         LinkedList<Integer> bridgeIds = new LinkedList();
         bridgeIds.add(agent.getGoalIds().get(0));
         for (TeammateAgent t : agent.getAllTeammates().values()) {
         if (t.isReady()) {
         continue;
         } else {
         bridgeIds.add(agent.getGoalIds().get(t.getID() - 1));
         }

         }

         //Here i have an id list of non replanning robots
         //build dijkstra graph
         SimpleWeightedGraph<String, DefaultWeightedEdge> graph
         = new SimpleWeightedGraph<String, DefaultWeightedEdge>(DefaultWeightedEdge.class);

         for (int i = 0; i < agent.getCommGrid().getCommMatrix().length; i++) {
         graph.addVertex(Integer.toString(i));
         }

         for (int i = 0; i < agent.getCommGrid().getCommMatrix().length; i++) {
         for (int j = i; j < agent.getCommGrid().getCommMatrix().length; j++) {
         if (agent.getCommGrid().getCommMatrix()[i][j] == 1 && i != j) {
         DefaultWeightedEdge e1 = graph.addEdge(Integer.toString(i), Integer.toString(j));
         graph.setEdgeWeight(e1, 1);
         }
         }
         }

         min_dist = -1;
         for (int id : bridgeIds) {
         //System.out.println("ID " + Integer.toString(id));
         for (int frontier : agent.getFrontierIds()) {
         //System.out.println("Frontier " + Integer.toString(frontier));
         DijkstraShortestPath sp = new DijkstraShortestPath(graph, Integer.toString(id), Integer.toString(frontier));
         if (((min_dist == -1 || sp.getPathLength() < min_dist)) && sp.getPathLength() != 0.0) {
         min_dist = sp.getPathLength();
         }
         }
         }

         }
         System.out.println("The min dist is " + Double.toString(min_dist));
         System.out.println("The number of ready robots is " + Integer.toString(numberReady));

         HashMap<Integer, Point> m_opt;
         if (timeElapsed == Constants.INIT_CYCLES || (min_dist <= numberReady && min_dist != -1)) {
         */
        //agent.writeNotReadyButComm();
        long startAlgo = System.currentTimeMillis();
        Async s = new Async();
        HashMap<Integer, Point> m_opt = s.callAsync(agent, simConfig);

        long replanTimeAll = System.currentTimeMillis() - startAll;

        long replanTimeAlgo = System.currentTimeMillis() - startAlgo;

        int replanCycles = (int) Math.ceil((double) replanTimeAll / (1000.0 * (double) SimulatorConfig.secondsPerCycle));

        if (!SimulatorConfig.instantReplan) {
            agent.setReplanCycles(replanCycles);
        }

        agent.setReplanTime(agent.getReplanTime() + replanTimeAll);

        updateReplanData((int) replanTimeAll, (int) replanTimeAlgo, replanCycles, timeElapsed, agent);

        System.out.println(agent.toString() + "replan took " + Long.toString(replanTimeAll) + "ms.");

        if (m_opt == null) {
            return null;
        }

        /*}else{
         System.out.println("Not calling python because budget is not sufficient to reach a frontier");
         m_opt = agent.getM_opt_p();
         }*/
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

    public static void calculateFrontiers2(RealAgent agent, SimulatorConfig.frontiertype frontierExpType) {
        long realtimeStart = System.currentTimeMillis();
        //System.out.print(agent.toString() + "Calculating frontiers. ");

        // If recalculating frontiers, must set old frontiers dirty for image rendering
        for (Frontier f : agent.getFrontiers()) {
            agent.addDirtyCells(f.getPolygonOutline());
        }

        LinkedList<Frontier> tmpFrontiers = Frontier.calculateFrontiers(agent.getOccupancyGrid());

        LinkedList<Frontier> frontiers = new LinkedList<Frontier>();

        int contourCounter = 0;
        for (Frontier currFrontier : tmpFrontiers) {
            if (currFrontier.getArea() >= Constants.MIN_FRONTIER_SIZE && (agent.getOccupancyGrid().freeSpaceAt(currFrontier.getCentre().x, currFrontier.getCentre().y))) {
                if (!agent.getOccupancyGrid().obstacleAt(currFrontier.getCentre().x, currFrontier.getCentre().y)) {
                    frontiers.add(currFrontier); //just to be sure
                }
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
    public static void logDelayFrontier(int frontier, int arrivalTime, int timeElapsed) {
        String userPath = System.getProperty("user.dir") + "/log_file/";

        File delay = new File(userPath + "delayFrontiers.txt");

        if (!delay.exists()) {
            BufferedWriter writer = null;
            try {
                writer = new BufferedWriter(new FileWriter(delay, false));
                String s = Integer.toString(frontier) + " " + Integer.toString(timeElapsed - arrivalTime) + "\n";
                writer.write(s);
                writer.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else {
            BufferedWriter writer = null;
            try {
                writer = new BufferedWriter(new FileWriter(delay, true));
                String s = Integer.toString(frontier) + " " + Integer.toString(timeElapsed - arrivalTime) + "\n";

                writer.write(s);
                writer.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public static void updateReplanData(int replanTimeAll, int replanTimeAlgo, int replanCycles, int timeSteps, RealAgent agent) {
        String userPath = System.getProperty("user.dir") + "/log_file/";

        File repTime = new File(userPath + "replanTimeSingle.txt");

        if (!repTime.exists()) {
            BufferedWriter writer = null;
            try {
                writer = new BufferedWriter(new FileWriter(repTime, false));
                String s = Integer.toString(timeSteps) + " " + Integer.toString(replanTimeAll) + " " + Integer.toString(replanTimeAlgo) + " " + Integer.toString(replanCycles) + " " + agent.getDimInstance() + "\n";
                writer.write(s);
                writer.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else {
            BufferedWriter writer = null;
            try {
                writer = new BufferedWriter(new FileWriter(repTime, true));
                String s = Integer.toString(timeSteps) + " " + Integer.toString(replanTimeAll) + " " + Integer.toString(replanTimeAlgo) + " " + Integer.toString(replanCycles) + " " + agent.getDimInstance() + "\n";

                writer.write(s);
                writer.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
