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

import communication.DataMessage;
import exploration.*;
import exploration.StumpExploration.Utility;
import environment.*;
import config.*;
import config.RobotConfig.roletype;
import environment.Environment.Status;

import java.util.*;
import java.util.List;
import java.awt.*;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;

import path.Path;

/**
 *
 * @author Julian de Hoog
 */
public class RealAgent extends BasicAgent implements Agent {

// <editor-fold defaultstate="collapsed" desc="Class variables and Constructors">
    // Data
    Point tempStep;

    int timeLastCentralCommand;   /* units of time elapsed since command 
     received from ComStation */

    double distanceTraveled;      // Distance traveled
    int areaKnown;                // How much area this agent knows about
    int areaGoal;                 // How much free space the agent needs to explore in this mission
    int prevAreaKnown; //area known last step (needed to decided whether replanning the paths yields any utility at all
    int lastContactAreaKnown;     // How much area this agent knew at the time of last contact with base
    int newInfo;                  // How much area that we know we think is not known at base station
    int prevNewInfo; //new info at last step
    int prevX, prevY;             // Previous position in environment
    public int timeLastDirectContactCS; // time since last contact with base station
    public int periodicReturnInterval;   // how long to wait until going back to BS
    public int frontierPeriodicState;   // bit of a hack: state of periodic return frontier exp robots (0=exploring; 1=returning)
    int timeElapsed;

    int relayID; //id of the agent acting as relay for this agent

    boolean envError;             // set to true by env when RealAgent's step is not legal
    public int timeSinceLastPlan;        // time passed since last plan was made

    boolean needUpdatingAreaKnown;

    // Occupancy Grid
    OccupancyGrid occGrid;
    LinkedList<Point> dirtyCells;   /* List of cells changed since last step
     (For faster update of image) */

    public LinkedList<Point> pathTaken;    // For display where robot has gone

    // Frontiers
    LinkedList<Frontier> frontiers; //MODFICATO
    Frontier lastFrontier;          // Keep track of last frontier of interest
    //goal history to prevent dejavue
    Hashtable<Point, Point> positionGoals;

    //history to backtrack if we get lost
    LinkedList<Point> backtrackPoints;

    int relayMarks;

    // Path
    Path path;
    Path pathToBase;

    // Teammates
    Hashtable<Integer, TeammateAgent> teammates;

    double maxRateOfInfoGatheringBelief;
    double currentTotalKnowledgeBelief;
    double currentBaseKnowledgeBelief;
    double lastTotalKnowledgeBelief;
    double lastBaseKnowledgeBelief;
    int lastNewInfo;

    // Role-based Exploration
    private int timeSinceLastRoleSwitch;  // keeps track of time since last switch
    private int timeSinceLastRVCalc;  // keeps track of time since last rendezvous calculation
    private RVLocation parentRendezvous;  // location of parent rendezvous
    private RVLocation parentBackupRendezvous; // location of parent backup rendezvous
    private RVLocation childRendezvous;   // location of child rendezvous
    private RVLocation childBackupRendezvous;   // location of child backup rendezvous
    private int timeUntilRendezvous; // estimated time left until due to rendezvous
    private boolean missionComplete; // true only when mission complete (env fully explored)
    private double percentageKnown;

    // rendezvous Points and Skeleton
    private LinkedList<Point> skeleton;
    private LinkedList<Point> rvPoints;

    private TopologicalMap topologicalMap;
    int timeTopologicalMapUpdated;

    // dynamic behavior
    private Point currentGoal;  // needed for calculating dynamic role switch

    private SimulatorConfig simConfig;

    private Point m_opt; //AGGIUNTO, DEVO PROVARE A FARGLIELO PASSARE DALLA BS CON UN MESSAGGIO

    private CommunicationGrid commGrid; //AGGIUNTO
    private WeightGrid weightGrid; //AGGIUNTO

    private ArrayList<Location> locList; //AGGIUNTO

    //The index will start from 0
    private Map<Integer, Location> locationIDs = new HashMap<Integer, Location>();

    private int locNumber = 0;

    private int timeToRecalculate; //AGGIUNTO

    private ArrayList<Integer> m_1; //AGGIUNTO

    private double mu; //AGGIUNTO

    private ArrayList<Integer> C; //AGGIUNTO

    private ArrayList<Integer> V; //AGGIUNTO

    private ArrayList<Integer> F; // Added

    private ArrayList<Double> FA; // Added

    int timeLastUpdate;

    private int[][] D;

    private double R;

    private int[][] D_opt;

    private int[][] adj;

    private ArrayList<Integer> m_opt_arr;

    private int[][] G;

    private int[][] w_p;

    private ArrayList<Integer> C_beta;

    private ArrayList<Integer> V_beta;

    //private ArrayList<Point> oldLocations;
    private HashMap<Integer, Point> m_opt_p;
    
    private HashMap<Integer, Integer> goalIds;

    private HashMap<Integer, ArrayList<Integer>> waitingLists;

    public int timeLastContactBS;

    private boolean recovery;

    private long replanTime;

    private HashMap<Integer, Integer> winner;

    private int[][] multiHopCommTable;

    private int[][] directHopCommTable;

    private boolean endExploration;

    private boolean deliveredUtilFinal;
    
    private String dimInstance;
    
    private int replanCycles;
    
    private HashMap<Integer, Integer> arrivalTimes;
    
    private HashMap<Integer, Integer> robotAtFrontier;
    
    private LinkedList<Integer> frontierIds;
    
    private int equalCounter;
    
    private boolean allEqual;
    
    
    private LinkedList<Integer> trueFrontiers;
    
    private LinkedList<Integer> alreadyLoggedFrontiers;
    
    private boolean alreadyRotated;
    
    private boolean completedRotation;
    
    public RealAgent(int envWidth, int envHeight, RobotConfig robot) {
        super(robot.getRobotNumber(),
                robot.getName(),
                robot.getRobotNumber(),
                robot.getStartX(),
                robot.getStartY(),
                robot.getStartHeading(),
                robot.getSensingRange(),
                robot.getCommRange(),
                robot.getBatteryLife(),
                robot.getRole(),
                robot.getParent(),
                robot.getChild(),
                Constants.DEFAULT_SPEED);

        //System.out.println("Speed is " + Double.toString(Constants.DEFAULT_SPEED));
        timeLastCentralCommand = 0;
        distanceTraveled = 0;
        areaKnown = 0;
        newInfo = 0;
        lastContactAreaKnown = 0;
        prevX = x;
        prevY = y;
        timeLastDirectContactCS = 0;
        periodicReturnInterval = 10;
        frontierPeriodicState = 0;
        envError = false;
        timeSinceLastPlan = 0;
        timeTopologicalMapUpdated = -1;
        timeElapsed = 0;
        relayID = -1;

        maxRateOfInfoGatheringBelief = 0;
        currentTotalKnowledgeBelief = 0;
        currentBaseKnowledgeBelief = 0;
        lastTotalKnowledgeBelief = -1;
        lastBaseKnowledgeBelief = -1;
        lastNewInfo = -1;

        occGrid = new OccupancyGrid(envWidth, envHeight);
        topologicalMap = new TopologicalMap(null);
        dirtyCells = new LinkedList<Point>();
        pathTaken = new LinkedList<Point>();
        backtrackPoints = new LinkedList<Point>();

        childRendezvous = new RVLocation(new Point(x, y));

        positionGoals = new Hashtable<Point, Point>();

        frontiers = new LinkedList<Frontier>();

        path = new Path();

        teammates = new Hashtable<Integer, TeammateAgent>();

        setState(BasicAgent.ExploreState.Initial);
        timeUntilRendezvous = 0;
        missionComplete = false;

        skeleton = new LinkedList<Point>();
        rvPoints = new LinkedList<Point>();
        timeSinceLastRVCalc = 0;

        currentGoal = new Point(x, y);

        //This must be the only place where NEW matrices are built.
        commGrid = new CommunicationGrid(this); //AGGIUNTO

        weightGrid = new WeightGrid(this); //AGGIUNTO
        locList = new ArrayList<Location>(); //AGGIUNTO

        m_opt = new Point(robot.getStartX(), robot.getStartY()); //AGGIUNTO

        timeToRecalculate = 1; //AGGIUNTO

        m_1 = new ArrayList<Integer>(); //AGGIUNTO

        mu = 0; //AGGIUNTO

        C = new ArrayList<Integer>(); //AGGIUNTO

        V = new ArrayList<Integer>();

        D = null;

        R = 0;

        D_opt = null;

        adj = null;

        m_opt_arr = null;

        G = null;

        w_p = null;

        C_beta = new ArrayList<Integer>();

        V_beta = new ArrayList<Integer>();

        //oldLocations = new ArrayList<Point>();
        m_opt_p = new HashMap<Integer, Point>();

        waitingLists = new HashMap<Integer, ArrayList<Integer>>();

        winner = new HashMap<Integer, Integer>();
        winner.put(1, 0); //1 ILP, 2 APP
        winner.put(2, 0);

        timeLastContactBS = 0;

        recovery = false;

        replanTime = 0;

        timeLastUpdate = 0;

        endExploration = false;

        tempStep = null;

        deliveredUtilFinal = false;
        
        dimInstance = null;
        
        replanCycles = -1;
        
        arrivalTimes = new HashMap<Integer, Integer>();
        
        robotAtFrontier = new HashMap<Integer, Integer>();
        
        goalIds = new HashMap<Integer, Integer>();
        
        frontierIds = new LinkedList<Integer>();
        
        allEqual = false;
        
        equalCounter = -1;
        
        trueFrontiers = new LinkedList<>();
        
        alreadyLoggedFrontiers = new LinkedList<>();
        
        alreadyRotated = false;
        
        completedRotation = false;
    }

// </editor-fold>     
// <editor-fold defaultstate="collapsed" desc="Get and Set">
    public void setCompletedRotation(boolean cr){
        this.completedRotation = cr;
    }
    
    public boolean getCompletedRotation(){
        return this.completedRotation;
    }
    
    public boolean getAlreadyRotated(){
        return this.alreadyRotated;
    }
    
    public void setAlreadyRotated(boolean ar){
        this.alreadyRotated = ar;
    }
    
    
    public LinkedList<Integer> getTrueFrontiers(){
        return this.trueFrontiers;
    }
    
    public LinkedList<Integer> getAlreadyLoggedFrontiers(){
        return this.alreadyLoggedFrontiers;
    }
    public void setAllEqual(boolean e){
        this.allEqual = e;
    }
    
    public boolean getAllEqual(){
        return this.allEqual;
    }
    
    public int getEqualCounter(){
        return this.equalCounter;
    }
    
    public void setEqualCounter(int ec){
        this.equalCounter = ec;
    }
    
    public void incrementEqualCounter(){
        this.equalCounter+=1;
    }
    
    public LinkedList<Integer> getFrontierIds(){
        return this.frontierIds;
    }
    
    public void setGoalIds(int robot, int frontierId){
        this.goalIds.put(robot, frontierId);
    }
    
    public HashMap<Integer, Integer> getGoalIds(){
        return this.goalIds;
    }
    
    public HashMap<Integer,Integer> getArrivalTimes(){
        return this.arrivalTimes;
    }
    
    public HashMap<Integer,Integer> getRobotAtFrontier(){
        return this.robotAtFrontier;
    }
    
    public void updateRobotAtFrontier(){
        //operates on the updated waiting list
        for(Map.Entry<Integer, ArrayList<Integer> > entry : this.waitingLists.entrySet()){
            
            int frontier = entry.getKey();
            System.out.println("On frontier " + Integer.toString(frontier));
            ArrayList<Integer> servingRobots = entry.getValue();
            int robotAt = -1;
            //find the robot in charge of being that placed on the frontier
            for(int robot: servingRobots){
                int idDest = -1;
                for(Map.Entry<Integer, Location> entry1 : this.locationIDs.entrySet()){
                    int id = entry1.getKey();
                    Point dest = entry1.getValue().getPosition();
                    
                    if(dest.equals(this.getM_opt_p().get(robot))){
                        idDest = id;
                        break;
                    }
                }
                
                if(frontier == idDest){
                    robotAt = robot;
                    break;
                }
            }
            System.out.println("Robot " + Integer.toString(robotAt));
            
            if(!robotAtFrontier.containsKey(frontier)){
                robotAtFrontier.put(frontier, robotAt);
                arrivalTimes.put(frontier, -1);
            }
            else{
                int prevRobot = robotAtFrontier.get(frontier);
                if(prevRobot != robotAt){
                    System.out.println("Changed frontier allocation!");
                    arrivalTimes.put(frontier, -1);
                    robotAtFrontier.put(frontier, robotAt);
                }      
            }
            
        }
    }
    
    public void setReplanCycles(int rc){
        this.replanCycles = rc;
    }
    
    public int getReplanCycles(){
        return this.replanCycles;
    }
    
    public void setDimInstance(String dim){
        this.dimInstance = dim;
    }
    
    public String getDimInstance(){
        return this.dimInstance;
    }

    public void setDeliveredUtilFinal() {
        this.deliveredUtilFinal = true;
    }

    public boolean getDeliveredUtilFinal() {
        return this.deliveredUtilFinal;
    }

    public void setEndExploration(boolean end) {
        this.endExploration = end;
    }

    public boolean isEndExploration() {
        return this.endExploration;
    }

    public void setMultiHopCommTable(int[][] mhct) {
        this.multiHopCommTable = mhct;
    }

    public int[][] getMultiHopCommTable() {
        return this.multiHopCommTable;
    }

    public void setDirectCommTable(int[][] dir) {
        this.directHopCommTable = dir;
    }

    public int[][] getDirectCommTable() {
        return this.directHopCommTable;
    }

    public int getTimeLastUpdate() {
        return this.timeLastUpdate;
    }

    public void setTimeLastUpdate(int t) {
        this.timeLastUpdate = t;
    }

    public HashMap<Integer, Integer> getWinner() {
        return winner;
    }

    public void updateWinner(int win) {
        winner.put(win, winner.get(win) + 1);
    }

    public void setLocationIDs(HashMap<Integer, Location> locationIDs) {
        this.locationIDs = locationIDs;
    }

    public void setLocNumber(int locNumber) {
        this.locNumber = locNumber;
    }

    public Map<Integer, Location> getLocationIDs() {
        return locationIDs;
    }

    public int getLocNumber() {
        return locNumber;
    }

    public void setRecovery(boolean recovery) {
        this.recovery = recovery;
    }

    public boolean getRecovery() {
        return recovery;
    }

    public HashMap<Integer, Point> getM_opt_p() {
        return m_opt_p;
    }

    public void setM_opt_p(HashMap<Integer, Point> m_opt_p) {
        //In this way each robot always has a destination
        System.out.println("Setting m_opt_p in BS");
        for (Map.Entry<Integer, Point> entry : m_opt_p.entrySet()) {
            this.m_opt_p.put(entry.getKey(), entry.getValue());
        }
    }

    public void setC_beta(ArrayList<Integer> c_beta) { //AGGIUNTO
        this.C_beta = c_beta;
    }

    public ArrayList<Integer> getC_beta() { //AGGIUNTO
        return this.C_beta;
    }

    public void setV_beta(ArrayList<Integer> v_beta) { //AGGIUNTO
        this.V_beta = v_beta;
    }

    public ArrayList<Integer> getV_beta() { //AGGIUNTO
        return this.V_beta;
    }

    public void setReplanTime(long replanTime) {
        this.replanTime = replanTime;
    }

    public long getReplanTime() {
        return replanTime;
    }

    /*public ArrayList<Point> getOldLocations() {
     return this.oldLocations;
     }

     public void setOldLocations(ArrayList<Point> oldLocations) {
     this.oldLocations = oldLocations;
     }*/
    public void setG(int[][] G) {
        this.G = G;
    }

    public void setAdj(int[][] adj) {
        this.adj = adj;
    }

    public void setW_p(int[][] w_p) {
        this.w_p = w_p;
    }

    public ArrayList<Integer> getM_1() {
        return this.m_1;
    }

    public double getMu() {
        return this.mu;
    }

    public double getR() {
        return this.R;
    }

    public ArrayList<Integer> getC() {
        return this.C;
    }

    public ArrayList<Integer> getV() {
        return this.V;
    }

    public int[][] getG() {
        return this.G;
    }

    public int[][] getD() {
        return this.D;
    }

    public int[][] getAdj() {
        return this.adj;
    }

    public int[][] getW_p() {
        return this.w_p;
    }

    public void setM_1(ArrayList<Integer> m_1) { //AGGIUNTO
        this.m_1 = m_1;
    }

    public void setMu(double mu) { //AGGIUNTO
        this.mu = mu;
    }

    public void setC(ArrayList<Integer> C) { //AGGIUNTO
        this.C = C;
    }

    public void setV(ArrayList<Integer> V) { //AGGIUNTO
        this.V = V;
    }

    public void setD(int[][] D) { //AGGIUNTO
        this.D = D;
    }

    public void setD_opt(int[][] D_opt) {
        this.D_opt = D_opt;
    }

    public void setR(double R) {
        this.R = R;
    }

    public ArrayList<Location> getList() { //AGGIUNTO
        return this.locList;
    }

    public WeightGrid getWeightGrid() { //AGGIUNTO
        return this.weightGrid;
    }

    public CommunicationGrid getCommGrid() { //AGGIUNTO
        return this.commGrid;
    }

    public void setM_opt_arr(ArrayList<Integer> m_opt_arr) {
        this.m_opt_arr = m_opt_arr;
    }

    public ArrayList<Integer> getM_opt_arr() {
        return this.m_opt_arr;
    }

    public Point getM_opt() { //AGGIUNTO
        return this.m_opt;
    }

    public void setM_opt(Point m_Opt) { //AGGIUNTO
        System.out.println(this.toString() + ": setting M_opt" + m_Opt);
        this.m_opt = m_Opt;
    }

    public void updateWaitingLists(HashMap<Integer, ArrayList<Integer>> waitingLists) {

        for (Map.Entry<Integer, ArrayList<Integer>> entry : waitingLists.entrySet()) {
            Integer fr = entry.getKey();
            ArrayList<Integer> robots = entry.getValue();

            if (!this.waitingLists.containsKey(fr)) {
                this.waitingLists.put(fr, robots);
            } else {
                for (Integer robot : robots) {
                    if (!(this.waitingLists.get(fr).contains(robot))) {
                        this.waitingLists.get(fr).add(robot);
                    }
                }
            }
        }

    }

    public void setWaitingLists(HashMap<Integer, ArrayList<Integer>> waitingLists) {
        this.waitingLists = waitingLists;
    }

    public HashMap<Integer, ArrayList<Integer>> getWaitingLists() {
        return waitingLists;
    }

    public void setTimeToRecalculate(int time) { //AGGIUNTO
        this.timeToRecalculate = time;
    }

    public int getTimeToRecalculate() { //AGGIUNTO
        return this.timeToRecalculate;
    }

    public void setCommGrid() { //AGGIUNTO
        this.commGrid = new CommunicationGrid(this);
    }

    public void updateCommGrid(SimulatorConfig simConfig) {
        this.commGrid.update(this, simConfig);
    }

    public void setWeightGrid() { //AGGIUNTO
        this.weightGrid = new WeightGrid(this);
    }

    public void updateWeightGrid(int[][] limitedVisMatrix) {
        this.weightGrid.update(this, limitedVisMatrix);
    }

    public void setList(ArrayList<Location> list) { //AGGIUNTO
        this.locList = list;
    }

    public void setCommRange(int commRange) { //AGGIUNTO
        this.commRange = commRange;
    }

    public int getTimeLastCentralCommand() {
        return this.timeLastCentralCommand;
    }

    public double getDistanceTraveled() {
        return this.distanceTraveled;
    }

    public int getAreaKnown() {
        return this.areaKnown;
    }

    public int getNewInfo() {
        return this.newInfo;
    }

    public int getLastContactAreaKnown() {
        return this.lastContactAreaKnown;
    }

    public void setLastContactAreaKnown(int val) {
        this.lastContactAreaKnown = val;
    }

    public int getPrevX() {
        return this.prevX;
    }

    public int getPrevY() {
        return this.prevY;
    }

    public boolean getEnvError() {
        return envError;
    }

    public void setEnvError(boolean err) {
        envError = err;
    }

    public int getTimeSinceLastPlan() {
        return timeSinceLastPlan;
    }

    public void setTimeSinceLastPlan(int t) {
        timeSinceLastPlan = t;
    }

    public OccupancyGrid getOccupancyGrid() {
        return occGrid;
    }

    public LinkedList<Point> getDirtyCells() {
        return dirtyCells;
    }

    public void setDirtyCells(LinkedList<Point> list) {
        dirtyCells = list;
    }

    public void addDirtyCells(LinkedList<Point> newDirt) {
        setDirtyCells(mergeLists(getDirtyCells(), newDirt));
    }

    public LinkedList<Frontier> getFrontiers() { //MODIFICATO; ERA PRIORITYQUEUE
        return (this.frontiers);
    }

    public void setFrontiers(LinkedList<Frontier> linkedList) {
        this.frontiers = linkedList;
    }

    /*public void updateFrontiers(LinkedList<Frontier> linkedList) {
     //this.frontiers = linkedList;
     //ListIterator<Frontier> it = linkedList.listIterator();
     //while (it.hasNext()) {
     //    if(!this.frontiers.contains(it.next())){
     //        this.frontiers.add(it.next());
     //    }
     //}
     for(int i = 0; i < linkedList.size(); i++){
     Frontier candidate = linkedList.get(i);
     if(!this.frontiers.contains(candidate)){
     this.frontiers.add(candidate);
     }
     }
     }*/
    public Frontier getLastFrontier() {
        return this.lastFrontier;
    }

    public void setLastFrontier(Frontier f) {
        this.lastFrontier = f;
    }

    public void updatePathDirt(Path p) {
        java.util.List pts = p.getPoints();
        if (pts != null) {
            for (int i = 0; i < pts.size() - 1; i++) {
                addDirtyCells(pointsAlongSegment(((Point) pts.get(i)).x, ((Point) pts.get(i)).y, ((Point) pts.get(i + 1)).x, ((Point) pts.get(i + 1)).y));
            }
        }
    }

    public Path getPath() {
        return path;
    }

    public void setPath(Path newPath) {
        this.setDirtyCells(mergeLists(dirtyCells, path.getAllPathPixels()));
        path = newPath;

        //check circularity
        /*if (path != null) {
         if (path.getPoints() != null) {
         if (path.getPoints().size() > 3) {
         boolean ok = true;
         LinkedList<Point> pathOk = new LinkedList<Point>();
         do{
         LinkedList<Point> checked = new LinkedList<Point>();
         for(Point p: path.getPoints()){
         if(checked.contains(p)){
                                
         ok = false;
         System.out.println("CIRCULAR PATH!!!");
         break;
         }
         checked.add(p);
         }
                    
         }while(!ok);
         }
         }
         }*/
    }

    public Point getNextPathPoint() {
        //System.out.println(this.toString() + " before next path point " + this.getCurrentGoal());
        if (path != null) {
            if (path.getPoints() != null) {
                if (!path.getPoints().isEmpty()) {
                    return ((Point) path.getPoints().remove(0));
                }
            }
        }

        //System.out.println(this.toString() + " after next path point " + this.getCurrentGoal());
        return null;
    }

    public void setPathToBaseStation() {
        setPath(getPathToBaseStation());
    }

    public void computePathToBaseStation() {
        long realtimeStartAgentCycle = System.currentTimeMillis();
        pathToBase = calculatePath(getLocation(), getTeammate(1).getLocation());
        System.out.println(this.toString() + "Path to base computation took " + (System.currentTimeMillis() - realtimeStartAgentCycle) + "ms.");
    }

    public Path getPathToBaseStation() {
        if ((pathToBase == null) || (pathToBase.getStartPoint().distance(this.getLocation()) > (Constants.DEFAULT_SPEED * 2))) {
            computePathToBaseStation();
        }
        return pathToBase;
    }

    public int getTimeSinceLastRoleSwitch() {
        return timeSinceLastRoleSwitch;
    }

    public void setTimeSinceLastRoleSwitch(int t) {
        timeSinceLastRoleSwitch = t;
    }

    public int getTimeSinceLastRVCalc() {
        return timeSinceLastRVCalc;
    }

    public void setTimeSinceLastRVCalc(int t) {
        timeSinceLastRVCalc = t;
    }

    public boolean isMissionComplete() {
        return missionComplete;
    }

    public void setMissionComplete() {
        missionComplete = true;
    }

    public void setMissionNotComplete() {
        missionComplete = false;
    }

    public int getTimeUntilRendezvous() {
        return timeUntilRendezvous;
    }

    public void setTimeUntilRendezvous(int n) {
        timeUntilRendezvous = n;
    }

    public LinkedList<Point> getSkeleton() {
        return skeleton;
    }

    public void setSkeleton(LinkedList<Point> list) {
        skeleton = list;
    }

    public LinkedList<Point> getRVPoints() {
        return rvPoints;
    }

    public void setRVPoints(LinkedList<Point> list) {
        rvPoints = list;
    }

    public Point getCurrentGoal() {
        return currentGoal;
    }

    public void setCurrentGoal(Point cg) {
        System.out.println("Setting current goal to " + cg);
        currentGoal = cg;
    }

    public TopologicalMap getTopologicalMap() {
        return topologicalMap;
    }

    public double getMaxRateOfInfoGatheringBelief() {
        return maxRateOfInfoGatheringBelief;
    }

    public double getCurrentTotalKnowledgeBelief() {
        return currentTotalKnowledgeBelief;
    }

    public double getCurrentBaseKnowledgeBelief() {
        return currentBaseKnowledgeBelief;
    }

    public void setLastTotalKnowledgeBelief(double val) {
        lastTotalKnowledgeBelief = val;
    }

    public void setLastBaseKnowledgeBelief(double val) {
        lastBaseKnowledgeBelief = val;
    }

    public double getLastTotalKnowledgeBelief() {
        return lastTotalKnowledgeBelief;
    }

    public double getLastBaseKnowledgeBelief() {
        return lastBaseKnowledgeBelief;
    }

    public int getLastNewInfo() {
        return lastNewInfo;
    }

    public void setLastNewInfo(int val) {
        lastNewInfo = val;
    }

    public void forceUpdateTopologicalMap() {
        System.out.println(this + " Updating topological map");
        long timeStart = System.currentTimeMillis();
        topologicalMap.setGrid(occGrid);
        System.out.println(toString() + "setGrid, " + (System.currentTimeMillis() - timeStart) + "ms.");
        topologicalMap.generateSkeleton();
        System.out.println(toString() + "generateSkeleton, " + (System.currentTimeMillis() - timeStart) + "ms.");
        topologicalMap.findKeyPoints();
        System.out.println(toString() + "findKeyPoints, " + (System.currentTimeMillis() - timeStart) + "ms.");
        topologicalMap.generateKeyAreas();
        timeTopologicalMapUpdated = timeElapsed;
        System.out.println(toString() + "generated topological map, " + (System.currentTimeMillis() - timeStart) + "ms.");
    }

// </editor-fold>    
// <editor-fold defaultstate="collapsed" desc="Parent and Child">
    public RVLocation getChildRendezvous() {
        return childRendezvous;
    }

    public void setChildRendezvous(RVLocation r) {
        childRendezvous = r;
    }

    public RVLocation getChildBackupRendezvous() {
        return childBackupRendezvous;
    }

    public void setChildBackupRendezvous(RVLocation r) {
        childBackupRendezvous = r;
    }

    public TeammateAgent getParentTeammate() {
        return getTeammate(parent);
    }

    public void setParent(int p) {
        parent = p;
    }

    public TeammateAgent getChildTeammate() {
        return getTeammate(child);
    }

    public void setChild(int c) {
        child = c;
    }

    public boolean isExplorer() {
        return (role == roletype.Explorer);
    }

    public RVLocation getParentRendezvous() {
        return parentRendezvous;
    }

    public void setParentRendezvous(RVLocation r) {
        parentRendezvous = r;
    }

    public RVLocation getParentBackupRendezvous() {
        return parentBackupRendezvous;
    }

    public void setParentBackupRendezvous(RVLocation r) {
        parentBackupRendezvous = r;
    }

// </editor-fold>     
// <editor-fold defaultstate="collapsed" desc="Teammates">
    public void addTeammate(TeammateAgent teammate) {
        teammates.put(teammate.getRobotNumber(), teammate);
    }

    public TeammateAgent getTeammate(int n) {
        return teammates.get(n);
    }

    // necessary when swapping roles with another agent
    public TeammateAgent removeTeammate(int n) {
        return teammates.remove(n);
    }

    public Hashtable<Integer, TeammateAgent> getAllTeammates() {
        return teammates;
    }

// </editor-fold>     
// <editor-fold defaultstate="collapsed" desc="Flush, take step, write step">
    // Overwrite any useless data from previous step
    public void flush() {
        prevX = x;
        prevY = y;
        for (TeammateAgent teammate : teammates.values()) {
            teammate.setInRange(false);
        }

        // Only for testing, uncommenting the line below leads to massive slow down
        //occGrid.initializeTestBits();
    }

    public Point takeStep(int timeElapsed, SimulatorConfig simConfig, RealAgent bs) throws FileNotFoundException {
        this.simConfig = simConfig;
        long realtimeStartAgentStep = System.currentTimeMillis();
        Point nextStep = new Point(0, 0);

        //previous time elapsed, used to check if we advanced to a new time cycle or are still in the old one
        int oldTimeElapsed = this.timeElapsed;
        this.timeElapsed = timeElapsed;

        //shall we go out of service?
        //if (Math.random() < Constants.PROB_OUT_OF_SERVICE)
        //if ((timeElapsed > (robotNumber * 150)) && (robotNumber > 0))
        //    setState(ExploreState.OutOfService);
        //if we are out of service, don't move, act as relay
        if (getState() == ExploreState.OutOfService) {
            setSpeed(0);
            return getLocation();
        }
        //System.out.println(this.toString() + "I am at location " + this.x + "," + this.y + ". Taking step ... ");

        if (timeElapsed == 0) {
            oldTimeElapsed = -1; //hack for initial time step
        }
        if (oldTimeElapsed != timeElapsed) {
            setDistanceToBase(getPathToBaseStation().getLength());

            switch (simConfig.getExpAlgorithm()) {
                case RunFromLog:
                    nextStep = RunFromLog.takeStep(timeElapsed, simConfig.getRunFromLogFilename(), this.robotNumber);
                    this.setHeading(RunFromLog.getHeading(timeElapsed, simConfig.getRunFromLogFilename(), this.robotNumber));
                    setState(RunFromLog.getState(timeElapsed, simConfig.getRunFromLogFilename(), this.robotNumber));
                    setRole(RunFromLog.getRole(timeElapsed, simConfig.getRunFromLogFilename(), this.robotNumber));
                    RunFromLog.getReady(timeElapsed, simConfig.getRunFromLogFilename(), this.robotNumber, bs);
                    //<editor-fold defaultstate="collapsed" desc="Make sure the GUI can display a path estimate">
                    Path straightLine = new Path();
                    straightLine.setStartPoint(getLocation());
                    straightLine.setGoalPoint(RunFromLog.getGoal(timeElapsed, simConfig.getRunFromLogFilename(), this.robotNumber));
                    straightLine.getPoints().add(straightLine.getStartPoint());
                    straightLine.getPoints().add(straightLine.getGoalPoint());
                    setPath(straightLine);
                    //</editor-fold>
                    break;
                case LeaderFollower:
                    nextStep = LeaderFollower.takeStep(this, timeElapsed);
                    break;
                case FrontierExploration:
                    if (simConfig.getFrontierAlgorithm().equals(SimulatorConfig.frontiertype.ReturnWhenComplete)) {
                        nextStep = FrontierExploration.takeStep(this,
                                timeElapsed, simConfig.getFrontierAlgorithm(), simConfig);
                    }
                    if (simConfig.getFrontierAlgorithm().equals(SimulatorConfig.frontiertype.UtilReturn)) {
                        nextStep = UtilityExploration.takeStep(this, timeElapsed, simConfig);
                    }
                    break;
                case StumpExploration:
                    if (simConfig.getFrontierAlgorithm().equals(SimulatorConfig.frontiertype.ReturnWhenComplete)) {
                        nextStep = StumpExploration.takeStep(this,
                                timeElapsed, simConfig.getFrontierAlgorithm(), simConfig);
                    }
                    break;
                case BirkExploration:
                    if (simConfig.getFrontierAlgorithm().equals(SimulatorConfig.frontiertype.ReturnWhenComplete)) {
                        nextStep = BirkExploration.takeStep(this,
                                timeElapsed, simConfig.getFrontierAlgorithm(), simConfig);
                    }
                    break;
                case IlpExploration:
                    if (simConfig.getFrontierAlgorithm().equals(SimulatorConfig.frontiertype.ReturnWhenComplete)) {
                        nextStep = AsyncExploration.takeStep(this,
                                timeElapsed, simConfig.getFrontierAlgorithm(), simConfig);
                    }
                    break;

                case ApproxExploration:
                    if (simConfig.getFrontierAlgorithm().equals(SimulatorConfig.frontiertype.ReturnWhenComplete)) {
                        nextStep = AsyncExploration.takeStep(this,
                                timeElapsed, simConfig.getFrontierAlgorithm(), simConfig);
                    }
                    break;
                case DemurIlpExploration:
                    if (simConfig.getFrontierAlgorithm().equals(SimulatorConfig.frontiertype.ReturnWhenComplete)) {
                        nextStep = DemurIlpExploration.takeStep(this,
                                timeElapsed, simConfig.getFrontierAlgorithm(), simConfig);
                    }
                    break;

                case MixedExploration:
                    if (simConfig.getFrontierAlgorithm().equals(SimulatorConfig.frontiertype.ReturnWhenComplete)) {
                        nextStep = AsyncExploration.takeStep(this,
                                timeElapsed, simConfig.getFrontierAlgorithm(), simConfig);
                    }
                    break;

                case RoleBasedExploration:
                    nextStep = RoleBasedExploration.takeStep(this, timeElapsed,
                            simConfig.useImprovedRendezvous(), simConfig.replanningAllowed(), simConfig);
                    break;
                case SwitchExploration:
                    nextStep = SwitchExploration.takestep(this,timeElapsed, simConfig.getFrontierAlgorithm(), simConfig);
                    break;
                default:
                    break;
            }
        } else {

            //First, check that I am not in the middle of a previous step.
            /*if (this.getTempStep() != null) {

             if (this.getCurrentGoal().equals(this.getLocation())) {
             System.out.println(this.toString() + " is already at goal.");
             this.tempStep = null;
             nextStep.x = this.x;
             nextStep.y = this.y;
             return nextStep;
             }

             nextStep.x = this.getTempStep().x;
             nextStep.y = this.getTempStep().y;
             System.out.println(this.toString() + " completing step from " + this.getLocation() + " to " + nextStep.toString());
             this.tempStep = null;

             if (!this.occGrid.directLinePossible(this.getX(), this.getY(), nextStep.x, nextStep.y)) {
             nextStep.x = this.x;
             nextStep.y = this.y;
             this.setEnvError(true);
             System.out.println(this.toString() + " tried to cross a wall in completing step!");
             }

             return nextStep;
             }*/
            switch (simConfig.getExpAlgorithm()) {

                case RunFromLog:
                    break;
                case LeaderFollower:
                    nextStep = this.getNextPathPoint();
                    break;
                case FrontierExploration:
                    nextStep = this.getNextPathPoint();
                    break;
                case StumpExploration:
                    nextStep = this.getNextPathPoint();
                    break;
                case IlpExploration:
                    nextStep = this.getNextPathPoint();
                    break;
                case ApproxExploration:
                    nextStep = this.getNextPathPoint();
                    break;
                case DemurIlpExploration:
                    nextStep = this.getNextPathPoint();
                    break;
                case MixedExploration:
                    nextStep = this.getNextPathPoint();
                    break;
                case BirkExploration:
                    nextStep = this.getNextPathPoint();
                    break;
                case SwitchExploration:
                    nextStep = this.getNextPathPoint();
                    break;
                case RoleBasedExploration: //TOCHECK IF NEW MOVEMENT STILL WORKS HERE
                    if ((this.getState() != ExploreState.GiveParentInfo)
                            && (this.getState() != ExploreState.GetInfoFromChild) && (this.getState() != ExploreState.WaitForChild)
                            && (this.getState() != ExploreState.WaitForParent)) {
                        nextStep = this.getNextPathPoint();
                    }
                    break;
                default:
                    break;
            }
        }

        //pruneUnexploredSpace();
        System.out.println(this.toString() + "Taking step complete, moving from " + getLocation() + " to " + nextStep + ", took " + (System.currentTimeMillis() - realtimeStartAgentStep) + "ms.");

        return nextStep;
    }

    //Since perfect sensing is assumed, I pass env object for now!
    public void writeStep(Point nextLoc, double[] sensorData, Environment env) {
        long realtimeStart = System.currentTimeMillis();
        //System.out.print(this.toString() + "Writing step ... ");

        int safeRange = (int) (sensRange * Constants.SAFE_RANGE / 100.0);
        Polygon newFreeSpace, newSafeSpace;

        x = nextLoc.x;
        y = nextLoc.y;

        pathTaken.add(new Point(x, y));

        if (!(prevX == x && prevY == y)) {
            heading = Math.atan2(y - prevY, x - prevX);
        }
        distanceTraveled += Math.sqrt(Math.pow(y - prevY, 2) + Math.pow(x - prevX, 2));
        //System.out.println("Agent " + this.ID + " distance travelled: " + distanceTraveled);

        // OLD METHOD RAY TRACING
        // Safe space slightly narrower than free space to make sure frontiers
        // are created along farthest edges of sensor radial polygon
        newFreeSpace = findRadialPolygon(sensorData, sensRange);
        newSafeSpace = findRadialPolygon(sensorData, safeRange);
        updateObstacles(newFreeSpace, env);
        updateFreeAndSafeSpace(newFreeSpace, newSafeSpace, env);

        // NEW METHOD FLOOD FILL
        //updateGrid(sensorData);
        //batteryPower--;
        batteryPower = newInfo;
        timeLastCentralCommand++;

        //System.out.println(this.toString() + "WriteStep complete, took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
    }

// </editor-fold>     
// <editor-fold defaultstate="collapsed" desc="Calculate paths">
    public Path calculatePath(Point startPoint, Point goalPoint) {
        return calculatePath(startPoint, goalPoint, false);
    }

    public Path calculatePath(Point startPoint, Point goalPoint, boolean pureAStar) {
        //topologicalMap = new TopologicalMap(occGrid);
        int rebuild_topological_map_interval = Constants.REBUILD_TOPOLOGICAL_MAP_INTERVAL;
        if (timeTopologicalMapUpdated < 0) {
            timeTopologicalMapUpdated = timeElapsed - rebuild_topological_map_interval;
        }
        if (timeElapsed - timeTopologicalMapUpdated >= rebuild_topological_map_interval) {
            forceUpdateTopologicalMap();
        }
        topologicalMap.setPathStart(startPoint);
        topologicalMap.setPathGoal(goalPoint);
        if (!pureAStar) {
            topologicalMap.getTopologicalPath();
        } else {
            //topologicalMap.getJumpPath();
            topologicalMap.getAStarPath();
            if (topologicalMap.getPath() == null) {
                System.out.println("!!!! CATASTROPHIC FAILURE !!!!!");
            }
        }
        return topologicalMap.getPath();
        //return new Path(occGrid, startPoint, goalPoint, false);
    }

    private LinkedList<Point> pointsAlongSegment(int x1, int y1, int x2, int y2) {
        LinkedList<Point> pts = new LinkedList<Point>();

        for (int i = Math.min(x1, x2); i <= Math.max(x1, x2); i++) {
            for (int j = Math.min(y1, y2); j <= Math.max(y1, y2); j++) {
                if (occGrid.distPointToLine(x1, y1, x2, y2, i, j) < 0.5) {
                    pts.add(new Point(i, j));
                }
            }
        }

        return pts;
    }
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Updating">
    public boolean isNeedUpdatePaths() {
        return ((areaKnown != prevAreaKnown) || (newInfo != prevNewInfo)
                || ((getState() == ExploreState.ReturnToParent) && (newInfo == 0)));
    }

    public boolean isNeedUpdateAreaKnown() {
        return needUpdatingAreaKnown;
    }

    public double getPercentageKnown() {
        return percentageKnown;
    }

    public void setGoalArea(int goalArea) {
        areaGoal = goalArea;
    }

    // update stats of what we know about the environment
    public void updateAreaKnown() {
        int counter = 0;
        int new_counter = 0;
        int gotRelayed = 0;
        int baseCounter = 0;
        for (int i = 0; i < occGrid.width; i++) {
            for (int j = 0; j < occGrid.height; j++) {
                if (occGrid.freeSpaceAt(i, j)) {
                    counter++;
                    if ((!occGrid.isKnownAtBase(i, j)) && (!occGrid.isGotRelayed(i, j))) {
                        new_counter++;
                    }
                    if ((!occGrid.isKnownAtBase(i, j)) && (occGrid.isGotRelayed(i, j))) {
                        gotRelayed++;
                    }
                    if (occGrid.isKnownAtBase(i, j))/* || occGrid.isGotRelayed(i, j))*/ {
                        baseCounter++;
                    }
                }
            }
        }

        prevAreaKnown = areaKnown;
        areaKnown = counter;
        prevNewInfo = newInfo;
        newInfo = new_counter;
        needUpdatingAreaKnown = false;
        percentageKnown = (double) areaKnown / (double) areaGoal;
        currentBaseKnowledgeBelief = baseCounter + gotRelayed; //can add them up, as they are disjoint;
        // may be a good idea to add a discounted value for gotRelayed, as we are not sure it is going to be delivered
        // to base soon. The reason we incorporate gotRelayed to reduce the probability of agents trying to go back to base
        // before the ratio is hit, and then having to go back to exploring when they learn that base station knows more
        // information than they thought, resulting in wasted effort.
        currentTotalKnowledgeBelief = getCurrentBaseKnowledgeBelief() + newInfo * (teammates.size() - 1);

        if (timeElapsed > 0) {
            double rateOfInfoGatheringBelief = (double) areaKnown / (double) timeElapsed;
            if (rateOfInfoGatheringBelief > maxRateOfInfoGatheringBelief) {
                maxRateOfInfoGatheringBelief = rateOfInfoGatheringBelief;
            }
        }
    }

    public void updateAreaRelayed(TeammateAgent ag) {
        if (ag.robotNumber == robotNumber) //we are the same as ag, nothing to do here
        {
            return;
        }
        if (getID() == Constants.BASE_STATION_ID) //base station
        {
            return;
        }
        int new_counter = 0;
        boolean iAmCloserToBase = (ag.timeToBase() > timeToBase());
        /*if (iAmCloserToBase)
         System.out.println(toString() + " relaying for " + ag.name + " (" + timeToBase() + " vs " + ag.timeToBase() + ")");
         else
         System.out.println(ag.name + " relaying for " + toString() + " (" + ag.timeToBase() + " vs " + timeToBase() + ")");
         */
        if (iAmCloserToBase) {
            for (int i = 0; i < ag.occGrid.width; i++) {
                for (int j = 0; j < ag.occGrid.height; j++) {
                    if ((ag.occGrid.freeSpaceAt(i, j))
                            && (!ag.occGrid.isKnownAtBase(i, j))
                            && (!ag.occGrid.isGotRelayed(i, j))) {
                        if (occGrid.isGotRelayed(i, j)) {
                            occGrid.setGotUnrelayed(i, j);
                            new_counter++;
                        }
                    }
                }
            }
            if (new_counter > 0) {
                if (newInfo == 0) {
                    newInfo = 1;
                }
                System.out.println(toString() + "setGotUnrelayed: " + new_counter);
            }
        } else {
            if (newInfo > 0) {
                for (int i = 0; i < occGrid.width; i++) {
                    for (int j = 0; j < occGrid.height; j++) {
                        if ((occGrid.freeSpaceAt(i, j))
                                && (!occGrid.isKnownAtBase(i, j))
                                && (!occGrid.isGotRelayed(i, j))) {
                            occGrid.setGotRelayed(i, j);
                            new_counter++;
                        }
                    }
                }
                if (new_counter > 0) {
                    System.out.println(toString() + "setGotRelayed: " + new_counter);
                }
            }
        }
    }

    protected void updateGrid(double sensorData[]) {
        Point first, second;
        double dist, angle;
        int currX, currY;
        double currRayAngle;
        int currRayX, currRayY;

        //If there's no sensor data, done
        if (sensorData == null) {
            return;
        }

        int safeRange = (int) (sensRange * Constants.SAFE_RANGE / 100.0);

        // To create a polygon, add robot at start and end
        Polygon polygon = new Polygon();
        polygon.addPoint(x, y);

        //For every degree
        for (int i = 0; i <= 180; i += 1) {
            currRayAngle = heading - Math.PI / 2 + Math.PI / 180 * i;

            if (sensorData[i] >= sensRange) {
                currRayX = x + (int) Math.floor(sensRange * (Math.cos(currRayAngle)));
                currRayY = y + (int) Math.floor(sensRange * (Math.sin(currRayAngle)));
            } else {
                currRayX = x + (int) Math.floor(sensorData[i] * (Math.cos(currRayAngle)));
                currRayY = y + (int) Math.floor(sensorData[i] * (Math.sin(currRayAngle)));
            }

            polygon.addPoint(currRayX, currRayY);
        }

        polygon.addPoint(x, y);

        // determine mins and maxes of polygon
        int xmin = polygon.getBounds().x;
        int ymin = polygon.getBounds().y;

        // Create temp grid 
        int[][] tempGrid = new int[polygon.getBounds().width + 1][polygon.getBounds().height + 1];
        for (int i = 0; i < polygon.getBounds().width + 1; i++) {
            for (int j = 0; j < polygon.getBounds().height + 1; j++) {
                tempGrid[i][j] = 0;
            }
        }

        /**
         * ******************************************
         */
        //   I   Make outline of polygon
        // go through all points in scan, set lines between them to obstacle
        for (int i = 0; i < polygon.npoints - 1; i++) {
            first = new Point(polygon.xpoints[i], polygon.ypoints[i]);
            second = new Point(polygon.xpoints[i + 1], polygon.ypoints[i + 1]);
            dist = first.distance(second);
            angle = Math.atan2(second.y - first.y, second.x - first.x);
            for (int j = 0; j < dist; j++) {
                currX = (int) Math.floor((first.x + j * Math.cos(angle)));
                currY = (int) Math.floor((first.y + j * Math.sin(angle)));
                tempGrid[currX - xmin][currY - ymin] = 1;
            }
        }

        /**
         * ******************************************
         */
        //   II  Fill free space from inside
        // first, create
        LinkedList<Point> toFill = new LinkedList<Point>();
        Point start = new Point((int) (x + 1 * Math.cos(heading) - xmin), (int) (y + 1 * Math.sin(heading) - ymin));

        // if start is obstacle (robot right in front of wall), don't bother updating anything
        if (occGrid.obstacleAt(start.x, start.y)) {
            return;
        }

        Point curr;
        toFill.add(start);

        while (!toFill.isEmpty()) {
            curr = toFill.pop();
            tempGrid[curr.x][curr.y] = 2;

            // add neighbours
            if (curr.x - 1 >= 0 && tempGrid[curr.x - 1][curr.y] == 0) {
                tempGrid[curr.x - 1][curr.y] = -1;
                toFill.add(new Point(curr.x - 1, curr.y));
            }
            if (curr.x + 1 <= polygon.getBounds().width && tempGrid[curr.x + 1][curr.y] == 0) {
                tempGrid[curr.x + 1][curr.y] = -1;
                toFill.add(new Point(curr.x + 1, curr.y));
            }
            if (curr.y - 1 >= 0 && tempGrid[curr.x][curr.y - 1] == 0) {
                tempGrid[curr.x][curr.y - 1] = -1;
                toFill.add(new Point(curr.x, curr.y - 1));
            }
            if (curr.y + 1 <= polygon.getBounds().height && tempGrid[curr.x][curr.y + 1] == 0) {
                tempGrid[curr.x][curr.y + 1] = -1;
                toFill.add(new Point(curr.x, curr.y + 1));
            }
        }

        /**
         * ******************************************
         */
        //   III Fill in obstacles
        // go through all points in polygon (avoid first and last because it's the robot)
        for (int i = 1; i < polygon.npoints - 2; i++) {
            first = new Point(polygon.xpoints[i], polygon.ypoints[i]);
            second = new Point(polygon.xpoints[i + 1], polygon.ypoints[i + 1]);

            // if they are close enough, fill line between them
            if (first.distance(x, y) < (sensRange - 2) && second.distance(x, y) < (sensRange - 2)
                    && first.distance(second) < 5) {
                angle = Math.atan2(second.y - first.y, second.x - first.x);
                for (double j = 0; j < first.distance(second); j++) {
                    currX = (int) (first.x + j * Math.cos(angle));
                    currY = (int) (first.y + j * Math.sin(angle));
                    tempGrid[currX - xmin][currY - ymin] = 4;
                }
            }
        }

        /**
         * ******************************************
         */
        //   IV  Update real grid
        for (int i = 0; i < polygon.getBounds().width; i++) {
            for (int j = 0; j < polygon.getBounds().height; j++) {
                if (tempGrid[i][j] == 4) {
                    occGrid.setObstacleAt(i + xmin, j + ymin);
                    dirtyCells.add(new Point(i + xmin, j + ymin));
                } else if (tempGrid[i][j] == 2) {
                    if ((new Point(x, y)).distance(new Point(i + xmin, j + ymin)) < safeRange) { // &&
                        //angleDiff(Math.atan2((j+ymin)-y, (i+xmin)-x), heading) < 80)
                        occGrid.setSafeSpaceAt(i + xmin, j + ymin);
                        dirtyCells.add(new Point(i + xmin, j + ymin));
                    } else {
                        occGrid.setFreeSpaceAt(i + xmin, j + ymin);
                        dirtyCells.add(new Point(i + xmin, j + ymin));

                    }
                }
            }
        }
    }

    protected int angleDiff(double theta1, double theta2) {
        //System.out.println(theta1 + " " + theta2);
        int angle1 = (int) (180 / Math.PI * theta1 + 360) % 360;
        int angle2 = (int) (180 / Math.PI * theta2 + 360) % 360;
        int diff = Math.abs(angle1 - angle2);
        return (Math.min(diff, 360 - diff));
    }

    protected void updateFreeAndSafeSpace(Polygon newFreeSpace, Polygon newSafeSpace, Environment env) {
        // May be possible to do some optimization here, I think a lot of cells are checked unnecessarily
        for (int i = newFreeSpace.getBounds().x; i <= newFreeSpace.getBounds().x + newFreeSpace.getBounds().width; i++) {
            innerloop:
            for (int j = newFreeSpace.getBounds().y; j <= newFreeSpace.getBounds().y + newFreeSpace.getBounds().height; j++) {
                if (occGrid.locationExists(i, j)) {
                    if (newFreeSpace.contains(i, j) && !occGrid.freeSpaceAt(i, j)) {
                        if (!env.obstacleAt(i, j)) {
                            occGrid.setFreeSpaceAt(i, j);
                            dirtyCells.add(new Point(i, j));
                        }
                    }
                    if (newSafeSpace.contains(i, j)) {
                        // double for loop to prevent empty-safe boundary (which
                        // would not qualify as a frontier)
                        for (int m = i - 1; m <= i + 1; m++) {
                            for (int n = j - 1; n <= j + 1; n++) {
                                if (occGrid.locationExists(m, n) && occGrid.emptyAt(m, n)) {
                                    continue innerloop;
                                }
                            }
                        }
                        if (!occGrid.safeSpaceAt(i, j) && !env.obstacleAt(i, j)) {
                            occGrid.setSafeSpaceAt(i, j);
                            dirtyCells.add(new Point(i, j));
                        }
                    }
                }
            }
        }
    }

    protected void updateObstacles(Polygon newFreeSpace, Environment env) {
        // Update obstacles -- all those bits for which radial polygon < sensRange
        // Ignore first point in radial polygon as this is the robot itself.
        Point first, second;
        double angle;
        int currX, currY;

        for (int i = 1; i < newFreeSpace.npoints - 1; i++) {
            first = new Point(newFreeSpace.xpoints[i], newFreeSpace.ypoints[i]);
            second = new Point(newFreeSpace.xpoints[i + 1], newFreeSpace.ypoints[i + 1]);

            // if two subsequent points are close and both hit an obstacle, assume there is a line between them.
            if (first.distance(x, y) < (sensRange - 2) && second.distance(x, y) < (sensRange - 2)
                    && first.distance(second) < 5) { // used to be 10
                angle = Math.atan2(second.y - first.y, second.x - first.x);
                for (int j = 0; j < first.distance(second); j++) {
                    currX = first.x + (int) (j * (Math.cos(angle)));
                    currY = first.y + (int) (j * (Math.sin(angle)));
                    double angleDepth = Math.atan2(currY - y, currX - x);
                    for (int k = 0; k < Constants.WALL_THICKNESS; k++) {
                        //occGrid.setObstacleAt(currX + (int) (k * Math.cos(angleDepth)), currY + (int) (k * Math.sin(angleDepth)));
                        int newX = currX + (int) (k * Math.cos(angleDepth));
                        int newY = currY + (int) (k * Math.sin(angleDepth));
                        //free space is final,
                        //otherwise, we can get inaccessible pockets of free space in our map that are actually
                        //accessible. This can lead to frontiers or RV points we cannot plan a path to, which can lead
                        //to all sorts of tricky problems.
                        if (!occGrid.freeSpaceAt(newX, newY) && env.obstacleAt(newX, newY)) {
                            occGrid.setObstacleAt(newX, newY);
                            //if (k == 0) occGrid.setSafeSpaceAt(newX, newY); //mark obstacles that we know for sure are there
                            dirtyCells.add(new Point(currX, currY));
                        }
                    }
                    dirtyCells.add(new Point(currX, currY));
                }
            }
        }
    }

    protected Polygon findRadialPolygon(double sensorData[], int maxRange) {
        double currRayAngle;
        int currRayX, currRayY;

        Polygon radialPolygon = new Polygon();
        radialPolygon.addPoint(x, y);

        //If there's no sensor data, done
        if (sensorData == null) {
            return radialPolygon;
        }

        //For every degree
        for (int i = 0; i <= SimulatorConfig.fov; i += 1) {
            currRayAngle = heading - Math.toRadians(((float)SimulatorConfig.fov)/2.0) + Math.PI / 180 * i;

            if (sensorData[i] >= maxRange) {
                currRayX = x + (int) Math.floor(maxRange * (Math.cos(currRayAngle)));
                currRayY = y + (int) Math.floor(maxRange * (Math.sin(currRayAngle)));
            } else {
                currRayX = x + (int) Math.floor(sensorData[i] * (Math.cos(currRayAngle)));
                currRayY = y + (int) Math.floor(sensorData[i] * (Math.sin(currRayAngle)));
            }

            radialPolygon.addPoint(currRayX, currRayY);
        }

        return radialPolygon;
    }

    // </editor-fold>     
// <editor-fold defaultstate="collapsed" desc="Communicate">
    public void receiveMessage(DataMessage msg, int timeStep) {
        TeammateAgent teammate = teammates.get(msg.ID);

        teammate.setInRange(true);
        teammate.setInDirectRange(msg.directComm);
        teammate.setX(msg.x);
        teammate.setY(msg.y);
        teammate.setOccupancyGrid(msg.occGrid);
        teammate.setTimeLastCentralCommand(msg.timeLastCentralCommand);
        teammate.setPathLength(msg.pathLength);
        teammate.setState(msg.state);
        teammate.setDistanceToBase(msg.distToBase);
        teammate.setSpeed(msg.speed);
        teammate.lastContactAreaKnown = msg.timeLastCentralCommand;
        teammate.relayID = msg.relayID;
        teammate.setChildRendezvous(msg.childRendezvous);
        teammate.setParentRendezvous(msg.parentRendezvous);
        teammate.setFrontierCentre(msg.frontierCentre);
        teammate.setIntendedPath(msg.intendedPath);

        //System.out.println(toString() + "Communicating...");
        if (teammate.ID == child && teammate.ID != Constants.BASE_STATION_ID) {
            this.childRendezvous = msg.parentRendezvous;
            this.childBackupRendezvous = msg.parentBackupRendezvous;
            this.missionComplete = msg.missionComplete;
        }

        teammate.setTimeSinceLastComm(0);
        boolean isBaseStation = false;
        if ((teammate.getRobotNumber() == Constants.BASE_STATION_ID)
                || (this.getRobotNumber() == Constants.BASE_STATION_ID)) {
            isBaseStation = true;
        }

        /*if (teammate.ID == Constants.BASE_STATION_ID && msg.m_opt.size() > 0) {
         Point m_opt = msg.m_opt.get(this.ID - 1);
         this.setM_opt(m_opt);
         teammate.setM_opt(m_opt);
         }*/
        //if (ID == teammate.relayID) amIRelay = true;
        mergeGrid(teammate.getOccupancyGrid(), isBaseStation);
        //if (sim.getTimeElapsed() > 47) sim.verifyNoInfoGotLost2();

        if (msg.ID == Constants.BASE_STATION_ID) {
            this.timeLastContactBS = 1;
        }
        
        //When the simulation is running and I'm using the UtilityExploration (Spirin) or Using the SwitchExploration (in Relaying mode) 
        //I need to communicate the area relayed (otherwise all agents in ReturnToParent state)
        if ((simConfig != null) && (((simConfig.getExpAlgorithm() == SimulatorConfig.exptype.FrontierExploration)
                && (simConfig.getFrontierAlgorithm() == SimulatorConfig.frontiertype.UtilReturn))
                || (simConfig.getExpAlgorithm() == SimulatorConfig.exptype.SwitchExploration && !SwitchExploration.isMultiHopping()))) {
            updateAreaRelayed(teammate);
        }
        if (teammate.getTimeLastCentralCommand() < timeLastCentralCommand) {
            timeLastCentralCommand = teammate.getTimeLastCentralCommand();
        }
        if (teammate.lastContactAreaKnown > lastContactAreaKnown) {
            lastContactAreaKnown = teammate.lastContactAreaKnown;
        }
        needUpdatingAreaKnown = true;

        double rateOfInfoGatheringBelief = msg.maxRateOfInfoGatheringBelief;
        if (rateOfInfoGatheringBelief > maxRateOfInfoGatheringBelief) {
            maxRateOfInfoGatheringBelief = rateOfInfoGatheringBelief;
        }
        
        teammate.setCompletedRotation(msg.completedRotation);
    }

    // Unisce le frontiere di due robot
    public void mergeFrontiers(LinkedList<Frontier> list) {
        boolean isIn = false;
        for (Frontier f1 : list) {
            for (Frontier f2 : this.getFrontiers()) {
                if (f1.getCentre().equals(f2.getCentre())) {
                    isIn = true;
                    break;
                }
            }
            if (!isIn) {
                this.getFrontiers().add(f1);
            }
            isIn = false;
        }
    }

    // Aggiorna alcuni parametri da passare a Stump (C, w_p e G)
    public void updateInfo(LinkedList<Frontier> frontiers) { //AGGIUNTO

        this.getFrontierIds().clear();
        /*if(this.getRobotNumber() != Constants.BASE_STATION_ID) {
         this.setFrontiers(new LinkedList<Frontier>());
         }*/
        //this.setList(new ArrayList<Location>());
        Map<Integer, Location> newLocationIDs = new HashMap(this.locationIDs);
        for (Frontier f : frontiers) {
            Location locF = new Location(f.getCentre());

            if (this.locationIDs.isEmpty()) {

                this.getList().add(locF);
                newLocationIDs.put(this.locNumber, locF);
                this.getFrontierIds().add(this.locNumber);
                this.getTrueFrontiers().add(this.locNumber);
                this.locNumber++;
                System.out.println("Adding new frontier with ID " + Integer.toString(this.locNumber) + " at pos. " + locF.getPosition().toString() );
                continue;
            }

            //If the frontier was not already present
            boolean present = false;
            for (Map.Entry<Integer, Location> entry : this.locationIDs.entrySet()) {
                Integer id = entry.getKey();
                Location loc = entry.getValue();
                if (loc.getPosition().equals(locF.getPosition())) {
                    present = true;
                    System.out.println("The frontier at (" + locF.getPosition().x + "," + locF.getPosition().y + ") was already present with ID " + id);
                    this.getFrontierIds().add(id);
                    break;
                }

            }

            if (!present) {
                this.getList().add(locF);

                System.out.println("Adding new frontier with ID " + Integer.toString(this.locNumber) + " at pos. " + locF.getPosition().toString() );
                newLocationIDs.put(this.locNumber, locF);
                this.getFrontierIds().add(this.locNumber);
                this.getTrueFrontiers().add(this.locNumber);
                this.locNumber++;

            }

        }

        this.locationIDs = newLocationIDs;

    }

    // Aggiorna alcuni parametri da passare a Stump (C_beta e V_beta)
    public void updateC_V_beta(PriorityQueue<Utility> utilities) { //AGGIUNTO

        ArrayList<Utility> ut = new ArrayList<Utility>();

        ArrayList<TeammateAgent> ta = new ArrayList<TeammateAgent>(this.getAllTeammates().values());

        while (utilities.peek() != null && ut.size() < ta.size()) {

            Utility max = utilities.peek();

            boolean contains = false;

            for (Utility u1 : utilities) {
                if (u1.compareTo(max) == -1) {
                    max = u1;
                }
            }

            utilities.remove(max);

            for (Utility u2 : ut) {
                if (u2.ID == max.ID
                        || max.ID == Constants.BASE_STATION_ID
                        || max.frontier.getCentre().equals(u2.frontier.getCentre())) {
                    contains = true;
                    break;
                }
            }

            if (!contains) {
                ut.add(max);
            }
        }

        //Rimuove una frontiera che non ? utilizzata
        /*int k = this.getList().size();
    	
         for(int i=0; i<k; i++) {
         boolean in = false;
         for(Utility u : ut) {
         if(u.frontier.getCentre().equals(this.getList().get(i).getPosition())) {
         in = true;
         break;
         }
         }
         if(!in) {
         this.getList().remove(i);
         k--;
         i--;
         }
         }*/
        ArrayList<Integer> C_beta = new ArrayList<Integer>();
        ArrayList<Integer> V_beta = new ArrayList<Integer>();

        boolean isIn = false;

        for (Map.Entry<Integer, Location> entry : this.locationIDs.entrySet()) {
            Integer id = entry.getKey();
            Location loc = entry.getValue();
            if (loc.getPosition().equals(this.getLocation())) {
                C_beta.add(id + 1);
                isIn = true;
                break;
            }
        }

        if (!isIn) {
            Location l = new Location(this.getLocation());
            this.getList().add(l);

            this.locationIDs.put(this.locNumber, l);
            this.locNumber++;

            //TODO check for stump
            C_beta.add(this.locNumber);

        }

        V_beta.add(this.getRobotNumber()); //VALE SE LA BS HA NUMERO 1

        for (Utility utility : ut) {
            for (Map.Entry<Integer, Location> entry : this.locationIDs.entrySet()) {
                Integer id = entry.getKey();
                Location loc = entry.getValue();
                if (loc.getPosition().equals(utility.frontier.getCentre())) {

                    C_beta.add(id + 1);

                    V_beta.add(utility.ID);
                }
            }
        }

        while (C_beta.size() < V_beta.size()) { //NON DOVREBBE SUCCEDERE
            V_beta.remove(V_beta.size() - 1);
        }
        /*while(V_beta.size() < C_beta.size()) { // NON DOVREBBE SUCCEDERE
         C_beta.remove(C_beta.size()-1);
         }*/

        this.setC_beta(C_beta);
        this.setV_beta(V_beta);

        BufferedWriter writer = null;
        try {
            writer = new BufferedWriter(new FileWriter("C_beta.txt"));
            for (int i = 0; i < C_beta.size(); i++) {
                //System.out.println(C_beta.get(i));
                writer.write(C_beta.get(i) + "\t");
                writer.flush();
            }
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

        BufferedWriter writer2 = null;
        try {
            writer2 = new BufferedWriter(new FileWriter("V_beta.txt"));
            for (int i = 0; i < V_beta.size(); i++) {
                //System.out.println(V_beta.get(i));
                writer2.write(V_beta.get(i) + "\t");
                writer2.flush();
            }
            writer2.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void updateCurrWaitingList() {

        /*HashMap<Integer, ArrayList<Integer>> waitingListsInt = new HashMap<Integer, ArrayList<Integer>>();
        for (Map.Entry<Integer, ArrayList<Integer>> entry : this.getWaitingLists().entrySet()) {
            ArrayList<Integer> relayIds = new ArrayList<Integer>();
            int frontier = entry.getKey() - 1;

            ArrayList<Integer> robots = entry.getValue();
            //for each robot, get its destination in terms of ID
            for (int robot : robots) {
                Point dest = this.getM_opt_p().get(robot);
                for (Map.Entry<Integer, Location> entry2 : this.getLocationIDs().entrySet()) {
                    if (entry2.getValue().getPosition().equals(dest)) {
                        //and there will be for sure

                        relayIds.add(entry2.getKey());
                        break;
                    }
                }
            }

            waitingListsInt.put(frontier, relayIds);

        }*/

        //write the file
        BufferedWriter writer = null;
        try {
            writer = new BufferedWriter(new FileWriter("currWaitingList.txt"));

            //write the paths to allocate
            for (Map.Entry<Integer, ArrayList<Integer>> entry : this.waitingLists.entrySet()) {
                writer.write(Integer.toString(entry.getKey()));
                /*for (int relay : entry.getValue()) {
                    writer.write(" " + Integer.toString(relay));
                }*/
                writer.write("\n");
            }

            writer.close();

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void updateApprox(SimulatorConfig sim){
        BufferedWriter writer = null;
        try {
            writer = new BufferedWriter(new FileWriter("approx.txt"));
            //System.out.print(this.commRange);
            String A = String.valueOf(sim.approximation);
            writer.write(A);
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    
    public void updateW_p(int[][] limitedVisMatrix) { //AGGIUNTO

        //this.setWeightGrid();
        this.updateWeightGrid(limitedVisMatrix);

        int[][] w_p = this.getWeightGrid().getWeightMatrix();

        this.setW_p(w_p);

        BufferedWriter writer = null;
        try {
            writer = new BufferedWriter(new FileWriter("w_p.txt"));
            for (int i = 0; i < w_p.length; i++) {
                for (int j = 0; j < w_p.length; j++) {
                    //System.out.println(w_p[i][j]);
                    writer.write(w_p[i][j] + "\t");
                    writer.flush();
                }
                //System.out.println();
                writer.newLine();
            }
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

        //write also not ready but comm
        HashMap<Integer, ArrayList<Integer>> notReady = this.weightGrid.getOtherDistances(this, limitedVisMatrix);

        try {
            writer = new BufferedWriter(new FileWriter("notReadyDist.txt"));
            for (Map.Entry<Integer, ArrayList<Integer>> entry : notReady.entrySet()) {
                int id = entry.getKey();
                ArrayList<Integer> distances = entry.getValue();
                writer.write(Integer.toString(id));
                for (int dist : distances) {
                    writer.write(" " + Integer.toString(dist));
                    writer.flush();
                }
                writer.newLine();
            }

            writer.write("COMM");
            //write also the IDS of the communicating robots (will be allowed to change destination)
            for (TeammateAgent t : this.getAllTeammates().values()) {
                if (!t.isReady() && t.isInRange()) {
                    writer.write(" " + Integer.toString(t.getID() - 1));
                }
            }
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    public void updateD() { //AGGIUNTO

        if (this.D_opt != null && this.D_opt.length != 0) {
            D = D_opt;
        }

        BufferedWriter writer = null;
        try {
            writer = new BufferedWriter(new FileWriter("D.txt"));
            for (int i = 0; i < D.length; i++) {
                for (int j = 0; j < D.length; j++) {
                    //System.out.println(D[i][j]);
                    writer.write(D[i][j] + "\t");
                    writer.flush();
                }
                //System.out.println();
                writer.newLine();
            }
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void updateG(SimulatorConfig simConfig) { //AGGIUNTO

        //Here a new CommGrid is built from scratch
        //this.setCommGrid();
        this.updateCommGrid(simConfig);

        int[][] G = this.getCommGrid().getCommMatrix();

        this.setG(G);

        BufferedWriter writer = null;
        try {
            writer = new BufferedWriter(new FileWriter("G.txt"));
            for (int i = 0; i < G.length; i++) {
                for (int j = 0; j < G.length; j++) {
                    //System.out.println(G[i][j]);
                    writer.write(G[i][j] + "\t");
                    writer.flush();
                }
                //System.out.println();
                writer.newLine();
            }
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void updateM_1() { //AGGIUNTO; 

        //this.setM_1(new ArrayList<Integer>());
        //get the BS key in the hashmap
        int idBs = -1;
        for (Map.Entry<Integer, Location> entry : this.locationIDs.entrySet()) {
            Integer id = entry.getKey();
            Location loc = entry.getValue();
            if (loc.getPosition().equals(this.getLocation())) {
                idBs = id + 1;
                break;
            }
        }

        ArrayList<TeammateAgent> readyList = new ArrayList<TeammateAgent>();

        for (TeammateAgent t : this.getAllTeammates().values()) {
            if (t.isReady()) {
                readyList.add(t);
            }
        }

        //final list where teammates are ordered
        ArrayList<TeammateAgent> list = new ArrayList<TeammateAgent>();
        for (int i = 0; i < readyList.size(); i++) {
            TeammateAgent min = null;
            for (TeammateAgent t : readyList) {
                if (min == null) {
                    min = t;
                } else if (t.ID < min.ID && !list.contains(t)) {
                    min = t;
                }
            }
            list.add(min);
        }

        //In this list, Robots are ordered by ID
        /*for (TeammateAgent t : list) {

         for (Map.Entry<Integer, Location> entry : this.locationIDs.entrySet()) {
         Integer id = entry.getKey();
         Location loc = entry.getValue();
         if (loc.getPosition().equals(t.getLocation())) {
         this.getM_1().add(id + 1);
         break;
         }
         }

         }

         ArrayList<Integer> m_1 = this.getM_1();*/
        BufferedWriter writer = null;
        try {
            writer = new BufferedWriter(new FileWriter("m_1.txt"));

            //write the bs
            writer.write(Integer.toString(0) + "\t" + Integer.toString(idBs) + "\n");
            writer.flush();

            for (TeammateAgent t : list) {
                Integer id = -1;
                for (Map.Entry<Integer, Location> entry : this.locationIDs.entrySet()) {
                    id = entry.getKey();
                    Location loc = entry.getValue();
                    if (loc.getPosition().equals(t.getLocation())) {
                        break;
                    }
                }
                writer.write(Integer.toString(t.ID - 1) + "\t" + Integer.toString(id + 1) + "\n");
                writer.flush();
            }
            /*for (int i = 0; i < m_1.size(); i++) {
             //System.out.print(m_1.get(i));
             if(this.getAllTeammates().get(i + 1).isReady()){
             writer.write(Integer.toString(i) + "\t" + m_1.get(i) + "\n");
             writer.flush();
             }
             }*/
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void updateM_1_stump() { //AGGIUNTO; 

        //this.setM_1(new ArrayList<Integer>());
        //get the BS key in the hashmap
        int idBs = -1;
        for (Map.Entry<Integer, Location> entry : this.locationIDs.entrySet()) {
            Integer id = entry.getKey();
            Location loc = entry.getValue();
            if (loc.getPosition().equals(this.getLocation())) {
                idBs = id + 1;
                break;
            }
        }

        ArrayList<TeammateAgent> readyList = new ArrayList<TeammateAgent>();

        for (TeammateAgent t : this.getAllTeammates().values()) {
            if (t.isReady()) {
                readyList.add(t);
            }
        }

        //final list where teammates are ordered
        ArrayList<TeammateAgent> list = new ArrayList<TeammateAgent>();
        for (int i = 0; i < readyList.size(); i++) {
            TeammateAgent min = null;
            for (TeammateAgent t : readyList) {
                if (min == null) {
                    min = t;
                } else if (t.ID < min.ID && !list.contains(t)) {
                    min = t;
                }
            }
            list.add(min);
        }

        //In this list, Robots are ordered by ID
        /*for (TeammateAgent t : list) {

         for (Map.Entry<Integer, Location> entry : this.locationIDs.entrySet()) {
         Integer id = entry.getKey();
         Location loc = entry.getValue();
         if (loc.getPosition().equals(t.getLocation())) {
         this.getM_1().add(id + 1);
         break;
         }
         }

         }

         ArrayList<Integer> m_1 = this.getM_1();*/
        BufferedWriter writer = null;
        try {
            writer = new BufferedWriter(new FileWriter("m_1.txt"));

            //write the bs
            writer.write(Integer.toString(idBs) + "\t");
            writer.flush();

            for (TeammateAgent t : list) {
                Integer id = -1;
                for (Map.Entry<Integer, Location> entry : this.locationIDs.entrySet()) {
                    id = entry.getKey();
                    Location loc = entry.getValue();
                    if (loc.getPosition().equals(t.getLocation())) {
                        break;
                    }
                }
                writer.write(Integer.toString(id + 1) + "\t");
                writer.flush();
            }
            /*for (int i = 0; i < m_1.size(); i++) {
             //System.out.print(m_1.get(i));
             if(this.getAllTeammates().get(i + 1).isReady()){
             writer.write(Integer.toString(i) + "\t" + m_1.get(i) + "\n");
             writer.flush();
             }
             }*/
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void updateC_V() { //AGGIUTO

        this.C = new ArrayList<Integer>();

        //this.getOldLocations().add(this.getLocation()); 
        ArrayList<TeammateAgent> list = new ArrayList<TeammateAgent>();

        for (int i = 0; i < this.getAllTeammates().values().size(); i++) {
            TeammateAgent min = null;
            for (TeammateAgent t : this.getAllTeammates().values()) {
                if (min == null) {
                    min = t;
                } else if (t.ID < min.ID && !list.contains(t)) {
                    min = t;
                }
            }
            list.add(min);
        }

        for (TeammateAgent t : list) {
            if (!t.isReady()) {
                continue;
            }
            //this.getOldLocations().add(t.getLocation());
            Location newLoc = new Location(t.getLocation());
            if (!newLoc.isIn(this.getList())) {
                this.getList().add(newLoc);
                this.locationIDs.put(this.locNumber, newLoc);
                this.locNumber++;
            }
        }

        //Aggiunge la posizione della BS
        /*if (!this.getOldLocations().contains(this.getLocation())) {
         this.getOldLocations().add(this.getLocation());
         }*/
        Location bs = new Location(this.getLocation());
        if (!bs.isIn(this.getList())) {
            this.getList().add(bs);
            this.locationIDs.put(this.locNumber, bs);
            this.locNumber++;
        }

        //Aggiunge tutte le posizioni occupate dai robot "esploratori"
        /*for (TeammateAgent t : list) {
         if (!this.getOldLocations().contains(t.getLocation())) {
         this.getOldLocations().add(t.getLocation());
         }
         }*/
        //Aggiunge tutte le nuove frontiere
        /*for (Location l : this.getList()) {
         Point p = new Point(l.getPosition());
         if (!this.getOldLocations().contains(p)) {
         this.getOldLocations().add(p);
         }
         }*/

        /*for (Point p : this.getOldLocations()) {
         Location newLoc = new Location(p);
         if (!newLoc.isIn(this.getList())) {
         this.getList().add(newLoc);
         this.locationIDs.put(this.locNumber, newLoc);
         this.locNumber++;
         }
         }*/
        
        //These files are actually used only by stump
        
        for (int i = 0; i < this.getList().size(); i++) {
            //System.out.println("LOCATION " + this.getList().get(i).getPosition());
            this.C.add(i + 1);
        }

        BufferedWriter writer = null;
        try {
            writer = new BufferedWriter(new FileWriter("C.txt"));
            for (int i = 0; i < C.size(); i++) {
                //System.out.print(C.get(i));
                writer.write(C.get(i) + "\t");
                writer.flush();
            }
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

        BufferedWriter writer2 = null;
        try {
            writer2 = new BufferedWriter(new FileWriter("V.txt"));
            for (int i = 0; i < V.size(); i++) {
                //System.out.print(V.get(i));
                writer2.write(V.get(i) + "\t");
                writer2.flush();
            }
            writer2.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    public void updateF_FA() { //AGGIUNTO
        this.F = new ArrayList<Integer>();
        this.FA = new ArrayList<Double>();

        for (Frontier f : this.getFrontiers()) {
            Location locF = new Location(f.getCentre());
            for (Map.Entry<Integer, Location> entry : this.locationIDs.entrySet()) {
                Integer id = entry.getKey();
                Location loc = entry.getValue();
                if (loc.getPosition().equals(locF.getPosition())) {
                    F.add(id + 1);
                    FA.add(f.getArea());
                    break;
                }
            }

        }

        BufferedWriter writer = null;
        BufferedWriter writer2 = null;
        try {
            writer = new BufferedWriter(new FileWriter("F.txt"));
            writer2 = new BufferedWriter(new FileWriter("FA.txt"));
            for (int i = 0; i < F.size(); i++) {
                //System.out.print(C.get(i));
                writer.write(F.get(i) + "\t");
                writer.flush();
                writer2.write(FA.get(i) + "\t");
                writer2.flush();
            }
            writer.close();
            writer2.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    public void updateR_mu() {
        BufferedWriter writer = null;
        try {
            writer = new BufferedWriter(new FileWriter("R.txt"));
            //System.out.print(this.commRange);
            String R = String.valueOf(this.R);
            writer.write(R);
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

        BufferedWriter writer2 = null;
        try {
            writer2 = new BufferedWriter(new FileWriter("mu.txt"));
            //System.out.print(Constants.MU);
            String mu = String.valueOf(Constants.MU);
            writer2.write(mu);
            writer2.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void writeNotReadyButComm() {
        BufferedWriter writer = null;
        int newLocId = this.locNumber;
        try {
            writer = new BufferedWriter(new FileWriter("notReadyComm.txt"));
            //It's ok if they are not ordered.
            for (TeammateAgent teammate : teammates.values()) {
                if (teammate.isInRange() && !(teammate.isReady())) {
                    writer.write(Integer.toString(teammate.ID - 1) + "\t" + newLocId + "\n");
                    newLocId++;
                    writer.flush();
                }
            }

            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public boolean isCommunicating() {
        for (TeammateAgent teammate : teammates.values()) {
            if (teammate.isInRange()) {
                return true;
            }
        }
        return false;
    }

    public void updateAfterCommunication() {
        //long realtimeStart = System.currentTimeMillis();
        //System.out.print(this.toString() + "Updating post-communication ... ");

        for (TeammateAgent teammate : teammates.values()) {
            if (!teammate.isInRange()) {
                teammate.setTimeSinceLastComm(teammate.getTimeSinceLastComm() + 1);
            }
        }

        //processRelayMarks();
        //System.out.println("Complete, took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
    }

    private void mergeGrid(OccupancyGrid partnerOccGrid, boolean withBaseStation) {
        for (int i = 0; i < occGrid.width; i++) {
            for (int j = 0; j < occGrid.height; j++) {
                if (occGrid.getByteNoRelay(i, j) != partnerOccGrid.getByteNoRelay(i, j)) {
                    // if the information is completely new, get all of it, including relay status
                    // otherwise, we may be the relay! So get all new info, apart from relay status
                    if (occGrid.getByte(i, j) == 0) {
                        occGrid.setByte(i, j, (byte) (occGrid.getByte(i, j) | partnerOccGrid.getByte(i, j)));
                    } else {
                        occGrid.setByte(i, j, (byte) (occGrid.getByte(i, j) | partnerOccGrid.getByteNoRelay(i, j)));
                    }
                    if (withBaseStation) {
                        occGrid.setKnownAtBase(i, j);
                    }
                    dirtyCells.add(new Point(i, j));
                }
            }
        }
    }
// </editor-fold>     

}
