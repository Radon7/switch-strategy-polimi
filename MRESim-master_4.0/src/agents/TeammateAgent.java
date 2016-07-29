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

import config.Constants;
import config.RobotConfig;
import config.RobotConfig.roletype;
import environment.CommunicationGrid;
import environment.Location;
import environment.OccupancyGrid;
import environment.WeightGrid;
import exploration.RVLocation;
import path.Path;

import java.awt.Point;
import java.util.ArrayList;
import java.util.LinkedList;

/**
 * The TeammateAgent class is used to store the knowledge of an agent about their teammates.
 * Fields in this class represent what the agent can know about their teammates.
 * @author julh
 */
public class TeammateAgent extends BasicAgent implements Agent {
    
    int timeLastCentralCommand;   /* units of time elapsed since command 
                                     received from ComStation */
    int lastContactAreaKnown;
    private boolean inRange;
    private boolean inDirectRange;

    private int timeSinceLastComm;
    OccupancyGrid occGrid;
    private RVLocation childRendezvous;
    private RVLocation parentRendezvous;
    private Point frontierCentre;
    private double pathLength;
    int relayID;
    public int newInfo;
    
    public WeightGrid weightGrid; //AGGIUNTO
    public ArrayList<Location> list; //AGGIUNTO
    public CommunicationGrid commGrid; //AGGIUNTO
    
    public Point m_opt;
    
    private boolean ready;
    
    private boolean wasReady;
    
    private int stepLastComm;
    
    LinkedList<Point> intendedPath;
    
    Point positionEstimate;
    
    private boolean completedRotation;

    public TeammateAgent(RobotConfig robot) {
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
        
        inRange = false;
        timeSinceLastComm = 0;
        pathLength = 0;
        
        //weightGrid = new WeightGrid(); //AGGIUNTO , si possono togliere?
        list = new ArrayList<Location>(); //AGGIUNTO
        //commGrid = new CommunicationGrid(); //AGGIUNTO
        
        m_opt = new Point(0,0);
        ready = true;
        stepLastComm = 0;
        intendedPath = null;
        positionEstimate = null;
        wasReady = true;
        completedRotation = false;
    }
    
    public TeammateAgent(TeammateAgent toCopy) {
        super(toCopy.getRobotNumber(), 
              toCopy.getName(), 
              toCopy.getID(),
              toCopy.getX(), 
              toCopy.getY(), 
              toCopy.getHeading(),
              toCopy.getSenseRange(),
              toCopy.getCommRange(), 
              toCopy.getBatteryPower(), 
              toCopy.getRole(),
              toCopy.getParent(),
              toCopy.getChild(),
              toCopy.getSpeed());
        this.childRendezvous = toCopy.childRendezvous;
        this.parentRendezvous = toCopy.parentRendezvous;
        this.timeLastCentralCommand = toCopy.timeLastCentralCommand;
        this.lastContactAreaKnown = toCopy.lastContactAreaKnown;
        
        this.weightGrid = toCopy.weightGrid; //AGGIUNTO
        this.list = toCopy.list; //AGGIUNTO
        this.commGrid = toCopy.commGrid; //AGGIUNTO
        
        m_opt = toCopy.m_opt;
        ready = toCopy.isReady();
        wasReady = toCopy.wasReady();
        stepLastComm = toCopy.getStepLastComm();
        if(toCopy.intendedPath == null){
            this.intendedPath = null;
        }
        else{
            this.intendedPath = new LinkedList<Point>(toCopy.intendedPath);
        }
        
        if(toCopy.positionEstimate == null){
            this.positionEstimate = null;
        }
        else{
            this.positionEstimate = new Point(toCopy.positionEstimate);
        }
        this.completedRotation = toCopy.completedRotation;
    }
    
    public void setCompletedRotation(boolean cr){
        this.completedRotation = cr;
    }
    
    public boolean getCompletedRotation(){
        return this.completedRotation;
    }
    
    public boolean wasReady(){
        return this.wasReady;
    }
    
    public void setWasReady(boolean wr){
        this.wasReady = wr;
    }
    
    public void setPositionEstimate(Point p){
        this.positionEstimate = p;
    }
    
    public Point getPositionEstimate(){
        return this.positionEstimate;
    }

    public void setIntendedPath(LinkedList<Point> p){
        this.intendedPath = p;
    }
    
    public LinkedList<Point> getIntendetPath(){
        return this.intendedPath;
    }
    
    public int getStepLastComm(){
        return this.stepLastComm;
    }
    
    public void setStepLastComm(int step){
        this.stepLastComm = step;
    }
       
    public int getTimeLastCentralCommand() {
        return this.timeLastCentralCommand;
    }
    
    public void setTimeLastCentralCommand(int t) {
        this.timeLastCentralCommand = t;
    }
    
    public int getTimeSinceLastComm() {
        return this.timeSinceLastComm;
    }
    
    public void setTimeSinceLastComm(int t) {
        this.timeSinceLastComm = t;
    }
    
    public boolean isInDirectRange() {
        return inDirectRange;
    }

    public void setInDirectRange(boolean r) {
        inDirectRange = r;
    }

    public boolean isInRange() {
        return inRange;
    }

    public void setInRange(boolean r) {
        inRange = r;
    }

    public Point getM_opt() { //AGGIUNTO
    	return this.m_opt;
    }
    
    public void setM_opt(Point m_opt) { //AGGIUNTO
    	this.m_opt = m_opt;
    }
    
    public WeightGrid getWeightGrid() { //AGGIUNTO
    	return weightGrid;
    }
    
    public ArrayList<Location> getListOfLocations() { //AGGIUNTO
    	return list;
    }
    
    public CommunicationGrid getCommGrid() { //AGGIUNTO
    	return commGrid;
    }
    
    public void setWeightGrid(WeightGrid w) { //AGGIUNTO
    	weightGrid = w;
    }
    
    public void setListOfLocations(ArrayList<Location> l) { //AGGIUNTO
    	list = l;
    }
    
    public void setCommGrid(CommunicationGrid c) { //AGGIUNTO
    	commGrid = c;
    }
    
    public OccupancyGrid getOccupancyGrid() {
        return occGrid;
    }
    
    public void setOccupancyGrid(OccupancyGrid og) {
        this.occGrid = og;
    }
    
    public TeammateAgent copy() {
        return new TeammateAgent(this);
    }
    
    public boolean isExplorer() {
        return role.equals(roletype.Explorer);
    }
    
    public void setExplorer() {
        role = roletype.Explorer;
    }
    
    public void setRelay() {
        role = roletype.Relay;
    }
    
    public void setParentRendezvous(RVLocation parentRendezvous) {
        this.parentRendezvous = parentRendezvous;
    }
    
    public void setChildRendezvous(RVLocation childRendezvous) {
        this.childRendezvous = childRendezvous;
    }
    
    public RVLocation getParentRendezvous()
    {
        return parentRendezvous;
    }
    
    public RVLocation getChildRendezvous()
    {
        return childRendezvous;
    }
    
    public Point getFrontierCentre() {
        return frontierCentre;
    }
    
    public void setFrontierCentre(Point frontierCentre) {
        this.frontierCentre = frontierCentre;
    }
    
    public double getPathLength() {
        return pathLength;
    }
    
    public void setPathLength(double lgth) {
        pathLength = lgth;
    }
    
    public boolean isReady(){
        return this.ready;
    }
    
    public void setReady(boolean ready){
        this.ready = ready;
    }
}
