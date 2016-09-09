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

import agents.*;
import agents.BasicAgent.ExploreState;
import environment.*;
import config.*;
import gui.*;
import communication.*;
import config.RobotConfig.roletype;
import config.SimulatorConfig.exptype;
import config.SimulatorConfig.frontiertype;
import environment.Environment.Status;

import javax.swing.*;

import java.awt.MultipleGradientPaint.CycleMethod;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.Point;
import java.awt.Polygon;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Random;

import path.Path;

/**
 *
 * @author Julian de Hoog
 */
public class SimulationFramework implements ActionListener {

// <editor-fold defaultstate="collapsed" desc="Class variables and Constructors">
    boolean pauseSimulation;                    // For stepping through simulation one step at a time

    boolean useGui;
    MainGUI mainGUI;                            // Allows simulator to change image, data
    ExplorationImage image;                     // Image of environment
    Environment env;                            // The environment (walls, obstacles)
    RealAgent agent[];                           // The agents
    int numRobots;

    SimulatorConfig simConfig;

    Polygon agentRange[];                       // For visualization of agents' comm ranges

    Timer timer;                                // Drives simulation steps
    Random random;                              // For generating random debris

    int[] debrisTimer;                          // For aisleRoom random debris exercise (AAMAS2010)

    // Communication
    int[][] directCommTable;
    int[][] directCommTableCopy;
    int[][] multihopCommTable;

    // Interesting data
    int timeElapsed;
    int realTimeElapsed;

    int jointAreaKnown;
    double pctAreaKnown;
    int avgCycleTime;
    long simStartTime;
    int totalArea;
    double avgComStationKnowledge;
    double avgAgentKnowledge;
    double avgTimeLastCommand;
    double totalDistanceTraveled;
    int numSwaps;

    //long simulationTime1; //AGGIUNTO
    //long simTimeNoReplan1;
    long[] timeNotCommunicating1; //AGGIUNTO
    long timeStartCycle; //AGGIUNTO
    long stumpTime1; //AGGIUNTO
    long oldSimulationTime1; //AGGIUNTO
    long lastStumpTime1; //AGGIUNTO
    long lastSimulationTime1; //AGGIUNTO
    double[] distanceLastStump1; //AGGIUNTO
    double[] oldPctAreaKnown1; //AGGIUNTO
    double[] pctAreaKnown1;
    double[] distance1;

    //long simulationTime2; //AGGIUNTO
    //long simTimeNoReplan2;
    long[] timeNotCommunicating2; //AGGIUNTO   
    long stumpTime2; //AGGIUNTO
    long oldSimulationTime2; //AGGIUNTO
    long lastStumpTime2; //AGGIUNTO
    long lastSimulationTime2; //AGGIUNTO
    double[] distanceLastStump2; //AGGIUNTO
    double[] oldPctAreaKnown2; //AGGIUNTO
    double[] pctAreaKnown2;
    double[] distance2;

    RobotTeamConfig robotTeamConfig;

    boolean timeToRecalculate = false;

    long[] replanTime1;
    long[] replanTime2;

    public SimulationFramework(boolean useGui, MainGUI maingui, RobotTeamConfig newRobotTeamConfig, SimulatorConfig newSimConfig, ExplorationImage img) {
        random = new Random();
        mainGUI = maingui;
        image = img;
        simConfig = newSimConfig;
        env = simConfig.getEnv();
        robotTeamConfig = newRobotTeamConfig;
        this.useGui = useGui;

        reset();
    }

    private void reset() {
        pauseSimulation = false;
        env = simConfig.getEnv();

        timeElapsed = 0;
        realTimeElapsed = 0;
        jointAreaKnown = 1;             // to prevent divide by 0
        pctAreaKnown = 0;
        totalArea = simConfig.getEnv().getTotalFreeSpace();
        avgComStationKnowledge = 0;
        avgAgentKnowledge = 0;
        avgTimeLastCommand = 0;
        avgCycleTime = 0;
        totalDistanceTraveled = 0;
        numSwaps = 0;

        //simulationTime1 = 0; //AGGIUNTO
        //simTimeNoReplan1 = 0;
        timeStartCycle = 0; //AGGIUNTO
        stumpTime1 = 0; //AGGIUNTO
        oldSimulationTime1 = 0; //AGGIUNTO
        lastStumpTime1 = 0; //AGGIUNTO
        lastSimulationTime1 = -1; //AGGIUNTO

        //simulationTime2 = 0; //AGGIUNTO
        //simTimeNoReplan2 = 0;
        stumpTime2 = 0; //AGGIUNTO
        oldSimulationTime2 = 0; //AGGIUNTO
        lastStumpTime2 = 0; //AGGIUNTO
        lastSimulationTime2 = -1; //AGGIUNTO

        createAgents(robotTeamConfig);

        // Initialize Timer
        timer = new Timer((Constants.TIME_INCREMENT * 10 + 1) - simConfig.getSimRate() * Constants.TIME_INCREMENT, this);
        timer.setInitialDelay(Constants.INIT_DELAY);
        timer.setCoalesce(true);

        // Initialize Debris timing
        debrisTimer = new int[6];
        for (int i = 0; i < 6; i++) {
            debrisTimer[i] = 0;
        }

        // Initialize Image
        if (this.useGui) {
            updateImage(true);
        }
    }

    private void createAgents(RobotTeamConfig robotTeamConfig) {
        numRobots = robotTeamConfig.getNumRobots();
        agent = new RealAgent[numRobots];
        TeammateAgent teammate[] = new TeammateAgent[numRobots];
        agentRange = new Polygon[numRobots];

        // Create ComStation
        agent[0] = new ComStation(env.getColumns(), env.getRows(), robotTeamConfig.getRobotTeam().get(1));
        teammate[0] = new TeammateAgent(robotTeamConfig.getRobotTeam().get(1));
        agent[0].setChild(1);
        agent[0].setState(RealAgent.ExploreState.GetInfoFromChild);

        for (int i = 1; i < numRobots; i++) {
            agent[i] = new RealAgent(env.getColumns(), env.getRows(), robotTeamConfig.getRobotTeam().get(i + 1));
            agentRange[i] = null;
            agent[i].setChildRendezvous(new RVLocation(agent[0].getLocation()));
            agent[i].setParentRendezvous(new RVLocation(agent[0].getLocation()));
            agent[i].setState(RealAgent.ExploreState.Initial);

            teammate[i] = new TeammateAgent(robotTeamConfig.getRobotTeam().get(i + 1));

        }

        // Give each agent its teammates
        for (int i = 0; i < numRobots; i++) {
            for (int j = 0; j < numRobots; j++) {
                if (j != i) {
                    agent[i].addTeammate(teammate[j].copy());
                }
            }
        }

        // Initialize some variables of the BaseStation used in Stump Algorithm
        initR(); //AGGIUNTO

        initMu(); //AGGIUNTO

        initD(); //AGGIUNTO

        initV(); //AGGIUNTO

        replanTime1 = new long[agent.length];
        replanTime2 = new long[agent.length];
        timeNotCommunicating1 = new long[agent.length]; //AGGIUNTO
        timeNotCommunicating2 = new long[agent.length]; //AGGIUNTO
        pctAreaKnown1 = new double[agent.length];
        pctAreaKnown2 = new double[agent.length];
        oldPctAreaKnown1 = new double[agent.length]; //AGGIUNTO
        oldPctAreaKnown2 = new double[agent.length]; //AGGIUNTO
        distance1 = new double[agent.length];
        distance2 = new double[agent.length];
        distanceLastStump1 = new double[agent.length]; //AGGIUNTO
        distanceLastStump2 = new double[agent.length]; //AGGIUNTO
    }

    public void initMu() {

        agent[0].setMu(Constants.MU);

        /*String myString = String.valueOf(0.5); //mu);
         BufferedWriter outputWriter = null;
         try {
         outputWriter = new BufferedWriter(new FileWriter("C:\\Users\\Mattia\\workspace\\MRESim-master\\mu.txt")); //forse devo mettere il percorso!
         outputWriter.write(myString);
         outputWriter.close();
         } catch (IOException e) {
         e.printStackTrace();
         }*/
    }

    public void initV() {
        ArrayList<Integer> V = new ArrayList<Integer>();

        for (int i = 0; i < numRobots; i++) {
            V.add(i + 1);
        }

        agent[0].setV(V);

        /*
         StringBuilder builder = new StringBuilder();
         for(Integer i : V) {
         builder.append(i + "\t");
         }
         String myString = builder.toString().trim();

         BufferedWriter outputWriter = null;
         try {
         outputWriter = new BufferedWriter(new FileWriter("C:\\Users\\Mattia\\workspace\\MRESim-master\\V.txt"));
         outputWriter.write(myString);
         outputWriter.close();
         } catch (IOException e) {
         e.printStackTrace();
         }*/
    }

    public void initD() {
        int[][] D = new int[numRobots][numRobots];

        for (int i = 0; i < numRobots; i++) {
            for (int j = i; j < numRobots; j++) {
                if (j == i + 1) {
                    D[i][j] = 1;
                    D[j][i] = 1;
                } else {
                    D[i][j] = 0;
                    D[j][i] = 0;
                }
            }
        }

        agent[0].setD(D);
        /*
         try {
    		
         BufferedWriter outputWriter = null;
         outputWriter = new BufferedWriter(new FileWriter("C:\\Users\\Mattia\\workspace\\MRESim-master\\D.txt"));
         for(int j=0; j<numRobots; j++) {

         StringBuilder builder = new StringBuilder();
         for(Integer i : D[j]) {
         builder.append(i + "\t");
         }

         String myString = builder.toString().trim();

    			
         outputWriter.write(myString);
         outputWriter.newLine();
         outputWriter.flush();
         }
    		
    		
         outputWriter.close();
         } catch (IOException e) {
         e.printStackTrace();
         }*/
    }

    //Due agenti sono in comunicazione se i loro CommunicationRange si intersecano
    public void initR() {
        agent[0].setR(agent[0].getCommRange() * 2);

        /*String myString = String.valueOf(commRange);
         BufferedWriter outputWriter = null;
         try {
         outputWriter = new BufferedWriter(new FileWriter("C:\\Users\\Mattia\\workspace\\MRESim-master\\R.txt"));
         outputWriter.write(myString);
         outputWriter.close();
         } catch (IOException e) {
         e.printStackTrace();
         }*/
    }

// </editor-fold>     
    public int getTotalArea() {
        return totalArea;
    }

    public int getTimeElapsed() {
        return timeElapsed;
    }

// <editor-fold defaultstate="collapsed" desc="Simulation Cycle">
    private void simulationCycle() throws FileNotFoundException {

        long realtimeStartCycle;
        realtimeStartCycle = System.currentTimeMillis();

        timeStartCycle = System.currentTimeMillis();

        if (timeElapsed == 1) {
            simStartTime = System.currentTimeMillis();
        }
        System.out.println("\n" + this.toString() + "************** CYCLE " + timeElapsed + " ******************\n");

        //set exploration goals at the start of the mission
        if (timeElapsed == 0) {
            for (int i = 0; i < numRobots; i++) {
                agent[i].setGoalArea(env.getTotalFreeSpace());
            }
        }

        agentSteps();               // move agents, simulate sensor data

        System.out.println(this.toString() + "agentSteps took " + (System.currentTimeMillis() - realtimeStartCycle) + "ms.\n");
        //if(timeElapsed % 7 == 0 || timeElapsed % 7 == 1) {
        long timer = System.currentTimeMillis();
        //if(timeElapsed % Constants.COMM_RATE == 0){
        simulateCommunication(simConfig);    // simulate communication
        System.out.println(this.toString() + "simulateCommunication took " + (System.currentTimeMillis() - timer) + "ms.\n");
        //}
        //timer = System.currentTimeMillis();
        if ((simConfig != null) && (simConfig.getExpAlgorithm() == SimulatorConfig.exptype.RoleBasedExploration)
                && (simConfig.roleSwitchAllowed())) {
            //Role switch should ideally be done by individual agents as they communicate, rather than here.
            switchRoles();              // switch roles
            //System.out.println(this.toString() + "switchRoles took " + (System.currentTimeMillis()-timer) + "ms.\n");
            //timer = System.currentTimeMillis();
            // second role switch check (to avoid duplicate relays)
            for (int i = 1; i < numRobots; i++) {
                for (int j = 1; j < numRobots; j++) {
                    if (i != j) {
                        if (agent[i].getState() == ExploreState.ReturnToParent && !agent[i].isExplorer()
                                && agent[j].getState() == ExploreState.ReturnToParent && !agent[j].isExplorer()
                                && agent[i].getTeammate(agent[j].getID()).isInRange()
                                && agent[i].getPath().getLength() < agent[j].getPath().getLength()) {
                            agent[i].setState(ExploreState.GoToChild);
                            agent[i].setStateTimer(0);
                            agent[i].addDirtyCells(agent[i].getPath().getAllPathPixels());
                            //System.out.println("\nSecondary switch: " + agent[i].getName() + " and " + agent[j].getName() + "\n");
                            Path path = agent[i].calculatePath(agent[i].getLocation(), agent[i].getChildRendezvous().getParentLocation());
                            agent[i].setPath(path);
                            agent[i].setCurrentGoal(agent[i].getChildRendezvous().getParentLocation());
                        }
                    }
                }
            }
            //System.out.println(this.toString() + "Second switch roles check took " + (System.currentTimeMillis()-timer) + "ms.\n");

        }
        //timer = System.currentTimeMillis();   
        //simulateDebris();           // simulate dynamic environment
        //System.out.println(this.toString() + "simulateDebris took " + (System.currentTimeMillis()-timer) + "ms.\n");
        timer = System.currentTimeMillis();

        //if (timeElapsed % Constants.UPDATE_AGENT_KNOWLEDGE_INTERVAL == 0) {
        updateAgentKnowledgeData();
        //}

        updateGlobalData(agent[0].getReplanCycles());         // update data, increments timeElapsed.
        //System.out.println(this.toString() + "updateGlobalData took " + (System.currentTimeMillis()-realtimeStartCycle) + "ms.\n");

        System.out.println(this.toString() + "updateGlobalData took " + (System.currentTimeMillis() - timer) + "ms.\n");
        //timer = System.currentTimeMillis();

        //System.out.println(this.toString() + "updateAgentKnowledgeData took " + (System.currentTimeMillis()-timer) + "ms.\n");
        //timer = System.currentTimeMillis();
        //System.out.println(this.toString() + "updateGUI took " + (System.currentTimeMillis()-timer) + "ms.\n");
        //timer = System.currentTimeMillis();
        logging();                  // perform logging as required
        //if ((timeElapsed % 10) == 0) verifyNoInfoGotLost(); //verify relaying works fine
        //System.out.println(this.toString() + "logging took " + (System.currentTimeMillis()-timer) + "ms.\n");
        int currentCycleTime = (int) (System.currentTimeMillis() - realtimeStartCycle);

        updateRelevantVariables(currentCycleTime);

        System.out.println(this.toString() + "Cycle complete, took " + currentCycleTime + "ms.\n");

        System.out.println(this.toString() + "Time " + (System.currentTimeMillis() - simStartTime) + "ms.\n");

        writeLogData(agent[0].getReplanCycles());

        if (!SimulatorConfig.instantReplan && agent[0].getReplanCycles() != -1) {
            realTimeElapsed = Math.min(realTimeElapsed + agent[0].getReplanCycles() - 1, SimulatorConfig.runDuration);
            agent[0].setReplanCycles(-1);
        }

        if (this.useGui) {
            updateGUI();                // update GUI
        }

        // check whether user wanted to pause
        avgCycleTime = (int) (System.currentTimeMillis() - simStartTime) / timeElapsed;
        checkRunFinish();           // for scripting multiple runs, to max number of cycles
        checkPause();
    }

    public void writeLogData(int replanCycles) {
        if ((timeElapsed - 1) % Constants.LOG_UPDATE_TIME == 0) {
            updateLogFiles(replanCycles);
        }
    }

    public void updateRelevantVariables(int currentCycleTime) {

        for (int i = 1; i < agent.length; i++) {
            if (multihopCommTable[0][i] == 0) {
                timeNotCommunicating1[i] = timeNotCommunicating1[i] + 1;
                timeNotCommunicating2[i] = timeNotCommunicating2[i] + 1;
            }
        }

        for (int i = 0; i < agent.length; i++) {
            replanTime1[i] = agent[i].getReplanTime();
            pctAreaKnown1[i] = 100 * (double) agent[i].getAreaKnown() / (double) totalArea;
        }

        for (int i = 0; i < agent.length; i++) {
            distance1[i] = agent[i].getDistanceTraveled();
        }

    }

    public void writeWinners() {
        String filePath = System.getProperty("user.dir") + "/log_file/winners.txt";
        File winners = new File(filePath);
        try (PrintWriter writer = new PrintWriter(new BufferedWriter(new FileWriter(winners, false)))) {
            writer.println(agent[0].getWinner().get(1));
            writer.println(agent[0].getWinner().get(2));
            writer.close();
        } catch (IOException e) {
        }
    }

    public void updateLogFiles(int replanCycles) {
        String userPath = System.getProperty("user.dir") + "/log_file/";

        File notCommTime = new File(userPath + "notCommTime.txt");
        File dist = new File(userPath + "distanceRobot.txt");
        File pct = new File(userPath + "pctArea.txt");
        File repTime = new File(userPath + "replanTime.txt");

        //log the same data for all the time needed for computing a new plan
        //if instant time is chosen, the loop is executed only once.
        int rc = 0;
        do {
            if (!notCommTime.exists()) {
                try (PrintWriter writer = new PrintWriter(new BufferedWriter(new FileWriter(notCommTime, false)))) {
                    if (replanCycles != -1 && rc == 0 && !SimulatorConfig.instantReplan) {
                        writer.write("#START#\n");
                    }
                    for (int i = 0; i < timeNotCommunicating1.length; i++) {
                        writer.write(timeNotCommunicating1[i] + "\t");
                        writer.flush();
                    }
                    writer.println();

                    if (replanCycles != -1 && rc == replanCycles - 1 && !SimulatorConfig.instantReplan) {
                        writer.write("#END#\n");
                    }
                    writer.close();
                } catch (IOException e) {
                }
            } else {
                try (PrintWriter out = new PrintWriter(new BufferedWriter(new FileWriter(notCommTime, true)))) {

                    if (replanCycles != -1 && rc == 0 && !SimulatorConfig.instantReplan) {
                        out.write("#START#\n");
                    }

                    for (int i = 0; i < timeNotCommunicating1.length; i++) {
                        out.write(timeNotCommunicating1[i] + "\t");
                        out.flush();
                    }
                    out.println();

                    if (replanCycles != -1 && rc == replanCycles - 1 && !SimulatorConfig.instantReplan) {
                        out.write("#END#\n");
                    }

                    out.close();
                } catch (IOException e) {
                }
            }

            if (!dist.exists()) {
                BufferedWriter writer = null;
                try {
                    writer = new BufferedWriter(new FileWriter(dist, false));
                    if (replanCycles != -1 && rc == 0 && !SimulatorConfig.instantReplan) {
                        writer.write("#START#\n");

                    }
                    for (int i = 0; i < distance1.length; i++) {
                        writer.write(distance1[i] + "\t");
                        writer.flush();
                    }
                    writer.newLine();

                    if (replanCycles != -1 && rc == replanCycles - 1 && !SimulatorConfig.instantReplan) {
                        writer.write("#END#\n");
                    }
                    writer.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            } else {
                BufferedWriter writer = null;
                try {
                    writer = new BufferedWriter(new FileWriter(dist, true));
                    if (replanCycles != -1 && rc == 0 && !SimulatorConfig.instantReplan) {
                        writer.write("#START#\n");
                    }
                    for (int i = 0; i < distance1.length; i++) {
                        writer.write(distance1[i] + "\t");
                        writer.flush();
                    }
                    writer.newLine();

                    if (replanCycles != -1 && rc == replanCycles - 1 && !SimulatorConfig.instantReplan) {
                        writer.write("#END#\n");
                    }

                    writer.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }

            if (!pct.exists()) {
                BufferedWriter writer = null;
                try {
                    writer = new BufferedWriter(new FileWriter(pct, false));
                    if (replanCycles != -1 && rc == 0 && !SimulatorConfig.instantReplan) {
                        writer.write("#START#\n");
                    }
                    for (int i = 0; i < pctAreaKnown1.length; i++) {
                        writer.write(pctAreaKnown1[i] + "\t");
                        writer.flush();
                    }
                    writer.newLine();
                    if (replanCycles != -1 && rc == replanCycles - 1 && !SimulatorConfig.instantReplan) {
                        writer.write("#END#\n");
                    }

                    writer.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            } else {
                BufferedWriter writer = null;
                try {
                    writer = new BufferedWriter(new FileWriter(pct, true));
                    if (replanCycles != -1 && rc == 0 && !SimulatorConfig.instantReplan) {
                        writer.write("#START#\n");

                    }
                    for (int i = 0; i < pctAreaKnown1.length; i++) {
                        writer.write(pctAreaKnown1[i] + "\t");
                        writer.flush();
                    }
                    writer.newLine();
                    if (replanCycles != -1 && rc == replanCycles - 1 && !SimulatorConfig.instantReplan) {
                        writer.write("#END#\n");
                    }
                    writer.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }

            if (!repTime.exists()) {
                BufferedWriter writer = null;
                try {
                    writer = new BufferedWriter(new FileWriter(repTime, false));
                    if (replanCycles != -1 && rc == 0 && !SimulatorConfig.instantReplan) {
                        writer.write("#START#\n");

                    }
                    for (int i = 0; i < replanTime1.length; i++) {
                        writer.write(replanTime1[i] + "\t");
                        writer.flush();
                    }
                    writer.newLine();
                    if (replanCycles != -1 && rc == replanCycles - 1 && !SimulatorConfig.instantReplan) {
                        writer.write("#END#\n");
                    }
                    writer.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            } else {
                BufferedWriter writer = null;
                try {
                    writer = new BufferedWriter(new FileWriter(repTime, true));
                    if (replanCycles != -1 && rc == 0 && !SimulatorConfig.instantReplan) {
                        writer.write("#START#\n");
                    }

                    for (int i = 0; i < replanTime1.length; i++) {
                        writer.write(replanTime1[i] + "\t");
                        writer.flush();
                    }
                    writer.newLine();

                    if (replanCycles != -1 && rc == replanCycles - 1 && !SimulatorConfig.instantReplan) {
                        writer.write("#END#\n");
                    }
                    writer.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
            rc += 1;
        } while (rc < replanCycles && (realTimeElapsed + rc <= SimulatorConfig.runDuration));

    }

// <editor-fold defaultstate="collapsed" desc="Start, Run and Stop">
    int runNumber;
    int runNumMax;
    Boolean justStartedSimulation;

    public void start() {

        /*runNumber = 1;
         runNumMax = 33;
         updateRunConfig();
         reset();*/
        //simConfig.TARGET_INFO_RATIO = 0.90;
        RandomWalk.generator.setSeed(SimulatorConfig.randomSeed);
        System.out.println(this.toString() + "Starting exploration!");
        justStartedSimulation = true;
        timer.start();
        //simStartTime = System.currentTimeMillis();

        /*TimerThread timerThread = new TimerThread(this);
         //Thread p = new Thread(timerRun);
         timerThread.setName("TimerThread");
         timerThread.start();

         TimerThread2 timerThread2 = new TimerThread2(this);
         //Thread p = new Thread(timerRun);
         timerThread2.setName("TimerThread2");
         timerThread2.start();*/
    }

    private void restart() {
        /*updateRunConfig();
         reset();*/
        System.out.println(this.toString() + "Restarting exploration!");
        justStartedSimulation = true;
        simStartTime = System.currentTimeMillis();
        //timer.start();
    }

    public void takeOneStep() {
        pauseSimulation = true;
    }

    private void checkPause() {
        if (pauseSimulation || timeElapsed % 15000 == 0) {
            pauseSimulation = false;
            this.pause();
        }
    }

    private boolean allAgentsDone() {
        /*for(RealAgent a: agent) {
         if(a.getID() == 1)
         continue;
         if(!a.isMissionComplete())
         return false;
         if(!a.getTeammate(1).isInRange())
         return false;
         }
         return true;*/

        return (((double) agent[0].getAreaKnown() / (double) totalArea) > Constants.TERRITORY_PERCENT_EXPLORED_GOAL);
    }

    private void checkRunFinish() {

        //Never the need to go back to the BS
        if ((SimulatorConfig.instantReplan && timeElapsed >= SimulatorConfig.runDuration)
                || (!SimulatorConfig.instantReplan && realTimeElapsed >= SimulatorConfig.runDuration) || allAgentsDone()) {// || simTimeNoReplan1 >= 1200000) { //- stumpTime

            writeWinners(); //JB
            System.exit(0);
            /*timer.stop();
             runNumber++;
             if (runNumber < runNumMax) {
             restart();
             }*/
        }
    }

    public void pause() {
        timer.stop();
        System.out.println(this.toString() + "Pausing exploration!");
    }

    public void kill() {
        timer.stop();
        System.out.println(this.toString() + "Resetting exploration!");
    }

    public void actionPerformed(ActionEvent e) {
        try {
            simulationCycle();
        } catch (FileNotFoundException e1) {
            e1.printStackTrace();
        }
    }
// </editor-fold>     

// <editor-fold defaultstate="collapsed" desc="Agent Steps, Sensor Data">
    private void agentSteps() throws FileNotFoundException {
        //long realtimeStartAgentCycle;        
        Point nextStep = new Point(0, 0);         // The next location that an agent wants to go to
        //double[] sensorData;    // Sensor data for an agent at its next location

        //agent[0].flush(); 
        //List<Thread> threads = new ArrayList<Thread>(); 
        //int needNewGoal = 0;
        //int updateFrontiers = 0;
        HashMap<Integer, Point> m_opt = new HashMap<Integer, Point>();

        for (int i = 0; i < agent.length; i++) { //default value: i=1

            /*Runnable task = new AgentStepRunnable(agent[i], simConfig, timeElapsed, env, this);
             Thread worker = new Thread(task);
             worker.setName(agent[i].toString());
             worker.start();
             threads.add(worker);   */
            // <editor-fold defaultstate="collapsed" desc="NoThreads">
            long realtimeStartAgentCycle = System.currentTimeMillis();
            if (agent[i].getRobotNumber() != Constants.BASE_STATION_ID) { //AGGIUNTO

                //Point nextStep = agent[i].getLocation();
                double[] sensorData;
                double distance_left = agent[i].getSpeed();

                if (justStartedSimulation) {
                    // If the simulation just started, ensure that the robots perform the perception first
                    // Note that it may be better to decouple motion and sensing
                    sensorData = findSensorData(agent[i], agent[i].getLocation(), simConfig);
                    agent[i].writeStep(agent[i].getLocation(), sensorData, env);
                }
                while (distance_left > 0) {
                    System.out.println(agent[i].toString() + " location is (" + agent[i].getX() + ", " + agent[i].getY() + ")");

                    if (distance_left < agent[i].getSpeed() && agent[i].getLocation().equals(agent[i].getCurrentGoal())) {
                        System.out.println(agent[i].toString() + " reached goal in a subsequent step.");
                        //agent[i].setTempStep(null);
                        break;
                    }

                    System.out.println(agent[i].toString() + " the goal is: " + agent[i].getCurrentGoal().toString());
                    System.out.println(agent[i].toString() + " distance left: " + distance_left);

                    //System.out.println(agent[i].toString() + " before taking step -> " + agent[i].getCurrentGoal());
                    nextStep = agent[i].takeStep(timeElapsed, simConfig, agent[0]);

                    //System.out.println(agent[i].toString() + " after taking step -> " + agent[i].getCurrentGoal());
                    if (nextStep == null) {
                        //mainGUI.runComplete();  // run complete
                        //return;
                        System.out.println("ERROR: agent " + i + " chose NULL step!");
                        nextStep = agent[i].getLocation();
                    }
                    agent[i].flush();

                    if (env.directLinePossible(agent[i].getX(), agent[i].getY(), nextStep.x, nextStep.y)) {
                        //check here we don't 'teleport'
                        double dist = agent[i].getLocation().distance(nextStep);
                        if (dist < 0.1) {
                            //agent[i].setTempStep(null);
                            break;
                        }

                        if (dist > 10) {
                            System.out.println(agent[i].toString() + " The point is too far");
                            System.out.println(agent[i].toString() + " dist is " + dist);
                            System.out.println(agent[i].toString() + " location is (" + agent[i].getX() + ", " + agent[i].getY() + ")");
                            System.out.println(agent[i].toString() + " next step is " + nextStep);
                            System.out.println(agent[i].toString() + " goal is " + agent[i].getCurrentGoal());
                        }
                        System.out.println(agent[i].toString() + " distance to next path point: " + dist);
                        if (dist > distance_left) {

                            //System.out.println(agent[i].toString() + " before correction -> " + agent[i].getCurrentGoal());
                            Point tempDest = new Point();
                            tempDest.x = nextStep.x;
                            tempDest.y = nextStep.y;

                            System.out.println(agent[i].toString() + " exceeded speed. Distance left: " + distance_left + ", dist to next path point: " + dist);

                            //double ratio = distance_left / dist;
                            //nextStep.x = agent[i].getX() + (int) Math.round((nextStep.x - agent[i].getX()) * ratio);
                            //nextStep.y = agent[i].getY() + (int) Math.round((nextStep.y - agent[i].getY()) * ratio);
                            double deltaY = nextStep.y - agent[i].getY();
                            double deltaX = nextStep.x - agent[i].getX();

                            double alpha = Math.atan2(deltaY, deltaX);
                            nextStep.x = (int) Math.round((double) agent[i].getX() + (Math.cos(alpha) * distance_left));
                            nextStep.y = (int) Math.round((double) agent[i].getY() + (Math.sin(alpha) * distance_left));

                            if (!env.directLinePossible(agent[i].getX(), agent[i].getY(), nextStep.x, nextStep.y)) {
                                nextStep.x = agent[i].getX();
                                nextStep.y = agent[i].getY();
                                agent[i].setEnvError(true);
                                //agent[i].setTempStep(null);
                                System.out.println(agent[i].toString() + " directLinePossible returned wrong result!");
                            }

                            System.out.println(agent[i].toString() + "step corrected. Now is " + nextStep.toString() + " (speed " + agent[i].getLocation().distance(nextStep) + ")");

                            //Here I reput the supposed step in the list, since I did not 
                            //reach it, but only if it is different from the nextStep (due to approx)
                            if (!tempDest.equals(nextStep)) {
                                //System.out.println(agent[i].toString() + " before setting -> " + agent[i].getCurrentGoal());
                                System.out.println(agent[i].toString() + " setting not completed step " + tempDest.toString());
                                //agent[i].setTempStep(tempDest);
                                agent[i].getPath().getPoints().add(0, tempDest);
                                //System.out.println(agent[i].toString() + " after setting -> " + agent[i].getCurrentGoal());
                            }

                            distance_left = 0;
                        } else {
                            distance_left = distance_left - dist;
                            //agent[i].setTempStep(null);

                        }
                        //System.out.println(agent[i].toString() + " before sensor -> " + agent[i].getCurrentGoal());
                        sensorData = findSensorData(agent[i], nextStep, simConfig);
                        agent[i].writeStep(nextStep, sensorData, env);
                        //System.out.println(agent[i].toString() + " after sensor -> " + agent[i].getCurrentGoal());
                    } else {
                        System.out.println("Something went wrong.");
                        distance_left = 0;
                        agent[i].setEnvError(true);
                    }

                    if (simConfig.getExpAlgorithm() == exptype.RunFromLog) {
                        //This is because the log represents what happens out of 
                        //the while loop
                        distance_left = 0;
                    }
                    //System.out.println(agent[i].toString() + " before looping -> " + agent[i].getCurrentGoal());
                }
                System.out.println(agent[i].toString() + "Agent cycle complete, took " + (System.currentTimeMillis() - realtimeStartAgentCycle) + "ms.");
                //System.out.println(agent[i].toString() + " -> " + agent[i].getCurrentGoal());

            } else {

                if ((simConfig.getExpAlgorithm() == exptype.StumpExploration || simConfig.getExpAlgorithm() == exptype.IlpExploration
                        || simConfig.getExpAlgorithm() == exptype.ApproxExploration || simConfig.getExpAlgorithm() == exptype.MixedExploration 
                        || (simConfig.getExpAlgorithm() == exptype.SwitchExploration && SwitchExploration.getState().equals(SwitchExploration.SwitchState.MultiHopping))                        
                        || simConfig.getExpAlgorithm() == exptype.BirkExploration 
                        || simConfig.getExpAlgorithm() == exptype.DemurIlpExploration) && timeElapsed >= Constants.INIT_CYCLES) {
                    try {
                        switch (simConfig.getExpAlgorithm()) {
                            case StumpExploration:
                                m_opt = StumpExploration.replan(agent[i], frontiertype.ReturnWhenComplete, timeElapsed, simConfig);
                                if (agent[0].getTimeToRecalculate() == 0) { //STUMP DISTANCE
                                    for (int k = 0; k < agent.length; k++) {
                                        distanceLastStump1[k] = distance1[k];
                                        distanceLastStump2[k] = distance2[k];
                                        oldPctAreaKnown1[k] = pctAreaKnown1[k];
                                        oldPctAreaKnown2[k] = pctAreaKnown2[k];
                                    }
                                }
                                break;

                            case IlpExploration:
                                m_opt = AsyncExploration.replan(agent[i], frontiertype.ReturnWhenComplete, realTimeElapsed, simConfig);
                                //AsyncExploration.reallocate(agent[i]);
                                break;
                            case ApproxExploration:
                                m_opt = AsyncExploration.replan(agent[i], frontiertype.ReturnWhenComplete, realTimeElapsed, simConfig);
                                //AsyncExploration.reallocate(agent[i]);
                                break;
                            case MixedExploration:
                                m_opt = AsyncExploration.replan(agent[i], frontiertype.ReturnWhenComplete, realTimeElapsed, simConfig);
                                //AsyncExploration.reallocate(agent[i]);
                                break;
                            case DemurIlpExploration:
                                m_opt = DemurIlpExploration.replan(agent[i], frontiertype.ReturnWhenComplete, timeElapsed, simConfig);
                                //AsyncExploration.reallocate(agent[i]);
                                break;
                            case BirkExploration:
                                m_opt = BirkExploration.replan(agent[i], frontiertype.ReturnWhenComplete, timeElapsed, simConfig);
                                break;
                            case SwitchExploration:
                                m_opt = SwitchExploration.replan(agent[i], frontiertype.ReturnWhenComplete, timeElapsed, simConfig);
                                break;
                        }

                        if (agent[0].isEndExploration()) {
                            System.exit(0);
                        }

                        if (m_opt != null) {

                        } else {
                            for (int k = 0; k < agent.length - 1; k++) {
                                agent[k].setMissionComplete();
                            }
                            /*timer.stop();
                             pause();*/
                        }

                    } catch (FileNotFoundException e) {
                        e.printStackTrace();
                    }
                }

                //A new plan is computed only when every robot has access to it. 
                if (simConfig.getExpAlgorithm() != exptype.FrontierExploration
                        && simConfig.getExpAlgorithm() != exptype.RunFromLog
                        && simConfig.getExpAlgorithm() != exptype.RoleBasedExploration && agent[0].getM_opt_p() != null && timeElapsed > Constants.INIT_CYCLES) {
                    for (int k = 1; k < agent.length; k++) {

                        if (agent[0].getTeammate(k + 1).isInRange()) {
                            agent[k].setM_opt(new Point(agent[0].getM_opt_p().get(k)));

                            if ((simConfig.getExpAlgorithm() == exptype.ApproxExploration || simConfig.getExpAlgorithm() == exptype.IlpExploration 
                                    || (simConfig.getExpAlgorithm() == exptype.SwitchExploration && SwitchExploration.getState().equals(SwitchExploration.SwitchState.MultiHopping))
                                    || simConfig.getExpAlgorithm() == exptype.MixedExploration) && !agent[k].getCurrentGoal().equals(agent[k].getM_opt())) {
                                agent[k].setCurrentGoal(new Point(agent[k].getM_opt()));
                                
                                agent[k].setAlreadyRotated(false);
                                agent[k].setCompletedRotation(false);
                                
                                System.out.println("Computing path of " + agent[k].toString() + " to destination " + agent[k].getCurrentGoal());
                                Path path = agent[k].calculatePath(agent[k].getLocation(), new Point(agent[k].getCurrentGoal()), true);

                                if (path.getPoints().size() > 0) {
                                    if (!path.getPoints().get(0).equals(agent[k].getCurrentGoal())) {
                                        path.getPoints().remove(0);
                                    }
                                } else {
                                    /*path = agent[k].calculatePath(agent[k].getLocation(), new Point(agent[k].getCurrentGoal()));
                                    
                                     if (!path.getPoints().get(0).equals(agent[k].getCurrentGoal())) {
                                     path.getPoints().remove(0);
                                     }*/
                                    System.out.println("PANICO!");

                                }
                  
                                if (path.getPoints() != null) {
                                    System.out.println("The path is:");
                                    for (Point p : path.getPoints()) {
                                        System.out.println(p);
                                    }
                                }

                                agent[k].addDirtyCells(agent[k].getPath().getAllPathPixels());
                                agent[k].setPath(path);
                            }

                        }
                    }
                }
            }

            //</editor-fold>                       
        }

        if (justStartedSimulation) {
            justStartedSimulation = false;
        }
        /*for(int i=0; i<threads.size(); i++) {
         try
         {
         threads.get(i).join();
         } catch (Exception e)
         {
         System.out.println("Thread " + i + " threw exception " + e.getMessage());
         }
         }*/
    }

    // Simulates data from laser range finder
    protected double[] findSensorData(RealAgent agent, Point nextLoc, SimulatorConfig simConfig) {
        double currRayAngle, heading;
        int prevRayX, prevRayY;
        int currRayX, currRayY;
        double sensorData[] = new double[SimulatorConfig.fov + 1];

        // TOCHECK: Does it really work taking the current heading of the robot?
        // Quick check: if agent hasn't moved, no new sensor data 
        // 22.04.2010 Julian commented this out to make frontier exp work
        // if(agent.getLocation().equals(nextLoc))
        //    return null;
        if (agent.getLocation().equals(nextLoc)) {
            heading = agent.getHeading();
        } else {
            heading = Math.atan2(nextLoc.y - agent.getY(), nextLoc.x - agent.getX());
        }

        //For every degree
        for (int i = 0; i <= SimulatorConfig.fov; i += 1) {
            prevRayX = nextLoc.x;
            prevRayY = nextLoc.y;

            currRayAngle = heading - Math.toRadians(((float) SimulatorConfig.fov) / 2.0) + Math.PI / 180 * i;

            for (double m = 1; m <= agent.getSenseRange(); m += 1) {
                currRayX = nextLoc.x + (int) Math.floor(m * Math.cos(currRayAngle));
                currRayY = nextLoc.y + (int) Math.floor(m * Math.sin(currRayAngle));

                if (!env.locationExists(currRayX, currRayY)) {
                    sensorData[i] = nextLoc.distance(prevRayX, prevRayY);
                    break;
                } else if (env.statusAt(currRayX, currRayY) == Environment.Status.obstacle) {
                    sensorData[i] = nextLoc.distance(currRayX, currRayY);
                    break;
                } else if (m >= agent.getSenseRange()) {
                    sensorData[i] = nextLoc.distance(currRayX, currRayY);
                    break;
                } else {
                    prevRayX = currRayX;
                    prevRayY = currRayY;
                }
            }
        }

        return sensorData;
    }

    // update area known if needed
    private void updateAgentKnowledgeData() {
        if (simConfig.getExpAlgorithm() == exptype.RunFromLog) {
            return; //Nothing to do here
        }
        for (int i = 0; i < agent.length; i++) {
            agent[i].updateAreaKnown();
        }

    }
// </editor-fold>     

// <editor-fold defaultstate="collapsed" desc="Communication">
    private void simulateCommunication(SimulatorConfig simConfig) {
        long realtimeStart = System.currentTimeMillis();
        //long realtimeStart2;
        System.out.println(this.toString() + "Simulating communication ... ");

        detectCommunication();
        agent[0].setMultiHopCommTable(this.multihopCommTable);
        agent[0].setDirectCommTable(this.directCommTableCopy);
        long elapsed = System.currentTimeMillis() - realtimeStart;
        System.out.println("Detect took " + Long.toString(elapsed));
        //System.out.println(Constants.INDENT + "detectCommunication took " + (System.currentTimeMillis()-realtimeStart) + "ms.");

        //"flush" the bs
        if (simConfig.getExpAlgorithm() == exptype.ApproxExploration
                || simConfig.getExpAlgorithm() == exptype.IlpExploration
                || simConfig.getExpAlgorithm() == exptype.MixedExploration
                || simConfig.getExpAlgorithm() == exptype.DemurIlpExploration
                //only when I behave as AsyncExploration
                || (simConfig.getExpAlgorithm() == exptype.SwitchExploration && SwitchExploration.getState().equals(SwitchExploration.SwitchState.MultiHopping))) {
            for (TeammateAgent t : agent[0].getAllTeammates().values()) {
                t.setInRange(false);
            }
            /*for (int i = 0; i < numRobots; i++) {
             for (Map.Entry<Integer, TeammateAgent> entry : agent[i].getAllTeammates().entrySet()) {
             entry.getValue().setPrevInRange(entry.getValue().isInRange());
             entry.getValue().setInRange(false);
             entry.getValue().setInDirectRange(false);
             //because isInRange will now be updated
             }
             }*/
        }
        // Exchange data
        realtimeStart = System.currentTimeMillis();
        for (int i = 0; i < numRobots - 1; i++) {
            for (int j = i + 1; j < numRobots; j++) {
                if (multihopCommTable[i][j] == 1) {
                    //realtimeStart2 = System.currentTimeMillis();
                    DataMessage msgFromFirst = new DataMessage(agent[i], directCommTable[i][j]);
                    DataMessage msgFromSecond = new DataMessage(agent[j], directCommTable[i][j]);

                    agent[i].receiveMessage(msgFromSecond, timeElapsed);
                    agent[j].receiveMessage(msgFromFirst, timeElapsed);

                    //System.out.println(Constants.INDENT + "Communication between " +
                    //                    agent[i].getName() + " and " +
                    //                    agent[j].getName() + " took " +
                    //                    (System.currentTimeMillis() - realtimeStart2) + "ms.");
                    // For periodic return frontier exp
                    if (simConfig.getExpAlgorithm() == SimulatorConfig.exptype.FrontierExploration
                            && simConfig.getFrontierAlgorithm() == SimulatorConfig.frontiertype.PeriodicReturn
                            && i == 0 && agent[j].frontierPeriodicState == 1) {
                        agent[j].frontierPeriodicState = 0;
                        agent[j].periodicReturnInterval += 10;
                    }
                }
            }
        }

        elapsed = System.currentTimeMillis() - realtimeStart;
        System.out.println("exchange took " + Long.toString(elapsed));
        //realtimeStart = System.currentTimeMillis();
        // Update post comm
        for (int q = 0; q < numRobots; q++) {
            agent[q].updateAfterCommunication();
        }

        //verifyNoInfoGotLost();
        //System.out.println(Constants.INDENT + "updateAfterCommunication took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        //System.out.println(Constants.INDENT + "Communication complete, took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
    }

    private void verifyNoInfoGotLost() {
        //build common occgrid of all agents
        int counter = 0;
        OccupancyGrid commonGrid = new OccupancyGrid(agent[0].getOccupancyGrid().width, agent[0].getOccupancyGrid().height);
        for (int j = 0; j < commonGrid.width; j++) {
            for (int k = 0; k < commonGrid.height; k++) {
                if (!agent[0].getOccupancyGrid().isKnownAtBase(j, k)) {
                    commonGrid.setGotRelayed(j, k);
                    for (int i = 1; i < numRobots; i++) {
                        boolean isRelayed = commonGrid.isGotRelayed(j, k);
                        commonGrid.setByte(j, k, (byte) (commonGrid.getByte(j, k) | agent[i].getOccupancyGrid().getByteNoRelay(j, k)));
                        if (isRelayed != commonGrid.isGotRelayed(j, k)) {
                            System.out.println("THIS SHOULD NEVER HAPPEN!");
                        }
                        if (!agent[i].getOccupancyGrid().isGotRelayed(j, k)) {
                            commonGrid.setGotUnrelayed(j, k);
                        }
                    }
                    //check that someone is relaying information known to agents but unknown at base
                    if (commonGrid.freeSpaceAt(j, k) && !commonGrid.isKnownAtBase(j, k) && commonGrid.isGotRelayed(j, k)) {
                        counter++;
                    }
                }
            }
        }
        if (counter > 0) {
            int res = verifyNoInfoGotLost2();
            if (res > 0) {
                System.out.println("ERROR: INFORMATION LOST!! " + counter);
                System.out.println("@@@@@@@@@@@@@@  DIFF = " + res + "     @@@@@@@@@@@@@@@@@");
            }
        }
    }

    public int verifyNoInfoGotLost2() {
        //build common occgrid of all agents
        int counter_allagents = 0;
        int counter_base = 0;
        int newCell = 0;
        OccupancyGrid commonGrid = new OccupancyGrid(agent[0].getOccupancyGrid().width, agent[0].getOccupancyGrid().height);
        for (int j = 0; j < commonGrid.width; j++) {
            for (int k = 0; k < commonGrid.height; k++) {
                boolean newCellFound = false;
                commonGrid.setBit(j, k, OccupancyGrid.OccGridBit.FreeSpace, 0);
                for (int i = 0; i < numRobots; i++) {
                    if (agent[i].getOccupancyGrid().freeSpaceAt(j, k)) {
                        commonGrid.setFreeSpaceAt(j, k);
                        break;
                    }
                }

                for (int i = 1; i < numRobots; i++) {
                    if (agent[i].getOccupancyGrid().freeSpaceAt(j, k)) {
                        if ((!agent[i].getOccupancyGrid().isKnownAtBase(j, k))
                                && (!agent[i].getOccupancyGrid().isGotRelayed(j, k))) {
                            newCell++;
                            newCellFound = true;
                            break;
                        }
                    }
                }

                /*if ((commonGrid.freeSpaceAt(j, k)) &&
                 !(agent[0].getOccupancyGrid().freeSpaceAt(j, k)) &&
                 !newCellFound)
                 System.out.println("~~~ CELL LOST: (" + j + ", " + k + ")");
                 */
                //check that someone is relaying information known to agents but unknown at base
                if (commonGrid.freeSpaceAt(j, k)) {
                    counter_allagents++;
                }
                if (agent[0].getOccupancyGrid().freeSpaceAt(j, k)) {
                    counter_base++;
                }
            }
        }
        int result = (counter_allagents - counter_base - newCell);
        return result;
    }

    private static int[][] detectMultiHopLinks(int commTable[][]) {
        for (int i = 0; i < commTable.length; i++) {
            for (int j = 0; j < commTable[0].length; j++) {
                if (commTable[i][j] == 1 || commTable[j][i] == 1) {
                    for (int k = 0; k < commTable.length; k++) {
                        if (commTable[k][i] == 1 || commTable[i][k] == 1) {
                            commTable[k][j] = 1;
                            commTable[j][k] = 1;
                        }
                    }
                }
            }
        }

        return commTable;
    }

    private void detectCommunication() {
        directCommTable = new int[numRobots][numRobots];
        multihopCommTable = new int[numRobots][numRobots];

        switch (simConfig.getCommModel()) {
            case StaticCircle:
                directCommTable = StaticCircle.detectCommunication(env, agent);
                break;
            case DirectLine:
                directCommTable = DirectLine.detectCommunication(env, agent);
                break;
            case PropModel1:
                directCommTable = PropModel1.detectCommunication(env, agent);
                if (this.useGui) {
                    for (int i = 0; i < numRobots; i++) {
                        if (mainGUI.getRobotPanel(i).showCommRange()) {
                            agentRange[i] = PropModel1.getRange(env, agent[i]);
                        }
                    }
                }
                break;
            default:
                break;
        }

        directCommTableCopy = new int[directCommTable.length][];
        for (int i = 0; i < directCommTable.length; i++) {
            directCommTableCopy[i] = directCommTable[i].clone();
        }

        multihopCommTable = detectMultiHopLinks(directCommTable);
    }

    // </editor-fold>     
// <editor-fold defaultstate="collapsed" desc="Role Switch">
    private void switchRoles(RealAgent agent1, RealAgent agent2, Path p1, Path p2) {
        //System.out.println(this.toString() + "Switching " + agent1.getName() + " and " + agent2.getName() + "... ");

        // create temp copy of each agent for teammate maintenance
        TeammateAgent copyAgent1 = agent2.getTeammate(agent1.getID()).copy();
        TeammateAgent copyAgent2 = agent1.getTeammate(agent2.getID()).copy();

        // switch ID
        int tempID = agent1.getID();
        agent1.setID(agent2.getID());
        agent2.setID(tempID);

        // reinit time since last plan 
        //agent1.setTimeSinceLastPlan(15);
        //agent2.setTimeSinceLastPlan(15);
        // reinit state timer
        //agent1.setStateTimer(15);
        //agent2.setStateTimer(15);
        // exchange frontiers
        LinkedList<Frontier> tempFrontiers = agent1.getFrontiers();
        agent1.setFrontiers(agent2.getFrontiers());
        agent2.setFrontiers(tempFrontiers);

        // exchange childRV
        RVLocation tempChildRV = agent1.getChildRendezvous();
        agent1.setChildRendezvous(agent2.getChildRendezvous());
        agent2.setChildRendezvous(tempChildRV);

        // exchange parentRV
        RVLocation tempParentRV = agent1.getParentRendezvous();
        agent1.setParentRendezvous(agent2.getParentRendezvous());
        agent2.setParentRendezvous(tempParentRV);

        // exchange exploreState
        ExploreState tempExploreState = agent1.getState();
        agent1.setState(agent2.getState());
        agent2.setState(tempExploreState);

        // exchange parent
        int tempParent = agent1.getParent();
        agent1.setParent(agent2.getParent());
        agent2.setParent(tempParent);

        // exchange child
        int tempChild = agent1.getChild();
        agent1.setChild(agent2.getChild());
        agent2.setChild(tempChild);

        // exchange role
        roletype tempRole = agent1.getRole();
        agent1.setRole(agent2.getRole());
        agent2.setRole(tempRole);

        // exchange current goal (important!)
        Point tempCurrGoal = agent1.getCurrentGoal();
        agent1.setCurrentGoal(agent2.getCurrentGoal());
        agent2.setCurrentGoal(tempCurrGoal);

        // set newly calculated path
        agent1.setPath(p1);
        agent1.addDirtyCells(p1.getAllPathPixels());
        agent2.setPath(p2);
        agent2.addDirtyCells(p2.getAllPathPixels());

        // exchange lastFrontier
        Frontier tempLastFrontier = agent1.getLastFrontier();
        agent1.setLastFrontier(agent2.getLastFrontier());
        agent2.setLastFrontier(tempLastFrontier);

        // replace Teammate agents
        agent1.removeTeammate(agent2.getID());
        agent1.addTeammate(copyAgent1);
        agent2.removeTeammate(agent1.getID());
        agent2.addTeammate(copyAgent2);
    }

    private boolean checkRoleSwitch(int first, int second) {
        RealAgent agent1 = agent[first];
        RealAgent agent2 = agent[second];

        if (agent1.getTimeSinceLastRoleSwitch() < 4
                || agent2.getTimeSinceLastRoleSwitch() < 4) {
            return false;
        }

        // Specific scenario which leads to oscillation must be avoided
        /*if((agent1.isExplorer() && agent1.getState() == ExploreState.Explore &&
         !agent2.isExplorer() && agent2.getState() == ExploreState.GoToChild) ||
         (agent2.isExplorer() && agent2.getState() == ExploreState.Explore &&
         !agent1.isExplorer() && agent1.getState() == ExploreState.GoToChild))
         return false;*/
        /* path.Path path1 = new path.Path(agent1.getOccupancyGrid(), agent1.getLocation(), agent1.getCurrentGoal());
         path.Path path2 = new path.Path(agent2.getOccupancyGrid(), agent2.getLocation(), agent2.getCurrentGoal());
         path.Path path3 = new path.Path(agent1.getOccupancyGrid(), agent1.getLocation(), agent2.getCurrentGoal());
         path.Path path4 = new path.Path(agent2.getOccupancyGrid(), agent2.getLocation(), agent1.getCurrentGoal());

         if(Math.max(path1.getLength(), path2.getLength()) > Math.max(path3.getLength(), path4.getLength())) {
         // Apply dynamic hierarchy rule, switch roles
         switchRoles(agent1, agent2);
         return true;
         }*/
        try {
            Path path_a1g2 = agent1.calculatePath(agent1.getLocation(), agent2.getCurrentGoal());
            Path path_a2g1 = agent2.calculatePath(agent2.getLocation(), agent1.getCurrentGoal());
            double agent1_goal1 = agent1.getPath().recalcLength();
            double agent2_goal2 = agent2.getPath().recalcLength();
            double agent1_goal2 = path_a1g2.getLength();
            double agent2_goal1 = path_a2g1.getLength();

            //System.out.println(Constants.INDENT + "ROLE SWITCH DEBUG" +
            //                   "\n       " + agent1.getName() + " to goal1 = " + (int)agent1_goal1 +
            //                   "\n       " + agent2.getName() + " to goal2 = " + (int)agent2_goal2 +
            //                   "\n       " + agent1.getName() + " to goal2 = " + (int)agent1_goal2 +
            //                   "\n       " + agent2.getName() + " to goal1 = " + (int)agent2_goal1);
            // First check:  is it possible to eliminate the longest path?  If not, don't switch.
            if (Math.max(agent1_goal1, agent2_goal2)
                    <= Math.max(agent1_goal2, agent2_goal1)) {
                return false;
            }

            // Second check (if desired): does the overall responsiveness improve?  If not, don't switch.
            if (simConfig.strictRoleSwitch()) {

                // Case 1:  Two explorers both in state explore
                if (agent1.isExplorer() && agent1.getState() == ExploreState.Explore
                        && agent2.isExplorer() && agent2.getState() == ExploreState.Explore) {

                    Path rv1ToCS = agent1.calculatePath(agent1.getParentRendezvous().getParentLocation(),
                            agent1.getTeammate(Constants.BASE_STATION_ID).getLocation());
                    Path rv2ToCS = agent2.calculatePath(agent2.getParentRendezvous().getParentLocation(),
                            agent2.getTeammate(Constants.BASE_STATION_ID).getLocation());

                    Path a1ToRV2 = agent1.calculatePath(agent1.getLocation(),
                            agent2.getParentRendezvous().getChildLocation());
                    Path a2ToRV1 = agent2.calculatePath(agent2.getLocation(),
                            agent1.getParentRendezvous().getChildLocation());

                    double noRoleSwitch = Math.max(agent1.getTimeUntilRendezvous() + rv1ToCS.getLength(),
                            agent2.getTimeUntilRendezvous() + rv2ToCS.getLength());
                    double yesRoleSwitch = Math.min(a1ToRV2.getLength() + rv2ToCS.getLength(),
                            a2ToRV1.getLength() + rv1ToCS.getLength());

                    //System.out.println("\n\n\n\n\n*********************************");
                    //System.out.println("     No role switch: " + noRoleSwitch);
                    //System.out.println("    Yes role switch: " + yesRoleSwitch);
                    //System.out.println("*********************************\n\n\n\n");
                    if (noRoleSwitch <= yesRoleSwitch) {
                        //System.out.println("\n\n*********************************");
                        //System.out.println("    SWITCH DENIED");
                        //System.out.println("*********************************\n");
                        return false;
                    }
                }

            }

            // If we reach this point, we want to switch roles.
            switchRoles(agent1, agent2, path_a1g2, path_a2g1);
            agent1.setTimeSinceLastRoleSwitch(0);
            agent2.setTimeSinceLastRoleSwitch(0);
            return true;
        } catch (NullPointerException e) {
        }

        return false;
    }

    private void switchRoles() {
        // Quick check if role switching is desired
        if (!simConfig.roleSwitchAllowed()) {
            return;
        }

        long realtimeStart = System.currentTimeMillis();
        //System.out.println(this.toString() + "Checking for role switch ... ");

        // Note, need timeout on number of swaps a robot can do (at least 3 timestep gap?)
        if (timeElapsed > 5 && timeElapsed % 10 == 0) {
            for (int i = 1; i < numRobots - 1; i++) {
                for (int j = i + 1; j < numRobots; j++) {
                    if (agent[i].getTeammate(agent[j].getID()).isInRange()) {
                        // Only one role switch per time step allowed at the moment
                        if (checkRoleSwitch(i, j)) {
                            // exit loops
                            numSwaps++;
                            j = numRobots;
                            i = numRobots;
                        }
                    }
                }
            }
        }

        //System.out.println(this.toString() + "Role switch check complete, took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
    }

// </editor-fold>     
// <editor-fold defaultstate="collapsed" desc="Logging">
    private void logging() {
        // Note, logging of data is performed in updateGlobalData, should change to here when i have the time

        // Log screenshot
        if (this.useGui && simConfig.logScreenshots()) {
            logScreenshot();
        }

        if (simConfig.getExpAlgorithm() == exptype.RunFromLog) {
            return; //Nothing to do here
        }
        // Log agent positions
        if (simConfig.logAgents()) {
            logAgents();
        }

    }

    private void logAgents() {
        try {
            PrintWriter outFile = new PrintWriter(new FileWriter(simConfig.getLogAgentFilename(), true));

            outFile.print(timeElapsed + " ");
            for (int i = 0; i < numRobots; i++) {
                outFile.print(agent[i].getX() + " ");
                outFile.print(agent[i].getY() + " ");
                outFile.print(agent[i].getCurrentGoal().getX() + " ");
                outFile.print(agent[i].getCurrentGoal().getY() + " ");
                outFile.print(agent[i].getRole() + " ");
                outFile.print(agent[i].getState() + " ");
                if (i == 0) {
                    outFile.print("true ");
                } else {
                    outFile.print(agent[0].getTeammate(i + 1).wasReady() + " ");
                }

                outFile.print(agent[i].getHeading() + " ");

            }
            outFile.println();
            outFile.close();
        } catch (IOException e) {
            System.out.println(this.toString() + "Agent logging - error writing data to file!" + e);
        }
    }

    private void logScreenshot() {
        image.fullUpdate(mainGUI.getShowSettings(), mainGUI.getShowSettingsAgents(), env, agent, agentRange);
        image.saveScreenshot(simConfig.getLogScreenshotsDirname(), timeElapsed);
    }

// </editor-fold>     
// <editor-fold defaultstate="collapsed" desc="GUI Interaction">
    public void simRateChanged(int newSimRate, MainGUI.runMode RUNMODE) {
        if (newSimRate == 0) {
            timer.stop();
        } else {
            if (!timer.isRunning() && !RUNMODE.equals(MainGUI.runMode.paused)) {
                timer.start();
            }
            timer.setDelay((Constants.TIME_INCREMENT * 10 + 1) - newSimRate * Constants.TIME_INCREMENT);
            //System.out.println(this.toString() + "Timer set to " + ((Constants.TIME_INCREMENT*10+1) - newSimRate*Constants.TIME_INCREMENT));
        }
    }

    private void updateGUI() {
        //long realtimeStart = System.currentTimeMillis();
        //System.out.print(this.toString() + "Updating GUI ... ");

        mainGUI.updateFromData(agent, realTimeElapsed, pctAreaKnown, avgCycleTime);
        updateImage(false); //was false

        //System.out.println(this.toString() + "GUI Update complete, took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
    }

    public void updateImage(boolean full) {
        long realtimeStart = System.currentTimeMillis();

        if (full) {
            //System.out.print(this.toString() + "Full Image Update ... ");
            image.fullUpdate(mainGUI.getShowSettings(), mainGUI.getShowSettingsAgents(), env, agent, agentRange);
        } else {
            //System.out.print(this.toString() + "Dirty Image Update ... ");
            image.dirtyUpdate(mainGUI.getShowSettings(), mainGUI.getShowSettingsAgents(), env, agent, agentRange);
        }
        mainGUI.getLabelImageHolder().setIcon(new ImageIcon(image.getImage()));

        //System.out.println("Complete, took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
    }

    public int getTrueJointAreaKnown() {
        int known = 0;
        for (int j = 0; j < env.getColumns(); j++) {
            for (int k = 0; k < env.getRows(); k++) {
                for (int i = 0; i < agent.length; i++) {
                    if ((agent[i].getOccupancyGrid().freeSpaceAt(j, k))
                            && (env.statusAt(j, k) != Status.obstacle)) {
                        known++; //"true" area known, excluding false empty spaces
                        i = agent.length;
                    }
                }
            }
        }
        return known;
    }

    private void updateGlobalData(int replanCycles) {
        long realtimeStart = System.currentTimeMillis();
        //System.out.print(this.toString() + "Updating Global Data ... ");
        timeElapsed++;
        realTimeElapsed++;

        pctAreaKnown = 100 * (double) agent[0].getAreaKnown() / (double) totalArea; //MODIFICATO

        //System.out.println(this.toString() + "% area known " + pctAreaKnown + "\n");
        if (simConfig.logData()) {
            avgAgentKnowledge = 0;
            avgTimeLastCommand = 0;
            totalDistanceTraveled = 0;
            //jointAreaKnown = 1;

            if ((timeElapsed % Constants.RECALC_JOINT_AREA) == 1) {
                jointAreaKnown = getTrueJointAreaKnown();
            } else {
                jointAreaKnown = Math.max(agent[0].getAreaKnown(), jointAreaKnown); //MODIFICATO
            }
            jointAreaKnown = Math.max(agent[0].getAreaKnown(), jointAreaKnown); //MODIFICATO

            for (int i = 1; i < agent.length; i++) {
                avgAgentKnowledge += agent[i].getAreaKnown();
                avgTimeLastCommand += agent[i].getTimeLastCentralCommand();
                totalDistanceTraveled += agent[i].getDistanceTraveled();
            }
            avgAgentKnowledge /= (agent.length - 1);  //ComStation not included in calculation
            avgAgentKnowledge = 100 * avgAgentKnowledge / jointAreaKnown;
            avgTimeLastCommand /= (agent.length - 1);

            if (jointAreaKnown == 0) {
                avgComStationKnowledge = 0;
            } else if (timeElapsed < 2) {
                avgComStationKnowledge = 100 * agent[0].getAreaKnown() / jointAreaKnown;
            } else {
                avgComStationKnowledge = ((timeElapsed - 1) * avgComStationKnowledge
                        + (100 * agent[0].getAreaKnown() / jointAreaKnown))
                        / timeElapsed;
            }
            pctAreaKnown = 100 * (double) jointAreaKnown / (double) totalArea;

            try {
                PrintWriter outFile = new PrintWriter(new FileWriter(simConfig.getLogDataFilename(), true));

                outFile.print(timeElapsed + " ");
                outFile.print(System.currentTimeMillis() + " ");
                outFile.print(pctAreaKnown + " ");
                outFile.print(100 * (double) agent[0].getAreaKnown() / (double) totalArea + " ");
                outFile.print(avgAgentKnowledge + " ");
                outFile.print(avgTimeLastCommand + " ");
                outFile.print((double) totalDistanceTraveled / (double) (agent.length - 1) + " ");
                outFile.print(numSwaps + " ");
                outFile.print(jointAreaKnown + " ");
                outFile.print(agent[0].getAreaKnown() + " ");
                outFile.print(100 * (double) agent[0].getAreaKnown() / (double) jointAreaKnown + " ");
                for (int i = 1; i < agent.length; i++) {
                    outFile.print(agent[i].getAreaKnown() + " ");
                    outFile.print(agent[i].getNewInfo() + " ");
                    outFile.print(agent[i].getMaxRateOfInfoGatheringBelief() + " ");
                    outFile.print(agent[i].getCurrentTotalKnowledgeBelief() + " ");
                    outFile.print(agent[i].getCurrentBaseKnowledgeBelief() + " ");
                }
                outFile.println();
                outFile.close();
            } catch (IOException e) {
                System.out.println(this.toString() + "Error writing data to file!" + e);
            }
        }
        //System.out.println("Complete, took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
    }
// </editor-fold>     

// <editor-fold defaultstate="collapsed" desc="Utility">
    @Override
    public String toString() {
        return ("[Simulator] ");
    }
// </editor-fold>     

// <editor-fold defaultstate="collapsed" desc="Debris">
    private void simulateDebris() {
        int debrisSize, currX, currY, nextX, nextY;

        /* The below puts random debris anywhere
         // according to constant NEW_DEBRIS_LIKELIHOOD, add debris
         if(random.nextInt(100) < (int)(Constants.NEW_DEBRIS_LIKELIHOOD * 100)) {
         debrisSize = random.nextInt(Constants.NEW_DEBRIS_MAX_SIZE) + 1;

         System.out.println(this.toString() + "Adding random debris of size " + debrisSize + "!");
            
         currX = random.nextInt(env.getColumns());
         currY = random.nextInt(env.getRows());
            
         for(int i=0; i<debrisSize; i++) {
         env.setStatus(currY, currX, Status.obstacle);
         do {
         nextX = currX + random.nextInt(3) - 1;
         nextY = currY + random.nextInt(3) - 1;
         }
         while(!env.locationExists(nextX, nextY));
         currX = nextX;
         currY = nextY;
         }
         } */

        /* The below is purely for the aisleRoom environment */
        // Gate 1
        /*if(debrisTimer[0] <= 0) {
         if(random.nextInt(100) < 5) {
         closeGate(46);
         debrisTimer[0] = 10;
         }
         }
         else if(debrisTimer[0] == 1) {
         openGate(46);
         debrisTimer[0] = 0;
         }
         else
         debrisTimer[0]--;

         if(debrisTimer[1] <= 0) {
         if(random.nextInt(100) < 5) {
         closeGate(121);
         debrisTimer[0] = 10;
         }
         }
         else if(debrisTimer[1] == 1) {
         openGate(121);
         debrisTimer[1] = 0;
         }
         else
         debrisTimer[1]--;

         if(debrisTimer[2] <= 0) {
         if(random.nextInt(100) < 5) {
         closeGate(196);
         debrisTimer[0] = 10;
         }
         }
         else if(debrisTimer[2] == 1) {
         openGate(196);
         debrisTimer[2] = 0;
         }
         else
         debrisTimer[2]--;*/
    }

    private void closeGate(int yTop) {
        for (int i = yTop; i < yTop + 67; i++) {
            for (int j = 250; j < 258; j++) {
                env.setStatus(j, i, Status.obstacle);
            }
        }
    }

    private void openGate(int yTop) {
        for (int i = yTop; i < yTop + 67; i++) {
            for (int j = 250; j < 258; j++) {
                env.setStatus(j, i, Status.unexplored);
            }
        }
    }

// </editor-fold>   
    //AGGIUNTO
    public RealAgent[] getAgents() {
        return this.agent;
    }
}
