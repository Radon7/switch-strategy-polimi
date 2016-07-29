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
package config;
import java.io.*;
import environment.*;

/**
 * Describes the model of the simulator configuration.
 * @author julh
 */

public class SimulatorConfig {
    
// <editor-fold defaultstate="collapsed" desc="Variables and Constructor">


    private boolean logAgents;
    private String logAgentsFilename;
    private boolean logData;
    private String logDataFilename;
    private boolean logScreenshots;
    private String logScreenshotsDirname;
    private String runFromLogFilename;
    
    public static enum commtype {StaticCircle, DirectLine, PropModel1}
    private commtype commModel;
    private int simRate;
    private Environment env;
    public static enum exptype {RunFromLog, LeaderFollower, FrontierExploration, RoleBasedExploration, StumpExploration, BirkExploration, IlpExploration, ApproxExploration, MixedExploration, DemurIlpExploration, SwitchExploration}
    public static enum frontiertype {RangeConstrained, PeriodicReturn, ReturnWhenComplete, UtilReturn}
    public double TARGET_INFO_RATIO;
    public int repl_threshold;
    public int approximation;
    private exptype expAlgorithm;
    private frontiertype frontierAlgorithm;
    private boolean useImprovedRendezvous;
    private boolean allowReplanning;
    private boolean allowRoleSwitch;
    private boolean useStrictRoleSwitch;
    private boolean RVThroughWallsEnabled;
    private boolean RVCommRangeEnabled;
    
    public static int randomSeed;
    public static int fov;
    public static int secondsPerCycle;
    public static boolean instantReplan;
    public static int runDuration;

    public SimulatorConfig() {
        boolean oldEnvVariableConfigFound = loadOldSimulatorConfig();
        if(!oldEnvVariableConfigFound) {
            //set defaults
            expAlgorithm = exptype.RoleBasedExploration;
            frontierAlgorithm = frontiertype.PeriodicReturn;
            runFromLogFilename = null;
            commModel = commtype.DirectLine;
            logAgents = false;
            logAgentsFilename = System.getProperty("user.dir") + "/logs/defaultAgentLog.txt";
            logData = false;
            logDataFilename = System.getProperty("user.dir") + "/logs/defaultDataLog.txt";
            simRate = 5;
            useImprovedRendezvous = false;
            allowReplanning = false;
            allowRoleSwitch = false;
            useStrictRoleSwitch = false;
            RVThroughWallsEnabled = false;
            RVCommRangeEnabled = false;
            repl_threshold = 0;
            randomSeed = 1;
            fov = 180;
            secondsPerCycle = 1;
            instantReplan = true;
            runDuration = 100;
        }
        
        boolean oldWallConfigFound = loadOldWallConfig();
        if(!oldWallConfigFound)
            env = new Environment(Constants.MAX_ROWS, Constants.MAX_COLS);
    }
// </editor-fold>
    
// <editor-fold defaultstate="collapsed" desc="Get and Set">

    public commtype getCommModel() {
        return commModel;
    }
    
    public void setCommModel(int n) {
        commModel = commtype.values()[n];
    }
    
    public exptype getExpAlgorithm() {
        return expAlgorithm;
    }

    public frontiertype getFrontierAlgorithm() {
        return frontierAlgorithm;
    }

    public String getRunFromLogFilename() {
        return runFromLogFilename;
    }

    public void setExpAlgorithm(exptype eType) {
        expAlgorithm = eType;
    }
    
    public void setRunFromLogFilename(String f) {
        runFromLogFilename = f;
    }

    public void setExpAlgorithm(int n) {
        expAlgorithm = exptype.values()[n];
    }

    public void setFrontierAlgorithm(int n) {
        frontierAlgorithm = frontiertype.values()[n];
    }

    public void setFrontierAlgorithm(frontiertype fType) {
        frontierAlgorithm = fType;
    }

    public int getSimRate() {
        return simRate;
    }
    
    public void setSimRate(int s) {
        simRate = s;
    }
    
    public Environment getEnv() {
        return env;
    }
    
    public boolean logAgents() {
        return logAgents;
    }
    
    public void setLogAgents(boolean log) {
        logAgents = log;
    }
    
    public String getLogAgentFilename() {
        return logAgentsFilename;
    }
    
    public void setLogAgentsFilename(String f) {
        logAgentsFilename = f;
    }

    public boolean logData() {
        return logData;
    }

    public void setLogData(boolean log) {
        logData = log;
    }

    public String getLogDataFilename() {
        return logDataFilename;
    }

    public void setLogDataFilename(String f) {
        logDataFilename = f;
    }

    public boolean logScreenshots() {
        return logScreenshots;
    }

    public void setLogScreenshots(boolean log) {
        logScreenshots = log;
    }

    public String getLogScreenshotsDirname() {
        return logScreenshotsDirname;
    }

    public void setLogScreenshotsDirname(String f) {
        logScreenshotsDirname = f;
    }

    public boolean roleSwitchAllowed() {
        return allowRoleSwitch;
    }

    public void setRoleSwitchAllowed(boolean rs) {
        allowRoleSwitch = rs;
    }

    public boolean strictRoleSwitch() {
        return useStrictRoleSwitch;
    }

    public void setStrictRoleSwitch(boolean rs) {
        useStrictRoleSwitch = rs;
    }

    public boolean replanningAllowed() {
        return allowReplanning;
    }

    public void setReplanningAllowed(boolean rs) {
        allowReplanning = rs;
    }
    public boolean useImprovedRendezvous() {
        return useImprovedRendezvous;
    }

    public void setUseImprovedRendezvous(boolean rv) {
        useImprovedRendezvous = rv;
    }
    
    public boolean RVCommRangeEnabled() {
        return RVCommRangeEnabled;
    }
    
    public void setRVCommRangeEnabled(boolean rvCommRangeEnabled)
    {
        this.RVCommRangeEnabled = rvCommRangeEnabled;
    }
    
    public boolean RVThroughWallsEnabled() {
        return RVThroughWallsEnabled;
    }
    
    public void setRVThroughWallsEnabled(boolean rvThroughWallsEnabled) {
        this.RVThroughWallsEnabled = rvThroughWallsEnabled;
    }
// </editor-fold>


// <editor-fold defaultstate="collapsed" desc="Load and Save">



    private boolean loadOldSimulatorConfig() {
        String oldConfigFilename = System.getProperty("user.dir") + "/config/lastSimulatorConfig.txt";
        File file = new File(oldConfigFilename);
        if (file.exists())
            return loadSimulatorConfig(oldConfigFilename);
        else
            return false;
    }
    
    public boolean loadSimulatorConfig(String fileName) {
        File file = new File(fileName); 

        if ( file.exists() )           
        {                                         
            try{
                BufferedReader inFile = new BufferedReader(new FileReader(file));

                expAlgorithm = exptype.valueOf(inFile.readLine());
                frontierAlgorithm = frontiertype.valueOf(inFile.readLine());
                runFromLogFilename = String.valueOf(inFile.readLine());
                commModel = commtype.valueOf(inFile.readLine());
                logAgents = Boolean.parseBoolean(inFile.readLine());
                logAgentsFilename = String.valueOf(inFile.readLine());
                logData = Boolean.parseBoolean(inFile.readLine());
                logDataFilename = String.valueOf(inFile.readLine());
                logScreenshots = Boolean.parseBoolean(inFile.readLine());
                logScreenshotsDirname = String.valueOf(inFile.readLine());
                useImprovedRendezvous = Boolean.parseBoolean(inFile.readLine());
                allowReplanning = Boolean.parseBoolean(inFile.readLine());
                allowRoleSwitch = Boolean.parseBoolean(inFile.readLine());
                useStrictRoleSwitch = Boolean.parseBoolean(inFile.readLine());
                simRate = Integer.parseInt(inFile.readLine());
                RVCommRangeEnabled = Boolean.parseBoolean(inFile.readLine());
                RVThroughWallsEnabled = Boolean.parseBoolean(inFile.readLine());
                try
                {
                    TARGET_INFO_RATIO = Double.parseDouble(inFile.readLine());
                    repl_threshold = Integer.parseInt(inFile.readLine());
                    randomSeed = Integer.parseInt(inFile.readLine());
                    approximation = Integer.parseInt(inFile.readLine());
                    fov = Integer.parseInt(inFile.readLine());
                    secondsPerCycle = Integer.parseInt(inFile.readLine());
                    instantReplan = Boolean.parseBoolean(inFile.readLine());
                    runDuration = Integer.parseInt(inFile.readLine());
                } catch (Exception e)
                {
                    TARGET_INFO_RATIO = 0.9;
                    repl_threshold = 1;
                    randomSeed = 1;
                    approximation = 1;
                    fov = 180;
                    secondsPerCycle = 1;
                }

                //System.out.println("The target info ratio is " + Double.toString(TARGET_INFO_RATIO));
                //Remove the possible currentTree file
                File ct = new File("currentTree.txt");
                File nr = new File("notReadyDist.txt");
                //File currWait = new File("waiting.txt");
                ct.delete();
                nr.delete();
                //wait.delete();
                
                inFile.close();
                return true;
            }
            catch (IOException e) {
                System.out.println(this.toString() + "Error: could not read data from " + fileName);
            }
            catch (NumberFormatException e) {
                System.out.println(this.toString() + "Error: incorrect data format in file " + fileName);
            }
        }
        return false;
    }

    
    private boolean loadOldWallConfig() {
        String oldConfigFilename = System.getProperty("user.dir") + "/config/lastEnvironment.png";
        File file = new File(oldConfigFilename);
        if (file.exists())
            return loadWallConfig(oldConfigFilename);
        else
            return false;
    }
    
    public boolean loadWallConfig(String fileName) {
        env = EnvLoader.loadWallConfig(fileName);

        if(env != null)
            return true;
        else
            return false;
    }
    
    public boolean saveSimulatorConfig() {
        return (this.saveSimulatorConfig(System.getProperty("user.dir") + "/config/lastSimulatorConfig.txt"));
    }

    public boolean saveSimulatorConfig(String fileName) {
        try{
            PrintWriter outFile = new PrintWriter(new FileWriter(fileName));
            
            outFile.println(expAlgorithm.toString());
            outFile.println(frontierAlgorithm.toString());
            outFile.println(runFromLogFilename);
            outFile.println(commModel.toString());
            outFile.println(logAgents);
            outFile.println(logAgentsFilename);
            outFile.println(logData);
            outFile.println(logDataFilename);
            outFile.println(logScreenshots);
            outFile.println(logScreenshotsDirname);
            outFile.println(useImprovedRendezvous);
            outFile.println(allowReplanning);
            outFile.println(allowRoleSwitch);
            outFile.println(useStrictRoleSwitch);
            outFile.println(simRate);
            outFile.println(RVCommRangeEnabled);
            outFile.println(RVThroughWallsEnabled);
            outFile.println(TARGET_INFO_RATIO);
            outFile.println(repl_threshold);
            outFile.println(randomSeed);
            outFile.println(approximation);
            outFile.println(fov);
            outFile.println(secondsPerCycle);
            outFile.println(instantReplan);
            outFile.println(runDuration);
            
            outFile.close();
            return true;
        }
        catch(IOException e){
            System.out.println(this.toString() + "Error writing to file " + fileName);
        }

        return false;
    }

    public boolean saveWallConfig() {
        return EnvLoader.saveWallConfig(env);
    }
    
// </editor-fold>
    
// <editor-fold defaultstate="collapsed" desc="Utility">
    @Override
    public String toString() {
        return ("[SimulatorConfig] ");
    }
// </editor-fold>
    
}
