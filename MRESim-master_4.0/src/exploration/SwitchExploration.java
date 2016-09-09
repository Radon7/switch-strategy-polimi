/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package exploration;

import agents.BasicAgent;
import agents.RealAgent;
import agents.TeammateAgent;
import config.Constants;
import config.RobotConfig;
import config.SimulatorConfig;
import java.awt.Point;
import java.io.FileNotFoundException;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;

/**
 *
 * @author alex
 */
public class SwitchExploration {


    public static enum SwitchState {MultiHopping, RendezVous, Wait};
    private static SwitchState state = SwitchState.MultiHopping;
    
    
    public static Point takestep(RealAgent agent, int timeElapsed,
            SimulatorConfig.frontiertype frontierExpType, SimulatorConfig simConfig)
            throws FileNotFoundException {
        // <editor-fold defaultstate="collapsed" desc="if initial i'll do multilhop">
        // </editor-fold>
        
        // <editor-fold defaultstate="collapsed" desc="if i need switch">
      
            
        // </editor-fold>
        switch(state){
            case MultiHopping:
                System.err.println("Doing MULTIHOPPING with ROBOT" + agent.getName());
                return AsyncExploration.takeStep(agent, timeElapsed, frontierExpType.ReturnWhenComplete, simConfig);
                
            case RendezVous:
                //trying to switch back to multiholoping by moving all the robots to BS
                
                //moving to Utility Exploration
                System.err.println("Doing UTILITYexp with ROBOT" + agent.getName());
                return UtilityExploration.takeStep(agent, timeElapsed, simConfig);
            
            case Wait:
                if (agent.getRobotNumber() == (Constants.BASE_STATION_ID)){
                    ArrayList<TeammateAgent> commTeam = new ArrayList<TeammateAgent>();
                    for (TeammateAgent t : agent.getAllTeammates().values()){
                        if (t.isInRange()){
                            commTeam.add(t);
                        }
                    }
                    if (commTeam.size() == agent.getAllTeammates().values().size()){
                        state = SwitchState.MultiHopping;
                    }
                }
                return agent.getLocation();
            default:
                return null;
        }
        
    }

    /*
        Replan only when AsyncExploration is needed
     */
    public static HashMap<Integer, Point> replan(RealAgent agent,
            SimulatorConfig.frontiertype frontierExpType, int timeElapsed,
            SimulatorConfig simConfig) throws FileNotFoundException {
        
        if (agent.getPercentageKnown() > 0.3 && agent.getRole().equals(RobotConfig.roletype.BaseStation)){
            state = SwitchState.RendezVous;
            System.err.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!FUUUUUUU");
        }
        return AsyncExploration.replan(agent, frontierExpType.ReturnWhenComplete, timeElapsed, simConfig);
    }
    
    public static SwitchState getState(){
        return state;
    }

    // </editor-fold>
}
