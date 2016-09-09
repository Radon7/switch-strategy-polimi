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
    
    //<editor-fold defaultstate="collapsed" desc="TAKE STEP">
    public static Point takestep(RealAgent agent, int timeElapsed,
            SimulatorConfig.frontiertype frontierExpType, SimulatorConfig simConfig)
            throws FileNotFoundException {
        
        switch(state){
            case MultiHopping:
                System.err.println("Doing MULTIHOPPING with ROBOT" + agent.getName());
                return AsyncExploration.takeStep(agent, timeElapsed, frontierExpType.ReturnWhenComplete, simConfig);
           
            case RendezVous:
                System.err.println("Doing UTILITYexp with ROBOT" + agent.getName());
                return UtilityExploration.takeStep(agent, timeElapsed, simConfig);

            case Wait:
                System.err.println("I'M in WAIT, robot" + agent.getName());

                ArrayList<TeammateAgent> commTeam = new ArrayList<TeammateAgent>();
                for (TeammateAgent t : agent.getAllTeammates().values()){
                    if (t.isInRange()){
                        commTeam.add(t);
                    }
                }
                System.err.println("commTeamSize = "+commTeam.size());
                System.err.println("Team size = "+agent.getAllTeammates().values().size());
                if (commTeam.size() == agent.getAllTeammates().values().size()){
                    state = SwitchState.MultiHopping;
                }
                
                return UtilityExploration.takeStep(agent, timeElapsed, simConfig);
           
            // This should never happen;
            default:
                return agent.getLocation();
        }
        
    }
    // </editor-fold>
    
    // <editor-fold defaultstate="collapsed" desc="Replanning funtion">
    /*
        Replan only when AsyncExploration is needed
     */
    public static HashMap<Integer, Point> replan(RealAgent agent,
            SimulatorConfig.frontiertype frontierExpType, int timeElapsed,
            SimulatorConfig simConfig) throws FileNotFoundException {
        //This happens when I need to switch the exploration Type
        if (agent.getPercentageKnown() > 0.3 && agent.getRole().equals(RobotConfig.roletype.BaseStation) && timeElapsed < 150){
            state = SwitchState.RendezVous;
        }
        return AsyncExploration.replan(agent, frontierExpType.ReturnWhenComplete, timeElapsed, simConfig);
    }
    // </editor-fold>
    
    public static SwitchState getState(){
        return state;
    }
    
    public static void setState(SwitchState s){
        state = s;
    }


}
