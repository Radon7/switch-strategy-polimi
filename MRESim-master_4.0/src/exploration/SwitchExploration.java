/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package exploration;

import agents.RealAgent;
import config.RobotConfig;
import config.SimulatorConfig;
import java.awt.Point;
import java.io.FileNotFoundException;
import java.time.Instant;
import java.util.Date;
import java.util.HashMap;

/**
 *
 * @author alex
 */
public class SwitchExploration {

    private static boolean multiHopping = true;
    private static long randomTime = System.currentTimeMillis();

    public static boolean isMultiHopping() {
        return multiHopping;
    }

    public void setMultiHopping(boolean multiHopping) {
        this.multiHopping = multiHopping;
    }

    // <editor-fold defaultstate="collapsed" desc="Take Step">
    public static Point takestep(RealAgent agent, int timeElapsed,
            SimulatorConfig.frontiertype frontierExpType, SimulatorConfig simConfig)
            throws FileNotFoundException {

        if (!isMultiHopping()) {
            System.err.println("doing a Frontier WITH MY ROBOT" + agent.getName() + "with frontier" + simConfig.getFrontierAlgorithm());
            return UtilityExploration.takeStep(agent, timeElapsed, simConfig);
        } else {
            System.out.println("trying a ASYNCEXPLORATION walkAsyncExplorationnt WITH ROBOT"
                    + agent.getName());
            return AsyncExploration.takeStep(agent, timeElapsed, frontierExpType.ReturnWhenComplete, simConfig);
        }
    }

    /*
        Replan only when AsyncExploration is needed
     */
    public static HashMap<Integer, Point> replan(RealAgent agent,
            SimulatorConfig.frontiertype frontierExpType, int timeElapsed,
            SimulatorConfig simConfig) throws FileNotFoundException {
        if (agent.getPercentageKnown() > 0.3 && agent.getRole().equals(RobotConfig.roletype.BaseStation)) {
            multiHopping = false;
        }

        return AsyncExploration.replan(agent, frontierExpType, timeElapsed, simConfig);
    }

    // </editor-fold>
}
