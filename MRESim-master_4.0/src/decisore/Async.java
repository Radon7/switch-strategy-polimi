package decisore;

import java.awt.Point;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Map;
import java.util.Scanner;

import agents.TeammateAgent;
import environment.Frontier;
import environment.Location;
import agents.RealAgent;
import config.SimulatorConfig;
import config.SimulatorConfig.exptype;

/**
 *
 * @author Jacopo
 *
 */
public class Async {

    public Async() {
    }

    public void reallocate(RealAgent agent){
        String userPath = System.getProperty("user.dir") + "/"; //.replace("\\", "\\\\") + "\\\\";
        String filePath = userPath + "scripts/reallocate.py";
        System.out.println("USERPATH: " + userPath);
        System.out.println("FILEPATH: " + filePath);
        try {
            
            Process proc = Runtime.getRuntime().exec(new String[]{"python", filePath});
            //Process proc = Runtime.getRuntime().exec(filePath);
            proc.waitFor();
            //System.out.println("After wait");
            
            HashMap<Integer,Integer> allocation = updateM_opt();
            HashMap<Integer, Point> allocation_phis = new HashMap<Integer, Point>();

            for (Map.Entry<Integer, Integer> entry : allocation.entrySet()) {
                allocation_phis.put(entry.getKey(), agent.getLocationIDs().get(entry.getValue() -1 ).getPosition());
            }
                

            agent.setM_opt_p(allocation_phis);

            agent.setWaitingLists(getAllNewWaitings(agent));
            
        } catch (Exception e) {
            e.printStackTrace();
        }
        
        return;
        
    }
    
    public HashMap<Integer, Point> callAsync(RealAgent agent, SimulatorConfig simConfig) throws FileNotFoundException {

		//java.awt.Toolkit.getDefaultToolkit().beep(); //Avvisa quando parte Stump
        ArrayList<Integer> C_b_copy = new ArrayList<Integer>(agent.getC_beta());

        ArrayList<Integer> V_b_copy = new ArrayList<Integer>(agent.getV_beta());

        /*String env[] = new String[12];
		
         String adj = String.valueOf(Arrays.deepToString(agent.getAdj()).replaceAll("],", "];"));
		
         String G = String.valueOf(Arrays.deepToString(agent.getG()).replaceAll("],", "];"));
		
         String D = String.valueOf(Arrays.deepToString(agent.getD()).replaceAll("],", "];"));
		
         String R = String.valueOf(agent.getR());
				
         String mu = String.valueOf(agent.getMu());
		
         String C = String.valueOf(agent.getC());
		
         String V = String.valueOf(agent.getV());
		
         String m_1 = String.valueOf(agent.getM_1());
		
         String w_p = String.valueOf(Arrays.deepToString(agent.getW_p()).replaceAll("],", "];"));*/
        String userPath = System.getProperty("user.dir") + "/"; //.replace("\\", "\\\\") + "\\\\";

        /*TODO if (System.getProperty("os.name").startsWith("Windows")) {
         // includes: Windows 2000,  Windows 95, Windows 98, Windows NT, Windows Vista, Windows XP
         userPath = System.getProperty("user.dir").replace("\\", "\\\\") + "\\\\";
         } else {
         userPath = System.getProperty("user.dir");
         // everything else
         } */
        String filePath;
        if(simConfig.getExpAlgorithm() == exptype.IlpExploration){
            filePath = userPath + "scripts/ilp_model.py";
        }
        else if(simConfig.getExpAlgorithm() == exptype.ApproxExploration
                || simConfig.getExpAlgorithm() == exptype.SwitchExploration){
            filePath = userPath + "scripts/approximate_deployement.py";
        }
        else{ //Both with time threshold
            filePath = userPath + "scripts/mixed_deployement.py";
        }
        System.out.println("USERPATH: " + userPath);
        System.out.println("FILEPATH: " + filePath);
        int k = 1;

        try {
            Process proc = Runtime.getRuntime().exec(new String[]{"python", filePath});
            //Process proc = Runtime.getRuntime().exec(filePath);
            proc.waitFor();
            //System.out.println("After wait");
        } catch (Exception e) {
            e.printStackTrace();
        }

        File fileCost = new File(userPath + "costMin.txt");

        if (!fileCost.exists()) {
            System.out.println(agent.toString() + "ERROR: FILE NOT FOUND!");
            return null;
        } else {

            Scanner scanner3 = new Scanner(fileCost);
            String result = new String();
            result = scanner3.nextLine();
            scanner3.close();

            if (result.equals("Inf")) {

                System.out.println(agent.toString() + " END EXPLORATION");
                agent.setEndExploration(true);
                return null;

            } else {
                System.out.println(agent.toString() + "Minimum cost structure found!");

                //Retrieve instance dimension
                File fileDim = new File(userPath + "dim.txt");
                scanner3 = new Scanner(fileDim);
                agent.setDimInstance(scanner3.nextLine());
                scanner3.close();
                
                HashMap<Integer,Integer> allocation = updateM_opt();
                
                
                HashMap<Integer, Point> allocation_phis = new HashMap<Integer, Point>();
                
                /*HashMap<Integer, Point> prevAllocationReady = new HashMap<Integer, Point>();
                for(TeammateAgent t: agent.getAllTeammates().values()){
                
                }*/
                
                boolean all_equal = true;

                for (Map.Entry<Integer, Integer> entry : allocation.entrySet()) {
                    System.out.println("Robot " + Integer.toString(entry.getKey()) + " goes to " + Integer.toString(entry.getValue() - 1));
                    //agent.getGoalIds().put(entry.getKey(), entry.getValue() - 1);
                    allocation_phis.put(entry.getKey(), agent.getLocationIDs().get(entry.getValue() -1 ).getPosition());
                    if(agent.getM_opt_p().containsKey(entry.getKey()) && !agent.getM_opt_p().get(entry.getKey()).equals(allocation_phis.get(entry.getKey()))){
                        all_equal = false;
                    }
                    if(entry.getKey() != 0)
                        setTeammateNotReady(agent, entry.getKey());
                }
                
                if(all_equal && agent.getEqualCounter() == -1){
                    agent.setEqualCounter(0);
                }
                
                else if(all_equal && agent.getEqualCounter() != -1){ // means initial plan
                    System.out.println("All equal");
                    agent.setEqualCounter(0);
                    agent.setAllEqual(true);
                }
                

                agent.setM_opt_p(allocation_phis);

                //agent.updateWaitingLists(getNewWaitings(agent));
                
                agent.setWaitingLists(getAllNewWaitings(agent));
                //For reallocation of the whole set of communicating robots
                
                //for logging, depends on the waiting list
                agent.updateRobotAtFrontier();
                
                if(simConfig.getExpAlgorithm() == exptype.MixedExploration){
                    agent.updateWinner(getWinner());
                }
                
                return new HashMap<Integer, Point>(allocation_phis);
            }
        }

    }

    public int getWinner() throws FileNotFoundException{
        String userPath = System.getProperty("user.dir")+ "/"; //TODO .replace("\\", "\\\\") + "\\\\";
        Scanner wScan = new Scanner(new File(userPath + "winner.txt"));
        if(wScan.hasNextInt()){
            return wScan.nextInt();
        }
        return 0; // così bomba nel caso
    }
    
    public HashMap<Integer, Integer> updateM_opt() throws FileNotFoundException {
        String userPath = System.getProperty("user.dir")+ "/"; //TODO .replace("\\", "\\\\") + "\\\\";
        Scanner m_opt = new Scanner(new File(userPath + "m_opt.txt"));
        HashMap<Integer, Integer> allocation = new HashMap<Integer, Integer>();
        //String line = m_opt.nextLine();
        //Scanner scanner = new Scanner(line);
        //scanner.useDelimiter("\t");
        while (m_opt.hasNext()) {
            //m_opt_copy.add(scanner.nextInt());
            String s = m_opt.nextLine();
            String[] arr = s.split("\t");
            int robotID = Integer.parseInt(arr[0]);
            int loc = Integer.parseInt(arr[1]);
            allocation.put(robotID, loc);
            
        }
        m_opt.close();
        
        return new HashMap<Integer, Integer>(allocation);

    }
    
    public void setTeammateNotReady(RealAgent agent, int robot){
        agent.getAllTeammates().get(robot + 1).setReady(false);
    }
    
    //this function is for the reallocation
    public HashMap<Integer, ArrayList<Integer>> getAllNewWaitings(RealAgent agent) throws FileNotFoundException{
        String userPath = System.getProperty("user.dir")+ "/"; //TODO .replace("\\", "\\\\") + "\\\\";
        Scanner input = new Scanner(new File(userPath + "waiting.txt"));
        HashMap<Integer, ArrayList<Integer>> waits = new HashMap<Integer, ArrayList<Integer>>();
        while (input.hasNextLine()) {
            Scanner rowReader = new Scanner(input.nextLine());
            ArrayList<Integer> robots = new ArrayList<Integer>();
            int frontier = rowReader.nextInt();
            System.out.println("Waiting for frontier  " + Integer.toString(frontier));
            while(rowReader.hasNextInt()){
                int robot = rowReader.nextInt();
                robots.add(robot);
                setTeammateNotReady(agent, robot);
            }
            
            waits.put(frontier, robots);
            
        }
        
        return new HashMap<Integer, ArrayList<Integer>>(waits);
    }
    
    public HashMap<Integer, ArrayList<Integer>> getNewWaitings(RealAgent agent) throws FileNotFoundException{
        String userPath = System.getProperty("user.dir")+ "/"; //TODO .replace("\\", "\\\\") + "\\\\";
        Scanner input = new Scanner(new File(userPath + "waiting.txt"));
        HashMap<Integer, ArrayList<Integer>> waits = new HashMap<Integer, ArrayList<Integer>>();
        while (input.hasNextLine()) {
            Scanner rowReader = new Scanner(input.nextLine());
            ArrayList<Integer> frontiers = new ArrayList<Integer>();
            int robot = rowReader.nextInt();
            setTeammateNotReady(agent, robot);
            while(rowReader.hasNextInt()){
                frontiers.add(rowReader.nextInt());
            }
            
            for(Integer fr : frontiers){
                if(!waits.containsKey(fr)){
                    waits.put(fr, new ArrayList<Integer>());
                }
                waits.get(fr).add(robot);
            }
        }
        return new HashMap<Integer, ArrayList<Integer>>(waits);
    }

    public int[][] updateD() throws FileNotFoundException {
        ArrayList<ArrayList<Integer>> D_opt_f = new ArrayList<ArrayList<Integer>>();
        String userPath = System.getProperty("user.dir") + "/";
        Scanner input = new Scanner(new File(userPath + "D_opt.txt"));
        while (input.hasNextLine()) {
            Scanner colReader = new Scanner(input.nextLine());
            ArrayList<Integer> col = new ArrayList<Integer>();
            while (colReader.hasNextInt()) {
                col.add(colReader.nextInt());
            }
            D_opt_f.add(col);
            colReader.close();
        }

        int[][] D_opt = new int[D_opt_f.size()][D_opt_f.size()];

        for (int i = 0; i < D_opt_f.size(); i++) {
            for (int j = 0; j < D_opt_f.size(); j++) {
                D_opt[i][j] = D_opt_f.get(i).get(j);
            }
        }
        input.close();
        return D_opt;
    }

}
