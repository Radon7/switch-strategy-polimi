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
import java.util.Map;
import java.util.Map;
import java.util.Scanner;

import environment.Frontier;
import environment.Location;
import agents.RealAgent;
import config.SimulatorConfig;
import config.SimulatorConfig.exptype;

/**
 * Eseguito soltanto dalla BS. Vengono presi tutti i parametri da passare
 * all'eseguibile "main.exe" (che rappresente l'algoritmo di Stump) e
 * trasformati in stringhe. Le stringhe vengono inserite in un array di stringhe
 * "env" necessario per eseguire "main.exe". L'eseguibile genera tre file:
 * costMin.txt, D_opt.txt e M_opt.txt. I file si trovano nella cartella
 * \\MRESim-master. Se il file costMin.txt non viene trovato, allora c'� stato
 * un errore e l'esecuzione termina. Se il file costMin.txt contiene la stringa
 * "Inf", allora non � stata trovata alcuna struttura per la disposizione dei
 * robot nell'ambiente. Altrimenti � stata trovata una struttura e viene
 * ritornato l'array delle posizioni ottimali dei robot. Se la struttura non
 * viene trovata per un certo numero di volte, allora l'esplorazione termina.
 *
 * @author Mattia
 *
 */
public class DemurIlp {

    public DemurIlp() {
    }

    
    public HashMap<Integer, Point> callDemurIlp(RealAgent agent, SimulatorConfig simConfig) throws FileNotFoundException {

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
        filePath = userPath + "scripts/ilp_model_demur.py";
        System.out.println("USERPATH: " + userPath);
        System.out.println("FILEPATH: " + filePath);
        /* System.out.println("adj = '" + adj + "';");
         System.out.println("G = '" + G + "';");
         System.out.println("D = '" + D + "';");
         System.out.println("R = '" + R + "';");
         System.out.println("mu = '" + mu + "';");
         System.out.println("C = '" + C + "';");
         System.out.println("V = '" + V + "';");
         System.out.println("m_1 = '" + m_1 + "';");
         System.out.println("w_p = '" + w_p + "';");*/
        /*for(Frontier f : agent.getFrontiers()) {
         System.out.println("FRONTIERA " + f.getCentre());
         }
		
         for(Location l : agent.getList()) {
         System.out.println("LOCATION " + l.getPosition());
         }
		
         System.out.println("CARDINALIT� G " + agent.getG().length);*/
        int k = 1;

        /*String C_b_copy_s = String.valueOf(C_b_copy);
         String V_b_copy_s = String.valueOf(V_b_copy);
			
         System.out.println("C_beta = '" + C_b_copy_s + "'");
         System.out.println("V_beta = '" + V_b_copy_s + "'");*/

        /*env[0] = filePath;
         env[1] = D;
         env[2] = w_p;
         env[3] = R;
         env[4] = C;
         env[5] = C_b_copy_s;
         env[6] = V;
         env[7] = V_b_copy_s;
         env[8] = adj;
         env[9] = G;
         env[10] = m_1;
         env[11] = mu;*/
        try {
            
            //ProcessBuilder pb = new ProcessBuilder("python", filePath);
            //Map<String, String> env = pb.environment();
            //env.put("GUROBI_HOME", "/opt/gurobi602/linux64");
            //env.put("PATH", "${PATH}:${GUROBI_HOME}/bin");
            //env.put("LD_LIBRARY_PATH", "/opt/gurobi602/linux64/lib");
            //Process proc = pb.start();
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

                HashMap<Integer,Integer> allocation = updateM_opt();
                HashMap<Integer, Point> allocation_phis = new HashMap<Integer, Point>();

                for (Map.Entry<Integer, Integer> entry : allocation.entrySet()) {
                    allocation_phis.put(entry.getKey(), agent.getLocationIDs().get(entry.getValue() -1 ).getPosition());
                }

                agent.setM_opt_p(allocation_phis);
        
                return new HashMap<Integer, Point>(allocation_phis);
            }
        }

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
        
}
