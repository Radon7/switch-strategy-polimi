package decisore;

import java.awt.Point;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Scanner;
import java.util.HashMap;

import environment.Frontier;
import environment.Location;
import agents.RealAgent;
import java.util.Map;

/**
 * Eseguito soltanto dalla BS. Vengono presi tutti i parametri da passare
 * all'eseguibile "main.exe" (che rappresente l'algoritmo di Stump) e
 * trasformati in stringhe. Le stringhe vengono inserite in un array di stringhe
 * "env" necessario per eseguire "main.exe". L'eseguibile genera tre file:
 * costMin.txt, D_opt.txt e M_opt.txt. I file si trovano nella cartella
 * \\MRESim-master. Se il file costMin.txt non viene trovato, allora c'è stato
 * un errore e l'esecuzione termina. Se il file costMin.txt contiene la stringa
 * "Inf", allora non è stata trovata alcuna struttura per la disposizione dei
 * robot nell'ambiente. Altrimenti è stata trovata una struttura e viene
 * ritornato l'array delle posizioni ottimali dei robot. Se la struttura non
 * viene trovata per un certo numero di volte, allora l'esplorazione termina.
 *
 * @author Mattia
 *
 */
public class Stump {

    public Stump() {
    }

    public HashMap<Integer, Point> callStump(RealAgent agent) throws FileNotFoundException {

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
        String userPath = System.getProperty("user.dir") + "/";

        //String filePath = userPath + "main.exe";

        /*System.out.println("FILEPATH: " + filePath);
         System.out.println("adj = '" + adj + "';");
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
		
         System.out.println("CARDINALITà G " + agent.getG().length);*/
        int k = 1;

        while (C_b_copy.size() >= 1 && k < agent.getC_beta().size()) {

            if (C_b_copy.size() == 1) {
                k++;
                if (k < agent.getC_beta().size()) {
                    C_b_copy.add(agent.getC_beta().get(k));
                    V_b_copy.add(agent.getV_beta().get(k));
                    System.out.println(agent.toString() + "Try to reach only one frontier!");
                } else {
                    break;
                }
            }

            BufferedWriter writer = null;
            try {
                writer = new BufferedWriter(new FileWriter("C_beta.txt"));
                for (int i = 0; i < C_b_copy.size(); i++) {
                    //System.out.print(C_b_copy.get(i));
                    writer.write(C_b_copy.get(i) + "\t");
                    writer.flush();
                }
                writer.close();
            } catch (IOException e) {
                e.printStackTrace();
            }

            BufferedWriter writer2 = null;
            try {
                writer2 = new BufferedWriter(new FileWriter("V_beta.txt"));
                for (int i = 0; i < V_b_copy.size(); i++) {
                    //System.out.print(V_b_copy.get(i));
                    writer2.write(V_b_copy.get(i) + "\t");
                    writer2.flush();
                }
                writer2.close();
            } catch (IOException e) {
                e.printStackTrace();
            }

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
            File fileCost = new File(userPath + "costMin.txt");

            try {

                //String launch = "matlab -nojvm -nosplash -nodesktop -r \"run " + userPath + "matlab_scripts/main.m;exit;\"";
                String launch = "./main";
                System.out.println(userPath);
                Process proc = Runtime.getRuntime().exec(launch);
                //Process proc = Runtime.getRuntime().exec(new String[]{"matlab", launch});
                proc.waitFor();
            } catch (Exception e) {
                e.printStackTrace();
            }

            if (!fileCost.exists()) {
                System.out.println(agent.toString() + "ERROR: FILE NOT FOUND!");
            } else {

                Scanner scanner3 = new Scanner(fileCost);
                String result = new String();
                result = scanner3.nextLine();
                scanner3.close();

                if (result.equals("Inf")) {

                    System.out.println(agent.toString() + "Can't connect desired locations! Trying again!");

                    C_b_copy.remove(C_b_copy.size() - 1);
                    V_b_copy.remove(V_b_copy.size() - 1);

                } else {
                    System.out.println(agent.toString() + "Minimum cost structure found!");

                    HashMap<Integer, Integer> allocation = updateM_opt();
                    HashMap<Integer, Point> allocation_phis = new HashMap<Integer, Point>();

                    for (Map.Entry<Integer, Integer> entry : allocation.entrySet()) {
                        allocation_phis.put(entry.getKey(), agent.getLocationIDs().get(entry.getValue() - 1).getPosition());
                    }

                    agent.setM_opt_p(allocation_phis);

                    agent.setD_opt(updateD());

                    return new HashMap<Integer, Point>(allocation_phis);
                }
            }
        }

        if (C_b_copy.size() == 1) {
            System.out.println(agent.toString() + "END OF EXPLORATION!!!!");
            agent.setEndExploration(true);
            return null;
        }
        return null;
    }

    public HashMap<Integer, Integer> updateM_opt() throws FileNotFoundException {
        String userPath = System.getProperty("user.dir") + "/";
        Scanner m_opt = new Scanner(new File(userPath + "m_opt.txt"));
        HashMap<Integer, Integer> m_opt_copy = new HashMap<Integer, Integer>();
        String line = m_opt.nextLine();
        Scanner scanner = new Scanner(line);
        scanner.useDelimiter("\t");
        int i = 0;
        while (scanner.hasNextInt()) {
            m_opt_copy.put(i, scanner.nextInt());
            i += 1;
        }
        scanner.close();
        m_opt.close();

        return new HashMap<Integer, Integer>(m_opt_copy);

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
