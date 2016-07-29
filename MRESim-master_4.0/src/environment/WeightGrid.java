package environment;

import java.awt.Point;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import agents.RealAgent;
import agents.TeammateAgent;
import path.Path;
import org.jgrapht.alg.*;
import org.jgrapht.*;
import org.jgrapht.graph.*;

/**
 * Build w_p
 *
 * @author Mattia
 *
 */
public class WeightGrid {

    private int[][] weightMatrix;
    private int size = 0;

    //private Dijkstra dijkstra;
    public WeightGrid(RealAgent agent) {
        this.size = 0;
        //this.dijkstra = new Dijkstra();
    }

    /*public void update(RealAgent agent){
     HashMap<Integer, Location> locations = (HashMap) agent.getLocationIDs();
     if (locations.isEmpty()) {
     return;
     }
        
     weightMatrix = dijkstra.computeMatrix(agent);


     }*/
    public void update(RealAgent agent, int[][] limVisMatrix) {
        System.out.println("Updating distance matrix.");

        HashMap<Integer, Location> locations = (HashMap) agent.getLocationIDs();
        if (locations.isEmpty()) {
            return;
        }

        int numLoc = agent.getLocNumber();

        int[][] oldWeightMatrix = weightMatrix;
        weightMatrix = new int[numLoc][numLoc];

        SimpleWeightedGraph<String, DefaultWeightedEdge> graph
                = new SimpleWeightedGraph<String, DefaultWeightedEdge>(DefaultWeightedEdge.class);
        for (int i = 0; i < numLoc; i++) {
            graph.addVertex(Integer.toString(i));
        }

        for (int i = 0; i < numLoc; i++) {
            for (int j = i; j < numLoc; j++) {
                if (limVisMatrix[i][j] == 1 && i != j) {
                    DefaultWeightedEdge e1 = graph.addEdge(Integer.toString(i), Integer.toString(j));
                    graph.setEdgeWeight(e1, agent.getLocationIDs().get(i).getPosition().distance(agent.getLocationIDs().get(j).getPosition()));
                }
            }
        }

        FloydWarshallShortestPaths fw = new FloydWarshallShortestPaths(graph);
        for (int i = 0; i < numLoc; i++) {
            for (int j = i; j < numLoc; j++) {
                weightMatrix[i][j] = (int) fw.shortestDistance(Integer.toString(i), Integer.toString(j));
                if(weightMatrix[i][j] > 1200)
                    weightMatrix[i][j] = 1200; //If no feasible path is found
                
                weightMatrix[j][i] = weightMatrix[i][j];
            }
        }

        size = numLoc;
        System.out.println("Done");

    }

    public int[][] getWeightMatrix() {
        return this.weightMatrix;
    }

    public HashMap<Integer, ArrayList<Integer>> getOtherDistances(RealAgent agent, int[][] limVisMatrix) {

        System.out.println("Getting other distances");
        HashMap<Integer, ArrayList<Integer>> notReady = new HashMap<Integer, ArrayList<Integer>>();

        SimpleWeightedGraph<String, DefaultWeightedEdge> graph
                = new SimpleWeightedGraph<String, DefaultWeightedEdge>(DefaultWeightedEdge.class);

        for (int i = 0; i < this.size; i++) {
            graph.addVertex(Integer.toString(i));
        }

        for (int i = 0; i < this.size; i++) {
            for (int j = i; j < this.size; j++) {
                if (limVisMatrix[i][j] == 1 && i != j) {
                    DefaultWeightedEdge e1 = graph.addEdge(Integer.toString(i), Integer.toString(j));
                    graph.setEdgeWeight(e1, agent.getLocationIDs().get(i).getPosition().distance(agent.getLocationIDs().get(j).getPosition()));
                }
            }
        }

        for (TeammateAgent t : agent.getAllTeammates().values()) {
            if (!t.isReady()) { 
                Point start;
                if (t.isInRange()) {
                    start = t.getLocation();
                } else {
                    start = t.getPositionEstimate();
                }
                graph.addVertex("Teammate" + t.getID());
                //add proper edges
                for(int i = 0; i < this.size; i++){
                    if(agent.getOccupancyGrid().directLinePossible3(start.x, start.y, agent.getLocationIDs().get(i).getPosition().x, agent.getLocationIDs().get(i).getPosition().y)){
                        DefaultWeightedEdge e1 = graph.addEdge("Teammate" + t.getID(), Integer.toString(i));
                        graph.setEdgeWeight(e1, start.distance(agent.getLocationIDs().get(i).getPosition())); 
                    }
                }

                ArrayList<Integer> distances = new ArrayList<Integer>();
                for (int i = 0; i < this.size; i++) {
                    Location loc2 = agent.getLocationIDs().get(i);

                    DijkstraShortestPath sp = new DijkstraShortestPath(graph, "Teammate" + t.getID(), Integer.toString(i));
                    distances.add((int)sp.getPathLength());

                }
                notReady.put(t.getID() - 1, distances);
            }
        }

        return notReady;

    }

}
