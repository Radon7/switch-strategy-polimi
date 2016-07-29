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
package environment;

import java.awt.Point;


/**
 *
 * @author julh
 */

public class Environment{

	private int[][] adjMatrix;
	
// <editor-fold defaultstate="collapsed" desc="Class variables and constructor">

    public int rows;           // The environment's size
    public int columns;

    public enum Status {explored, unexplored, obstacle}
    private Status status[][];


    // Simple constructor for setup stage -- rows and columns only
    public Environment(int numrows, int numcolumns) {
        rows = numrows;
        columns = numcolumns;
        initCells();
    }
    
// </editor-fold>     

// <editor-fold defaultstate="collapsed" desc="Get and Set">

    
    public int getRows() {
        return this.rows;
    }
    
    public int getColumns() {
        return this.columns;
    }
    
    public void setStatus(int i, int j, Status newStat) {
        status[i][j] = newStat;
    }
    
    public Status statusAt(int i, int j) {
        return status[i][j];
    }

    public boolean obstacleAt(int i, int j) {
        return (statusAt(i,j) == Status.obstacle);
    }

    public Status[][] getFullStatus() {
        return status;
    }

// </editor-fold>  
    
// <editor-fold defaultstate="collapsed" desc="Initialization">
    private void initCells() {
        status = new Status[columns][rows];
        for(int i=0; i<rows; i++)
            for(int j=0; j<columns; j++)
                status[j][i] = Status.unexplored;
    }
// </editor-fold>     

// <editor-fold defaultstate="collapsed" desc="Information of Interest">
    public boolean locationExists(int x, int y) {
        return(x < columns  && x >= 0 &&  y < rows && y >= 0);
    }
    
    public boolean obstacleWithinDistance(int x, int y, int minDistance) {
        for(int i=x-minDistance; i<=x+minDistance; i++)
            for(int j=y-minDistance; j<=y+minDistance; j++)
                if(locationExists(i,j) &&
                   new Point(x,y).distance(i,j) <= minDistance &&
                   obstacleAt(i,j))
                    return true;
        return false;
    }


    public int getTotalFreeSpace() {
        int runningTotal = 0;
        for(int i=0; i<rows; i++)
            for(int j=0; j<columns; j++)
                if(status[j][i]!=Status.obstacle)
                    runningTotal++;

        return runningTotal;
    }
    
    
    public boolean directLinePossible(int sourceX, int sourceY, int destX, int destY) {
        if(!locationExists(sourceX, sourceY) || !locationExists(destX, destY))
            return false;
        if (status[destX][destY] == Environment.Status.obstacle)
            return false;
        for(int i=Math.min(sourceX, destX); i<=Math.max(sourceX, destX); i++)
            for(int j=Math.min(sourceY, destY); j<=Math.max(sourceY, destY); j++)
                if((distPointToLine(sourceX,sourceY,destX,destY,i,j) < 0.5) &&
                   (status[i][j] == Environment.Status.obstacle))
                    return false;
        
        return true;
        
        /*if (sourceX != destX){
            double slope = (double)(destY - sourceY) / (destX - sourceX);
            int minX = Math.min(sourceX, destX), maxX = Math.max(sourceX, destX);
            double y_c;
            int y;
            //System.out.println("CHECKING " + sourceX + "," + sourceY + ";" + destX + "," + destY);
            for (int x=minX+1; x < maxX; x++){
                y_c = slope * (x - sourceX) + sourceY;
                y = (int)Math.round(y_c);
                //System.out.println(x + "," + y);
                if(obstacleAt(x,y))
                        return false;
            }
        }
        else{
            int minY = Math.min(sourceY, destY), maxY = Math.max(sourceY, destY);
            for (int y=minY+1; y < maxY; y++){
                if(obstacleAt(sourceX,y))
                        return false;
            }
        }
        
        return true;*/
        
    }
    
    // This function finds the shortest distance from P3 to the line between P1 and P2
    public double distPointToLine(int x1, int y1, int x2, int y2, int x3, int y3) {
        if((x3 == x1 && y3 == y1)  || (x3 == x2 && y3 == y2)) return 0;
        
        double dist = Math.sqrt(Math.pow(y2 - y1,2) + Math.pow(x2 - x1, 2));
        double slope = ((x3 - x1) * (x2 - x1) + (y3 - y1) * (y2 - y1)) / Math.pow(dist,2);
        
        double xIntersection = x1 + slope * (x2 - x1);
        double yIntersection = y1 + slope * (y2 - y1);
        
        double shortestDist = Math.sqrt(Math.pow(x3 - xIntersection, 2) + Math.pow(y3 - yIntersection, 2));
        
        return shortestDist;
    }
    
    public int numObstaclesOnLine(int x1, int y1, int x2, int y2) {
        int counter = 0;
        double angle = Math.atan2(y2 - y1, x2 - x1);
        int distance = (int)(Math.sqrt(Math.pow(y2-y1, 2) + Math.pow(x2-x1, 2)));
        int currX, currY;
        
        for(int i=0; i<=distance; i++) {
            currX = x1 + (int)(Math.cos(angle) * i);
            currY = y1 + (int)(Math.sin(angle) * i);
            
            if(this.statusAt(currX, currY) == Status.obstacle){
                counter++;
                i++;  // To prevent the same pixel counting as an obstacle twice.
            }
        }
        
        return counter;
    }

// </editor-fold>
    
    
}
