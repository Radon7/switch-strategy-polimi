package environment;

import java.awt.Point;
import java.util.ArrayList;
import java.util.LinkedList;


/**
 * Represent C_i
 * @author Mattia
 *
 */

public class Location {
	 
	 private Point position;
	    
	 public Location(Point position) {
	        this.position = position;
	 }
	 
	 public Point getPosition() {
		 return this.position;
	 }
	 
	 public boolean isIn(ArrayList<Location> list) {
		 for(Location loc : list) {
			 if(loc.getPosition().equals(this.getPosition())) {
				 return true;
			 }
		 }
		 return false;
	 }
	 
	 
}
