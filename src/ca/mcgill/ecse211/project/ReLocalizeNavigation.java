package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;

/**
 * A navigator that actively corrects its heading and position.
 * 
 *
 * @author Matthew
 *
 */
public class ReLocalizeNavigation extends WaggleNavigation {
  
  @Override
  public void travelTo(double x, double y) {
//    colorSensorR.close();
//    colorSensorL.close();
    new PlainNavigation().travelTo(x, y);
    System.out.println("X:"+x);
    System.out.println("Y:"+y);
  }
  
  public void reLocalize(double x, double y) {
    turnTo(Navigation.angleToTarget(x, y)+180);
    
    double[] xyt = Odometer.getOdometer().getXYT();
    double dx, dy;
    int direction, axis;
    dx = Math.abs(xyt[0]-x);
    dy = Math.abs(xyt[1]-y);
    if(dx>dy) {
      direction = 0;
      if(xyt[0]>x) {
        axis = 1; 
      } else {
        axis = -1;
      }
      waggle(axis, direction);
      moveTo(-TILE_SIZE/2);
      
      direction = 1;
      axis = 1;
      turnToHeading(0);
      waggle(axis, direction);
      moveTo(-TILE_SIZE/2);
      turnTo(Navigation.angleToTarget(x, y));
      
    } else {  //dy>dx
      direction = 1;
      if(xyt[1]>y) {
        axis = 1; 
      } else {
        axis = -1;
      }
      waggle(axis, direction);
      moveTo(-TILE_SIZE/2);
      
      direction = 0;
      axis = 1;
      turnToHeading(90);
      waggle(axis, direction);
      moveTo(-TILE_SIZE/2);
      turnTo(Navigation.angleToTarget(x, y));
    }
  }
}
