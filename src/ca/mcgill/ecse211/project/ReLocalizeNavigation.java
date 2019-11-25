package ca.mcgill.ecse211.project;

import lejos.robotics.SampleProvider;
import static ca.mcgill.ecse211.project.Resources.*;

/**
 * A navigator that actively corrects its heading and position.
 * 
 *
 * @author Matthew
 *
 */
public class ReLocalizeNavigation extends PlainNavigation {
  
  @Override
  public void travelTo(double x, double y) {
    colorSensorR.close();
    colorSensorL.close();
    super.travelTo(x, y);
//    relocalize(); 
  }
  
  public void reLocalize(double x, double y) {
    WaggleNavigation waggle = new WaggleNavigation();
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
      waggle.waggle(axis, direction);
      moveTo(-TILE_SIZE/2);
      
      direction = 1;
      axis = 1;
      turnToHeading(0);
      waggle.waggle(axis, direction);
      moveTo(-TILE_SIZE/2);
      turnTo(Navigation.angleToTarget(x, y));
      
    } else {  //dy>dx
      direction = 1;
      if(xyt[1]>y) {
        axis = 1; 
      } else {
        axis = -1;
      }
      waggle.waggle(axis, direction);
      moveTo(-TILE_SIZE/2);
      
      direction = 0;
      axis = 1;
      turnToHeading(90);
      waggle.waggle(axis, direction);
      moveTo(-TILE_SIZE/2);
      turnTo(Navigation.angleToTarget(x, y));
    }
  }
}
