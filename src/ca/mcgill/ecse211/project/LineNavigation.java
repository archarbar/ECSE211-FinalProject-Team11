package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import java.util.LinkedList;
import ca.mcgill.ecse211.odometer.OdometryCorrection;

public class LineNavigation {
  public static boolean lineNavigating = false;
  public static boolean turning = false; 
  
  public static void findLine() {
    lineNavigating = true;
    long startTime;
    int direction = -1, count = 0;
    
    leftMotor.stop();
    rightMotor.stop();
    Navigation.turn(-90);
    while (true) {
      startTime = System.currentTimeMillis();
      direction*=-1;
      count++;
      int angle = Navigation.convertDistance(count*direction*searchDistance);
      leftMotor.rotate(angle, true);
      rightMotor.rotate(angle, true);
      
      while(!OdometryCorrection.lineDetected(Resources.centerLightSensor)||(System.currentTimeMillis()-startTime)/1000>searchTime*count) {
        try {
          Thread.sleep(Resources.CORRECTION_PERIOD);
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  //    TODO
  //    stop motors
  //    turn left/right (whichever is most likely drift) 90 degrees
  //    move forwards a bit and try to detect line
  //    if line detected, stop, turn back to center and end method
  //    if no line detected in small distance, move backwards
  //    if still no line detected, expand a bit more
    }
  
  
}
