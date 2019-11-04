package ca.mcgill.ecse211.navigation;

import static ca.mcgill.ecse211.project.Resources.*;
import java.util.LinkedList;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import lejos.robotics.SampleProvider;

public class LineNavigation extends Navigation{
  private static boolean lineNavigating = false;
  public static boolean turning = false;

  
  
  /*
  public static void findLine() {
    lineNavigating = true;
    long startTime;
    int direction = -1, count = 0;

    leftMotor.stop();
    rightMotor.stop();
    turnTo(-90);
    while (true) {
      startTime = System.currentTimeMillis();
      direction *= -1;
      count++;
      int angle = convertDistance(count * direction * searchDistance);
      leftMotor.rotate(angle, true);
      rightMotor.rotate(angle, true);

      while (!OdometryCorrection.lineDetected())
          || (System.currentTimeMillis() - startTime) / 1000 > searchTime * count) {
        try {
          Thread.sleep(CORRECTION_PERIOD);
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
    // TODO
    // stop motors
    // turn left/right (whichever is most likely drift) 90 degrees
    // move forwards a bit and try to detect line
    // if line detected, stop, turn back to center and end method
    // if no line detected in small distance, move backwards
    // if still no line detected, expand a bit more
  }
  */

  /**
   * @return the lineNavigating
   */
  public static boolean isLineNavigating() {
    return lineNavigating;
  }

  /**
   * @return the turning
   */
  public static boolean isTurning() {
    return turning;
  }
  
  public static void travelTo(int x, int y) {
    double ax = (x+0.5)*TILE_SIZE;
    double ay = (y+0.5)*TILE_SIZE;
    travelTo(ax, ay);
  }
  
  public static void travelTo(double x, double y) {
    if (outOfBounds(x, y)) {
      return;
    }
    lineNavigating = true;
    
    
    double[] position = Odometer.getOdometer().getXYT();
    double dx = x-position[0];
    double dy = y-position[1];
    double angle;
    double distance;
    
    angle = angleToTarget(x, y);
    angle = Math.round(angle/90)*90;
    turnToHeading(angle);
    if(Math.abs(dx)>Math.abs(dy)) {
     
    }
    else {
      
    }
    angle = angleToTarget(x, y);
    distance = calculateDistanceTo(x, y);
    turnTo(angle);
    moveTo(distance);
    lineNavigating = false;
  }

  private static double[] nearestIntersect() {
    double position[] = Odometer.getOdometer().getXYT();
    return nearestIntersect(position[0], position[1]);
  }

  private static double[] nearestIntersect(double x, double y) {
    double intersect[] = new double[2];
    double position[] = Odometer.getOdometer().getXYT();
    intersect[0] = Math.round(x/TILE_SIZE)*TILE_SIZE;
    intersect[1] = Math.round(y/TILE_SIZE)*TILE_SIZE;
    
    return intersect;
  }
  
}
