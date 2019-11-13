package ca.mcgill.ecse211.navigation;

import static ca.mcgill.ecse211.project.Resources.*;
import java.util.LinkedList;
import ca.mcgill.ecse211.lightSensor.DifferentialLineDetector;
import ca.mcgill.ecse211.lightSensor.LineDetectorController;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import lejos.robotics.SampleProvider;
@Deprecated
public class LineNavigation extends Navigation {
  private static boolean lineNavigating = false;
  public static boolean turning = false;
  private static final double sideSensorOffset = 3;
  public static double currentTargetX, currentTargetY;



  /*
   * public static void findLine() { lineNavigating = true; long startTime; int direction = -1, count = 0;
   * 
   * leftMotor.stop(); rightMotor.stop(); turnTo(-90); while (true) { startTime = System.currentTimeMillis(); direction
   * *= -1; count++; int angle = convertDistance(count * direction * searchDistance); leftMotor.rotate(angle, true);
   * rightMotor.rotate(angle, true);
   * 
   * while (!OdometryCorrection.lineDetected()) || (System.currentTimeMillis() - startTime) / 1000 > searchTime * count)
   * { try { Thread.sleep(CORRECTION_PERIOD); } catch (InterruptedException e) { // there is nothing to be done } } } //
   * TODO // stop motors // turn left/right (whichever is most likely drift) 90 degrees // move forwards a bit and try
   * to detect line // if line detected, stop, turn back to center and end method // if no line detected in small
   * distance, move backwards // if still no line detected, expand a bit more }
   */

  public static void findLine() {
    stop();
    double theta = Odometer.getOdometer().getXYT()[2];
    double heading = Math.round(theta / 90) * 90;
    double dTheta = heading - theta;
    if (dTheta >= 0) {
      dTheta += 90;
    } else {
      dTheta -= 90;
    }
    turnTo(dTheta);
    moveTo(-sideSensorOffset);
    forwards();
    LineDetectorController sideDetector = new DifferentialLineDetector(sideLightSensor.getRedMode());
    long updateStart, updateEnd;
    while(true) {
      updateStart = System.currentTimeMillis();
      if(sideDetector.lineDetected()) {
        break;
      }
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < CORRECTION_PERIOD / 2) {
        try {
          Thread.sleep(CORRECTION_PERIOD / 2 - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
    stop();
    moveTo(sideSensorOffset);
    turnToHeading(heading);
  }

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

  public void travelTo(double x, double y) {
    currentTargetX = x;
    currentTargetY = y;
    // if (outOfBounds(x, y)) {
    // return;
    // }
    lineNavigating = true;

    new Thread(new OdometryCorrection().setLineThread(Thread.currentThread())).start();
    double[] position = Odometer.getOdometer().getXYT();
    double[] nearestInt = nearestIntersect(x, y);
    double dx = Math.abs(nearestInt[0] - position[0]);
    double dy = Math.abs(nearestInt[1] - position[1]);
    double angle;
    double distance;


    angle = angleToTarget(x, y);
    angle = Math.round(angle / 90) * 90;
    turning = true;
    turnToHeading(angle);
    turnToHeading(angle);
    turning = false;
    if (dx > dy) {
      moveTo(dx);
      position = Odometer.getOdometer().getXYT();
      dx = Math.abs(nearestInt[0] - position[0]);
      turnToHeading(angle);
      moveTo(dx);
      turning = true;
      turnTo(angleToTarget(nearestInt[0], nearestInt[1]));
      turnTo(angleToTarget(nearestInt[0], nearestInt[1]));
      turning = false;
      moveTo(dy);
      position = Odometer.getOdometer().getXYT();
      dy = Math.abs(nearestInt[1] - position[1]);
      turnTo(angleToTarget(nearestInt[0], nearestInt[1]));
      moveTo(dy);
    } else {
      moveTo(dy);
      position = Odometer.getOdometer().getXYT();
      dy = Math.abs(nearestInt[1] - position[1]);
      turnToHeading(angle);
      moveTo(dy);
      turning = true;
      turnTo(angleToTarget(nearestInt[0], nearestInt[1]));
      turnTo(angleToTarget(nearestInt[0], nearestInt[1]));
      turning = false;
      moveTo(dx);
      position = Odometer.getOdometer().getXYT();
      dx = Math.abs(nearestInt[0] - position[0]);
      turnTo(angleToTarget(nearestInt[0], nearestInt[1]));
      moveTo(dx);
    }
    lineNavigating = false;
    angle = angleToTarget(x, y);
    distance = calculateDistanceTo(x, y);
    turning = true;
    turnTo(angle);
    turning = false;
    moveTo(distance);

  }

  private static double[] nearestIntersect() {
    double position[] = Odometer.getOdometer().getXYT();
    return nearestIntersect(position[0], position[1]);
  }

  private static double[] nearestIntersect(double x, double y) {
    double intersect[] = new double[2];
    double position[] = Odometer.getOdometer().getXYT();
    intersect[0] = Math.round(x / TILE_SIZE) * TILE_SIZE;
    intersect[1] = Math.round(y / TILE_SIZE) * TILE_SIZE;
    if (position[0] > intersect[0] + TILE_SIZE / 2) {
      intersect[0] += TILE_SIZE;
    }
    if (position[1] > intersect[1] + TILE_SIZE / 2) {
      intersect[1] += TILE_SIZE;
    }
    return intersect;
  }

}
