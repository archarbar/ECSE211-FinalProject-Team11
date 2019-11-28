package ca.mcgill.ecse211.project;

/**
 * A simple navigation implementation.
 * 
 * @author Matthew
 */
public class PlainNavigation extends Navigation {

  /**
   * Tells the robot to travel in a straight line from its current location to a given point in cm.
   * 
   * @param x location in cm
   * @param y location in cm
   */
  @Override
  public void travelTo(double x, double y) {
    turnTo(angleToTarget(x, y));
    double dist = calculateDistanceTo(x, y);
    moveTo(dist);
    dist = calculateDistanceTo(x, y);
    while (dist>1) {
      turnTo(angleToTarget(x, y));
      moveTo(dist);
      dist = calculateDistanceTo(x, y);
    }
  }

}
