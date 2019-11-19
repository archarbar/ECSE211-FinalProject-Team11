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
   * @param x in cm
   * @param y in cm
   */
  @Override
  public void travelTo(double x, double y) {
    turnTo(angleToTarget(x, y));
    double dist = calculateDistanceTo(x, y);
    moveTo(dist);
    turnTo(angleToTarget(x, y));
    dist = calculateDistanceTo(x, y);
    moveTo(dist);
  }

}
