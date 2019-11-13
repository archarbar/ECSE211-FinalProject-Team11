package ca.mcgill.ecse211.navigation;

public class PlainNavigation extends Navigation {

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
