package ca.mcgill.ecse211.navigation;

import static ca.mcgill.ecse211.project.Resources.*;

public class PlainNavigation extends Navigation {  

  @Override
  public void travelTo(double x, double y) {
    turnTo(angleToTarget(x, y));

    double dist = calculateDistanceTo(x,y);

    leftMotor.setSpeed(180);
    rightMotor.setSpeed(180);
    leftMotor.rotate(convertDistance(dist), true);
    rightMotor.rotate(convertDistance(dist), false);

  }

}
