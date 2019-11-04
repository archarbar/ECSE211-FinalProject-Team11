package ca.mcgill.ecse211.odometer;

import static ca.mcgill.ecse211.project.Resources.*;
import ca.mcgill.ecse211.lightSensor.DifferentialLineDetector;
import ca.mcgill.ecse211.lightSensor.LineDetectorController;
import ca.mcgill.ecse211.lightSensor.LineDetectorController.Edge;
import ca.mcgill.ecse211.navigation.LineNavigation;

public class OdometryCorrection {

  private static Odometer odometer = Odometer.getOdometer();
  
  /**
   * Diplacement from sensor to wheelbase parallel to the direction of travel in cm.
   */
  private static double sensorOffset = -2.5;

  public void run() {
    long updateStart, updateEnd;
    LineDetectorController centreDetector = new DifferentialLineDetector(centreLightSensor.getRedMode());
    LineDetectorController sideDetector = new DifferentialLineDetector(sideLightSensor.getRedMode());


    while (LineNavigation.isLineNavigating()) {
      //side
      updateStart = System.currentTimeMillis();
      if (sideDetector.lineDetected()) {
        double angle = odometer.getXYT()[2];
        if ((angle >= 350 || angle <= 10) || ((angle >= 170 && angle <= 190))) {
          double xPos = (odometer.getXYT()[0]+sideDetector.getEdgeX())/2;
          odometer.setX(roundToLine(xPos));
        } else if ((angle >= 80 && angle <= 100) || (angle >= 260 && angle <= 280)) {
          double yPos = (odometer.getXYT()[1]+sideDetector.getEdgeY())/2;
          odometer.setY(roundToLine(yPos));
        }
      }
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < CORRECTION_PERIOD/2) {
        try {
          Thread.sleep(CORRECTION_PERIOD/2 - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
      
      //centre
      updateStart = System.currentTimeMillis();
      if (!LineNavigation.isTurning()) {
        boolean centerLineDetected = (centreDetector.edgeDetected() == Edge.RisingEdge);
        if (centerLineDetected) {
          final double acceptableAngle = 25;
          double angle = odometer.getXYT()[2];
          if (angle >= 360-acceptableAngle || angle <= 0+acceptableAngle) {
            odometer.setTheta(0);
            double yPos = (odometer.getXYT()[1]+sideDetector.getEdgeY())/2;
            odometer.setY(roundToLine(yPos));
          } else if (angle >= 90-acceptableAngle && angle <= 90+acceptableAngle) {
            odometer.setTheta(90);
            double xPos = (odometer.getXYT()[0]+sideDetector.getEdgeX())/2;
            odometer.setX(roundToLine(xPos));
          } else if (angle >= 180-acceptableAngle && angle <= 180+acceptableAngle) {
            odometer.setTheta(180);
            double yPos = (odometer.getXYT()[1]+sideDetector.getEdgeY())/2;
            odometer.setY(roundToLine(yPos));
          } else if (angle >= 270-acceptableAngle && angle <= 270+acceptableAngle) {
            odometer.setTheta(270);
            double xPos = (odometer.getXYT()[0]+sideDetector.getEdgeX())/2;
            odometer.setX(roundToLine(xPos));
          }
        } else { //if isLineNavigating && !isTurning &&
          LineNavigation.findLine();
        }
      }


      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < CORRECTION_PERIOD/2) {
        try {
          Thread.sleep(CORRECTION_PERIOD/2 - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }

  }
  
  /**
   * given a position on an axis, return the nearest value that's on a gridline.
   * @param pos
   * @return
   */
  private static double roundToLine(double pos) {
    double round = Math.round(pos / TILE_SIZE) * TILE_SIZE;
    return round+sensorOffset;
  }
}