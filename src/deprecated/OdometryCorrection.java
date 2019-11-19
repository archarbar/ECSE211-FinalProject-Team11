package deprecated;

import static ca.mcgill.ecse211.project.Resources.*;
import ca.mcgill.ecse211.project.DifferentialLineDetector;
import ca.mcgill.ecse211.project.LineDetectorController;
import ca.mcgill.ecse211.project.Odometer;
import ca.mcgill.ecse211.project.LineDetectorController.Edge;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;

@Deprecated
public class OdometryCorrection implements Runnable {

  private static Odometer odometer = Odometer.getOdometer();

  /**
   * Diplacement from sensor to wheelbase parallel to the direction of travel in cm.
   */
  private static double sensorOffset = -3;
  private Thread mainThread;

  private EV3ColorSensor centreLightSensor;

  private EV3ColorSensor sideLightSensor;

  /**
   * the detection of line thread.
   */
  public void run() {
    long updateStart, updateEnd;
    LineDetectorController centreDetector = new DifferentialLineDetector(centreLightSensor.getRedMode());
    LineDetectorController sideDetector = new DifferentialLineDetector(sideLightSensor.getRedMode());


    while (LineNavigation.isLineNavigating()) {
      // side
      updateStart = System.currentTimeMillis();
      if (sideDetector.lineDetected()) {
        if (!SILENT_VERIFICATION) {
          Sound.beep();
        }
        boolean north = false, south = false, east = false, west = false;
        double angle = odometer.getXYT()[2];
        if (angle >= 350 || angle <= 10) {
          north = true;
        } else if (angle >= 170 && angle <= 190) {
          south = true;
        } else if (angle >= 80 && angle <= 100) {
          east = true;
        } else if (angle >= 260 && angle <= 280) {
          west = true;
        }

        if (north || south) {
          double yPos = (odometer.getXYT()[0] + sideDetector.getEdgeY()) / 2;
          odometer.setY(roundToLine(yPos));
        } else if (east || west) {
          double xPos = (odometer.getXYT()[1] + sideDetector.getEdgeX()) / 2;
          odometer.setX(roundToLine(xPos));
        }
        double[] xy = odometer.getXYT();
        if (north) {
          odometer.setY(xy[0] + sensorOffset);
        } else if (south) {
          odometer.setY(xy[0] - sensorOffset);
        } else if (east) {
          odometer.setX(xy[1] + sensorOffset);
        } else if (west) {
          odometer.setX(xy[1] - sensorOffset);
        }
      }
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < CORRECTION_PERIOD / 2) {
        try {
          Thread.sleep(CORRECTION_PERIOD / 2 - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }

      // centre
      updateStart = System.currentTimeMillis();
      if (!LineNavigation.isTurning()) {
        boolean centerLineDetected = (centreDetector.edgeDetected() == Edge.RisingEdge);
        if (centerLineDetected) {
          final double acceptableAngle = 25;
          double angle = odometer.getXYT()[2];
          if (angle >= 360 - acceptableAngle || angle <= 0 + acceptableAngle) {
            // odometer.setTheta(0);
            double yPos = (odometer.getXYT()[1] + sideDetector.getEdgeY()) / 2;
            odometer.setY(roundToLine(yPos));
          } else if (angle >= 90 - acceptableAngle && angle <= 90 + acceptableAngle) {
            // odometer.setTheta(90);
            double xPos = (odometer.getXYT()[0] + sideDetector.getEdgeX()) / 2;
            odometer.setX(roundToLine(xPos));
          } else if (angle >= 180 - acceptableAngle && angle <= 180 + acceptableAngle) {
            // odometer.setTheta(180);
            double yPos = (odometer.getXYT()[1] + sideDetector.getEdgeY()) / 2;
            odometer.setY(roundToLine(yPos));
          } else if (angle >= 270 - acceptableAngle && angle <= 270 + acceptableAngle) {
            // odometer.setTheta(270);
            double xPos = (odometer.getXYT()[0] + sideDetector.getEdgeX()) / 2;
            odometer.setX(roundToLine(xPos));
          }
        } else { // if isLineNavigating && !isTurning &&
//           try {
//            mainThread.wait();
//          } catch (InterruptedException e) {
//            // TODO Auto-generated catch block
//            e.printStackTrace();
//          }
//           LineNavigation.findLine();
////           new LineNavigation().travelTo(LineNavigation.currentTargetX, LineNavigation.currentTargetY);
//           mainThread.notify();
          // Sound.beep();
          int a = 1;
        }
      }

      double[] xyt = odometer.getXYT();
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < CORRECTION_PERIOD / 2) {
        try {
          Thread.sleep(CORRECTION_PERIOD / 2 - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }

  }

  /**
   * given a position on an axis, return the nearest value that's on a gridline.
   * 
   * @param pos
   * @return
   */
  private static double roundToLine(double pos) {
    double round = Math.round(pos / TILE_SIZE) * TILE_SIZE;
    return round + sensorOffset;
  }
  
  /**
   * sets the thread to pause while executing.
   * @param mainThread
   * @return
   */
  public OdometryCorrection setLineThread(Thread mainThread) {
    this.mainThread = mainThread;
    return this;
  }
}
