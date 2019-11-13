package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.robotics.SampleProvider;
import static ca.mcgill.ecse211.project.Resources.*;


public class WaggleNavigation extends Navigation {
  enum Axis{X,Y};
  enum Direction{POS,NEG};
  Odometer odometer = Odometer.getOdometer();
  private static final long CORRECTION_PERIOD = 5;
  private float[] csDataL;
  private float[] csDataR;
  private double distance;
  private double[] startPos, endPos;
  private double offTheta;
  private boolean isLeft = false;
  private SampleProvider colorSampleProviderL = colorSensorL.getRedMode(); // use a red light to compare luminence level
  private SampleProvider colorSampleProviderR = colorSensorR.getRedMode();
  private static final float LINE_RED_INTENSITY = (float) 0.3; // cast to float since default is double
  double[] position;

  @Override
  public void travelTo(double x, double y) {
    double heading = angleToTarget(x, y);
    heading = Math.round(heading / 90) * 90;
    position = Odometer.getOdometer().getXYT();
    int xTileDis = (int) Math.floor(Math.abs((x - position[0]) / TILE_SIZE));
    int yTileDis = (int) Math.floor(Math.abs((y - position[1]) / TILE_SIZE));

    
    // turn to heading
    turnTo(heading);
    
    toWaggle(xTileDis, yTileDis);
    heading = angleToTarget(x, y);
    heading = Math.round(heading / 90) * 90;
    turnTo(heading);
    toWaggle(xTileDis, yTileDis);
    heading = angleToTarget(x, y);
    turnTo(heading);
    double distance = calculateDistanceTo(x,y);
    moveTo(distance);
  }
  private void toWaggle(int xTileDis, int yTileDis) {
    int dis=0;
    Axis axis = null;
    Direction direction = null;
    if (position[2] >= 315 || position[2] <= 45) {
      axis = Axis.Y;
      dis = yTileDis;
      direction = Direction.POS;
    } else if (position[2] >= 135 && position[2] <= 225) {
      axis = Axis.Y;
      dis = yTileDis;
      direction = Direction.NEG;
    } else if (position[2] >= 45 && position[2] <= 135) {
      axis = Axis.X;
      dis = xTileDis;
      direction = Direction.POS;
    } else if (position[2] >= 225 && position[2] <= 315) {
      axis = Axis.X;
      dis = xTileDis;
      direction = Direction.NEG;
    }
    for(int i = 0;i<dis;++i) {
      waggle(axis, direction);
    }
  }
  private void waggle(Axis a, Direction d) {
    int axis = 0, direction = 0;
    if (a==Axis.X) {
      axis = 0;
    }
    else if (a==Axis.Y) {
      axis = 1;
    }
    if (d==Direction.POS) {
      direction = 1;
    }
    else if (d==Direction.NEG) {
      direction = -1;
    }
    waggle(axis, direction);
  }
  
  private void waggle(int direction, int side) {
    long correctionStart, correctionEnd;
    leftMotor.stop(true);
    rightMotor.stop(false);
    forwards();
    while (true) {
      correctionStart = System.currentTimeMillis();
      colorSampleProviderL.fetchSample(csDataL, 0); // get data from sensor
      colorSampleProviderR.fetchSample(csDataR, 0);
      if ((csDataL[0] < LINE_RED_INTENSITY) || (csDataR[0] < LINE_RED_INTENSITY)) { // if light read by sensor is
                                                                                    // smaller (darker) than red light,
                                                                                    // eg., black lines
        leftMotor.stop(true);
        rightMotor.stop(false);
        if (csDataR[0] < LINE_RED_INTENSITY) {
          isLeft = false;
        } else if (csDataL[0] < LINE_RED_INTENSITY) {
          isLeft = true;
        }
        startPos = odometer.getXYT();
        break;
      }
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    }
    forwards();
    if (isLeft) {
      while (true) {
        correctionStart = System.currentTimeMillis();
        colorSampleProviderR.fetchSample(csDataR, 0);
        if (csDataR[0] < LINE_RED_INTENSITY) {
          leftMotor.stop(true);
          rightMotor.stop(false);
          endPos = odometer.getXYT();
          distance = endPos[direction] - startPos[direction];
          offTheta = Math.toDegrees(Math.atan(distance / LSwidth));
          LCD.drawString("theta:  " + String.valueOf(offTheta), 0, 4);
          LCD.drawString("distance:  " + String.valueOf(distance), 0, 5);
          leftMotor.rotate(-convertAngle(offTheta), true);
          rightMotor.rotate(convertAngle(offTheta), false);
          break;
        }
        correctionEnd = System.currentTimeMillis();
        if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
          try {
            Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
      }
    } else {
      while (true) {
        correctionStart = System.currentTimeMillis();
        colorSampleProviderL.fetchSample(csDataL, 0);
        if (csDataL[0] < LINE_RED_INTENSITY) {
          leftMotor.stop(true);
          rightMotor.stop(false);
          endPos = odometer.getXYT();
          distance = endPos[direction] - startPos[direction];
          offTheta = Math.toDegrees(Math.atan(distance / LSwidth));
          LCD.drawString("theta:  " + String.valueOf(offTheta), 0, 4);
          LCD.drawString("distance:  " + String.valueOf(distance), 0, 5);
          leftMotor.rotate(convertAngle(offTheta), true);
          rightMotor.rotate(-convertAngle(offTheta), false);
          break;
        }
        correctionEnd = System.currentTimeMillis();
        if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
          try {
            Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
      }
    }
    double ave = (endPos[direction] + startPos[direction]) / 2;
    if (direction == 0) {
      odometer.setX(roundToLine(ave)+sensorOffset*side);
    }
    else if (direction == 1) {
      odometer.setY(roundToLine(ave)+sensorOffset*side);
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
    return round;
  }
}
