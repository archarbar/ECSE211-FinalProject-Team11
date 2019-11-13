package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import ca.mcgill.ecse211.navigation.PlainNavigation;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;

public class LightLocalizer extends PlainNavigation {
  private static final long CORRECTION_PERIOD = 5;
  private float[] csDataL;
  private float[] csDataR;
  private double distance;
  private double[] startPos;
  private Odometer odometer;
  private double offTheta;
  private boolean isLeft = false;
  private SampleProvider colorSampleProviderL = colorSensorL.getRedMode(); // use a red light to compare luminence level
  private SampleProvider colorSampleProviderR = colorSensorR.getRedMode();
  private static final float LINE_RED_INTENSITY = (float) 0.3; // cast to float since default is double
  private int x, y;

  public LightLocalizer() {
    this.odometer = Odometer.getOdometer();
    csDataL = new float[colorSensorL.sampleSize()];
    csDataR = new float[colorSensorL.sampleSize()];
  }

  public void localize(int x, int y) {
    findDistance(x,y);
  }

  private void findDistance(int x, int y) {
    odometer.setXYT(0, 0, 0);
    long correctionStart, correctionEnd;
    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    double[] ini_pos = odometer.getXYT();
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
        // sound alert to know when the sensor hits a line
        leftMotor.stop(true);
        rightMotor.stop(false);
        if (csDataR[0] < LINE_RED_INTENSITY) {
          isLeft = false;
        } else if (csDataL[0] < LINE_RED_INTENSITY) {
          isLeft = true;
        }
        startPos = odometer.getXYT();
        // new Thread(odometer).start();
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
          distance = (odometer.getXYT())[1] - startPos[1];
          offTheta = Math.toDegrees(Math.atan(distance / LSwidth));
          LCD.drawString("theta:  " + String.valueOf(offTheta), 0, 4);
          LCD.drawString("distance:  " + String.valueOf(distance), 0, 5);
          leftMotor.rotate(-convertAngle(offTheta), true);
          rightMotor.rotate(convertAngle(offTheta), false);
          leftMotor.rotate(convertDistance(-5), true);
          rightMotor.rotate(convertDistance(-5), false);
          leftMotor.rotate(convertAngle(90.0), true);
          rightMotor.rotate(-convertAngle(90.0), false);
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
          distance = (odometer.getXYT())[1] - startPos[1];
          offTheta = Math.toDegrees(Math.atan(distance / LSwidth));
          LCD.drawString("theta:  " + String.valueOf(offTheta), 0, 4);
          LCD.drawString("distance:  " + String.valueOf(distance), 0, 5);
          leftMotor.rotate(convertAngle(offTheta), true);
          rightMotor.rotate(-convertAngle(offTheta), false);
          leftMotor.rotate(convertDistance(-5), true);
          rightMotor.rotate(convertDistance(-5), false);
          leftMotor.rotate(convertAngle(90.0), true);
          rightMotor.rotate(-convertAngle(90.0), false);
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

    stop();
    forwards();

    while (true) {
      correctionStart = System.currentTimeMillis();
      colorSampleProviderL.fetchSample(csDataL, 0); // get data from sensor
      colorSampleProviderR.fetchSample(csDataR, 0);
      if ((csDataL[0] < LINE_RED_INTENSITY) || (csDataR[0] < LINE_RED_INTENSITY)) {
        // if light read by sensor is smaller (darker) than red light, eg., black lines
        // Sound.beep();
        moveTo(sensorOffset);
        leftMotor.rotate(-convertAngle(90.0), true);
        rightMotor.rotate(convertAngle(90.0), false);
        moveTo(5+sensorOffset+1);
        break;
        // motor.stop();
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

    odometer.setX(x*TILE_SIZE);
    odometer.setY(y*TILE_SIZE);
    odometer.setTheta(0);
  }


}
