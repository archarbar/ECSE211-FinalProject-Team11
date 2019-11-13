package ca.mcgill.ecse211.lightSensor;

import static ca.mcgill.ecse211.project.Resources.*;
import ca.mcgill.ecse211.project.odometer.Odometer;
import ca.mcgill.ecse211.project.Main;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;

public class LightLocalizer implements Runnable{
  private static final long CORRECTION_PERIOD = 5;
  private float[] usData;
//  private Odometer odometer;
  private float[] csDataL;
  private float[] csDataR;
  private double distance;
  private double[] startPos;
  private Odometer odometer;
  private double offTheta;
  private boolean isLeft=false;
  private SampleProvider colorSampleProviderL= colorSensorL.getRedMode(); // use a red light to compare luminence level
  private SampleProvider colorSampleProviderR= colorSensorR.getRedMode();
  private static final float LINE_RED_INTENSITY = (float) 0.3; // cast to float since default is double
  
  public LightLocalizer() {
    this.odometer = Odometer.getOdometer();
    csDataL = new float[colorSensorL.sampleSize()];
    csDataR = new float[colorSensorL.sampleSize()];
  }

  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   * 
   * @param distance
   * @return the wheel rotations necessary to cover the distance
   */
  public static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that
   * angle.
   * 
   * @param angle
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int convertAngle(double angle) {
    return convertDistance(Math.PI * TRACK * angle / 360.0);
  }
  
  public void run() {
    findDistance();
  }
  private void findDistance() {
    odometer.setXYT(0,0,0);
    long correctionStart, correctionEnd;
    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
//  leftMotor.stop();
//  rightMotor.stop();
//  leftMotor.setAcceleration(ACCELERATION);
//  rightMotor.setAcceleration(ACCELERATION);
//    leftMotor.setSpeed(ROTATE_SPEED);
//    rightMotor.setSpeed(ROTATE_SPEED);
//    leftMotor.rotate(-convertAngle(720), true);
//  rightMotor.rotate(convertAngle(720), false);
    double[] ini_pos=odometer.getXYT();
    leftMotor.stop(true);
    rightMotor.stop(false);
    leftMotor.setAcceleration(ACCELERATION);
    rightMotor.setAcceleration(ACCELERATION);
    leftMotor.setSpeed(QUANT_SPEED);
    rightMotor.setSpeed(QUANT_SPEED);
    leftMotor.forward();
    rightMotor.forward();

    while (true) {
      correctionStart = System.currentTimeMillis();
      colorSampleProviderL.fetchSample(csDataL, 0); //get data from sensor
      colorSampleProviderR.fetchSample(csDataR, 0);
      if((csDataL[0] < LINE_RED_INTENSITY)||(csDataR[0] < LINE_RED_INTENSITY)) { //if light read by sensor is smaller (darker) than red light, eg., black lines
        // sound alert to know when the sensor hits a line
        leftMotor.stop(true);
        rightMotor.stop(false);
        if(csDataR[0] < LINE_RED_INTENSITY) {
          isLeft=false;
        }
        else if(csDataL[0] < LINE_RED_INTENSITY) {
          isLeft=true;
        }
        startPos  = odometer.getXYT();
//        new Thread(odometer).start();
        break;
      }
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        Main.sleepFor(CORRECTION_PERIOD - (correctionEnd - correctionStart));
      }
    }
    leftMotor.setAcceleration(ACCELERATION);
    rightMotor.setAcceleration(ACCELERATION);
    leftMotor.setSpeed(QUANT_SPEED);
    rightMotor.setSpeed(QUANT_SPEED);
    leftMotor.forward();
    rightMotor.forward();
    if(isLeft) {
      while(true) {
        correctionStart = System.currentTimeMillis();
        colorSampleProviderR.fetchSample(csDataR, 0);
        if(csDataR[0] < LINE_RED_INTENSITY) {
          leftMotor.stop(true);
          rightMotor.stop(false);
          distance=(odometer.getXYT())[1]-startPos[1];
          offTheta=Math.toDegrees(Math.atan(distance/LSwidth));
          LCD.drawString("theta:  "+String.valueOf(offTheta), 0, 4);
          LCD.drawString("distance:  "+String.valueOf(distance), 0, 5);
          leftMotor.rotate(-convertAngle(offTheta), true);
          rightMotor.rotate(convertAngle(offTheta), false);
          leftMotor.rotate(convertDistance(-5), true);
          rightMotor.rotate(convertDistance(-5), false);
          leftMotor.rotate(convertAngle(90.0), true);
          rightMotor.rotate(-convertAngle(90.0), false);
//          leftMotor.forward();
//          rightMotor.forward();
          break;
        }
        correctionEnd = System.currentTimeMillis();
        if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
          Main.sleepFor(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        }
      }
    }
    else {
      while(true) {
        correctionStart = System.currentTimeMillis();
        colorSampleProviderL.fetchSample(csDataL, 0);
        if(csDataL[0] < LINE_RED_INTENSITY) {
          leftMotor.stop(true);
          rightMotor.stop(false);
          distance=(odometer.getXYT())[1]-startPos[1];
          offTheta=Math.toDegrees(Math.atan(distance/LSwidth));
          LCD.drawString("theta:  "+String.valueOf(offTheta), 0, 4);
          LCD.drawString("distance:  "+String.valueOf(distance), 0, 5);
          leftMotor.rotate(convertAngle(offTheta), true);
          rightMotor.rotate(-convertAngle(offTheta), false);
          leftMotor.rotate(convertDistance(-5), true);
          rightMotor.rotate(convertDistance(-5), false);
          leftMotor.rotate(convertAngle(90.0), true);
          rightMotor.rotate(-convertAngle(90.0), false);
//          leftMotor.forward();
//          rightMotor.forward();
          break;
        }
        correctionEnd = System.currentTimeMillis();
        if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
          Main.sleepFor(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        }
      }
    }
    
    leftMotor.stop(true);
    rightMotor.stop(false);
    leftMotor.setAcceleration(ACCELERATION);
    rightMotor.setAcceleration(ACCELERATION);
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.forward();
    rightMotor.forward();
    
    while(true) {
      correctionStart = System.currentTimeMillis();
      colorSampleProviderL.fetchSample(csDataL, 0); //get data from sensor
      colorSampleProviderR.fetchSample(csDataR, 0);
      if((csDataL[0] < LINE_RED_INTENSITY)||(csDataR[0] < LINE_RED_INTENSITY)) {
        //if light read by sensor is smaller (darker) than red light, eg., black lines
        //          Sound.beep();
        leftMotor.rotate(convertDistance(3.5), true);
        rightMotor.rotate(convertDistance(3.5), false);
        leftMotor.rotate(-convertAngle(90.0), true);
        rightMotor.rotate(convertAngle(90.0), false);
        leftMotor.rotate(convertDistance(6.7), true);
        rightMotor.rotate(convertDistance(6.7), false);
        break;
        //          motor.stop();
      }
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        Main.sleepFor(CORRECTION_PERIOD - (correctionEnd - correctionStart));
      }
    }

      }

  
}
