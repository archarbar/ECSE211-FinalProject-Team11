package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import java.util.Arrays;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;
/**
 * The class to use to avoid objects.
 * 
 * @author Matthew
 *
 */
public class ObjectAvoidance implements Runnable{
  private boolean running = false;
  private GridRectangle safeArea = islandRectangle;
  private double distanceDetected;
  
  /**
   * ultrasonic sensor data
   */
  private float[] usData;
  
  /**
   * ultrasonic sensor sample provider
   */
  private SampleProvider usSamples;

  /**
   * distance to wall in cm
   */
  private static final int WALL_DISTANCE = 22;
  
  /**
   * error margin of 3 cm to counter the effect of noise
   */
  private int errorMargin = 3;

  /**
   * Current location within the filter array.
   */
  int filterControl = 0;

  /**
   * Size of the median filter buffer.
   */
  int filterSize = 10;
  
  /**
   * Array of previous detected distances.
   */
  int[] sample = new int[filterSize];
  /**
   * A sorted array populated with the previous samples.
   */
  int[] sortedSamples = new int[filterSize];
  
  Navigation navigator;
  
  public ObjectAvoidance(Navigation navigator) {
    this.navigator = navigator;
  }
  
  public void stop() {
    running = false;
  }

  @Override
  public void run() {
    fillFilter();
    running = true;
    while (running) {
      long startTime = System.currentTimeMillis();
      boolean detected = getDistance() < WALL_DISTANCE - errorMargin;
      if (detected) {
        navigator.avoided = true;
        navigator.avoidance();
        Navigation.stop();
        avoid(1);
        navigator.avoidanceover();
      }
      
      long endTime = System.currentTimeMillis();
      if (endTime-startTime < AVOIDANCE_PERIOD) {
        try {
          Thread.sleep(AVOIDANCE_PERIOD - (endTime - startTime));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    }
  }
  
  /**
   * 
   * @param direction to go (-1 for left, 1 for right)
   */
  private void avoid(int direction) {
      double[] xyt = Odometer.getOdometer().getXYT();
      
      
      //TODO turn until rising edge.
      boolean searching = true;
      while (searching) {
        long startTime = System.currentTimeMillis();
        searching = getDistance() < WALL_DISTANCE + errorMargin;
        if (direction == 1) {
          turnClockwise();
        } else if (direction == -1) {
          turnCounterclockwise();
        }
        long endTime = System.currentTimeMillis();
        int diff = (int) (endTime-startTime);
        if (diff<AVOIDANCE_PERIOD) {
          try {
            Thread.sleep(AVOIDANCE_PERIOD-diff);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
        
        
      }
      if (!SILENT_VERIFICATION) {
        Sound.beep();
      }
      
      
      double[] xyt2 = Odometer.getOdometer().getXYT();
      double diffT = Math.abs(xyt[2]-xyt2[2]);
      if (diffT<80) {
        Navigation.turnTo(direction*(90-diffT));
        double distanceToTravel = Math.sin(Math.toRadians(diffT))*distanceDetected+TRACK*0.75;
        double currentT = Odometer.getOdometer().getXYT()[2];
        Navigation.safeArea = safeArea;
        if(Navigation.outOfBounds(distanceToTravel*Math.sin(Math.toRadians(currentT)), distanceToTravel*Math.cos(Math.toRadians(currentT)))) {
          if (direction == -1) {
            return;
          }
          avoid(-1);
          return;
        } else {
          Navigation.moveTo(distanceToTravel);
          Navigation.turnTo(-direction*90);
        }
      } else {
        Navigation.turnTo(-direction*90);
        double currentT = Odometer.getOdometer().getXYT()[2];
        if (Navigation.outOfBounds(TRACK*Math.sin(Math.toRadians(currentT)), TRACK*Math.cos(Math.toRadians(currentT)))) {
          avoid(-1);
          return;
        }
        Navigation.moveTo(-TRACK);
      }
      
  }
  /**
   * Fills the filter with the current distance, so not skew the initial values.
   */
  private void fillFilter() {
    usSamples.fetchSample(usData, 0); // acquire distance data in meters
    int distance = (int) (usData[0] * 100.0); // extract from buffer, convert to cm, cast to int
    for (int i = 0; i < filterSize; i++) { // we know that sample has a length of filter size so we can just use filterSize to save up calculations
      sample[i] = distance;
    }
  }
  /**
   * A median filter for the ultrasonic sensor.
   * 
   * @param distance
   * @return int filtered distance
   */
  private int medianFilter(int distance) {
    sample[filterControl] = distance;
    filterControl++;
    filterControl %= filterSize;

    sortedSamples = Arrays.copyOf(sample, filterSize);
    Arrays.sort(sortedSamples);
    int median = sortedSamples[sortedSamples.length / 2];
    if (distance > median) {
      distance = median;
    }
    return distance;
  }
  /**
   * makes the robot turn clockwise
   */
  public void turnClockwise() {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.forward();
    rightMotor.backward();
  }

  /**
   * makes the robot turn counterclockwise
   */
  public void turnCounterclockwise() {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.backward();
    rightMotor.forward();
  }
  /**
   * this is a simple getter method that gets the distance from the ultrasonic sensor
   * 
   * @return
   */
  public int getDistance() {
    usSamples.fetchSample(usData, 0);
    //int distance = (int) (usData[0] * 100);
    return medianFilter((int) (usData[0] * 100));

  }
}
