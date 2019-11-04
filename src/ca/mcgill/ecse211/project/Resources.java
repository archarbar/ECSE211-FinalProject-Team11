package ca.mcgill.ecse211.project;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class Resources {

  /**
   * The launcher motor.
   */
  public static final EV3LargeRegulatedMotor launcher1 = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  /**
   * The launcher motor.
   */
  public static final EV3LargeRegulatedMotor launcher2 = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));

  /**
   * The LCD.
   */
  public static final TextLCD LCD = LocalEV3.get().getTextLCD();

  public static final int LAUNCH_ANGLE = 55; // angle at which launching motor is placed before launch

  public static final int LAUNCH_HEIGHT = 35; // height of the launcher motor when extended

  public static final double LAUNCH_ARM_LENGTH = 18; // length of launcher arm

  public static final double G = 9.81; // gravity constant


  /**
   * the robot's wheel radius
   */

  public static final double WHEEL_RAD = 2.128; // temporary value 2.148

  /**
   * the robot's track size
   */

  public static final double TRACK = 16.24; // temporary value 15.1232

  /**
   * the distance at which we want to perform the launch
   *
   */

  public static final double LAUNCH_DISTANCE = 125; // temporary value


  public static final double TILE_SIZE = 30.48;
  public static final double TARGET_DISTANCE = 4 * TILE_SIZE;

  /**
   * The tile which the ping pong ball has to hit
   */


  public static double[][] targetTile = new double[][] {{3 * TILE_SIZE, 1 * TILE_SIZE},

  };

  public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));


  public static final int numTilesX = 8;
  public static final int numTilesY = 8;

  public static final int searchDistance = 5; // 5cm

  public static final EV3UltrasonicSensor US_SENSOR = new EV3UltrasonicSensor(LocalEV3.get().getPort("S4"));

  public static final EV3ColorSensor centreLightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S3"));

  public static final EV3ColorSensor sideLightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));

  public static final int CORRECTION_PERIOD = 50;

  public static final int searchTime = 5;

  /**
   * The motor acceleration in degrees per second squared.
   */
  public static final int LAUNCHER_ACCELERATION = 10000;

  /**
   * The angle which the launcher rotates by in degrees.
   */
  public static final int LAUNCHER_ANGLE = 120;

  /**
   * The speed at which the launcher rotates in degrees per second.
   */
  public static final int LAUNCH_SPEED = 326;
  // public static final int LAUNCH_SPEED = 1300;
  /**
   * The speed at which the launcher rotates in degrees per second.
   */
  public static final int RESET_SPEED = 70;
  
  /**
   * Represents a coordinate point on the competition map grid.
   * 
   * @author Younes Boubekeur
   */
  public static class Point {
    /** The x coordinate. */
    public double x;
    
    /** The y coordinate. */
    public double y;
    
    /**
     * Constructs a Point.
     * 
     * @param x the x coordinate
     * @param y the y coordinate
     */
    public Point(double x, double y) {
      this.x = x;
      this.y = y;
    }
    
    public String toString() {
      return "(" + x + ", " + y + ")";
    }
    
  }


}
