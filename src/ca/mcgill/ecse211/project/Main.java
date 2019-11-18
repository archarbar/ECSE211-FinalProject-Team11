package ca.mcgill.ecse211.project;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import static ca.mcgill.ecse211.project.Resources.*;
import java.text.DecimalFormat;
import ca.mcgill.ecse211.project.Display;
import ca.mcgill.ecse211.project.LightLocalizer;
import ca.mcgill.ecse211.project.UltrasonicLocalizer;
import ca.mcgill.ecse211.navigation.GridRectangle;
import ca.mcgill.ecse211.navigation.LineNavigation;
import ca.mcgill.ecse211.navigation.Navigation;
import ca.mcgill.ecse211.navigation.PlainNavigation;
import ca.mcgill.ecse211.navigation.WaggleNavigation;
import ca.mcgill.ecse211.odometer.Odometer;

public class Main {
  private static Odometer odometer;
  private static Point TNG_LL = tng.ll, TNG_UR = tng.ur, BIN = bin;
  private static double TNR_UR_x = targetAngle;


  /**
   * The main method initializes all the threads and runs the main flow.
   *
   * @param args
   */
  public static void main(String[] args) {
    // TNG_LL = new Point(4,3);
    // TNG_UR = new Point(5,5);
    // BIN = new Point(2,6);
    // TNR_UR_x = 315;
    // turnTest();
    betaDemo();
    // startOdometer();
    // Resources.SILENT_VERIFICATION = true;
    //// importData();
    // localize(1,1);
    //// localizationTimeTest();
    // // lineNavigationTest();
    // // plainNavigationTest();
    //
    // tunnelTest();
    // waggleNavigationTest();
    // launchTest();
  }


  /**
   * The main flow of the project.
   */
  private static void mainFlow() {
    // set to silent verification
    Resources.SILENT_VERIFICATION = true;
  }

  /**
   * The flow used for the beta demo.
   */
  private static void betaDemo() {
    // set to silent verification
    Resources.SILENT_VERIFICATION = true;
    // import wifi data is done by default
    importData();
    // odometry start
    startOdometer();

    // localize
    localize(1, 1);
    Sound.beep();

    // navigate to tunnel
    navigateThroughTunnel(new WaggleNavigation());

    // navigate to specified coords
    // turn to specified angle
    navigateToLaunch(new WaggleNavigation());
    Sound.beep();
    Sound.beep();
    Sound.beep();
    // launch
    int maxSpeed = 1300;
    launch(1, maxSpeed);
    Sound.beep();

    System.exit(0);
  }

  /**
   * Imports data over wifi.
   */
  private static void importData() {
    System.out.println("Running...");

    // Example 1: Print out all received data
    System.out.println("Map:\n" + wifiParameters);

    // Example 2: Print out specific values
    System.out.println("Green Team: " + greenTeam);
    System.out.println("Green Zone: " + green);
    System.out.println("Island Zone, upper right: " + island.ur);
    System.out.println("Bin location: x=" + bin.x + ", y=" + bin.y);
    System.out.println("Target Angle: " + targetAngle);
  }

  /**
   * Creates an odometer and starts it in a thread.
   */
  private static void startOdometer() {
    leftMotor.resetTachoCount();
    rightMotor.resetTachoCount();
    odometer = Odometer.getOdometer();
    new Thread(odometer).start();
  }

  /**
   * Localises the robot once using the ultrasonic localizer, and once with the light localizer, and sets its location
   * to the given grid coords.
   * 
   * @param x
   * @param y
   */
  private static void localize(int x, int y) {
    initUSSensor();
    SampleProvider usDistance = US_SENSOR.getMode("Distance");
    LightLocalizer lsLocalizer = new LightLocalizer();
    UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(UltrasonicLocalizer.edgeType.FallingEdge, usDistance);
    usLocalizer.localize();
    US_SENSOR.close();
    US_SENSOR = null;
    Navigation.moveTo(-2);
    // Navigation.turnTo(-2);
    initLightSensors();
    lsLocalizer.localize(x, y);
  }

  /**
   * Navigates to, and through the tunnel to the launching side.
   * 
   * @param navigator to determine navigation type.
   */
  private static void navigateThroughTunnel(Navigation navigator) {
    Point tunnelEntr = Navigation.findTunnelEntrance(TNG_LL, TNG_UR);
    System.out.println("X val:" + tunnelEntr.x);
    System.out.println("Y val:" + tunnelEntr.y);
    navigator.travelTo(tunnelEntr.x, tunnelEntr.y);
    Navigation.moveTo(0.1);

    // find angle of tunnel
    double aveX, aveY, theta;
    aveX = (TNG_LL.x + TNG_UR.x) / 2;
    aveY = (TNG_LL.y + TNG_UR.y) / 2;
    theta = Navigation.angleToTarget(aveX, aveY);
    Navigation.turnTo(theta);
    initLaunchers();
    LauncherControl.lowerArm();
    closeLightSensors();
    Navigation.moveTo(4 * TILE_SIZE);
    initLightSensors();
    LauncherControl.raiseArm();
    closeLaunchers();
  }

  /**
   * navigates from the tunnel exit to the launch location.
   * 
   * @param navigator to determine navigation type.
   */
  private static void navigateToLaunch(Navigation navigator) {
    System.out.println("(" + BIN.x + "," + BIN.y + ")");
    System.out.println("(" + odometer.getXYT()[0] + "," + odometer.getXYT()[1] + ")");
    navigator.travelTo(BIN.x, BIN.y);
    Navigation.turnToHeading(TNR_UR_x);
  }

  /**
   * Launches the ping-pong balls a given number of times at a given speed.
   * 
   * @param numLaunches
   * @param speed
   */
  private static void launch(int numLaunches, int speed) {
    closeMotors();
    initLaunchers();
    for (int i = 0; i < numLaunches; ++i) {
      LauncherControl.launch(speed);
    }
    closeLaunchers();
  }

  /**
   * initialises the launcher motors.
   */
  private static void initLaunchers() {
    launcher1 = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
    launcher2 = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  }

  /**
   * initialises the movement motors.
   */
  private static void initMotors() {
    leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
    rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
  }

  /**
   * initialises the light sensors.
   */
  private static void initLightSensors() {
    colorSensorR = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
    colorSensorL = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
  }

  /**
   * initialises the ultrasonic sensor.
   */
  private static void initUSSensor() {
    US_SENSOR = new EV3UltrasonicSensor(LocalEV3.get().getPort("S3"));
  }

  /**
   * closes the launcher motors.
   */
  private static void closeLaunchers() {
    launcher1.close();
    launcher2.close();
  }

  /**
   * closes the movement motors.
   */
  private static void closeMotors() {
    leftMotor.close();
    rightMotor.close();
  }

  /**
   * closes the light sensors.
   */
  private static void closeLightSensors() {
    colorSensorR.close();
    colorSensorL.close();
  }

  /**
   * closes the ultrasonic sensor.
   */
  private static void closeUSSensor() {
    US_SENSOR.close();
  }

  /**
   * flow to test the plain navigation technique.
   */
  private static void plainNavigationTest() {
    double x = 3 * TILE_SIZE, y = 4 * TILE_SIZE;
    Navigation navigator = new PlainNavigation();
    navigator.safeArea = new GridRectangle(0, 0, 15, 15);
    navigator.travelTo(x, y);
    System.exit(0);
  }

  /**
   * flow to test the waggle navigation technique.
   */
  private static void waggleNavigationTest() {
    int x = 8, y = 4;
    Navigation navigator = new WaggleNavigation();
    navigator.travelTo(x, y);
    Navigation.turnToHeading(135);
  }

  /**
   * flow to test the localization method and time.
   */
  private static void localizationTimeTest() {
    long startTime = System.currentTimeMillis();
    localize(1, 1);
    long endTime = System.currentTimeMillis();
    long totalTime = endTime - startTime;
    if (totalTime > 30 * 1000)
      Sound.beepSequence();
    else
      Sound.beepSequenceUp();
  }

  /**
   * flow to test the tunnel navigation method.
   */
  private static void tunnelTest() {
    TNG_LL = new Point(4, 4);
    TNG_UR = new Point(6, 5);
    navigateThroughTunnel(new WaggleNavigation());
  }

  /**
   * flow to test launching at max speed.
   */
  private static void launchTest() {
    int maxSpeed = 1300;
    maxSpeed = 500;
    launch(3, maxSpeed);
  }

  /**
   * flow to test reloading the launcher without firing.
   */
  private static void resetTest() {
    int lowSpeed = 100;
    launch(2, lowSpeed);
  }

  /**
   * flow to test accurate turning.
   */
  private static void turnTest() {
    for (int i = 0; i < 20; ++i) {
      Navigation.turnTo(180);
    }
  }
}
