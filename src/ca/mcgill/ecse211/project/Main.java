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
   * The main method initializes all the threads and depending on user input, it runs either Falling edge or Rising edge
   * method.
   *
   * @param args
   */
  public static void main(String[] args) {
    TNG_LL = new Point(4,3);
    TNG_UR = new Point(5,4);
    BIN = new Point(2,6);
    TNR_UR_x = 315;
    
//     betaDemo();
importData();
     startOdometer();
//     turnTest();
//     Resources.SILENT_VERIFICATION = true;

//     localize(1,1);
////     localizationTimeTest();
//    // lineNavigationTest();
//    // plainNavigationTest();
//
//     tunnelTest();
     waggleNavigationTest();
//     launchTest();
  }

  private static void mainFlow() {
    // set to silent verification
    Resources.SILENT_VERIFICATION = true;
  }

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
    navigateThroughTunnel(new WaggleNavigation()); //use a waggle navigator to navigate

    // navigate to specified coords
    // turn to specified angle
    navigateToLaunch(new WaggleNavigation()); //use a waggle navigator to navigate
    Sound.beep();
    Sound.beep();
    Sound.beep();
    // launch
    int maxSpeed = 1300;
    launch(1, maxSpeed);
    Sound.beep();

    System.exit(0);
  }

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

  private static void startOdometer() {
    leftMotor.resetTachoCount();
    rightMotor.resetTachoCount();
    EV3LargeRegulatedMotor[] otherMotor = {rightMotor};
    leftMotor.synchronizeWith(otherMotor);
    odometer = Odometer.getOdometer();
    new Thread(odometer).start();
  }

  private static void localize(int x, int y) {
    SampleProvider usDistance = US_SENSOR.getMode("Distance");
    LightLocalizer lsLocalizer = new LightLocalizer();
    UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(UltrasonicLocalizer.edgeType.FallingEdge, usDistance);
    usLocalizer.mainMethod();
    US_SENSOR.close();
    Navigation.moveTo(-2);
//    Navigation.turnTo(-2);
    lsLocalizer.localize(x, y);
  }

  private static void navigateThroughTunnel(Navigation navigator) {
    Point tunnelEntr = Navigation.findTunnelEntrance(TNG_LL, TNG_UR);
    System.out.println("X val:"+tunnelEntr.x);
    System.out.println("Y val:"+tunnelEntr.y);
    navigator.travelTo(tunnelEntr.x, tunnelEntr.y);
    Navigation.moveTo(0.1); //align to tunnel

    // find angle of tunnel
    double aveX, aveY, theta;
    aveX = (TNG_LL.x + TNG_UR.x) / 2;
    aveY = (TNG_LL.y + TNG_UR.y) / 2;
    theta = Navigation.angleToTarget(aveX, aveY);
    Navigation.turnTo(theta);
    launcher1 = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
    launcher2 = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
    LauncherControl.lowerArm();
    Navigation.moveTo(4 * TILE_SIZE);
    LauncherControl.raiseArm();
    launcher1.close();
    launcher2.close();
    launcher1 = null;
    launcher2 = null;

  }

  private static void navigateToLaunch(Navigation navigator) {
    System.out.println("("+BIN.x+","+BIN.y+")");
    System.out.println("("+odometer.getXYT()[0]+","+odometer.getXYT()[1]+")");
    navigator.travelTo(BIN.x, BIN.y);
    Navigation.turnToHeading(TNR_UR_x);
  }

  private static void launch(int numLaunches, int speed) {
    rightMotor.close();
    leftMotor.close();
    launcher1 = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
    launcher2 = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
    for (int i = 0; i < numLaunches; ++i) {
      LauncherControl.launch(speed);
    }
  }

  private static void lineNavigationTest() {
    double x = 3 * TILE_SIZE, y = 4 * TILE_SIZE;
    Navigation navigator = new LineNavigation();
    navigator.safeArea = new GridRectangle(0, 0, 15, 15);
    navigator.travelTo(x, y);
    System.exit(0);
  }

  private static void plainNavigationTest() {
    double x = 3 * TILE_SIZE, y = 4 * TILE_SIZE;
    Navigation navigator = new PlainNavigation();
    navigator.safeArea = new GridRectangle(0, 0, 15, 15);
    navigator.travelTo(x, y);
    System.exit(0);
  }
  private static void waggleNavigationTest() {
    int x = 0, y = 2;
    Navigation navigator = new WaggleNavigation();
    navigator.travelTo(x, y);
//    Navigation.turnToHeading(135);
  }

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

  private static void tunnelTest() {
    TNG_LL = new Point(4, 4);
    TNG_UR = new Point(6, 5);
    navigateThroughTunnel(new WaggleNavigation());
  }

  private static void launchTest() {
    int maxSpeed = 1300;
    maxSpeed = 500;
    launch(3, maxSpeed);
  }
  private static void resetTest() {
    int lowSpeed = 100;
    launch(2, lowSpeed);
  }
  private static void turnTest() {
    for(int i =0;i<20;++i) {
      Navigation.turnTo(180);
    }
//    Navigation.turnTo(90);
//    Navigation.turnTo(-90);
//    Navigation.turnToHeading(-90);
//    Navigation.turnToHeading(0);
  }
}
