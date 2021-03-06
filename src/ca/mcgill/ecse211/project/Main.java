package ca.mcgill.ecse211.project;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import static ca.mcgill.ecse211.project.Resources.*;

/**
 * The running point of the entire project.
 * @author Team11
 *
 */
public class Main {

  /**
   * our odometer
   */
  private static Odometer odometer;

  /**
   * tunnel corners and bin location imported from resources
   */
  private static Point TNG_LL, TNG_UR, BIN;

  /**
   * home point to go back to after launching
   */
  private static IntPoint home, homeLocalize;
  
  /**
   * the distance between the selected launch point and the bin
   */
  public static double launchDist;

  /**
   * Starting theta
   */
  private static double startingT = 0;

  /**
   * use relocalize navigation type
   *
   */
  private static Navigation navigator = new ReLocalizeNavigation(); //new WaggleNavigation();
  /**
   * Avoidance type;
   */
  private static ObjectAvoidance avoider = null;


  /**
   * The main method initializes all the threads and runs the main flow.
   *
   * @param args no command line use.
   */
  public static void main(String[] args) {
    mainFlow();
  }


  /**
   * The main flow of the project.
   */
  private static void mainFlow() {
    //section 1
    System.out.println("init");
    initialization();
    //section 2
    System.out.println("travel to launch");
    int launchSpeed = travelToLaunch();
    //section3
    System.out.println("launch");
    launch(4, launchSpeed);
    //section4
    System.out.println("return");
    returnToBase();
  }

  /**
   * Handles the initialization step of the main flow.
   */
  private static void initialization() {
 // set to silent verification
    Resources.SILENT_VERIFICATION = true;
    // import wifi data is done by default
    initLightSensors();
    importData();
    // odometry start
    startOdometer();

    // localize
    System.out.println("localizing...");
    localize(homeLocalize.x, homeLocalize.y, startingT);
    System.out.println("localize to ("+homeLocalize.x+","+homeLocalize.y+","+startingT+")");
    beep(3);
  }

  /**
   * Handles the travel to launch step of the main flow.
   * @return launchSpeed depending on distance from target.
   */
  private static int travelToLaunch() {
    navigateThroughTunnel(navigator);
    // get launch location
    Point launchLocation = Navigation.findBestLaunchPoint(BIN);
    // get launch speed depending on location distance to bin
//    System.out.println(distanceToSpeed);
    int launchSpeed = distanceToSpeed.get(launchDist);
    //enable obstacle avoidance
    avoider = new ObjectAvoidance(navigator);
    new Thread(avoider).start();
    // navigate to launch location
    navigateToLaunch(navigator, launchLocation.x, launchLocation.y);
    if (navigator instanceof ReLocalizeNavigation) {
      if (Navigation.calculateDistanceTo(launchLocation.x, launchLocation.y)>1.5*TILE_SIZE) {
        double xyt[] = Odometer.getOdometer().getXYT();
        ((ReLocalizeNavigation) navigator).reLocalize(xyt[0]+TILE_SIZE, xyt[1]); 
      }
    }
    initUSSensor();
    navigateToLaunch(navigator, launchLocation.x, launchLocation.y);
    
    //disable obstacle avoidance
    Navigation.stop();
    beep(3);

    return launchSpeed;
  }

  /**
   * Handles the return to base step of the main flow.
   */
  private static void returnToBase() {
    //enable obstacle avoidance
    navigateThroughTunnel(navigator);
    closeUSSensor();
    //disable obstacle avoidance
    navigator.travelTo(home.x, home.y, false); // navigate back to home, false for no need to go to center of tile
    Navigation.stop();
    beep(5);
  }
  /**
   * makes the robot beep a specified number of times
   * @param n is the number of beeps
   */
  private static void beep(int n) {
    for(;n>0;--n) {
      Sound.beep();
    }
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
    System.out.println("Red Bin location: x=" + redBin.x + ", y=" + redBin.y);
    System.out.println("Green Bin location: x=" + greenBin.x + ", y=" + greenBin.y);

    int corner = -1;
    if (redTeam == TEAM_NUMBER) {
      System.out.println("red");
      corner = redCorner;
      BIN = redBin;
      TNG_LL = tnr.ll;
      TNG_UR = tnr.ur;
    } else if(greenTeam == TEAM_NUMBER) {
      System.out.println("green");
      corner = greenCorner;
      BIN = greenBin;
      TNG_LL = tng.ll;
      TNG_UR = tng.ur;
    }
    // set point to localize to, home to come back to, and starting angle relative to origin depending on starting corner
    System.out.println("corner"+corner);
    if (corner == 0) {
      homeLocalize = new IntPoint(1, 1);
      home = new IntPoint(0,0);
      startingT = 0;
    } else if (corner == 1) {
      homeLocalize = new IntPoint(14, 1);
      home = new IntPoint(15,0);
      startingT = 270;
    } else if (corner == 2) {
      homeLocalize = new IntPoint(14, 8);
      home = new IntPoint(15,9);
      startingT = 180;
    } else if (corner == 3) {
      homeLocalize = new IntPoint(1, 8);
      home = new IntPoint(0,9);
      startingT = 90;
    }
    System.out.println("tngLL"+TNG_LL.x+","+ TNG_LL.y);
    System.out.println("tngUR"+TNG_UR.x+","+ TNG_UR.y);
    // fine tuned launch distances obtained from trial and error lanches
    // each distance is mapped to an appropriate launch speed
    validDistances.add(3.7*TILE_SIZE);
    validDistances.add(5*TILE_SIZE);
    validDistances.add(5.5*TILE_SIZE);
    validDistances.add(6*TILE_SIZE);
    validDistances.add(6.4*TILE_SIZE);
    validDistances.add(6.7*TILE_SIZE);
    validDistances.add(8*TILE_SIZE);
    validDistances.add(10*TILE_SIZE);
    distanceToSpeed.put(3.7*TILE_SIZE, 240);
    distanceToSpeed.put(5*TILE_SIZE, 250);
    distanceToSpeed.put(5.5*TILE_SIZE, 250);
    distanceToSpeed.put(6*TILE_SIZE, 260);
    distanceToSpeed.put(6.4*TILE_SIZE, 280);
    distanceToSpeed.put(6.7*TILE_SIZE, 300);
    distanceToSpeed.put(8*TILE_SIZE, 325);
    distanceToSpeed.put(10*TILE_SIZE, 400);
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
   * @param x grid location
   * @param y grid location
   * @param theta is angle relative to 0 degrees at the start.
   */
  private static void localize(int x, int y, double theta) {
//    initUSSensor();
    SampleProvider usDistance = US_SENSOR.getMode("Distance");
    UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(UltrasonicLocalizer.edgeType.FallingEdge, usDistance);
    usLocalizer.localize();
    sleep();
    closeUSSensor();
    Navigation.moveTo(-2);
    // Navigation.turnTo(-2);
    initLightSensors();
    LightLocalizer lsLocalizer = new LightLocalizer();
    lsLocalizer.localize(x, y, theta);
    sleep();
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
    if (avoider!=null) {
      do {
        navigator.avoided = false;
        navigator.travelTo(tunnelEntr.x, tunnelEntr.y);
      } while(navigator.avoided);
    }
    else {
      System.out.println("going to tunnel");
      navigator.travelTo(tunnelEntr.x, tunnelEntr.y);
    }
    Navigation.moveTo(0.1);
    sleep();
    System.out.println("arrived before tunnel");

    //if obstacle avoidance exists, stop it for the sake of going through the tunnel.
    if (avoider!=null) {
      avoider.stop();
    }

    // find angle of tunnel
    double aveX, aveY, theta;
    aveX = (TNG_LL.x + TNG_UR.x) / 2;
    aveY = (TNG_LL.y + TNG_UR.y) / 2;
    if (navigator instanceof ReLocalizeNavigation) {
      System.out.println("point towards("+aveX+","+aveY+")");
      ((ReLocalizeNavigation) navigator).reLocalize(aveX, aveY);
    } else {
      theta = Navigation.angleToTarget(aveX, aveY);
      Navigation.turnTo(theta);
    }
    sleep();
    initLaunchers();
    LauncherControl.lowerArm();
    Navigation.moveThroughTunnel();
    sleep();
    initLightSensors();
    LauncherControl.raiseArm();
    closeLaunchers();
    sleep();
  }

  /**
   * navigates from the tunnel exit to the launch location.
   *
   * @param navigator to determine navigation type.
   * @param x position to launch from.
   * @param y position to launch from.
   */
  private static void navigateToLaunch(Navigation navigator, double x, double y) {
    System.out.println("(" + BIN.x + "," + BIN.y + ")");
    System.out.println("(" + odometer.getXYT()[0] + "," + odometer.getXYT()[1] + ")");
    do {
      navigator.avoided = false;
      navigator.travelTo(x,y);
    } while(navigator.avoided);
    Navigation.turnTo(Navigation.angleToTarget(BIN.x, BIN.y));
    sleep();
  }

  /**
   * Launches the ping-pong balls a given number of times at a given speed.
   *
   * @param numLaunches is the number of launches to make.
   * @param speed is the motor speed to launch at.
   */
  private static void launch(int numLaunches, int speed) {
    System.out.println("launching "+ numLaunches+" times at "+speed+" speed");
    initLaunchers();
    sleep();
    for (int i = 0; i < numLaunches; ++i) {
      LauncherControl.launch(speed);
    }
    closeLaunchers();
    sleep();
  }

  /**
   * initialises the launcher motors.
   */
  private static void initLaunchers() {
    if (launcher1==null) {
      launcher1 = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
    }
    if (launcher2==null) {
      launcher2 = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
    }
  }

  /**
   * initialises the movement motors.
   */
  private static void initMotors() {
    if (leftMotor == null) {
      leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
    }
    if (rightMotor == null) {
      rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
    }
  }

  /**
   * initialises the light sensors.
   */
  public static void initLightSensors() {
    if (colorSensorR==null) {
      colorSensorR = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
    }
    if (colorSensorL == null) {
      colorSensorL = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
    }
  }

  /**
   * initialises the ultrasonic sensor.
   */
  private static void initUSSensor() {
    if (US_SENSOR == null) {
      US_SENSOR = new EV3UltrasonicSensor(LocalEV3.get().getPort("S3"));
    }
  }

  /**
   * closes the launcher motors.
   */
  private static void closeLaunchers() {
    if (launcher1!=null) {
      launcher1.close();
    }
    if (launcher2!=null) {
      launcher2.close();
    }
    launcher1 = null;
    launcher2 = null;
  }

  /**
   * closes the ultrasonic sensor.
   */
  private static void closeUSSensor() {
    if (US_SENSOR!=null) {
      US_SENSOR.close();
    }
    US_SENSOR = null;
  }

  /**
   * sleeps the thread for 500 ms.
   */
  public static void sleep() {
    try {
      Thread.sleep(500);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }





















  /**
   * flow to test the plain navigation technique.
   */
  private static void plainNavigationTest() {
    double x = 3 * TILE_SIZE, y = 4 * TILE_SIZE;
    navigator = new PlainNavigation();
    navigator.safeArea = new GridRectangle(0, 0, 15, 15);
    navigator.travelTo(x, y);
    System.exit(0);
  }

  /**
   * flow to test the waggle navigation technique.
   */
  private static void waggleNavigationTest() {
    int x = 8, y = 4;
    navigator = new WaggleNavigation();
    navigator.travelTo(x, y, true);
    Navigation.turnToHeading(135);
  }

  /**
   * flow to test the relocalization navigation.
   */
  private static void relocalizationTest() {
    int x=4,y=3;
    navigator = new ReLocalizeNavigation();
    navigator.travelTo(x, y);
    ((ReLocalizeNavigation) navigator).reLocalize(x, y);
  }
  /**
   * flow to test the localization method and time.
   */
  private static void localizationTimeTest() {
    long startTime = System.currentTimeMillis();
    localize(1, 1, startingT);
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
    TNG_LL = new Point(4, 3);
    TNG_UR = new Point(6, 4);
    navigateThroughTunnel(navigator);
  }

  /**
   * flow to test launching at max speed.
   */
  private static void launchTest() {
    int maxSpeed = 1300;
    maxSpeed = 325;
    launch(4, maxSpeed);
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
  /**
   * The flow used for the beta demo.
   */
  @Deprecated
  private static void betaDemo() {
    // set to silent verification
    Resources.SILENT_VERIFICATION = true;
    // import wifi data is done by default
    importData();
    // odometry start
    startOdometer();

    // localize
    localize(1, 1, startingT);
    beep(1);

    // navigate to tunnel
    navigateThroughTunnel(new WaggleNavigation());

    // navigate to specified coords
    // turn to specified angle
    navigateToLaunch(new WaggleNavigation(), BIN.x, BIN.y);
    beep(3);
    // launch
    int maxSpeed = 1300;
    launch(1, maxSpeed);
    beep(1);

    System.exit(0);
  }
}
