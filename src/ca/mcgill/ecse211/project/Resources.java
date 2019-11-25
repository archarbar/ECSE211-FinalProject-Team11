package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.wificlient.WifiConnection;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Contains all of the constants and many of the variables shared by multiple classes, as well as containing all the
 * resources imported over wifi.
 */
public class Resources {
  /**
   * The default server IP used by the profs and TA's.
   */
  public static final String DEFAULT_SERVER_IP = "192.168.2.53";

  /**
   * The IP address of the server that transmits data to the robot. Set this to the default for the beta demo and
   * competition.
   */
  public static final String SERVER_IP = "192.168.2.45";

  /**
   * Your team number.
   */
  public static final int TEAM_NUMBER = 11;

  /**
   * Enables printing of debug info from the WiFi class.
   */
  public static final boolean ENABLE_DEBUG_WIFI_PRINT = false;

  /**
   * Enable this to attempt to receive Wi-Fi parameters at the start of the program.
   */
  public static final boolean RECEIVE_WIFI_PARAMS = false;

  // DECLARE YOUR CURRENT RESOURCES HERE
  // eg, motors, sensors, constants, etc

  /**
   * The launcher motor 1.
   */
  public static EV3LargeRegulatedMotor launcher1 = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  /**
   * The launcher motor 2.
   */
  public static EV3LargeRegulatedMotor launcher2 = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  /**
   * Theangle at which launching motor is placed before launch.
   */
  public static final int LAUNCH_ANGLE = 55;

  /**
   * The height of the launcher motor when extended.
   */
  public static final int LAUNCH_HEIGHT = 35;

  /**
   * The length of the launcher arm.
   */
  public static final double LAUNCH_ARM_LENGTH = 18;

  /**
   * Gravitational constant.
   */
  public static final double G = 9.81;


  /**
   * the robot's wheel radius
   */

  public static final double WHEEL_RAD = 2.128; // temporary value 2.148

  /**
   * the robot's track size
   */
  public static final double TRACK = 15.62; // best value now 15.62

  /**
   * the distance at which we want to perform the launch
   *
   */
  public static final double LAUNCH_DISTANCE = 125; // temporary value

  /**
   * The size of each tile in cm.
   */
  public static final double TILE_SIZE = 30.48;
  /**
   * Minimum distance to launch from.
   */
  public static final double TARGET_DISTANCE = 4 * TILE_SIZE;

  /**
   * The left motor of the robot used for moving.
   */
  public static EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

  /**
   * The right motor of the robot used for moving.
   */
  public static EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));

  /**
   * The Ultrasonic Sensor.
   */
  public static EV3UltrasonicSensor US_SENSOR = new EV3UltrasonicSensor(LocalEV3.get().getPort("S3"));

  /**
   * The right light sensor.
   */
  public static EV3ColorSensor colorSensorR = null;

  /**
   * The left light sensor.
   */
  public static EV3ColorSensor colorSensorL = null;

  /**
   * The Correction period for the light sensor in milliseconds.
   */
  public static final int CORRECTION_PERIOD = 60;

  /**
   * The object detection period for the US sensor.
   */
  public static final int AVOIDANCE_PERIOD = 50;
  
  /**
   * back off distance for robot in cm
   */
  public static final int REVERSE_DIST = 6;

  /**
   * The motor acceleration in degrees per second squared.
   */
  public static final int LAUNCHER_ACCELERATION = 10000;

  /**
   * The angle which the launcher rotates by in degrees.
   */
  public static final int LAUNCHER_ANGLE = 105;

  /**
   * Extra angle to lower the launcher by.
   */
  public static final int extra_angle = 15;

  /**
   * Angle to lower launcher arm by in order to pass through the tunnel.
   */
  public static final int LOWER_ANGLE = 70;

  /**
   * Speed to lower arm in degrees/s.
   */
  //public static final int LOWER_SPEED = 50;
  public static final int LOWER_SPEED = 300;

  /**
   * The speed at which the launcher rotates in degrees per second.
   */
  public static final int LAUNCH_SPEED = 326;
  // public static final int LAUNCH_SPEED = 1300;
  /**
   * The speed at which the launcher rotates in degrees per second.
   */
  // public static final int RESET_SPEED = 70;
  public static final int RESET_SPEED = 125;

  public static List<Double> validDistances = new ArrayList<>();
  public static Map<Double, Integer> distanceToSpeed = new HashMap<>();

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

    /**
     * Constructs a Point.
     * @param x the x grid coordinate
     * @param y the y grid coordinate
     */
    public Point(int x, int y) {
      this.x = x * TILE_SIZE;
      this.y = y * TILE_SIZE;
    }

    /**
     * Constructs a Point from another point.
     * @param p another point
     */
    public Point(Point p) {
      this(p.x, p.y);
    }

    /**
     * Constructs a Point from an IntPoint.
     * @param p the integer point
     */
    public Point(IntPoint p) {
      this(p.x, p.y);
    }

    public String toString() {
      return "(" + x + ", " + y + ")";
    }

  }

  /**
   * Represents an integer value coordinate point on the competition map grid.
   *
   * @author Matthew Williams
   */
  public static class IntPoint {
    /** The x coordinate. */
    public int x;

    /** The y coordinate. */
    public int y;

    /**
     * Constructs an IntPoint.
     *
     * @param x the x coordinate
     * @param y the y coordinate
     */
    public IntPoint(int x, int y) {
      this.x = x;
      this.y = y;
    }

    /**
     * Constructs an IntPoint.
     * @param x the x coordinate
     * @param y the y coordinate
     */
    public IntPoint(double x, double y) {
      this.x = (int) (x / TILE_SIZE);
      this.y = (int) (y / TILE_SIZE);
    }

    /**
     * Constructs an IntPoint from another IntPoint.
     * @param p another point
     */
    public IntPoint(IntPoint p) {
      this(p.x, p.y);
    }
    /**
     * Constructs an IntPoint from a Point.
     * @param p the integer point
     */
    public IntPoint(Point p) {
      this(p.x, p.y);
    }

    /**
     * converts coordinates to string
     * @return string of coordinates
     */
    public String toString() {
      return "(" + x + ", " + y + ")";
    }

  }

  /**
   * The motor acceleration in degrees per second squared.
   */
  public static final int ACCELERATION = 3000;

  /**
   * Distance between light sensors in cm.
   */
  public static final double LSwidth = 11.6;

  // public static final int ROTATE_SPEED = 175;
  // public static final int MOTOR_SPEED = 250;
  /**
   * Speed of motors when turning in degrees/s.
   */
  public static final int ROTATE_SPEED = 150;
  /**
   * Speed of motors when moving straight in degrees/s.
   */
  public static final int MOTOR_SPEED = 150;

  /**
   * Offset of the light sensors perpendicular to the wheelbase.
   */
  public static final double sensorOffset = 3.5;

  /**
   * Boolean for whether to play debug noises.
   */
  public static boolean SILENT_VERIFICATION = false;

  /**
   * Container for the Wi-Fi parameters.
   */
  public static Map<String, Object> wifiParameters;

  // This static initializer MUST be declared before any Wi-Fi parameters.
  static {
    receiveWifiParameters();
  }

  /**
   * Red team number.
   */
  public static int redTeam = get("RedTeam");

  /**
   * Red team's starting corner.
   */
  public static int redCorner = get("RedCorner");

  /**
   * Green team number.
   */
  public static int greenTeam = get("GreenTeam");

  /**
   * Green team's starting corner.
   */
  public static int greenCorner = get("GreenCorner");

  /**
   * The Red Zone.
   */
  public static Region red = new Region("Red_LL_x", "Red_LL_y", "Red_UR_x", "Red_UR_y");
  public static GridRectangle redRectangle =
      new GridRectangle(get("Red_LL_x"), get("Red_LL_y"), get("Red_UR_x"), get("Red_UR_y"));

  /**
   * The Green Zone.
   */
  public static Region green = new Region("Green_LL_x", "Green_LL_y", "Green_UR_x", "Green_UR_y");
  public static GridRectangle greenRectangle =
      new GridRectangle(get("Green_LL_x"), get("Green_LL_y"), get("Green_UR_x"), get("Green_UR_y"));

  /**
   * The Island.
   */
  public static Region island = new Region("Island_LL_x", "Island_LL_y", "Island_UR_x", "Island_UR_y");
  public static GridRectangle islandRectangle =
      new GridRectangle(get("Island_LL_x"), get("Island_LL_y"), get("Island_UR_x"), get("Island_UR_y"));

  /**
   * The red tunnel footprint.
   */
   public static Region tnr = new Region("TNR_LL_x", "TNR_LL_y", "TNR_UR_x", "TNR_UR_y");
   public static GridRectangle tnrRectangle =
   new GridRectangle(get("TNR_LL_x"), get("TNR_LL_y"), get("TNR_UR_x"), get("TNR_UR_y"));

  /**
   * The green tunnel footprint.
   */
  public static Region tng = new Region("TNG_LL_x", "TNG_LL_y", "TNG_UR_x", "TNG_UR_y");
  public static GridRectangle tngRectangle =
      new GridRectangle(get("TNG_LL_x"), get("TNG_LL_y"), get("TNG_UR_x"), get("TNG_UR_y"));

  /**
   * The location of the red target bin.
   */
  public static Point redBin = new Point(get("Red_BIN_x"), get("Red_BIN_y"));

  /**
   * The location of the green target bin.
   */
  public static Point greenBin = new Point(get("Green_BIN_x"), get("Green_BIN_y"));

  /**
   * Receives Wi-Fi parameters from the server program.
   */
  public static void receiveWifiParameters() {
    // Only initialize the parameters if needed
    if (!RECEIVE_WIFI_PARAMS || wifiParameters != null) {
      return;
    }
    System.out.println("Waiting to receive Wi-Fi parameters.");

    // Connect to server and get the data, catching any errors that might occur
    try (WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT)) {
      /*
       * getData() will connect to the server and wait until the user/TA presses the "Start" button in the GUI on their
       * laptop with the data filled in. Once it's waiting, you can kill it by pressing the upper left hand corner
       * button (back/escape) on the EV3. getData() will throw exceptions if it can't connect to the server (e.g. wrong
       * IP address, server not running on laptop, not connected to WiFi router, etc.). It will also throw an exception
       * if it connects but receives corrupted data or a message from the server saying something went wrong. For
       * example, if TEAM_NUMBER is set to 1 above but the server expects teams 17 and 5, this robot will receive a
       * message saying an invalid team number was specified and getData() will throw an exception letting you know.
       */
      wifiParameters = conn.getData();
    } catch (Exception e) {
      System.err.println("Error: " + e.getMessage());
    }
  }

  /**
   * Returns the Wi-Fi parameter int value associated with the given key.
   *
   * @param key the Wi-Fi parameter key
   * @return the Wi-Fi parameter int value associated with the given key
   */
  public static int get(String key) {
    if (wifiParameters != null) {
      return ((BigDecimal) wifiParameters.get(key)).intValue();
    } else {
      return 0;
    }
  }

  /**
   * Represents a region on the competition map grid, delimited by its lower-left and upper-right corners (inclusive).
   *
   * @author Younes Boubekeur
   */
  public static class Region {
    /** The lower left corner of the region. */
    public Point ll;

    /** The upper right corner of the region. */
    public Point ur;

    /**
     * Constructs a Region.
     *
     * @param lowerLeft the lower left corner of the region
     * @param upperRight the upper right corner of the region
     */
    public Region(Point lowerLeft, Point upperRight) {
      validateCoordinates(lowerLeft, upperRight);
      ll = lowerLeft;
      ur = upperRight;
    }

    /**
     * Helper constructor to make a Region directly from parameter names.
     *
     * @param llX the Wi-Fi parameter key representing the lower left corner of the region x coordinate
     * @param llY the Wi-Fi parameter key representing the lower left corner of the region y coordinate
     * @param urX the Wi-Fi parameter key representing the upper right corner of the region x coordinate
     * @param urY the Wi-Fi parameter key representing the upper right corner of the region y coordinate
     */
    public Region(String llX, String llY, String urX, String urY) {
      this(new Point(get(llX), get(llY)), new Point(get(urX), get(urY)));
    }

    /**
     * Validates coordinates.
     *
     * @param lowerLeft the lower left corner of the region
     * @param upperRight the upper right corner of the region
     */
    private void validateCoordinates(Point lowerLeft, Point upperRight) {
      if (lowerLeft.x > upperRight.x || lowerLeft.y > upperRight.y) {
        throw new IllegalArgumentException("Upper right cannot be below or to the left of lower left!");
      }
    }

    public String toString() {
      return "[" + ll + ", " + ur + "]";
    }
  }

}
