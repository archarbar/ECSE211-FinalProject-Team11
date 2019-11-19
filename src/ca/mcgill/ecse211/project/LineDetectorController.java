package ca.mcgill.ecse211.project;

/**
 * A class that implements LineDetectorController should use a single color sensor in order to detect lines. The
 * LineDetectorController interface provides two methods for detecting lines. Either detecting a full line, or detecting
 * an edge. The interface also contains two methods for determining the location of the most recent edge. The interface
 * has one last method that is used to process the light value that was detected.
 * 
 * @author Matthew
 *
 */
public interface LineDetectorController {
  /**
   * The Edge enum contains the 3 different possible returns of the edgeDetected method.
   * 
   * @author Matthew
   */
  public enum Edge {
  NoEdge, RisingEdge, FallingEdge
  }

  /**
   * Determines if a line has been detected.
   * 
   * @return true if a line has been detected.
   */
  public boolean lineDetected();

  /**
   * Determines if an edge has been detected, and returns an Edge value depending on what was detected.
   * 
   * @return Either a rising edge or falling edge if either were detected, or NoEdge.
   */
  public Edge edgeDetected();

  /**
   * Interprets a light value depending on the chosen implementation.
   * 
   * @param lightVal the most recent light value detected
   * @return the processed data
   */
  public double processCSData(double lightVal);

  /**
   * Finds the X location of the last detected edge.
   * 
   * @return X location in cm
   */
  public double getEdgeX();

  /**
   * Finds the Y location of the last detected edge.
   * 
   * @return Y location in cm
   */
  public double getEdgeY();

}
