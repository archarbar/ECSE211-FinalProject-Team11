package ca.mcgill.ecse211.project;

import lejos.robotics.SampleProvider;

/**
 * A simple line detector that detects lines based on a threshold proportional to its initial value.
 * 
 * @author Matthew
 */
public class ProportionalLineDetector implements LineDetectorController {

  /**
   * proportional threshold for light value
   */
  public final double THRESHOLD = 0.85;
  
  /**
   * old value read by light sensor
   */
  private double oldVal;

  /**
   * sample containing values read by light sensor
   */
  private float[] sample;
  
  /**
   * sample provider for the light sensor
   */
  SampleProvider cs;
  Edge lastEdge = Edge.NoEdge;
  
  /**
   * x position of edge in cm
   */
  Double edgeX;
  
  /**
   * y position of edge in cm
   */
  Double edgeY;

  /**
   * Create a ProportionalLineDetector with a lightSensor's sample provider.
   * 
   * @param cs is the colour sensor to use.
   */
  public ProportionalLineDetector(SampleProvider cs) {
    lastEdge = Edge.NoEdge;
    sample = new float[cs.sampleSize()];
    cs.fetchSample(sample, 0);
    oldVal = sample[0];
  }

  /**
   * check if the edge detected is falling or not. if it is, then a line is detected
   * 
   * @return true if a line is detected, false if not
   */
  @Override
  public boolean lineDetected() {
    return edgeDetected() == Edge.FallingEdge;
  }

  /**
   * method to know what kind of edge is detected
   * 
   * @return type of Edge, falling or rising
   */
  @Override
  public Edge edgeDetected() {
    cs.fetchSample(sample, 0);
    if (processCSData(sample[0]) < THRESHOLD) {
      double[] position = Odometer.getOdometer().getXYT();
      edgeX = position[0];
      edgeY = position[1];
      return Edge.FallingEdge;
    }
    return Edge.RisingEdge;
  }

  /**
   * @return the proportion of the new light value to the original value.
   */
  @Override
  public double processCSData(double lightVal) {
    return lightVal / oldVal;
  }

  /**
   * @return x position of edge
   */
  @Override
  public double getEdgeX() {
    return edgeX;
  }

  /**
   * @return y position of edge
   */
  @Override
  public double getEdgeY() {
    return edgeY;
  }

}
