package ca.mcgill.ecse211.project;

import lejos.robotics.SampleProvider;

/**
 * A simple line detector that detects lines based on a threshold proportional to its initial value.
 * 
 * @author Matthew
 */
public class ProportionalLineDetector implements LineDetectorController {

  // proportional
  public final double THRESHOLD = 0.85;
  private double oldVal;

  private static final double MAX_DISTANCE = 3;
  private float[] sample;
  SampleProvider cs;
  Edge lastEdge = Edge.NoEdge;
  Double edgeX;
  Double edgeY;

  /**
   * Create a ProportionalLineDetector with a lightSensor's sample provider.
   * 
   * @param cs
   */
  public ProportionalLineDetector(SampleProvider cs) {
    lastEdge = Edge.NoEdge;
    sample = new float[cs.sampleSize()];
    cs.fetchSample(sample, 0);
    oldVal = sample[0];
  }

  @Override
  public boolean lineDetected() {
    return edgeDetected() == Edge.FallingEdge;
  }

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

  @Override
  public double getEdgeX() {
    return edgeX;
  }

  @Override
  public double getEdgeY() {
    return edgeY;
  }

}
