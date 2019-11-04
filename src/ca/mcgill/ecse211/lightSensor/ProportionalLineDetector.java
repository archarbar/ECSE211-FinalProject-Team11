package ca.mcgill.ecse211.lightSensor;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.robotics.SampleProvider;

public class ProportionalLineDetector implements LineDetectorController {

  //proportional
  public final double THRESHOLD = 0.1;
  private double oldVal;
  
  private static final double MAX_DISTANCE = 3;
  private float[] sample;
  SampleProvider cs;
  Edge lastEdge = Edge.NoEdge;
  Double edgeX;
  Double edgeY;
  
  public ProportionalLineDetector(SampleProvider cs) {
    lastEdge = Edge.NoEdge;
    sample = new float[cs.sampleSize()];
    cs.fetchSample(sample, 0);
    oldVal = sample[0];
  }
  @Override
  public boolean lineDetected() {
    return edgeDetected()==Edge.FallingEdge;
  }

  @Override
  public Edge edgeDetected() {
    cs.fetchSample(sample, 0);
    if(sample[0]<0.85*oldVal) {
      double[] position = Odometer.getOdometer().getXYT();
      edgeX = position[0];
      edgeY = position[1];
      return Edge.FallingEdge;
    }
    return Edge.NoEdge;
  }

  @Override
  public double processCSData(double lightVal) {
    // TODO Auto-generated method stub
    return 0;
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
