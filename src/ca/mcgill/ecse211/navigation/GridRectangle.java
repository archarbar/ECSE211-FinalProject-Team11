package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.project.Resources.Point;

public class GridRectangle {
  private int xLow, xHigh, yLow, yHigh;

  public GridRectangle(int x1, int y1, int x2, int y2) {
    if (x1 < x2) {
      xLow = x1;
      xHigh = x2;
    } else {
      xLow = x2;
      xHigh = x1;
    }

    if (y1 < y2) {
      yLow = y1;
      yHigh = y2;
    } else {
      yLow = y2;
      yHigh = y1;
    }

  }

  /**
   * @return the xLow
   */
  public int getxLow() {
    return xLow;
  }

  /**
   * @return the xHigh
   */
  public int getxHigh() {
    return xHigh;
  }

  /**
   * @return the yLow
   */
  public int getyLow() {
    return yLow;
  }

  /**
   * @return the yHigh
   */
  public int getyHigh() {
    return yHigh;
  }

  public boolean contains(Point point) {
    return (point.x <= getxHigh() && point.x >= getxLow() && point.y <= getyHigh() && point.y >= getyLow());
  }
}
