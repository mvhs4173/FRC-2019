package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robot;

// SmoothJoystck is a drop-in replacement for Joystick that will
// not let the X, Y, and Throttle values change by more than set
// limits each time it is called.
// It also can square the raw joystick x and y values to make low-speed
// control easier.  It does the squaring before limiting the delta.
// It overrides the getX, getY, and getThrottle methods from Joystick
// but should leave everything else in Joystick alone.
//
// To use, change a line in IO.java from
//    public static Joystick joy = new Joystick(0);
// to
//    public static SmoothedJoystick joy = new SmoothedJoystick(
//             0,    // the port the joystick is plugged into (same as in old Joystick(0))
//             true, // do square the raw x and y values from joystick
//            -2., 2., // maximum allowed change in x (since x is in [-1,1] this means no limit)
//          -0.02, 0.02, // maximum allowed change in y (1 second from 0 to full throttle and reverse)
//            -2., 2.); // maximum allowed change in throttle
// Then when you use 'joy', don't square x and y and don't try to limit accelerations -
//   the output of joy.getX() and joy.getY() will already be squared and limited.

public class SmoothedJoystick extends Joystick {
  private boolean squareXY;
  private double previousX;
  private double previousY;
  private double previousThrottle;
  private double minDeltaX; // min allowed change in [possibly squared] X (generally negative)
  private double maxDeltaX; // max allowsed change in  [possibly squared]X (generally positive)
  private double minDeltaY; // min allowed change in [possibly squared] Y (generally negative)
  private double maxDeltaY; // max allowsed change in [possibly squared] Y (generally positive)
  private double minDeltaThrottle; // min allowed change in [possibly squared] Throttle (generally negative)
  private double maxDeltaThrottle; // max allowsed change in [possibly squared] Throttle (generally positive)
  
  /** The constructor: pass in a port number along with
  //   @param squareXY : true if you want raw joystick x and y squared
  //   @param minDeltaX, maxDeltaX : min and max change allowed in X from one call to the next
  //   @param minDeltaY, maxDeltaY : min and max change allowed in Y from one call to the next
  //   @param minDeltaThrottle, maxDeltaThrottle : min and max change allowed in Throttle from one call to the next
  //   If you don't want to limit changes, just pass in numbers that are bigger that the largest possible change,
  //     e.g., -2.0 and 2.0 if the range of values is -1 to 1.
  //   In FRC robot, this will be called c. 50 times a second, so if you want to make change
  //     from full speed ahead to 0 in 1 second use minDeltaY=-0.02 (assuming positive y from joystick
  //     indicates to move forward)**/
  public SmoothedJoystick(int joystick_port_number, boolean squareXY_arg,
    double minDeltaX_arg, double maxDeltaX_arg,
    double minDeltaY_arg, double maxDeltaY_arg,
    double minDeltaThrottle_arg, double maxDeltaThrottle_arg)
  {
    super(joystick_port_number); // initialize the raw joystick stuff
    squareXY = squareXY_arg;
    previousX = transformXY(super.getX());
    previousY = transformXY(super.getY());
    previousThrottle = super.getThrottle();
    minDeltaX = minDeltaX_arg;
    maxDeltaX = maxDeltaX_arg;
    minDeltaY = minDeltaY_arg;
    maxDeltaY = maxDeltaY_arg;
    minDeltaThrottle = minDeltaThrottle_arg;
    maxDeltaThrottle = maxDeltaThrottle_arg;
  }

  // limitRange: if v is outside of range, push it to closest point in range
  private static double limitRange(double v, double minV, double maxV)
  {
    return Math.min( Math.max(v, minV), maxV) ;
  }

  // limitDelta: user wants to change previousV to newV, but does not want
  //   change to be ouside of range.
  private static double limitDelta(double previousV, double newV, double minDeltaV, double maxDeltaV)
  {
    return previousV + limitRange(newV - previousV, minDeltaV, maxDeltaV) ;
  }

  // transformXY: square or don't square the x and y value, depending on value of squareXY
  private double transformXY(double v)
  {
    // Math.copySign(x, y) returns x with the sign of y, y>=0 ? Math.abs(x) : -Math.abs(x).
    return squareXY ? Math.copySign(Math.pow(v, 2), v) : v ;
  }

  // The the current joystick x value, possible squared, and with a limited difference from the previous one
  //@Override
  public double getXValue() {
    minDeltaX = Robot.prefs.getDouble("minDeltaX", 0);
    maxDeltaX = Robot.prefs.getDouble("maxDeltaX", 0);
    double newX = transformXY(super.getX());
    previousX = limitDelta(previousX, newX, minDeltaX, maxDeltaX);
    return previousX;
  }

  // The the current joystick y value, possible squared, and with a limited difference from the previous one
  //@Override
  public double getYValue() {
    minDeltaY = Robot.prefs.getDouble("minDeltaY", 0);
    maxDeltaY = Robot.prefs.getDouble("maxDeltaY", 0);
    double newY = transformXY(super.getY());
    previousY = limitDelta(previousY, newY, minDeltaY, maxDeltaY);
    return previousY;
  }

  // The the current joystick throttle value, possible squared, and with a limited difference from the previous one
  @Override
  public double getThrottle()
  {
    minDeltaThrottle = Robot.prefs.getDouble("minDeltaThrottle", 0);
    maxDeltaThrottle = Robot.prefs.getDouble("maxDeltaThrottle", 0);
    double newThrottle = super.getThrottle();
    previousThrottle = limitDelta(previousThrottle, newThrottle, minDeltaThrottle, maxDeltaThrottle);
    return previousThrottle;
  }
}
