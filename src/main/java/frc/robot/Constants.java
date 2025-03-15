package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  /*public static final Mode currentMode = Mode.REAL;
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
 
  private static final Double WHEEL_RADIUS = Units.inchesToMeters(2.0) ;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              WHEEL_RADIUS,
              //maxSpeedMetersPerSec,
              wheelCOF,
              //driveGearbox.withReduction(driveMotorReduction),
              //driveMotorCurrentLimit,
              1),
              //moduleTranslations);*/

  
  public static enum Mode {
    /** Running on a real robot. */
    REAL,
    

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  } 
} 
