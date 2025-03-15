package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.CANcoder;



import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;

import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {
  // Gear ratios for SDS MK4i L2, adjust as necessary
  private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private static final double TURN_GEAR_RATIO = (150 / 7.0);

  private final SparkMax driveSparkMax;
  private final SparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  // private final AnalogInput turnAbsoluteEncoder;
  private final CANcoder turnAbsoluteEncoder;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkMax(int index) {
    switch (index) {
      case 0: // FL
        driveSparkMax = new SparkMax(3, MotorType.kBrushless);
        turnSparkMax = new SparkMax(4, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(12);
        absoluteEncoderOffset =
            new Rotation2d(Units.rotationsToRadians(0.162842)); // MUST BE CALIBRATED
        break;
      case 1: // FR
        driveSparkMax = new SparkMax(1, MotorType.kBrushless);
        turnSparkMax = new SparkMax(2, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(11);
        absoluteEncoderOffset =
            new Rotation2d(Units.rotationsToRadians(-0.430420)); // MUST BE CALIBRATED
        break;
      case 2: // BL
        driveSparkMax = new SparkMax(5, MotorType.kBrushless);
        turnSparkMax = new SparkMax(6, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(10);
        absoluteEncoderOffset =
            new Rotation2d(Units.rotationsToRadians(0.465820)); // MUST BE CALIBRATED
        break;
      case 3: // BR
        driveSparkMax = new SparkMax(7, MotorType.kBrushless);
        turnSparkMax = new SparkMax(8, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(9);
        absoluteEncoderOffset =
            new Rotation2d(Units.rotationsToRadians(-0.289795)); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    SparkMaxConfig DriveConfig = new SparkMaxConfig();

//DRRRRRRRRRRRRRRRRRRRRRRIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIVVVVVVVVVVVVVVVVVVVVVEEEEEEEEEEEEEEEEEEEEE
DriveConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake);
    
DriveConfig.encoder
    .positionConversionFactor(1000)
    .velocityConversionFactor(1000);
    
DriveConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(1.0, 0.0, 0.0);
DriveConfig.
      smartCurrentLimit(40);
      setDriveVoltage(10);  
DriveConfig.voltageCompensation(12);



driveSparkMax.configure(DriveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); //applying the config above 

 //TURRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRNNNN
   SparkMaxConfig TurnConfig = new SparkMaxConfig();

TurnConfig
    .inverted(true)
    .idleMode(IdleMode.kBrake);
TurnConfig.absoluteEncoder
    .averageDepth(2)
    .positionConversionFactor(1000)
    .velocityConversionFactor(1000);

    
TurnConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(1.0, 0.0, 0.0);
TurnConfig.
    smartCurrentLimit(12);
    setDriveVoltage(6);
TurnConfig.voltageCompensation(12);

turnSparkMax.configure(TurnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turnAbsoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());

    driveSparkMax.setCANTimeout(250);
    turnSparkMax.setCANTimeout(250);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    //turnSparkMax.setInverted(isTurnMotorInverted);
    
 

    driveEncoder.setPosition(0.0);
    //driveEncoder.setMeasurementPeriod(10);
   

    turnRelativeEncoder.setPosition(0.0);
   // turnRelativeEncoder.setMeasurementPeriod(10);
   

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);


    DriveConfig.signals
    .primaryEncoderPositionPeriodMs( (int)(1000/ Module.ODOMETRY_FREQUENCY));   // Previously status 2

    DriveConfig.signals     
         .appliedOutputPeriodMs(10) //Previously 0 
          .primaryEncoderVelocityPeriodMs(20)  // Previously status 1
          .analogVoltagePeriodMs(50);  //Previously status 3 
    DriveConfig.signals
        .faultsPeriodMs(20);
//SPLITTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT

TurnConfig.signals
    .primaryEncoderPositionPeriodMs( (int)(1000/ Module.ODOMETRY_FREQUENCY));   // Previously status 2

    TurnConfig.signals     
         .appliedOutputPeriodMs(10) //Previously 0 
          .primaryEncoderVelocityPeriodMs(20)  // Previously status 1
          .analogVoltagePeriodMs(50);  //Previously status 3 
    TurnConfig.signals
        .faultsPeriodMs(20);
           
          

    /*driveSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));

    driveSparkMax.setPeriodi cFramePeriod(PeriodicFrame.kStatus0, 10);
    driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50);
    driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 20);
    driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 200);
    driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 200);*/
    // driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 500);

   /* turnSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));*/


    timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(driveEncoder::getPosition);
    turnPositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(turnRelativeEncoder::getPosition);

    
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

    inputs.turnAbsolutePosition =
        new Rotation2d(turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI)
            .minus(absoluteEncoderOffset);

    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value / TURN_GEAR_RATIO))
            .toArray(Rotation2d[]::new);

    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  

  }

  
  
  

