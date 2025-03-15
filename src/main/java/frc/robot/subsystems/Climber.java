package frc.robot.subsystems;






import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;


import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final SparkMax ClimberMotor = new SparkMax(15, MotorType.kBrushless);
  private final RelativeEncoder ClimberEncoder = ClimberMotor.getEncoder();
  private final SparkMaxConfig config = new SparkMaxConfig();

  @Override
  public void periodic() {
    SmartDashboard.putNumber(" Climber Encoder", ClimberEncoder.getPosition());

    SmartDashboard.putNumber(" Climber Velocity", ClimberEncoder.getVelocity());
  }

  public void setMotor(double speed) {
    ClimberMotor.set(speed);
  }

  public double getLeftEncoder() {
    return ClimberEncoder.getPosition();
  }

  public void resetEncoder() {
    ClimberEncoder.setPosition(0);
    config
    .inverted(true)
    .idleMode(IdleMode.kBrake);
    
    
  }

  public void setPeriodicStatus() {

config.signals.primaryEncoderPositionPeriodMs(10);
config.signals.appliedOutputPeriodMs(10);
config.signals.faultsPeriodMs(10);
config.signals.motorTemperaturePeriodMs(20);


ClimberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    /*ClimberMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    ClimberMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    ClimberMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    ClimberMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50);
    ClimberMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 20);
    ClimberMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 200);
    ClimberMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 200);*/
  }
}
