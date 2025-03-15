package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final SparkMax IntakeMotor = new SparkMax(14, MotorType.kBrushless);
  private final RelativeEncoder IntakeEncoder = IntakeMotor.getEncoder();
  private final SparkMaxConfig IntakeConfig = new SparkMaxConfig();
  @Override
  public void periodic() {
 SmartDashboard.putNumber(" Intake Position", IntakeEncoder.getPosition());

    SmartDashboard.putNumber(" Intake Velocity", IntakeEncoder.getVelocity());

  }

  public void setMotor(double speed){
    IntakeMotor.set(0.5);

  }
  public double getIntakeEncoder() {
    return IntakeEncoder.getPosition();
  }

  public void resetEncoder() {

    IntakeConfig
    .inverted(true)
    .idleMode(IdleMode.kBrake);
    IntakeMotor.configure(IntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }

  public void setPeriodicStatus() {
    
  }

  
}

