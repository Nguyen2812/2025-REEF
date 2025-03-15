package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakePIDCommand extends Command{
private final Intake intakeSubsystem;
  private final PIDController pidController;

  public IntakePIDCommand(Intake intakeSubsystem, double setpoint) {
    this.intakeSubsystem = intakeSubsystem;
    this.pidController = new PIDController(0.05, 0, 0);
    pidController.setSetpoint(setpoint);
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("IntakePID starting!!!!!!!! ");
    pidController.reset();
  }

  @Override
  public void execute() {
    double speed = pidController.calculate(intakeSubsystem.getIntakeEncoder());
    intakeSubsystem.setMotor(speed);
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setMotor(0);
    System.out.println("ArmPIDCmd ended!");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}


