package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberPIDCommand extends Command {
  private final Climber climberSubsystem;
  private final PIDController pidController;

  public ClimberPIDCommand(Climber climberSubsystem, double setpoint) {
    this.climberSubsystem = climberSubsystem;
    this.pidController = new PIDController(0.05, 0, 0);
    pidController.setSetpoint(setpoint);
    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("ArmPIDCmd started!");
    pidController.reset();
  }

  @Override
  public void execute() {
    double speed = pidController.calculate(climberSubsystem.getLeftEncoder());
    climberSubsystem.setMotor(speed);
  }

  @Override
  public void end(boolean interrupted) {
    climberSubsystem.setMotor(0);
    System.out.println("ArmPIDCmd ended!");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
