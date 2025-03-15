package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;

public class IntakeCommands extends Command{
    private final Intake intakeSubsystem;
  private final double intakespeed;

  public IntakeCommands(Intake intakeSubsystem, double intakespeed) {
    this.intakeSubsystem = intakeSubsystem;
    this.intakespeed = 1;
    addRequirements(intakeSubsystem); 
  }

  @Override
  public void initialize() {
    System.out.println("ArmCommand started!");
  }

  @Override
  public void execute() {
    intakeSubsystem.setMotor(intakespeed);
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setMotor(0);
    System.out.println("stopped!!");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
     


