// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorHeights;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Intake extends Command {
  private ElevatorSubsystem ElevatorSub;
  private ShooterSubsystem ShooterSub;
  /** Creates a new Intake. */
  public Intake(ElevatorSubsystem Elevator_Subsystem, ShooterSubsystem Shooter_Subsystem) {
    ElevatorSub = Elevator_Subsystem;
    ShooterSub = Shooter_Subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ElevatorSub, ShooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ShooterSub.setRollers(.10);
    ElevatorSub.setElevatorSetPoint(ElevatorHeights.LOAD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ElevatorSub.runElevatorFromSetHeight();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterSub.setRollers(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ShooterSub.coralIsInShooter();
  }
}
