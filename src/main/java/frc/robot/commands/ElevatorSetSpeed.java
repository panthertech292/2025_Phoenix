// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorSetSpeed extends Command {
  private ElevatorSubsystem ElevatorSub;
  private double elvSpeed;
  /** Creates a new ElevatorSetSpeed. */
  public ElevatorSetSpeed(ElevatorSubsystem Elevator_Subsystem, double speed) {
    ElevatorSub = Elevator_Subsystem;
    elvSpeed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ElevatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ElevatorSub.setElevator(elvSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ElevatorSub.setElevator(0);
    ElevatorSub.setHeightToCurrentPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
