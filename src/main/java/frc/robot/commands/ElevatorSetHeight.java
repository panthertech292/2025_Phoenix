// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorHeights;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorSetHeight extends Command {
  private ElevatorSubsystem ElevatorSub;
  private ElevatorHeights height;
  /** Creates a new ElevatorSetSpeed. */
  public ElevatorSetHeight(ElevatorSubsystem Elevator_Subsystem, ElevatorHeights Height_SetPoint) {
    ElevatorSub = Elevator_Subsystem;
    height = Height_SetPoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ElevatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ElevatorSub.setElevatorSetPoint(height);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   ElevatorSub.runElevatorFromSetHeight();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ElevatorSub.setElevator(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
