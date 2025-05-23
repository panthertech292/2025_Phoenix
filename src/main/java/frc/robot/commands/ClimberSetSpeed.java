// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberSetSpeed extends Command {
  private ClimberSubsystem ClimberSub;
  private double speed;
  /** Creates a new ClimberSetSpeed. */
  public ClimberSetSpeed(ClimberSubsystem Climber_Subsystem, double Climber_Speed) {
    ClimberSub = Climber_Subsystem;
    speed = Climber_Speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ClimberSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ClimberSub.setClimb(speed);
    if(speed > 0){
      ClimberSub.lockServo();
    }else{
      ClimberSub.unLockServo();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ClimberSub.setClimb(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
