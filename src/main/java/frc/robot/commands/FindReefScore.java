// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Constants.ElevatorConstants.ElevatorHeights;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FindReefScore extends SequentialCommandGroup {
  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  /** Creates a new FindReef. */
  public FindReefScore(CommandSwerveDrivetrain Drive_Subsystem, ShooterSubsystem Shooter_Substem, ElevatorSubsystem Elevator_Subsystem, ElevatorHeights height) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.runOnce(() -> Elevator_Subsystem.setElevatorSetPoint(ElevatorHeights.SENSE), Elevator_Subsystem),
      //Drive_Subsystem.applyRequest(() -> robotCentricDrive.withVelocityY(-0.5)).until(Drive_Subsystem.againstReef()),
      new WaitUntilElevatorAtHeight(Elevator_Subsystem),
      Drive_Subsystem.applyRequest(() -> robotCentricDrive.withVelocityX(0.35)).until(Shooter_Substem.reefIsInFrontRobot()).withTimeout(0.5),
      Drive_Subsystem.applyRequest(() -> robotCentricDrive.withVelocityX(-0.35)).until(Shooter_Substem.reefIsInFrontRobot()).withTimeout(1.25),
      Drive_Subsystem.applyRequest(() -> robotCentricDrive.withVelocityX(0.35)).until(Shooter_Substem.reefIsInFrontRobot()).withTimeout(0.5),
      Drive_Subsystem.applyRequest(() -> brake).withTimeout(0.1),
      new autoScoreEndless(Elevator_Subsystem, Shooter_Substem, height)
      //new autoScoreIfAligned(Elevator_Subsystem, Shooter_Substem, height)
    );
  }
}
