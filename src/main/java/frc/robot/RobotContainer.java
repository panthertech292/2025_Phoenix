// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants.ElevatorHeights;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DefaultElevator;
import frc.robot.commands.ElevatorSetSpeed;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

public class RobotContainer {
  //Joysticks
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  //Subsystems
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  //Drive Stuff
  //Drive Config
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
  public RobotContainer() {
    CameraServer.startAutomaticCapture();
    configureBindings();
    m_ElevatorSubsystem.setDefaultCommand(new DefaultElevator(m_ElevatorSubsystem));
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
    // Drivetrain will execute this command periodically
    drivetrain.applyRequest(() ->
    drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    .withRotationalRate(-driverController.getRightX() * MaxAngularRate))); // Drive counterclockwise with negative X (left)
    driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    driverController.rightTrigger().whileTrue(drivetrain.applyRequest(() ->
    drive.withVelocityX(-driverController.getLeftY() * MaxSpeed*0.15) // Drive forward with negative Y (forward)
    .withVelocityY(-driverController.getLeftX() * MaxSpeed*0.15) // Drive left with negative X (left)
    .withRotationalRate(-driverController.getRightX() * MaxAngularRate*0.15)));
    //Operator Controller
    operatorController.y().onTrue(Commands.runOnce(() -> m_ElevatorSubsystem.setElevatorSetPoint(ElevatorHeights.L4), m_ElevatorSubsystem));
    operatorController.x().onTrue(Commands.runOnce(() -> m_ElevatorSubsystem.setElevatorSetPoint(ElevatorHeights.L2), m_ElevatorSubsystem));
    operatorController.b().onTrue(Commands.runOnce(() -> m_ElevatorSubsystem.setElevatorSetPoint(ElevatorHeights.L3), m_ElevatorSubsystem));
    operatorController.a().onTrue(Commands.runOnce(() -> m_ElevatorSubsystem.setElevatorSetPoint(ElevatorHeights.LOAD), m_ElevatorSubsystem));
    operatorController.leftTrigger().whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.setRollers(.10), () -> m_ShooterSubsystem.setRollers(0), m_ShooterSubsystem));
    operatorController.rightTrigger().whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.setRollers(.20), () -> m_ShooterSubsystem.setRollers(0), m_ShooterSubsystem));
    operatorController.leftBumper().whileTrue(new ElevatorSetSpeed(m_ElevatorSubsystem, -0.15));
    operatorController.rightBumper().whileTrue(new ElevatorSetSpeed(m_ElevatorSubsystem, 0.15));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
