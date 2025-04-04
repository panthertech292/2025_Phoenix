// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants.ElevatorHeights;
import frc.robot.Constants.PositionConstants.Positions;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberSetSpeed;
import frc.robot.commands.DefaultElevator;
import frc.robot.commands.ElevatorSetHeight;
import frc.robot.commands.ElevatorSetSpeed;
import frc.robot.commands.FindReef;
import frc.robot.commands.Intake;
import frc.robot.commands.autoScore;
import frc.robot.commands.autoScoreIfAligned;
import frc.robot.commands.FindReefScore;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
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
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  //Drive Stuff
  //Drive Config
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentric drive5DeadBand = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final Command driveUntilTouch = drivetrain.applyRequest(() ->
  robotCentricDrive.withVelocityY(-0.5)).until(drivetrain.againstReef()); // Drive left with negative X (left)

  private final Command findTheReef = new FindReef(drivetrain, m_ShooterSubsystem).andThen(drivetrain.applyRequest(() -> brake).withTimeout(0.1));
  private final Command findAutoL4 = new FindReefScore(drivetrain, m_ShooterSubsystem, m_ElevatorSubsystem, ElevatorHeights.L4);
  
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    CameraServer.startAutomaticCapture();
    configureBindings();
    //configureButtonBoard();
    registerCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto", autoChooser);
    m_ElevatorSubsystem.setDefaultCommand(new DefaultElevator(m_ElevatorSubsystem));
  }

  private void registerCommands(){
    NamedCommands.registerCommand("Reset to Photon", drivetrain.runOnce(() -> drivetrain.poseToPhoton()));
    NamedCommands.registerCommand("Intake", new Intake(m_ElevatorSubsystem, m_ShooterSubsystem));
    NamedCommands.registerCommand("Score-L4", new autoScore(m_ElevatorSubsystem, m_ShooterSubsystem, ElevatorHeights.L4));
    NamedCommands.registerCommand("Score-L2", new autoScore(m_ElevatorSubsystem, m_ShooterSubsystem, ElevatorHeights.L2));
    NamedCommands.registerCommand("Elevator-L4", new ElevatorSetHeight(m_ElevatorSubsystem, ElevatorHeights.L4));
    NamedCommands.registerCommand("Elevator-SenseHeight", new ElevatorSetHeight(m_ElevatorSubsystem, ElevatorHeights.SENSE));
    //NamedCommands.registerCommand("FindReef", new FindReef(drivetrain, m_ShooterSubsystem));
    NamedCommands.registerCommand("FindReef", findTheReef);
    NamedCommands.registerCommand("DriveUntilTouch", driveUntilTouch);
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
    drive5DeadBand.withVelocityX(-driverController.getLeftY() * MaxSpeed*0.15) // Drive forward with negative Y (forward)
    .withVelocityY(-driverController.getLeftX() * MaxSpeed*0.15) // Drive left with negative X (left)
    .withRotationalRate(-driverController.getRightX() * MaxAngularRate*0.15)));

    driverController.leftTrigger().whileTrue(drivetrain.applyRequest(() ->
    drive5DeadBand.withVelocityX(-driverController.getLeftY() * MaxSpeed*0.05) // Drive forward with negative Y (forward)
    .withVelocityY(-driverController.getLeftX() * MaxSpeed*0.05) // Drive left with negative X (left)
    .withRotationalRate(-driverController.getRightX() * MaxAngularRate*0.05)));

    //driverController.leftBumper().whileTrue(drivetrain.goToPosition(Positions.TOPMID).alongWith(new Intake(m_ElevatorSubsystem, m_ShooterSubsystem)));
    //driverController.rightBumper().whileTrue(drivetrain.goToPosition(Positions.BOTTOMMID).alongWith(new Intake(m_ElevatorSubsystem, m_ShooterSubsystem)));
    driverController.rightBumper().whileTrue(findTheReef);
    driverController.leftBumper().whileTrue(findAutoL4);
    //driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.poseToPhoton()));
    //driverController.y().whileTrue(driveToPosition);
    //Operator Controller
    operatorController.back().onTrue(Commands.runOnce(() -> m_ElevatorSubsystem.setElevatorSetPoint(ElevatorHeights.SENSE), m_ElevatorSubsystem));
    operatorController.y().onTrue(Commands.runOnce(() -> m_ElevatorSubsystem.setElevatorSetPoint(ElevatorHeights.L4), m_ElevatorSubsystem));
    operatorController.x().onTrue(Commands.runOnce(() -> m_ElevatorSubsystem.setElevatorSetPoint(ElevatorHeights.L2), m_ElevatorSubsystem));
    operatorController.b().onTrue(Commands.runOnce(() -> m_ElevatorSubsystem.setElevatorSetPoint(ElevatorHeights.L3), m_ElevatorSubsystem));
    operatorController.a().onTrue(Commands.runOnce(() -> m_ElevatorSubsystem.setElevatorSetPoint(ElevatorHeights.LOAD), m_ElevatorSubsystem));
    operatorController.leftTrigger().whileTrue(new Intake(m_ElevatorSubsystem, m_ShooterSubsystem));
    operatorController.rightTrigger().whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.setRollers(.10), () -> m_ShooterSubsystem.setRollers(0), m_ShooterSubsystem));
    operatorController.leftStick().whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.setRollers(-.10), () -> m_ShooterSubsystem.setRollers(0), m_ShooterSubsystem));
    operatorController.rightStick().whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.setRollers(.20), () -> m_ShooterSubsystem.setRollers(0), m_ShooterSubsystem));//.whileTrue(Commands.startEnd(() -> m_ShooterSubsystem.setRollers(.10), () -> m_ShooterSubsystem.setRollers(0), m_ShooterSubsystem));
    operatorController.leftBumper().whileTrue(new ElevatorSetSpeed(m_ElevatorSubsystem, -0.15));
    operatorController.rightBumper().whileTrue(new ElevatorSetSpeed(m_ElevatorSubsystem, 0.15));
    operatorController.start().whileTrue(Commands.startEnd(() -> m_ClimberSubsystem.setClimbIntake(.60), () -> m_ClimberSubsystem.setClimbIntake(0), m_ClimberSubsystem));
    operatorController.povLeft().whileTrue(new ClimberSetSpeed(m_ClimberSubsystem, 0.10));
    operatorController.povUp().whileTrue(new ClimberSetSpeed(m_ClimberSubsystem, 0.35));
    operatorController.povRight().whileTrue(new ClimberSetSpeed(m_ClimberSubsystem, 0.60));
    operatorController.povDown().whileTrue(new ClimberSetSpeed(m_ClimberSubsystem, -0.35));

    drivetrain.registerTelemetry(logger::telemeterize);
  }
  
  public Command getAutonomousCommand() {
    System.out.println("AUTO RUNNING: " + autoChooser.getSelected().getName());
    return autoChooser.getSelected();
  }
}
