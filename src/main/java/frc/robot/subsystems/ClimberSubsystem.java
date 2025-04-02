// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX ClimberMotor;
  private final TalonFXS ClimberIntakeMotor;
  private final Servo ClimberServo;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    ClimberMotor = new TalonFX(ClimberConstants.kClimberMotor);
    TalonFXConfiguration climbConfig = new TalonFXConfiguration();
    climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ClimberMotor.getConfigurator().apply(climbConfig);

    ClimberIntakeMotor = new TalonFXS(ClimberConstants.kClimberIntakeMotor);
    TalonFXSConfiguration intakeConfig = new TalonFXSConfiguration();
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    ClimberIntakeMotor.getConfigurator().apply(intakeConfig);

    ClimberServo = new Servo(ClimberConstants.kClimberServo);
  }
  public void setClimb(double speed){
    ClimberMotor.set(speed);
  }
  public void setClimbIntake(double speed){
    ClimberIntakeMotor.set(speed);
  }
  public double getRelativePosition(){
    return ClimberMotor.getPosition().getValueAsDouble();
  }
  public void setServo(double degrees){
    ClimberServo.setAngle(degrees);
  }
  public void unLockServo(){
    setServo(ClimberConstants.kServoUnlocked);
  }
  public void lockServo(){
    setServo(ClimberConstants.kServoLocked);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
