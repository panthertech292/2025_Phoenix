// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX lowerRollerMotor;
  private final TalonFX upperRollerMotor;
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    lowerRollerMotor = new TalonFX(ShooterConstants.kLowerRollerMotor);
    upperRollerMotor = new TalonFX(ShooterConstants.kUpperRollerMotor);
    configLowerRollerMotor();
    configUpperRollerMotor();
  }
  private void configLowerRollerMotor(){
    TalonFXConfiguration lowerConfig = new TalonFXConfiguration();
    lowerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    lowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    lowerRollerMotor.getConfigurator().apply(lowerConfig);
  }
  private void configUpperRollerMotor(){
    TalonFXConfiguration upperConfig = new TalonFXConfiguration();
    upperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    upperConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    upperRollerMotor.getConfigurator().apply(upperConfig);
  }
  public void setRollers(double speed){
    lowerRollerMotor.set(speed);
    upperRollerMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Stator Current Upper", upperRollerMotor.getStatorCurrent().getValueAsDouble());
  }
}
