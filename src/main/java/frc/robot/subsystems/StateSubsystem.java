// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StateConstants;

public class StateSubsystem extends SubsystemBase {
  StateConstants.ReefPositions reefPosition;
  /** Creates a new StateSubsystem. */
  public StateSubsystem() {
    reefPosition = StateConstants.ReefPositions.ALPHABRAVO;
  }

  public void setSelectedReefState(StateConstants.ReefPositions position){
    reefPosition = position;
  }
  public StateConstants.ReefPositions getSelectedReefState(){
    return reefPosition;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
