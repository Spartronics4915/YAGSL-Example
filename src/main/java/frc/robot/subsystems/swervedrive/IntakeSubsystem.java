// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  public static enum IntakeState {
    OFF,
    INTAKE,
    EXPEL
  }

  IntakeState mState;
  public IntakeSubsystem() {
    setIntakeState(IntakeState.OFF);
    
  }

  public void setIntakeState(IntakeState state) {
    mState = state;
  }
  public IntakeState getIntakeState() {

    return mState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
