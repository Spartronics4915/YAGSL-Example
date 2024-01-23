// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.IntakeRollerSubsystem;
import frc.robot.subsystems.rollers.IntakeRollerSubsystem.IntakeState;
import frc.robot.subsystems.wrists.IntakeWristSubsystem;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  public IntakeRollerSubsystem rollerSubsystem;
  public IntakeWristSubsystem wristSubsystem;

  public enum IntakeWristState {
    STOWED,
    DEPLOY_GROUND_INTAKE
  }

  IntakeWristState mWristState;

  public IntakeSubsystem() {
    rollerSubsystem = new IntakeRollerSubsystem();
    wristSubsystem = new IntakeWristSubsystem();

  }

  public void deployShooterGroundPickup() {
    wristSubsystem.setUserSetPoint(55);
    setWristState(IntakeWristState.DEPLOY_GROUND_INTAKE);
  }

  public void activateRollersForIntake() {
    rollerSubsystem.setIntakeRollerState(IntakeState.INTAKE);
  }

  private void setWristState(IntakeWristState state) {

    mWristState = state;
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
