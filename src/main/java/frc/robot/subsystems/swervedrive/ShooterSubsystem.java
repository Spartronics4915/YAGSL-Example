// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private boolean mIsShooterOn;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    mIsShooterOn = false;
  }

  /**
   * Turns shooter on
   */
  public void turnShooterOn() {
    mIsShooterOn = true;
    System.out.println("shooterOn:" + mIsShooterOn);
  }

  /**
   * Turns shooter off
   */
  public void turnShooterOff() {
    mIsShooterOn = false;
    System.out.println("shooterOn:" + mIsShooterOn);
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
