// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag

  public static final class AutoAimConstants {
    public static final double kP = 0.1;
  }

  public static final class Auton {

    public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 1;
  }

  public static final class Drivebase {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {
    public static final int xboxID = 0;
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.01;
    public static final double LEFT_Y_DEADBAND = 0.01;
    public static final double RIGHT_X_DEADBAND = 0.01;
    public static final double TURN_CONSTANT = 6;
  }

  public static class BlingConstants {
    public static final int kLedPort = 0; // Port that light strip is on
    public static final int kLedLength = 200; // Length of light strip
    public static final int kAlertLength = 60; // Pulse Length in Frames
    public static final int kPulseLength = 200; // Pulse Length in Frames
    public static final double kAroundSpeedMultiplier = .2; // Around speed multiplier
    public static final double kAroundStripLength = 15; // Around speed multiplier
    public static final double kBrightness = 1; // Percentage
    public static final BlingModes kDefaultBlingMode = BlingModes.AROUND_SECONDARY_BG;
    public static final Color kDefaultBlingColor = Color.kGold;
    public static final Color kDefaultBlingColorSecondary = Color.kRoyalBlue;
  }

  public enum BlingModes {
    OFF,
    SOLID,
    SOLID_SECONDARY,
    GRADIENT,
    GRADIENT_REVERSED,
    PULSE,
    PULSE_SWITCH,
    AROUND,
    AROUND_SECONDARY_BG,
    WARNING,
    ERROR
  }
}
