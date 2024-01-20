package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public final class DriveCommands {

    public static class RotateFixedAngleCommand extends Command {
        SwerveSubsystem m_drive;
        double targetAngle;
        Rotation2d driveRotation;
        TrapezoidProfile.Constraints motionConstraints;
        TrapezoidProfile motionProfile;
        TrapezoidProfile.State targetState;
        TrapezoidProfile.State nextState;
        boolean completed;
        final double rotationThreshold = Rotation2d.fromDegrees(2).getRadians();

        public RotateFixedAngleCommand(SwerveSubsystem drive, Rotation2d angle) {

            m_drive = drive;
            driveRotation = angle;

            addRequirements(m_drive);

            motionConstraints = new TrapezoidProfile.Constraints(
                    m_drive.getSwerveController().config.maxAngularVelocity,
                    2 * m_drive.getSwerveController().config.maxAngularVelocity);

            motionProfile = new TrapezoidProfile(motionConstraints);
            completed = false;
        }

        public void initialize() {
            System.out.println("Initialized!");
            
            Rotation2d currHeading = m_drive.getHeading();
            
            targetAngle = currHeading.getRadians() + driveRotation.getRadians();
            
            targetState = new TrapezoidProfile.State(
                    targetAngle,
                    0);

            System.out.println(currHeading + " " + targetAngle);
            completed = false;
        }

        public void execute() {
            final double dT = 1 / 50.;
            TrapezoidProfile.State currState = new TrapezoidProfile.State(
                    m_drive.getHeading().getRadians(),
                    m_drive.getRobotVelocity().omegaRadiansPerSecond);

            nextState = motionProfile.calculate(dT, currState, targetState);

            m_drive.drive(new Translation2d(0, 0), nextState.velocity, false);

            System.out.println("targetstate: " + targetState.position + " currstate: " + currState.position);

            if (Math.abs(targetState.position - currState.position) < rotationThreshold) {
                System.out.println("Completed!");
                completed = true;
            }
        }

        public boolean isFinished() {

            return completed;
        }

    }
}
