package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ShooterAssistSubsystem extends SubsystemBase {

    ShooterSubsystem shooterSubsystem;
    SwerveSubsystem swerveSubsystem;
    Translation2d speakerTarget;
    boolean autoAimElevationEnabled;
    double speakerTargetHeight;

    public ShooterAssistSubsystem(ShooterSubsystem shooter, SwerveSubsystem swerve) {
        shooterSubsystem = shooter;
        swerveSubsystem = swerve;

        autoAimElevationEnabled = false;
        setSpeakerTarget();
    }

    public void enableAutoAimElevation() {

        autoAimElevationEnabled = true;
    }

    private void setSpeakerTarget() {
        speakerTarget = new Translation2d(0.474, 5.569);
        speakerTargetHeight = 3.5;
    }

    @Override
    public void periodic() {

        if (autoAimElevationEnabled) {
            Translation2d currLoc = swerveSubsystem.getPose().getTranslation();
            double distToTarget = currLoc.getDistance(speakerTarget);
            Rotation2d aimElevation = Rotation2d.fromRadians(Math.atan2(speakerTargetHeight, distToTarget));
            shooterSubsystem.shooterWrist.setUserSetPoint(aimElevation.getDegrees());
        }
    }
}
