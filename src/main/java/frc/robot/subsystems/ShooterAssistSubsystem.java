package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDDriverCommunicationSubsystem.ShooterAssistState;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ShooterAssistSubsystem extends SubsystemBase {

    ShooterSubsystem shooterSubsystem;
    SwerveSubsystem swerveSubsystem;
    Translation2d speakerTarget;
    boolean autoAimElevationEnabled, shooterAimFeedbackEnabled;
    double speakerTargetHeight;
    Rotation2d aimElevation;
    LEDDriverCommunicationSubsystem driverCommsSubsystem;

    final double SHOOTING_RANGE = 4; // meters

    public ShooterAssistSubsystem(ShooterSubsystem shooter, SwerveSubsystem swerve, LEDDriverCommunicationSubsystem driverComms) {
        shooterSubsystem = shooter;
        swerveSubsystem = swerve;

        autoAimElevationEnabled = false;
        shooterAimFeedbackEnabled = true;

        driverCommsSubsystem = driverComms;
        setSpeakerTarget();
    }

    public void enableAutoAimElevation() {

        autoAimElevationEnabled = true;
    }

    private void setSpeakerTarget() {
        speakerTarget = new Translation2d(0.474, 5.569);
        speakerTargetHeight = 3.5;
    }

    public void setDriverFeedbackState(Pose2d currPose) {
        double speakerRange = currPose.getTranslation().getDistance(speakerTarget);
        boolean withinRange = (speakerRange < SHOOTING_RANGE);
        
        if(withinRange) {
            driverCommsSubsystem.setShooterAssistState(ShooterAssistState.IN_RANGE_ONLY);

        }

        else {
            driverCommsSubsystem.setShooterAssistState(ShooterAssistState.OUT_OF_RANGE);
        }
    }

    @Override
    public void periodic() {

        Pose2d currPose = swerveSubsystem.getPose();
        setDriverFeedbackState(currPose);
        
        if (autoAimElevationEnabled) {
            Translation2d currLoc = currPose.getTranslation();
            double distToTarget = currLoc.getDistance(speakerTarget);
            aimElevation = Rotation2d.fromRadians(Math.atan2(speakerTargetHeight, distToTarget));
            shooterSubsystem.shooterWrist.setUserSetPoint(aimElevation.getDegrees());
        }
    }
}
