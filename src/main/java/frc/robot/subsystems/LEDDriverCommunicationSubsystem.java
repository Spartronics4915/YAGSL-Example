package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;

public class LEDDriverCommunicationSubsystem extends SubsystemBase {

    // The design of this subsystem is that other systems can set a number of
    // conditions
    // and the prioritization logic of what to send is determined here.

    public enum ShooterAssistState {
        NOT_ASSISTING,
        OUT_OF_RANGE,
        IN_RANGE_ONLY,
        AUTO_AIMING,
        READY_TO_FIRE
    }

    ShooterAssistState shooterAssistState;

    public LEDDriverCommunicationSubsystem() {

        shooterAssistState = null;

    }

    public void setShooterAssistState(ShooterAssistState state) {
        shooterAssistState = state;
    }

    public Color getShooterAssistColor() {

        if (shooterAssistState == null) {

            return Color.kWhite;
        }

        switch (shooterAssistState) {
            case NOT_ASSISTING:
                return Color.kGray;
            case OUT_OF_RANGE:
                return Color.kPurple;
            case IN_RANGE_ONLY:
                return Color.kYellow;
            case AUTO_AIMING:
                return Color.kBlue;
            case READY_TO_FIRE:
                return Color.kGreen;
            default:
                return Color.kBlack;
        }
    }

}
