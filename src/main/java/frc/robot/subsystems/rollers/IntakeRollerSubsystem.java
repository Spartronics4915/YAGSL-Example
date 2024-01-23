package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollerSubsystem extends SubsystemBase{
    public static enum IntakeState {
        OFF,
        INTAKE,
        EXPEL
    }

    IntakeState mState;

    public IntakeRollerSubsystem() {
        setIntakeRollerState(IntakeState.OFF);
    }

    public void setIntakeRollerState(IntakeState state) {
        mState = state;
    }

    public IntakeState getIntakeRollerState() {

        return mState;
    }

}
