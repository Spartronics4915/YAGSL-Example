package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimVisualizationSubsystem extends SubsystemBase {

    double wristAngleOffset = 30;
    ElevatorSubsystem mElevatorSubsystem;
    IntakeSubsystem mIntakeSubsystem;
    ShooterSubsystem mShooterSubsystem;
    LEDDriverCommunicationSubsystem mLEDDriverCommsSubssystem;

    public SimVisualizationSubsystem(ElevatorSubsystem elevatorSubsystem,
            IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem,
            LEDDriverCommunicationSubsystem driverComms
            ) {

        mElevatorSubsystem = elevatorSubsystem;
        mIntakeSubsystem = intakeSubsystem;
        mShooterSubsystem = shooterSubsystem;
        mLEDDriverCommsSubssystem = driverComms;

        buildWristElevatorViz();
        buildShooterViz();
        buildFeedbackViz();
    }

    Mechanism2d wristElevatorMechanism;
    MechanismLigament2d elevator;
    MechanismLigament2d wrist;
    MechanismLigament2d feedback;
    MechanismRoot2d elevatorBase;
    MechanismRoot2d shooterBase;
    MechanismRoot2d feedbackBase;

    MechanismLigament2d shooter;

    public void buildWristElevatorViz() {
        wristElevatorMechanism = new Mechanism2d(3, 1.2);
        elevatorBase = wristElevatorMechanism.getRoot("elevator_base", 1.5, 0.2);
        shooterBase = wristElevatorMechanism.getRoot("shooter_base", 2, 0.2);
        elevator = elevatorBase.append(new MechanismLigament2d("Elevator", 1, 90, 6, new Color8Bit(Color.kBlue)));
        wrist = elevator.append(new MechanismLigament2d("wrist", 0.3, 90, 6, new Color8Bit(Color.kRed)));

    }

    public void buildShooterViz() {

        shooter = shooterBase.append(new MechanismLigament2d("Shooter", 0.5, 0, 6 ,new Color8Bit(Color.kOrange)) );
    }

    public void buildFeedbackViz() {
        feedbackBase = wristElevatorMechanism.getRoot("feedback_base", 2, 1.1);
        feedback = feedbackBase.append(new MechanismLigament2d("feedback", 0.25, 270, 6, new Color8Bit(Color.kOrange)));
    }

    private Color getIntakeColor() {
        switch (mIntakeSubsystem.rollerSubsystem.getIntakeRollerState()) {
            case OFF:
                return Color.kRed;
            case INTAKE:
                return Color.kLimeGreen;
            case EXPEL:
                return Color.kYellow;
            default:
                return Color.kAntiqueWhite;
        }

    }

    private Color getDriverCommsColor() {

        return Color.kRed;
    }

    public void simulationPeriodic() {

        final double angle_offset = 75;
        SmartDashboard.putData("WristElevator", wristElevatorMechanism);
        elevator.setLength(mElevatorSubsystem.getProfileSetPoint());
        wrist.setAngle(mIntakeSubsystem.wristSubsystem.getProfileSetPoint() + angle_offset);

        Color8Bit intakeColor = new Color8Bit(getIntakeColor());
        wrist.setColor(intakeColor);

        shooter.setAngle(mShooterSubsystem.shooterWrist.getProfileSetPoint());
        feedback.setColor(new Color8Bit(mLEDDriverCommsSubssystem.getShooterAssistColor()));
        
    }

}
