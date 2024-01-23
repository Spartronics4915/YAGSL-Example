package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimVisualizationSubsystem extends SubsystemBase{
    
    double wristAngleOffset = 30;
    ElevatorSubsystem mElevatorSubsystem;
    WristSubsystem mWristSubsystem;
    public SimVisualizationSubsystem(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {

        mElevatorSubsystem = elevatorSubsystem;
        mWristSubsystem = wristSubsystem;
        buildWristElevatorViz();
    }

    Mechanism2d wristElevatorMechanism;
    MechanismLigament2d elevator;
    MechanismLigament2d wrist;

    public void buildWristElevatorViz() {
        wristElevatorMechanism = new Mechanism2d(3, 3);
        MechanismRoot2d elevatorBase = wristElevatorMechanism.getRoot("elevator_base", 1.5, 0);
        elevator = elevatorBase.append(new MechanismLigament2d("Elevator", 1, 90, 6, new Color8Bit(Color.kBlue)));
        wrist = elevator.append(new MechanismLigament2d("wrist", 0.3, 90, 6, new Color8Bit(Color.kOrange)));
        
    }

    public void simulationPeriodic() {

        final double angle_offset = 75;
        SmartDashboard.putData("WristElevator", wristElevatorMechanism);
        elevator.setLength(mElevatorSubsystem.getProfileSetPoint());
        wrist.setAngle(mWristSubsystem.getProfileSetPoint()+angle_offset);
    }

}
