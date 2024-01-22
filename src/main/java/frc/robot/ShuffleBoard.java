package frc.robot;

import java.util.EnumMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ShuffleBoard {
    public static String UserTab = "Overview"; //anything the drivers need to see should be on this tab
    public static String DebugTab = "Debug"; //anything that will need to be referenced for debugging should be on this tab
    
    public interface ShuffleBoardUpdaters {
        public ShuffleBoardUpdaters init();
        public void updateShuffle();
    }

    public record FieldShuffleBoard(SwerveSubsystem driveBase, Field2d field) implements ShuffleBoardUpdaters{
        @Override
        public ShuffleBoardUpdaters init(){
            ShuffleboardTab mainTab = Shuffleboard
                .getTab(UserTab);
            mainTab.add("field", field);
            return this;
        }

        @Override
        public void updateShuffle() {
            field.setRobotPose(driveBase.getPose());
        }
    }
}
