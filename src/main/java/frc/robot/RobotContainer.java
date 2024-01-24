// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.auto.drivecommands.DriveStraightCommands;
import frc.robot.commands.auto.drivecommands.DriveStraightCommands.DriveStraightFixedDistance;
import frc.robot.commands.swervedrive.auto.ChoreoPickUpNoteCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SimVisualizationSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterAssistSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import frc.robot.ShuffleBoard.ShuffleBoardUpdaters;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...
        private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                        "swerve/neo"));
        private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
        public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        public final ShooterAssistSubsystem shooterAssistSubsystem = new ShooterAssistSubsystem(shooterSubsystem,
                        drivebase);
        private final SimVisualizationSubsystem simVizSubsystem = new SimVisualizationSubsystem(elevatorSubsystem,
                        intakeSubsystem, shooterSubsystem);

        // CommandJoystick rotationController = new CommandJoystick(1);
        // Replace with CommandPS4Controller or CommandJoystick if needed
        CommandJoystick driverController = new CommandJoystick(1);

        // CommandJoystick driverController = new
        // CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
        XboxController driverXbox = new XboxController(0);

        Field2d m_field = new Field2d();
        ChoreoTrajectory traj;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                traj = Choreo.getTrajectory("TestNote3");

                m_field.getObject("traj").setPoses(
                                traj.getInitialPose(), traj.getFinalPose());
                m_field.getObject("trajPoses").setPoses(
                                traj.getPoses());

                // Configure the trigger bindings
                configureBindings();

                AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                OperatorConstants.LEFT_Y_DEADBAND),
                                () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                OperatorConstants.LEFT_X_DEADBAND),
                                () -> MathUtil.applyDeadband(driverXbox.getRightX(),
                                                OperatorConstants.RIGHT_X_DEADBAND),
                                driverXbox::getYButtonPressed,
                                driverXbox::getAButtonPressed,
                                driverXbox::getXButtonPressed,
                                driverXbox::getBButtonPressed);

                // Applies deadbands and inverts controls because joysticks
                // are back-right positive while robot
                // controls are front-left positive
                // left stick controls translation
                // right stick controls the desired angle NOT angular rotation
                Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
                                () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                                () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                                () -> driverXbox.getRightX(),
                                () -> driverXbox.getRightY());

                // Applies deadbands and inverts controls because joysticks
                // are back-right positive while robot
                // controls are front-left positive
                // left stick controls translation
                // right stick controls the angular velocity of the robot
                Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
                                () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                                () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                                () -> driverXbox.getRawAxis(2));

                Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
                                () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                                () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                                () -> driverXbox.getRawAxis(2));

                drivebase.setDefaultCommand(
                                !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle
                                                : driveFieldOrientedDirectAngleSim);
        }

        /*
         * Activates systems that have flags controlling whether the system operates
         */

        public void activateSystems() {
                intakeSubsystem.wristSubsystem.activate();
                shooterSubsystem.shooterWrist.activate();
        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary predicate, or via the
         * named factories in
         * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
         * for
         * {@link CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
         * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
         * Flight joysticks}.
         */
        private void configureBindings() {
                // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

                new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
                new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
                // new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new
                // InstantCommand(drivebase::lock, drivebase)));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                var initTraj = Choreo.getTrajectory("TestNote3");

                Command autoCommand = ChoreoPickUpNoteCommand.createChoreoPickUpNoteCommand(drivebase, "TestNote3",
                                new Translation2d(2.877, 7.025),
                                intakeSubsystem);
                Command initTrajCommand = Commands.runOnce(() -> drivebase.resetOdometry(initTraj.getInitialPose()));

                // return getPingPongTestAutonomousCommand();
                // return Commands.sequence(initTrajCommand, autoCommand);
                return new DriveStraightCommands.DriveStraightFixedDistance(drivebase, Rotation2d.fromDegrees(0), 
                3, new TrapezoidProfile.Constraints(0.5,0.5/2));
        }

        public ShuffleBoardUpdaters getFieldUpdater() {
                return new ShuffleBoard.FieldShuffleBoard(drivebase, m_field).init();
        }

        public void setDriveMode() {
                // drivebase.setDefaultCommand();
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }

        public void enableTeleopFeatures() {

                shooterAssistSubsystem.enableAutoAimElevation();
        }

        public Command getPingPongTestAutonomousCommand() {
                var thetaController = new PIDController(0, 0, 0);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                drivebase.resetOdometry(traj.getInitialPose());

                Command swerveCommand = Choreo.choreoSwerveCommand(
                                traj, // Choreo trajectory from above
                                drivebase::getPose, // A function that returns the current field-relative pose of the
                                                    // robot: your
                                // wheel or vision odometry
                                new PIDController(1, 0.0, 0.0), // PIDController for field-relative X
                                // translation (input: X error in meters,
                                // output: m/s).
                                new PIDController(1, 0.0, 0.0), // PIDController for field-relative Y
                                // translation (input: Y error in meters,
                                // output: m/s).
                                thetaController, // PID constants to correct for rotation
                                // error
                                (ChassisSpeeds speeds) -> {

                                        drivebase.drive(speeds);
                                },
                                () -> false, // Whether or not to mirror the path based on alliance (this assumes the
                                             // path is
                                             // created for the blue alliance)
                                drivebase // The subsystem(s) to require, typically your drive subsystem only
                );

                // return Commands.sequence(
                // Commands.runOnce(() -> drivebase.resetOdometry(traj.getInitialPose())),
                // swerveCommand,
                // drivebase.run(() -> drivebase.drive(new Translation2d(0,0), 0,false)));

                Command pingPongCommand = Commands.sequence(Commands.runOnce(() -> elevatorSubsystem.activate()),
                                Commands.runOnce(() -> intakeSubsystem.wristSubsystem.activate()),
                                Commands.parallel(elevatorSubsystem.pingPongCommand(),
                                                intakeSubsystem.wristSubsystem.pingPongCommand(),
                                                shooterSubsystem.shooterWrist.pingPongCommand()));
                Command autoTrajCommand = Commands.sequence(
                                Commands.runOnce(() -> drivebase.resetOdometry(traj.getInitialPose())),
                                swerveCommand,
                                drivebase.run(() -> drivebase.drive(new Translation2d(0, 0), 0, false)));

                return Commands.parallel(pingPongCommand, autoTrajCommand);
        }

}
