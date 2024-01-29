// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import frc.robot.commands.swervedrive.auto.PrintDistFromTrajBegin;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SimVisualizationSubsystem;
import frc.robot.subsystems.TimerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDDriverCommunicationSubsystem;
import frc.robot.subsystems.ShooterAssistSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import frc.robot.ShuffleBoard.ShuffleBoardUpdaters;
import frc.robot.commands.swervedrive.auto.ChoreoDriveCommand;

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
        public final LEDDriverCommunicationSubsystem driverLEDSubsystem = new LEDDriverCommunicationSubsystem();
        public final ShooterAssistSubsystem shooterAssistSubsystem = new ShooterAssistSubsystem(shooterSubsystem,
                        drivebase, driverLEDSubsystem);
        private final SimVisualizationSubsystem simVizSubsystem = new SimVisualizationSubsystem(elevatorSubsystem,
                        intakeSubsystem, shooterSubsystem, driverLEDSubsystem);
        public final TimerSubsystem timerSubsystem = new TimerSubsystem();

        // CommandJoystick rotationController = new CommandJoystick(1);
        // Replace with CommandPS4Controller or CommandJoystick if needed
        CommandJoystick driverController = new CommandJoystick(1);

        // CommandJoystick driverController = new
        // CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
        XboxController driverXbox = new XboxController(0);

        Field2d m_field = new Field2d();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

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
                var traj1 = Choreo.getTrajectory("Note1.1");
                var traj2 = Choreo.getTrajectory("Note2.1");
                var traj3 = Choreo.getTrajectory("Note3.1");
                var traj4 = Choreo.getTrajectory("Note4.1");

                final double MAX_NOISE = 0.0;
                double xNoise = Math.random() * MAX_NOISE;
                double yNoise = Math.random() * MAX_NOISE;

                Transform2d noise = new Transform2d(xNoise, yNoise, new Rotation2d());
                Pose2d initPose = traj1.getInitialPose().plus(noise);

                Command note1Command = ChoreoPickUpNoteCommand.createChoreoPickUpNoteCommand(drivebase, "Note1.1",
                                new Translation2d(2.877, 7.025),
                                intakeSubsystem);
                Command initTrajCommand = Commands.runOnce(() -> drivebase.resetOdometry(initPose));
                Command printDist1 = PrintDistFromTrajBegin.buildPrintDistFromTrajStart(drivebase, traj1);
                Command printDist2 = PrintDistFromTrajBegin.buildPrintDistFromTrajStart(drivebase, traj2);
                Command printDist3 = PrintDistFromTrajBegin.buildPrintDistFromTrajStart(drivebase, traj3);
                Command note2Command = ChoreoPickUpNoteCommand.createChoreoPickUpNoteCommand(drivebase, "Note2.1",
                                new Translation2d(3.176, 6.409),
                                intakeSubsystem);

                Command note3Command = ChoreoPickUpNoteCommand.createChoreoPickUpNoteCommand(drivebase, "Note3.1",
                                new Translation2d(3.176, 6.409),
                                intakeSubsystem);
                Command note4Command = ChoreoPickUpNoteCommand.createChoreoPickUpNoteCommand(drivebase, "Note4.1",
                                new Translation2d(3.176, 6.409),
                                intakeSubsystem);
                Command driveShootNote4Command = ChoreoDriveCommand.createChoreoDriveCommand(drivebase, "Shoot4.1");
                Command holdStillCommand = Commands.run(() -> {
                        drivebase.drive(new Translation2d(), 0, false);
                });

                // return getPingPongTestAutonomousCommand();
                return Commands.sequence(initTrajCommand, timerSubsystem.startTimerCommand(), printDist1, note1Command,
                                printDist2, note2Command,
                                printDist3, note3Command, note4Command, driveShootNote4Command,
                                timerSubsystem.printElapsedTimeCommand(),
                                holdStillCommand);
                // return new DriveStraightCommands.DriveStraightFixedDistance(drivebase,
                // Rotation2d.fromDegrees(180),
                // 3, new TrapezoidProfile.Constraints(0.5, 0.5 / 2));
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

                ChoreoTrajectory traj = Choreo.getTrajectory("Note1.1");
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
