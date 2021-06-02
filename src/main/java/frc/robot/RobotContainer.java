// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.DriveTrainConst;
import frc.robot.Constants.IntakeConst;
import frc.robot.Constants.JoystickConst;
import frc.robot.Constants.LoadingConst;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.StopShooterAndLm;
import frc.robot.commands.drivetrain.HeadChangingDrive;
import frc.robot.commands.intake.PushOutIntake;
import frc.robot.commands.intake.SuckerIn;
import frc.robot.commands.intake.SuckerOut;
import frc.robot.commands.intake.SuckerStop;
import frc.robot.commands.intake.TakeBackIntake;
import frc.robot.commands.loading.AutoLoadingBalls;
import frc.robot.commands.loading.LoadingIn;
import frc.robot.commands.loading.PreShootingCtrl;
import frc.robot.commands.loading.StopLoading;
import frc.robot.commands.shooter.AcceleratingShooter;
import frc.robot.commands.adjuster.AdjustHorizontalAngle;
import frc.robot.commands.adjuster.AdjustToTarget;
import frc.robot.commands.adjuster.AdjustVerDown;
import frc.robot.commands.adjuster.AdjustVerUp;
import frc.robot.commands.adjuster.LimelightOff;
import frc.robot.commands.auto.AutoDelievering;
import frc.robot.commands.auto.AutoLoadingIn;
import frc.robot.commands.auto.AutoPreShootingCtrl;
import frc.robot.commands.auto.AutoShoot;
import frc.robot.commands.auto.CenterAutoCommand;
import frc.robot.commands.auto.GoStraight;
import frc.robot.commands.auto.SimpleInitialLine;
import frc.robot.commands.auto.ThrBallShootAuto;
import frc.robot.commands.auto.ThrBallShootMinor;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.subsystems.Adjuster;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loading;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // Joysticks and joystick buttons are defined here...
        private final Joystick main_stick = new Joystick(JoystickConst.MAIN_STICK_PORT);
        private final JoystickButton high_speed_drivetrain_btn = new JoystickButton(main_stick,
                        JoystickConst.MainStick.HIGH_SPEED_DRIVETRAIN_BUTTON);
        private final JoystickButton mid_speed_drivetrain_btn = new JoystickButton(main_stick,
                        JoystickConst.MainStick.MID_SPEED_DRIVETRAIN_BUTTON);
        private final JoystickButton low_speed_drivetrain_btn = new JoystickButton(main_stick,
                        JoystickConst.MainStick.LOW_SPEED_DRIVETRAIN_BUTTON);

        // =======================================================================
        private final Joystick assist_stick = new Joystick(JoystickConst.ASSIST_STICK_PORT);
        private final JoystickButton sucker_in_btn = new JoystickButton(main_stick, 5);
        private final JoystickButton sucker_out_btn = new JoystickButton(main_stick, 4);
        private final JoystickButton sucker_off_btn = new JoystickButton(main_stick, 7);
        private final JoystickButton push_out_intake_btn = new JoystickButton(main_stick, 8);
        private final JoystickButton take_back_intake_btn = new JoystickButton(main_stick, 6);
        private final JoystickButton shooter_trigger_btn = new JoystickButton(assist_stick,
                        JoystickConst.AssistStick.SHOOTER_TRIGGER_BUTTON);
        private final JoystickButton stop_shooter_btn = new JoystickButton(assist_stick,
                        JoystickConst.AssistStick.STOP_SHOOTER_BUTTON);
        private final JoystickButton adjuster_ver_up_btn = new JoystickButton(assist_stick,
                        JoystickConst.AssistStick.ADJUSTER_VER_UP_BUTTON);
        private final JoystickButton adjuster_ver_down_btn = new JoystickButton(assist_stick,
                        JoystickConst.AssistStick.ADJUSTER_VER_DOWN_BUTTON);

        private final Joystick test_stick = new Joystick(3);

        // The robot's subsystems are defined here...
        private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
        private final DriveTrain drivetrain = new DriveTrain();
        private final Intake intake_subsys = new Intake();
        private final Loading loading_subsys = new Loading();
        private final Shooter shooter_subsys = new Shooter();
        private final Adjuster adjuster_subsys = new Adjuster();

        // The robot's commands are defined here...
        private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

        private String trajectoryJSON = "paths/auto1.wpilib.json";
        public Trajectory trajectory_center = new Trajectory();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                try {
                        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                        trajectory_center = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
                } catch (IOException ex) {
                        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
                }

                // Configure the button bindings
                configureButtonBindings();

                // set the default command to subsystems
                setDefaultCommand();
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by instantiating a {@link GenericHID} or one of its subclasses
         * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
         * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                sucker_in_btn.whenPressed(new SuckerIn(intake_subsys, () -> IntakeConst.SUCKER_IN_SPEED), true);
                sucker_out_btn.whileHeld(new SuckerOut(intake_subsys, () -> IntakeConst.SUCKER_OUT_SPEED), true);
                sucker_off_btn.whenPressed(new SuckerStop(intake_subsys), true);
                push_out_intake_btn.whenPressed(new PushOutIntake(intake_subsys));
                take_back_intake_btn.whenPressed(new TakeBackIntake(intake_subsys));
                high_speed_drivetrain_btn.whenPressed(new HeadChangingDrive(drivetrain,
                                () -> DriveTrainConst.HIGH_SPEED_DRIVETRAIN
                                                * main_stick.getRawAxis(JoystickConst.MainStick.LEFT_DRIVETRAIN_AXIS),
                                () -> DriveTrainConst.HIGH_SPEED_DRIVETRAIN * main_stick
                                                .getRawAxis(JoystickConst.MainStick.RIGHT_DRIVETRAIN_AXIS)));
                mid_speed_drivetrain_btn.whenPressed(new HeadChangingDrive(drivetrain,
                                () -> DriveTrainConst.MID_SPEED_DRIVETRAIN
                                                * main_stick.getRawAxis(JoystickConst.MainStick.LEFT_DRIVETRAIN_AXIS),
                                () -> DriveTrainConst.MID_SPEED_DRIVETRAIN * main_stick
                                                .getRawAxis(JoystickConst.MainStick.RIGHT_DRIVETRAIN_AXIS)));
                low_speed_drivetrain_btn.whenPressed(new HeadChangingDrive(drivetrain,
                                () -> DriveTrainConst.LOW_SPEED_DRIVETRAIN
                                                * main_stick.getRawAxis(JoystickConst.MainStick.LEFT_DRIVETRAIN_AXIS),
                                () -> DriveTrainConst.LOW_SPEED_DRIVETRAIN * main_stick
                                                .getRawAxis(JoystickConst.MainStick.RIGHT_DRIVETRAIN_AXIS)));
                shooter_trigger_btn.whenPressed(new AcceleratingShooter(shooter_subsys), true);
                stop_shooter_btn.whenPressed(new StopShooterAndLm(adjuster_subsys, shooter_subsys), true);
                new JoystickButton(assist_stick, 6).whenPressed(new AutoLoadingBalls(loading_subsys), true);
                adjuster_ver_up_btn.whenPressed(new AdjustVerUp(adjuster_subsys), true);
                adjuster_ver_down_btn.whenPressed(new AdjustVerDown(adjuster_subsys), true);
                new JoystickButton(assist_stick, 11).whenPressed(new AdjustToTarget(adjuster_subsys), true);
                // new JoystickButton(assist_stick, 12).toggleWhenPressed(
                // new AdjustHorizontalAngle(adjuster_subsys, () -> assist_stick.getRawAxis(2)),
                // true);
                new JoystickButton(assist_stick, 12).whenPressed(new LimelightOff(adjuster_subsys), true);
                new POVButton(assist_stick, 0).whenPressed(new StopLoading(loading_subsys), true);
                new POVButton(assist_stick, 0).whenReleased(
                                new LoadingIn(loading_subsys,
                                                () -> LoadingConst.PRE_SHOOTING_SPEED * assist_stick
                                                                .getRawAxis(JoystickConst.AssistStick.LOADING_AXIS),
                                                () -> LoadingConst.ROUTER_IN_SPEED * assist_stick
                                                                .getRawAxis(JoystickConst.AssistStick.LOADING_AXIS)),
                                true);
                new POVButton(assist_stick, 90).whileHeld(new AdjustHorizontalAngle(adjuster_subsys, () -> 0.3));
                new POVButton(assist_stick, 270).whileHeld(new AdjustHorizontalAngle(adjuster_subsys, () -> -0.3));
                new POVButton(assist_stick, 180).whileHeld(new PreShootingCtrl(loading_subsys, () -> 0.5));

        }

        private void setDefaultCommand() {
                CommandScheduler.getInstance().setDefaultCommand(drivetrain, new HeadChangingDrive(drivetrain,
                                () -> DriveTrainConst.MID_SPEED_DRIVETRAIN
                                                * main_stick.getRawAxis(JoystickConst.MainStick.LEFT_DRIVETRAIN_AXIS),
                                () -> DriveTrainConst.MID_SPEED_DRIVETRAIN * main_stick
                                                .getRawAxis(JoystickConst.MainStick.RIGHT_DRIVETRAIN_AXIS)));
                CommandScheduler.getInstance().setDefaultCommand(loading_subsys, new LoadingIn(loading_subsys,
                                () -> LoadingConst.PRE_SHOOTING_SPEED
                                                * assist_stick.getRawAxis(JoystickConst.AssistStick.LOADING_AXIS),
                                () -> LoadingConst.ROUTER_IN_SPEED
                                                * assist_stick.getRawAxis(JoystickConst.AssistStick.LOADING_AXIS)));
                CommandScheduler.getInstance().setDefaultCommand(adjuster_subsys, new LimelightOff(adjuster_subsys));
                // CommandScheduler.getInstance().setDefaultCommand(intake_subsys, new
                // TakeBackIntake(intake_subsys));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An ExampleCommand will run in autonomous
                TrajectoryConfig config = new TrajectoryConfig(1, 1);
                config.setKinematics(drivetrain.getKinematics());
                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(), new Pose2d(1, -1, new Rotation2d(-90))), config);
                RamseteCommand command = new RamseteCommand(trajectory, drivetrain::getPose,
                                new RamseteController(2.0, 0.7), drivetrain.getFeedForward(),
                                drivetrain.getKinematics(), drivetrain::getSpeeds, drivetrain.getLeftPIDController(),
                                drivetrain.getRightPIDController(), drivetrain::setOutput, drivetrain);
                // return new CenterAutoCommand(drivetrain, shooter_subsys, adjuster_subsys,
                // loading_subsys)
                // return new AutoShoot(shooter_subsys, adjuster_subsys, loading_subsys,
                // drivetrain);
                // return new SimpleInitialLine(drivetrain);
                // return new ThrBallShootMinor(drivetrain, loading_subsys);
                return new ThrBallShootAuto(drivetrain, loading_subsys, shooter_subsys, adjuster_subsys);
                // return new GoStraight(drivetrain);
                // return new AutoPreShootingCtrl(loading_subsys, () -> 0.5);
                // return new AutoLoadingIn(loading_subsys, () -> -0.5, () -> -0.5);
        }
}
