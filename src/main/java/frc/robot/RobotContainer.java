// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.IntakeConst;
import frc.robot.Constants.JoystickConst;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.drivetrain.HeadChangingDrive;
import frc.robot.commands.intake.SuckerIn;
import frc.robot.commands.intake.SuckerOut;
import frc.robot.commands.intake.SuckerStop;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Joysticks and joystick buttons are defined here...
  Joystick main_stick = new Joystick(JoystickConst.MAIN_STICK_PORT);
  JoystickButton high_speed_drivetrain_btn = new JoystickButton(main_stick,
      JoystickConst.MainStick.HIGH_SPEED_DRIVETRAIN_BUTTON);
  JoystickButton mid_speed_drivetrain_btn = new JoystickButton(main_stick,
      JoystickConst.MainStick.MID_SPEED_DRIVETRAIN_BUTTON);
  JoystickButton low_speed_drivetrain_btn = new JoystickButton(main_stick,
      JoystickConst.MainStick.LOW_SPEED_DRIVETRAIN_BUTTON);
  Joystick assist_stick = new Joystick(JoystickConst.ASSIST_STICK_PORT);
  JoystickButton sucker_in_btn = new JoystickButton(assist_stick, JoystickConst.AssistStick.SUCKER_IN_BUTTON);
  JoystickButton sucker_out_btn = new JoystickButton(assist_stick, JoystickConst.AssistStick.SUCKER_OUT_BUTTON);
  JoystickButton sucker_off_btn = new JoystickButton(assist_stick, JoystickConst.AssistStick.SUCKER_OFF_BUTTON);

  // The robot's subsystems are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveTrain drivetrain = new DriveTrain();
  private final Intake intake_subsys = new Intake();

  // The robot's commands are defined here...
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
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
    sucker_off_btn.whenPressed(new SuckerStop(intake_subsys));
  }

  private void setDefaultCommand() {
    CommandScheduler.getInstance().setDefaultCommand(drivetrain,
        new HeadChangingDrive(drivetrain,
            () -> 0.5 * main_stick.getRawAxis(JoystickConst.MainStick.LEFT_DRIVETRAIN_AXIS),
            () -> 0.5 * main_stick.getRawAxis(JoystickConst.MainStick.RIGHT_DRIVETRAIN_AXIS)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
