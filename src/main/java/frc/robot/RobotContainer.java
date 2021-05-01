// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.JoystickConst;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.drivetrain.HeadChangingDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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
  Joystick assist_stick = new Joystick(JoystickConst.ASSIST_STICK_PORT);

  // The robot's subsystems are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveTrain drivetrain = new DriveTrain();

  // The robot's commands are defined here...
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final HeadChangingDrive head_changing_driving_Command = new HeadChangingDrive(drivetrain,
      () -> 0.5 * main_stick.getRawAxis(JoystickConst.MainStick.LEFT_DRIVETRAIN_AXIS),
      () -> 0.5 * main_stick.getRawAxis(JoystickConst.MainStick.RIGHT_DRIVETRAIN_AXIS));

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
  }

  private void setDefaultCommand() {
    CommandScheduler.getInstance().setDefaultCommand(drivetrain, head_changing_driving_Command);
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
