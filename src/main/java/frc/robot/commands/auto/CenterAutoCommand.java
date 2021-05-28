// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Adjuster;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Loading;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterAutoCommand extends SequentialCommandGroup {
  private final DriveTrain drivetrain;
  private final Trajectory trajectory;

  /** Creates a new CenterAutoCommand. */
  public CenterAutoCommand(DriveTrain drivetrain, Shooter shooter_subsys, Adjuster adjuster_subsys,
      Loading loading_subsys, Trajectory trajectory) {
    this.drivetrain = drivetrain;
    this.trajectory = trajectory;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoShoot(shooter_subsys, adjuster_subsys, loading_subsys, drivetrain));
  }

  private Command getTrajectoryCommand() {
    TrajectoryConfig config = new TrajectoryConfig(1, 1);
    config.setKinematics(drivetrain.getKinematics());
    // Trajectory trajectory = TrajectoryGenerator
    // .generateTrajectory(List.of(new Pose2d(), new Pose2d(1, -1, new
    // Rotation2d(-90))), config);
    RamseteCommand command = new RamseteCommand(trajectory, drivetrain::getPose, new RamseteController(2.0, 0.7),
        drivetrain.getFeedForward(), drivetrain.getKinematics(), drivetrain::getSpeeds,
        drivetrain.getLeftPIDController(), drivetrain.getRightPIDController(), drivetrain::setOutput, drivetrain);
    return command;
  }
}
