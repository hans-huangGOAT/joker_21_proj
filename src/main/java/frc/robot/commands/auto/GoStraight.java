// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class GoStraight extends CommandBase {
  private final DriveTrain drivetrain;
  private double angle;

  /** Creates a new GoStraight. */
  public GoStraight(DriveTrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.tankDrive(0, 0);
    drivetrain.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = drivetrain.getGyroAngle();
    if (angle < 1.5 && angle > -1.5) {
      drivetrain.tankDrive(0.4, 0.4);
    } else if (angle > 0) {
      if (angle > 15) {
        drivetrain.tankDrive(-0.4, 0.6);
      } else {
        drivetrain.tankDrive(-0.3, 0.5);
      }
    } else if (angle < 0) {
      if (angle < -15) {
        drivetrain.tankDrive(0.6, -0.4);
      } else {
        drivetrain.tankDrive(0.5, -0.3);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
