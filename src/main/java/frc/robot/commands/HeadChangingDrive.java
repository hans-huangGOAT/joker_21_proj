// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class HeadChangingDrive extends CommandBase {
  private final DriveTrain drivetrain;
  private final double left_speed;
  private final double right_speed;
  private double angle;

  /** Creates a new HeadChangingDrive. */
  public HeadChangingDrive(DriveTrain drivetrain, DoubleSupplier left_speed, DoubleSupplier right_speed) {
    this.drivetrain = drivetrain;
    this.left_speed = left_speed.getAsDouble();
    this.right_speed = right_speed.getAsDouble();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = drivetrain.getGyroAngle();
    if((angle > 90 && angle < 270) || (angle > -270 && angle < -90) ){
      drivetrain.tankDrive(left_speed,right_speed);
    }
    if((angle < 90 && angle > -90) || (angle > 270) || (angle < -270)){
      drivetrain.tankDrive(-left_speed, -right_speed);
    }
    if((angle >= 358 && angle <= 362)  || (angle <= -358 && angle >= -362)){
      drivetrain.resetGyro();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
