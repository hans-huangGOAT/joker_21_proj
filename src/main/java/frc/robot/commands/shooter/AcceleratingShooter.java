// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConst;
import frc.robot.subsystems.Shooter;

public class AcceleratingShooter extends CommandBase {
  private final Shooter shooter_subsys;
  private double shooter_speed = 0;

  /** Creates a new AcceleratingShooter. */
  public AcceleratingShooter(Shooter shooter_subsys) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter_subsys = shooter_subsys;
    addRequirements(this.shooter_subsys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter_subsys.setShooter(0);
    shooter_speed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double acceleration = 0;
    while (shooter_speed < ShooterConst.SHOOTER_MAX_SPEED) {
      if (acceleration < ShooterConst.SHOOTER_MAX_ACCELERATION) {
        acceleration += ShooterConst.SHOOTER_STEP_ACCELERATION;
      }
      shooter_speed += acceleration;
      shooter_subsys.setShooter(shooter_speed);
    }
    do {
      System.out.println("success");
    } while (false);
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
