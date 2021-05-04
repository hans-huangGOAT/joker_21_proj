// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class SetShooter extends CommandBase {
  private final Shooter shooter_subsys;
  private final DoubleSupplier shooter_speed;

  /** Creates a new SetShooter. */
  public SetShooter(Shooter shooter_subsys, DoubleSupplier shooter_speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter_subsys = shooter_subsys;
    this.shooter_speed = shooter_speed;
    addRequirements(shooter_subsys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("a");
    shooter_subsys.setShooter(shooter_speed.getAsDouble());
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
