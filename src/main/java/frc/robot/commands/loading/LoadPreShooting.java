// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.loading;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Loading;

public class LoadPreShooting extends CommandBase {
  private final Loading loading_subsys;
  private final DoubleSupplier pre_shooting_speed;

  /** Creates a new LoadPreShooting. */
  public LoadPreShooting(Loading loading_subsys, DoubleSupplier pre_shooting_speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.loading_subsys = loading_subsys;
    this.pre_shooting_speed = pre_shooting_speed;
    addRequirements(loading_subsys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    loading_subsys.loadPreShooting(pre_shooting_speed.getAsDouble());
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
