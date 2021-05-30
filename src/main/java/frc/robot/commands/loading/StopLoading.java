// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.loading;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Loading;

public class StopLoading extends CommandBase {
  private final Loading loading_subsys;

  /** Creates a new StopLoading. */
  public StopLoading(Loading loading_subsys) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.loading_subsys = loading_subsys;
    addRequirements(this.loading_subsys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    loading_subsys.loadPreShooting(0);
    loading_subsys.router_set_same_speed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
