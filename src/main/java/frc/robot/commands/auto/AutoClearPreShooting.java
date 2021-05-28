// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Loading;

public class AutoClearPreShooting extends CommandBase {
  private final Loading loading_subsys;
  private boolean upper;
  private boolean flag = false;

  /** Creates a new AutoClearPreShooting. */
  public AutoClearPreShooting(Loading loading_subsys) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.loading_subsys = loading_subsys;
    addRequirements(loading_subsys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    loading_subsys.loadPreShooting(-0.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    loading_subsys.loadPreShooting(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    upper = loading_subsys.getUpperDetec();
    // System.out.println(upper + " " + flag);
    if (!upper) {
      flag = true;
    }
    if (upper && flag) {
      return true;
    } else {
      return false;
    }
  }
}
