// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.loading;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LoadingConst;
import frc.robot.subsystems.Loading;

public class AutoLoadingBalls extends CommandBase {
  private final Loading loading_subsys;
  private boolean lower;
  private boolean mid;
  private boolean upper;

  /** Creates a new AutoLoadingBalls. */
  public AutoLoadingBalls(Loading loading_subsys) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.loading_subsys = loading_subsys;
    addRequirements(this.loading_subsys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lower = loading_subsys.getLowerDetec();
    mid = loading_subsys.getMidDetec();
    upper = loading_subsys.getUpperDetec();
    // System.out.println(upper + " " + mid + " " + lower);
    if (!upper && !(!lower && mid)) {
      loading_subsys.router_set_same_speed(-0.6 * LoadingConst.ROUTER_IN_SPEED);
      loading_subsys.loadPreShooting(-0.45 * LoadingConst.PRE_SHOOTING_SPEED);
    } else if (!lower && !(!mid && !upper)) {
      loading_subsys.router_set_same_speed(-0.6 * LoadingConst.ROUTER_IN_SPEED);
      loading_subsys.loadPreShooting(0);
    } else {
      loading_subsys.router_set_same_speed(0);
      loading_subsys.loadPreShooting(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (upper) {
      return true;
    }
    return false;
  }
}
