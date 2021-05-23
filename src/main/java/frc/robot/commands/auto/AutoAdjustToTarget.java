// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AdjusterConst;
import frc.robot.subsystems.Adjuster;

public class AutoAdjustToTarget extends CommandBase {
  private final Adjuster adjuster_subsys;
  private boolean flag = true;

  /** Creates a new AdjustToTarget. */
  public AutoAdjustToTarget(Adjuster adjuster_subsys) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.adjuster_subsys = adjuster_subsys;
    addRequirements(adjuster_subsys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    adjuster_subsys.light_on();
    while (flag) {

      if (adjuster_subsys.get_lm_off_center_Xvalue() > 3) {
        adjuster_subsys.set_adjuster_hor(-AdjusterConst.ADJUST_TO_TARGET_SPEED);
      } else if (adjuster_subsys.get_lm_off_center_Xvalue() < -3) {
        adjuster_subsys.set_adjuster_hor(AdjusterConst.ADJUST_TO_TARGET_SPEED);
      } else {
        adjuster_subsys.set_adjuster_hor(0);
        flag = false;
      }
    }
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
    return true;
  }
}
