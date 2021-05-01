// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class SuckerOut extends CommandBase {
  private final Intake intake_subsys;
  private final DoubleSupplier out_speed;

  /** Creates a new SuckerOut. */
  public SuckerOut(Intake intake_subsys, DoubleSupplier out_speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake_subsys = intake_subsys;
    this.out_speed = out_speed;
    addRequirements(intake_subsys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake_subsys.sucker_out(out_speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake_subsys.sucker_stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
