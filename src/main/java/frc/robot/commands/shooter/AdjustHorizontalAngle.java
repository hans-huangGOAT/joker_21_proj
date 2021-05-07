// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Adjuster;
import frc.robot.subsystems.Shooter;

public class AdjustHorizontalAngle extends CommandBase {
  private final Adjuster adjuster_subsys;
  private final DoubleSupplier speed;

  /** Creates a new AdjustHorizontalAngle. */
  public AdjustHorizontalAngle(Adjuster adjuster_subsys, DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.adjuster_subsys = adjuster_subsys;
    this.speed = speed;
    addRequirements(this.adjuster_subsys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    adjuster_subsys.set_adjuster_hor(speed.getAsDouble());
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
