// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class GetTXofLM extends CommandBase {
  private final Shooter shooter_subsys;

  /** Creates a new GetTXofLM. */
  public GetTXofLM(Shooter shooter_subsys) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter_subsys = shooter_subsys;
    addRequirements(shooter_subsys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tx = NetworkTableInstance.getDefault().getTable("limelight-joker").getEntry("tx").getDouble(0);
    System.out.println(tx);
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
