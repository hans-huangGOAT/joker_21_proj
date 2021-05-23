// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.adjuster.LimelightOff;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.subsystems.Adjuster;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopShooterAndLm extends ParallelCommandGroup {
  private final Adjuster adjuster_subsys;
  private final Shooter shooter_subsys;

  /** Creates a new StopShooterAndLm. */
  public StopShooterAndLm(Adjuster adjuster_subsys, Shooter shooter_subsys) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.adjuster_subsys = adjuster_subsys;
    this.shooter_subsys = shooter_subsys;
    addCommands(new StopShooter(this.shooter_subsys), new LimelightOff(this.adjuster_subsys));
  }
}
