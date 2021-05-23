// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.loading.PreShootingCtrl;
import frc.robot.commands.shooter.AcceleratingShooter;
import frc.robot.subsystems.Adjuster;
import frc.robot.subsystems.Loading;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShoot extends ParallelCommandGroup {
  /** Creates a new AutoShoot. */
  public AutoShoot(Shooter shooter_subsys, Adjuster adjuster_subsys, Loading loading_subsys) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AcceleratingShooter(shooter_subsys),
        new AutoDelievering(shooter_subsys, adjuster_subsys, loading_subsys),
        new PreShootingCtrl(loading_subsys, () -> -0.4));
  }
}
