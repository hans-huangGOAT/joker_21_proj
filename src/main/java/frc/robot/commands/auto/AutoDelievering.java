// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.adjuster.AdjustToTarget;
import frc.robot.commands.loading.PreShootingCtrl;
import frc.robot.subsystems.Adjuster;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Loading;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDelievering extends SequentialCommandGroup {
  /** Creates a new AutoDelievering. */
  public AutoDelievering(Shooter shooter_subsys, Adjuster adjuster_subsys, Loading loading_subsys,
      DriveTrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new GoStraight(drivetrain), new WaitCommand(2), new AutoClearPreShooting(loading_subsys),
        new AutoClearLoading(loading_subsys));
  }
}
