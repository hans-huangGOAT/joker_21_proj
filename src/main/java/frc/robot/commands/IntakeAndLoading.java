// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.intake.PushOutSucking;
import frc.robot.commands.loading.AutoLoadingBalls;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loading;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAndLoading extends ParallelRaceGroup {
  /** Creates a new IntakeAndLoading. */
  public IntakeAndLoading(Loading loading_subsys, Intake intake_subsys) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new PushOutSucking(), new AutoLoadingBalls(loading_subsys));
  }
}
