// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.AutoConst;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PushOutSucking extends ParallelCommandGroup {
  private Intake intake_subsys = new Intake();

  /** Creates a new PushOutSucking. */
  public PushOutSucking() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new PushOutIntake(intake_subsys), new SuckerIn(intake_subsys, () -> AutoConst.SUCKER_IN_SPEED));
  }
}
