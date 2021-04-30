// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final WPI_VictorSPX sucker_motor;
  private final Solenoid expan_solen;

  /** Creates a new Intake. */
  public Intake() {
    sucker_motor = new WPI_VictorSPX();
    expan_solen = new Solenoid();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
