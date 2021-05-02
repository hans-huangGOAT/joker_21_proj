// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConst;

public class Intake extends SubsystemBase {
  private final WPI_VictorSPX sucker_motor;
  private final Solenoid expan_solen;

  /** Creates a new Intake. */
  public Intake() {
    sucker_motor = new WPI_VictorSPX(IntakeConst.SUCKER_MOTOR_PORT);
    expan_solen = new Solenoid(IntakeConst.EXPAN_SOLEN_PORT);
  }

  public void sucker_in(double sucking_speed) {
    sucker_motor.set(sucking_speed);
  }

  public void sucker_out(double out_speed) {
    sucker_motor.set(-out_speed);
  }

  public void sucker_stop() {
    sucker_motor.set(0);
  }

  public void push_out() {
    expan_solen.set(IntakeConst.EXPAN_SOLEN_PUSH_OUT_MODE);
  }

  public void take_back() {
    expan_solen.set(IntakeConst.EXPAN_SOLEN_TAKE_BACK_MODE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
