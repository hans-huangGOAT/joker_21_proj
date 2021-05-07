// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AdjusterConst;

public class Adjuster extends SubsystemBase {
  private final WPI_VictorSPX adjuster_hor;

  private final DigitalInput left_bound_limit;
  private final DigitalInput right_bound_limit;

  /** Creates a new Adjuster. */
  public Adjuster() {
    adjuster_hor = new WPI_VictorSPX(AdjusterConst.ADJUSTER_HOR_MOTOR_PORT);
    left_bound_limit = new DigitalInput(AdjusterConst.LEFT_BOUND_LIMIT_PORT);
    right_bound_limit = new DigitalInput(AdjusterConst.RIGHT_BOUND_LIMIT_PORT);

  }

  public void set_adjuster_hor(double speed) {
    // if speed < 0, turn right; speed > 0 turn left
    if (get_left_bound_limit() && speed > 0) {
      adjuster_hor.set(0);
    } else if (get_right_bound_limit() && speed < 0) {
      adjuster_hor.set(0);
    } else {
      adjuster_hor.set(speed);
    }
  }

  public boolean get_left_bound_limit() {
    return !left_bound_limit.get();
  }

  public boolean get_right_bound_limit() {
    return !right_bound_limit.get();
  }

  public double get_lm_off_center_Xvalue() {
    // tx < 0, target is on the left; tx > 0, target is on the right(of the shooter)
    return NetworkTableInstance.getDefault().getTable("limelight-joker").getEntry("tx").getDouble(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
