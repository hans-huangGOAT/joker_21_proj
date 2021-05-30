// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LoadingConst;

public class Loading extends SubsystemBase {
  private final WPI_VictorSPX router_left;
  private final WPI_VictorSPX router_right;
  private final WPI_VictorSPX pre_shooting1;
  private final WPI_VictorSPX pre_shooting2;
  private final SpeedControllerGroup router;
  private final SpeedControllerGroup pre_shooting;
  private final DigitalInput lower_detector;
  private final DigitalInput mid_detector;
  private final DigitalInput upper_detector;

  /** Creates a new Loading. */
  public Loading() {
    router_left = new WPI_VictorSPX(LoadingConst.ROUTER_MOTOR_PORT1);
    router_right = new WPI_VictorSPX(LoadingConst.ROUTER_MOTOR_PORT2);
    router = new SpeedControllerGroup(router_left, router_right);
    pre_shooting1 = new WPI_VictorSPX(LoadingConst.PRE_SHOOTING_MOTOR_PORT1);
    pre_shooting2 = new WPI_VictorSPX(LoadingConst.PRE_SHOOTING_MOTOR_PORT2);
    pre_shooting = new SpeedControllerGroup(pre_shooting1, pre_shooting2);
    lower_detector = new DigitalInput(LoadingConst.LOWER_DETECTOR_PORT);
    mid_detector = new DigitalInput(LoadingConst.MID_DETECTOR_PORT);
    upper_detector = new DigitalInput(LoadingConst.UPPER_DETECTOR_PORT);
    router_right.setInverted(true);
    router.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void router_set_same_speed(double router_speed) {
    router.set(router_speed);
  }

  public void router_set_left_faster(double router_speed_upper_bound, double diff_percentage) {
    router_left.set(router_speed_upper_bound);
    router_right.set(router_speed_upper_bound * diff_percentage);
  }

  public void router_set_right_faster(double router_speed_upper_bound, double diff_percentage) {
    router_left.set(router_speed_upper_bound * diff_percentage);
    router_right.set(router_speed_upper_bound);
  }

  public void loadPreShooting(double pre_shooting_speed) {
    pre_shooting.set(pre_shooting_speed);
  }

  public boolean getLowerDetec() {
    return !lower_detector.get();
  }

  public boolean getMidDetec() {
    return !mid_detector.get();
  }

  public boolean getUpperDetec() {
    return !upper_detector.get();
  }

}
