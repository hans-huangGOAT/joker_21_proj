// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

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

  /** Creates a new Loading. */
  public Loading() {
    router_left = new WPI_VictorSPX(LoadingConst.ROUTER_MOTOR_PORT1);
    router_right = new WPI_VictorSPX(LoadingConst.ROUTER_MOTOR_PORT2);
    router = new SpeedControllerGroup(router_left, router_right);
    pre_shooting1 = new WPI_VictorSPX(LoadingConst.ROUTER_MOTOR_PORT1);
    pre_shooting2 = new WPI_VictorSPX(LoadingConst.ROUTER_MOTOR_PORT2);
    pre_shooting = new SpeedControllerGroup(pre_shooting1, pre_shooting2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void router_in_same_speed(double router_speed) {
    router.set(router_speed);
  }

  public void router_in_left_faster(double router_speed_upper_bound, double diff) {
    router_left.set(router_speed_upper_bound);
    router_right.set(router_speed_upper_bound - diff);
  }

  public void router_in_right_faster(double router_speed_upper_bound, double diff) {
    router_left.set(router_speed_upper_bound - diff);
    router_right.set(router_speed_upper_bound);
  }

  public void router_out(double router_speed) {
    router.set(-router_speed);
  }

  public void pre_shooting_up(double pre_shooting_speed) {
    pre_shooting.set(pre_shooting_speed);
  }

  public void pre_shooting_down(double pre_shooting_speed) {
    pre_shooting.set(-pre_shooting_speed);
  }

}
