// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConst;

public class DriveTrain extends SubsystemBase {
  private final WPI_VictorSPX left_motor_1;
  private final WPI_VictorSPX left_motor_2;
  private final WPI_VictorSPX left_motor_3;
  private final WPI_VictorSPX right_motor_1;
  private final WPI_VictorSPX right_motor_2;
  private final WPI_VictorSPX right_motor_3;
  private final SpeedControllerGroup left_motor_group;
  private final SpeedControllerGroup right_motor_group;
  private final DifferentialDrive drivetrain;

  private final ADXRS450_Gyro gyro;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    left_motor_1 = new WPI_VictorSPX(DriveTrainConst.LEFT_MOTOR_PORT1);
    left_motor_2 = new WPI_VictorSPX(DriveTrainConst.LEFT_MOTOR_PORT2);
    left_motor_3 = new WPI_VictorSPX(DriveTrainConst.LEFT_MOTOR_PORT3);
    right_motor_1 = new WPI_VictorSPX(DriveTrainConst.RIGHT_MOTOR_PORT1);
    right_motor_2 = new WPI_VictorSPX(DriveTrainConst.RIGHT_MOTOR_PORT2);
    right_motor_3 = new WPI_VictorSPX(DriveTrainConst.RIGHT_MOTOR_PORT3);
    left_motor_group = new SpeedControllerGroup(left_motor_1, left_motor_2, left_motor_3);
    right_motor_group = new SpeedControllerGroup(right_motor_1, right_motor_2, right_motor_3);
    drivetrain = new DifferentialDrive(left_motor_group, right_motor_group);

    gyro = new ADXRS450_Gyro();
  }

  /*
   * control the drivetrain via tank drive mode
   */
  public void tankDrive(double left_speed, double right_speed) {
    drivetrain.tankDrive(left_speed, right_speed);
  }

  /*
   * get the angle of gyroscope
   */
  public double getGyroAngle() {
    return gyro.getAngle();
  }

  /*
   * reset the angle of gyro
   */
  public void resetGyro() {
    gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
