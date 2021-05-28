// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
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
  private final Encoder left_encoder;
  private final Encoder right_encoder;

  private final DifferentialDriveKinematics kinematics;
  private final DifferentialDriveOdometry odometry;
  private final SimpleMotorFeedforward feedforward;
  private final PIDController leftPIDController;
  private final PIDController rightPIDController;
  private Pose2d pose;

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
    left_encoder = new Encoder(DriveTrainConst.LEFT_ENCODER_PORT[0], DriveTrainConst.LEFT_ENCODER_PORT[1]);
    right_encoder = new Encoder(DriveTrainConst.RIGHT_ENCODER_PORT[0], DriveTrainConst.RIGHT_ENCODER_PORT[1]);
    right_encoder.setReverseDirection(true);
    left_encoder.setDistancePerPulse(15.24 * Math.PI / 204800);
    right_encoder.setDistancePerPulse(15.24 * Math.PI / 204800);

    kinematics = new DifferentialDriveKinematics(0.73);
    odometry = new DifferentialDriveOdometry(getHeading());
    feedforward = new SimpleMotorFeedforward(1.06, 4.33, 0.052);
    leftPIDController = new PIDController(0.442, 0, 0);
    rightPIDController = new PIDController(0.442, 0, 0);
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

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(left_encoder.getRate(), right_encoder.getRate());
  }

  public SimpleMotorFeedforward getFeedForward() {
    return feedforward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return pose;
  }

  public double getAverageDistance() {
    return (left_encoder.getDistance() + right_encoder.getDistance()) / 2;
  }

  public void setOutput(double leftVolts, double rightVolts) {
    drivetrain.tankDrive(leftVolts / 12, rightVolts / 12);
  }

  /*
   * reset the angle of gyro
   */
  public void resetGyro() {
    gyro.reset();
  }

  public void resetEncoder() {
    left_encoder.reset();
    right_encoder.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = odometry.update(getHeading(), left_encoder.getDistance(), right_encoder.getDistance());
    System.out.println(left_encoder.getDistance() + " " + right_encoder.getDistance());
    // System.out.println(pose.toString());
  }
}
