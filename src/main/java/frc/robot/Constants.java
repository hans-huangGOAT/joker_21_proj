// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveTrainConst {
        public static final int LEFT_MOTOR_PORT1 = 0;
        public static final int LEFT_MOTOR_PORT2 = 1;
        public static final int LEFT_MOTOR_PORT3 = 2;
        public static final int RIGHT_MOTOR_PORT1 = 3;
        public static final int RIGHT_MOTOR_PORT2 = 4;
        public static final int RIGHT_MOTOR_PORT3 = 5;
        
    }

    public static final class JoystickConst {
        public static final int MAIN_STICK_PORT = 0;
        public static final int ASSIST_STICK_PORT = 1;

        public static final class MainStick {
            public static final int LEFT_DRIVETRAIN_AXIS = 1;
            public static final int RIGHT_DRIVETRAIN_AXIS = 3;
        }
        public static final class AssistStick {

        }
    }

}
