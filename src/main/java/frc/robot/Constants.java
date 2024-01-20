// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int PDH_ID = 0;

  public static class IOConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public static final class SwerveConstants{
    //Drivetrain motor/encoder IDs
    public static final int LEFT_FRONT_DRIVE_ID = 7;
    public static final int RIGHT_FRONT_DRIVE_ID = 1;
    public static final int LEFT_BACK_DRIVE_ID = 5;
    public static final int RIGHT_BACK_DRIVE_ID = 3;
    
    public static final int LEFT_FRONT_TURN_ID = 6;
    public static final int RIGHT_FRONT_TURN_ID = 8;
    public static final int LEFT_BACK_TURN_ID = 4;
    public static final int RIGHT_BACK_TURN_ID = 2;
    
    public static final int LEFT_FRONT_CANCODER_ID = 2;
    public static final int RIGHT_FRONT_CANCODER_ID = 4;
    public static final int LEFT_BACK_CANCODER_ID = 1;
    public static final int RIGHT_BACK_CANCODER_ID = 3;

    public static final int PIGEON_ID = 0;

    //Drivetrain characteristics
    public static final double LEFT_FRONT_OFFSET = 327;//94.307; //78.838;//330; Id: 2
    public static final double RIGHT_FRONT_OFFSET = 55;//148.975; //94.307;//49; Id: 4
    public static final double LEFT_BACK_OFFSET = 272;//78.838;  //230.713;// 275; Id: 1
    public static final double RIGHT_BACK_OFFSET = 264;//final//230.713; //148.975;//260.5; Id: 3

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.5); //originally 4 in template
    public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
    public static final double TURN_MOTOR_GEAR_RATIO = 150.0/7;
    public static final double DRIVE_MOTOR_PCONVERSION = WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO;
    public static final double TURN_MOTOR_PCONVERSION = 2 * Math.PI / TURN_MOTOR_GEAR_RATIO;
    public static final double DRIVE_MOTOR_VCONVERSION = DRIVE_MOTOR_PCONVERSION / 60.0;
    public static final double TURN_MOTOR_VCONVERSION = TURN_MOTOR_PCONVERSION / 60.0;
    public static final double KP_TURNING = 0.6;

    public static final double DRIVETRAIN_MAX_SPEED = 1.75; //4.0
    public static final double DRIVETRAIN_MAX_ANGULAR_SPEED = 1.25 * Math.PI; //3.5

    //Teleop constraints
    public static final double TELE_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED / 1;
    public static final double TELE_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 1.75;
    public static final double TELE_DRIVE_MAX_ACCELERATION = 3; //3
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION = 3; //3

    //Auton constraints
    public static final double AUTO_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED / 1.5;
    public static final double AUTO_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 2.0;
    public static final double AUTO_DRIVE_MAX_ACCELERATION = 3;
    public static final double AUTO_DRIVE_MAX_ANGULAR_ACCELERATION = Math.PI;

    public static final double AUTO_kP_FRONT = 0.4;
    public static final double AUTO_kP_SIDE = 0.4;
    public static final double AUTO_kP_TURN = 0.5; //2.4

    //Swerve Kinematics
    public static final double TRACK_WIDTH = Units.inchesToMeters(18.75);
    public static final double WHEEL_BASE = Units.inchesToMeters(18.75);
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    );

    public static final double kS_PERCENT = 0.035;
    public static final double kP_PERCENT = 0.009;

    //Odometry
    public static final Vector<N3> ODOMETRY_STD_DEV = VecBuilder.fill(0.02, 0.02, 0.005);
  }
}