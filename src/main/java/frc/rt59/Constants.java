// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.rt59;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // public static final class AutonConstants
  // {
  //
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
  // 0);
  // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class DriverControlConstants {

    public static final double DEADBAND = 0.1;// Joystick Deadband
    public static final double TURN_CONSTANT = 6;
  }

  public static final class ElevatorConstants {

    public static final int ELEVATOR_CAN_ID = 51;
    public static final double ELEVATOR_P = 0.300000011920928;
    public static final double ELEVATOR_I = 0.000000999999997475242;
    public static final double ELEVATOR_D = 0.300000011920928;
    public static final double ELEVATOR_CONVERSION = 1.889880952380952380952380952381;
    public static final double ELEVATOR_FW_LIMIT = 48.0; // TODO: Verify
    public static final double ELEVATOR_REVERSE_LIMIT = 0.0;
    public static final boolean ELEVATOR_INVERTED = false; // TODO: Verify
    public static final int ELEVATOR_CURRENT_LIMIT = 20; // TODO: Verify

  }

  public static final class ArmConstants {
    // Basic Info
    public static final int ARM_CAN_ID = 52;
    // Ratios
    public static final double ARM_CONVERSION = 3.5;
    public static final double GEAR_RATIO = 102.8571428571429;
    // Basic Config
    public static final boolean ARM_BRAKE = true;
    public static final boolean ARM_INVERTED = true;
    public static final int ARM_CURRENT_LIMIT = 50;
    // Kinematics
    public static final double ARM_P = 0.17;
    public static final double ARM_I = 0.4;
    public static final double ARM_D = 0.035;
    public static final double ARM_S = 0.0;
    public static final double ARM_V = 2.0;
    public static final double ARM_A = 0.19;
    public static final double ARM_G = 0.54;

    public static final double ARM_MAX_SPEED = 580;
    public static final double ARM_MAX_ACCEL = 600;
  

    // Subsystem Info
    public static final double ARM_LENGTH = 0.9652;

  }
}
