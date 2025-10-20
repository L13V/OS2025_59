// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.rt59;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants {

    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED = Units.feetToMeters(14.5);

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
        public static final double ELEVATOR_P = 1.78;
        public static final double ELEVATOR_I = 0.24;
        public static final double ELEVATOR_D = 0;
        public static final double ELEVATOR_S = 0;
        public static final double ELEVATOR_G = 0.23;
        public static final double ELEVATOR_V = 0;
        public static final double ELEVATOR_A = 0;
        public static final double ELEVATOR_CONVERSION = 1.889880952380952380952380952381; // In rotations to inches
        public static final double ELEVATOR_FW_LIMIT = 26;
        public static final double ELEVATOR_REVERSE_LIMIT = 0.0;
        public static final boolean ELEVATOR_INVERTED = true;
        public static final int ELEVATOR_CURRENT_LIMIT = 40;
        public static final double ELEVATOR_MAX_SPEED = 800;
        public static final double ELEVATOR_MAX_ACCEL = 800;

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
        public static final double ARM_P = 0.2;
        public static final double ARM_I = 0.5;
        public static final double ARM_D = 0.0325;
        public static final double ARM_S = 0.0;
        public static final double ARM_V = 2.0;
        public static final double ARM_A = 0.19;
        public static final double ARM_G = 0.54;

        public static final double ARM_MAX_SPEED = 580;
        public static final double ARM_MAX_ACCEL = 600;

        // Subsystem Infox`
        public static final double ARM_LENGTH = 0.9652;
        public static final double SAFE_ARM_ANGLE = 90.0;

    }

    public static final class IndexerConstants {
        public static final int LEFT_INDEXER_MOTOR_CAN_ID = 61;
        public static final int RIGHT_INDEXER_MOTOR_CAN_ID = 62;
        public static final int INDEXER_RANGE_CAN_ID = 60;

        public static final double INDEXER_CORAL_THRESHOLD = 0.07;

        public static final boolean INDEXER_BRAKE = false;
        public static final int INDEXER_CURRENT_LIMIT = 200;

        public static final double INDEXER_P = 0.000115;
        public static final double INDEXER_I = 0.000001;
        public static final double INDEXER_D = 0.01;

    }

    public static final class endEffectorConstants {
        public static final int ENDEFFECTOR_MOTOR_CAN_ID = 55;
        public static final int CANRANGE_CAN_ID = 56;

        // Sensor Settings
        public static final double CORAL_DETECTION_THRESHOLD = 0.067;

        // Powers
        public static final double PLUCK_POWER = 0.45;
        public static final double IDLE_WITH_CORAL = 0.05;

    }

    public static final class floorIntakeConstants {
        public static final int FLOOR_INTAKE_PIVOT_CAN_ID = 57;

        public static final int FLOOR_INTAKE_PIVOT_CURRENT_LIMIT = 80;
        public static final boolean FLOOR_INTAKE_PIVOT_INVERTED = true;
        public static final double FLOOR_INTAKE_PIVOT_TOLLERANCE = 1; // degree
        public static final double PIVOT_FW_LIMIT = 135.0; // TODO: FIX
        public static final double PIVOT_REVERSE_LIMIT = 20.0; // TODO: FIX

        public static final double FLOOR_INTAKE_PIVOT_P = 0.007400000002235174; // TODO: FIX
        // public static final double FLOOR_INTAKE_PIVOT_I = 0.000004999999873689376; // TODO: FIX
        public static final double FLOOR_INTAKE_PIVOT_I = 0.0000015999999873689376; // TODO: FIX
        public static final double FLOOR_INTAKE_PIVOT_D = 0; // TODO: FIX
        public static final double FLOOR_INTAKE_PIVOT_F = 0; // TODO: FIX

        public static final int FLOOR_INTAKE_WHEELS_CAN_ID = 58;
        public static final int FLOOR_INTAKE_WHEELS_CURRENT_LIMIT = 80;
        public static final boolean FLOOR_INTAKE_WHEELS_INVERTED = false;

        public static final double FLOOR_INTAKE_PIVOT_DEADZONE_MIN = 19; // degree
        public static final double FLOOR_INTAKE_PIVOT_DEADZONE_MAX = 25; // degree



    }
}
