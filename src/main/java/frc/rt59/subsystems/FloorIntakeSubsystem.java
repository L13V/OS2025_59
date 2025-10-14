// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.rt59.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.rt59.Constants.floorIntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FloorIntakeSubsystem extends SubsystemBase {
    private final SparkMax floorIntakePivotMotor = new SparkMax(floorIntakeConstants.FLOOR_INTAKE_PIVOT_CAN_ID, MotorType.kBrushless);
    private final SparkMax floorIntakeWheelsMotor = new SparkMax(floorIntakeConstants.FLOOR_INTAKE_WHEELS_CAN_ID, MotorType.kBrushless);

    private SparkClosedLoopController floorIntakePivotClosedLoop = floorIntakePivotMotor.getClosedLoopController();
    private SparkMaxConfig floorIntakePivotConfig = new SparkMaxConfig();
    private SparkMaxConfig floorIntakeWheelsConfig = new SparkMaxConfig();

    private double intakePivotTargetAngle = 90.0;

    /*
     * Debugging/Testing
     */
    final LoggedNetworkNumber intakePosPub = new LoggedNetworkNumber("Floor Intake/Position (in)");
    final LoggedNetworkNumber intakeTargetPub = new LoggedNetworkNumber("Floor Intake/Target (in)");
    final LoggedNetworkNumber intakeVelocityPub = new LoggedNetworkNumber("Floor Intake/Velocity (rpm)");
    final LoggedNetworkNumber intakeVoltagePub = new LoggedNetworkNumber("Floor Intake/Voltage (V)");
    final LoggedNetworkNumber intakeCurrentPub = new LoggedNetworkNumber("Floor Intake/Current (A)");
    final LoggedNetworkNumber intakeTempPub = new LoggedNetworkNumber("Floor Intake/Motor Temp (C)");
    final LoggedNetworkBoolean intakeOnTargetPub = new LoggedNetworkBoolean("Floor Intake/OnTarget");

    final LoggedNetworkNumber TotalVoltagePub = new LoggedNetworkNumber("Floor Intake/Total Voltage (V)");

    public FloorIntakeSubsystem() {
        /*
         * Floor Intake Pivot Config
         */
        // Basic Params
        floorIntakePivotConfig.idleMode(IdleMode.kBrake);
        floorIntakePivotConfig.inverted(floorIntakeConstants.FLOOR_INTAKE_PIVOT_INVERTED);
        floorIntakePivotConfig.smartCurrentLimit(floorIntakeConstants.FLOOR_INTAKE_PIVOT_CURRENT_LIMIT);
        // Limits
        floorIntakePivotConfig.softLimit.forwardSoftLimitEnabled(true);
        floorIntakePivotConfig.softLimit.forwardSoftLimit(floorIntakeConstants.PIVOT_FW_LIMIT);
        floorIntakePivotConfig.softLimit.reverseSoftLimitEnabled(true);
        floorIntakePivotConfig.softLimit.reverseSoftLimit(floorIntakeConstants.PIVOT_REVERSE_LIMIT);
        // PID
        floorIntakePivotConfig.closedLoop.pid(floorIntakeConstants.FLOOR_INTAKE_PIVOT_P, floorIntakeConstants.FLOOR_INTAKE_PIVOT_I,
                floorIntakeConstants.FLOOR_INTAKE_PIVOT_D);
        floorIntakePivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        floorIntakePivotMotor.configure(floorIntakePivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        /*
         * Floor Intake Wheels Config
         */
        floorIntakeWheelsConfig.idleMode(IdleMode.kCoast);
        floorIntakeWheelsConfig.inverted(floorIntakeConstants.FLOOR_INTAKE_WHEELS_INVERTED);
        floorIntakeWheelsConfig.smartCurrentLimit(floorIntakeConstants.FLOOR_INTAKE_WHEELS_CURRENT_LIMIT);
        floorIntakeWheelsMotor.configure(floorIntakeWheelsConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        /*
         * Init
         */
        // Inital Position
        floorIntakePivotClosedLoop.setReference(intakePivotTargetAngle, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        intakePosPub.set(getPivotAngle());
        intakeTargetPub.set(intakePivotTargetAngle);
        intakeVelocityPub.set(getPivotVelocity());
        intakeVoltagePub.set(getPivotVoltage());
        intakeCurrentPub.set(getPivotCurrent());
        intakeTempPub.set(getPivotTemperature());
        intakeOnTargetPub.set(onPivotTarget());
        TotalVoltagePub.set(getPivotVoltage());
    }

    /*
     * Setters
     */
    public void setPivotAngle(double angle) {
        intakePivotTargetAngle = angle;
        floorIntakePivotClosedLoop.setReference(intakePivotTargetAngle, ControlType.kPosition);
    }

    public void stopPivot() {
        floorIntakePivotMotor.stopMotor();
    }

    public void setWheelPower(double power) {
        floorIntakeWheelsMotor.set(power);
    }
    public void stopWheels() {
        floorIntakeWheelsMotor.stopMotor();
    }

    /*
     * Getters
     */
    // Pivot
    public double getPivotPIDTarget() {
        return intakePivotTargetAngle;
    }

    public double getPivotAngle() {
        return floorIntakePivotMotor.getEncoder().getPosition();
    }

    public double getPivotVelocity() {
        return floorIntakePivotMotor.getEncoder().getVelocity();
    }

    public double getPivotVoltage() {
        return floorIntakePivotMotor.getBusVoltage();
    }

    public double getPivotCurrent() {
        return floorIntakePivotMotor.getOutputCurrent();
    }

    public double getPivotTemperature() {
        return floorIntakePivotMotor.getMotorTemperature();
    }

    public boolean onPivotTarget() {
        return floorIntakePivotMotor.getEncoder().getPosition() == intakePivotTargetAngle;
    }

    public boolean atPivotPosition(double test) {
        return (Math.abs(intakePivotTargetAngle - getPivotAngle()) <= floorIntakeConstants.FLOOR_INTAKE_PIVOT_TOLLERANCE);
    }
    // Wheels
    public double getWheelMotorVelocity() {
        return floorIntakeWheelsMotor.getEncoder().getVelocity();
    }

    public double getWheelMotorVoltage() {
        return floorIntakeWheelsMotor.getBusVoltage();
    }

    public double getWheelMotorCurrent() {
        return floorIntakeWheelsMotor.getOutputCurrent();
    }

    public double getWheelMotorTemperature() {
        return floorIntakeWheelsMotor.getMotorTemperature();
    }

    /* 
     * Commands
     */

}
