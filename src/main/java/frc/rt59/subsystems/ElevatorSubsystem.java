package frc.rt59.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.rt59.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    /*
     * Hardware
     */
    final SparkMax elevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_CAN_ID, MotorType.kBrushless);
    /*
     * Control Loops/ PIDs
     */
    SparkClosedLoopController elevatorPID = elevatorMotor.getClosedLoopController();
    SparkMaxConfig elevatorConfig = new SparkMaxConfig();

    private double targetPosition = 0.0;

    /*
     * Debugging/Testing
     */
    // NT Publishers
    final LoggedNetworkNumber elevatorPosPub = new LoggedNetworkNumber("Elevator/Position (in)");
    final LoggedNetworkNumber elevatorTargetPub = new LoggedNetworkNumber("Elevator/Target (in)");
    final LoggedNetworkNumber elevatorVelocityPub = new LoggedNetworkNumber("Elevator/Velocity (rpm)");
    final LoggedNetworkNumber elevatorVoltagePub = new LoggedNetworkNumber("Elevator/Voltage (V)");
    final LoggedNetworkNumber elevatorCurrentPub = new LoggedNetworkNumber("Elevator/Current (A)");
    final LoggedNetworkNumber elevatorTempPub = new LoggedNetworkNumber("Elevator/Motor Temp (C)");
    final LoggedNetworkBoolean elevatorOnTargetPub = new LoggedNetworkBoolean("Elevator/OnTarget");

    public ElevatorSubsystem() {
        /*
         * Elevator Motor Configuration
         */
        // Basic Params
        elevatorConfig.idleMode(IdleMode.kBrake);
        elevatorConfig.inverted(ElevatorConstants.ELEVATOR_INVERTED);
        elevatorConfig.smartCurrentLimit(ElevatorConstants.ELEVATOR_CURRENT_LIMIT);
        // Limits
        elevatorConfig.softLimit.forwardSoftLimitEnabled(true);
        elevatorConfig.softLimit.forwardSoftLimit(ElevatorConstants.ELEVATOR_FW_LIMIT);
        elevatorConfig.softLimit.reverseSoftLimitEnabled(true);
        elevatorConfig.softLimit.reverseSoftLimit(ElevatorConstants.ELEVATOR_REVERSE_LIMIT);

        // ClosedLoop
        elevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        elevatorConfig.closedLoop.pidf(ElevatorConstants.ELEVATOR_P, ElevatorConstants.ELEVATOR_I,
                ElevatorConstants.ELEVATOR_D, 0.00, ClosedLoopSlot.kSlot0);
        elevatorConfig.encoder.positionConversionFactor(1);
        elevatorConfig.encoder.velocityConversionFactor(1);
        // Apply Config!
        elevatorMotor.configure(elevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void periodic() {

    }

    public boolean onTarget() {
        double error = Math.abs(targetPosition - getElevatorPos());
        return error < 0.25;
    }

    public void setElevatorPos(Double target) {
        targetPosition = target;
        elevatorPID.setReference((targetPosition * ElevatorConstants.ELEVATOR_CONVERSION), ControlType.kPosition);
    }

    /*
     * Getters
     */

    public double getPIDTarget() {
        return targetPosition;
    }

    public double getElevatorPos() {
        return (elevatorMotor.getEncoder().getPosition() * ElevatorConstants.ELEVATOR_CONVERSION);
    }

    public double getVelocity() {
        return (elevatorMotor.getEncoder().getVelocity() * ElevatorConstants.ELEVATOR_CONVERSION);
    }

    public double getVoltage() {
        return (elevatorMotor.getBusVoltage());
    }

    public double getCurrent() {
        return (elevatorMotor.getOutputCurrent());
    }

    public double getTemperature() {
        return (elevatorMotor.getMotorTemperature());
    }

    /*
     * Commands
     */
    public Command setElevatorPosCommand(double position) {
        return runOnce(() -> setElevatorPos(position));
    }
    public Command stopCommand() {
        return runOnce(() -> elevatorMotor.close());
    }
}
