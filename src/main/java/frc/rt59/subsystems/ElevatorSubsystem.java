package frc.rt59.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.rt59.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private enum ControlMode {
        OPEN_LOOP,
        POSITION,
        VELOCITY,
    }

    /*
     * Hardware
     */
    final SparkMax elevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_CAN_ID, MotorType.kBrushless);

    /*
     * Tunables/ Targets
     */
    // Tunable PID gains
    private LoggedNetworkNumber kP = new LoggedNetworkNumber("/Elevator/Tuning/kP", ElevatorConstants.ELEVATOR_P);
    private LoggedNetworkNumber kI = new LoggedNetworkNumber("/Elevator/Tuning/kI", ElevatorConstants.ELEVATOR_I);
    private LoggedNetworkNumber kD = new LoggedNetworkNumber("/Elevator/Tuning/kD", ElevatorConstants.ELEVATOR_D);

    // Tunable feedforward
    private LoggedNetworkNumber kS = new LoggedNetworkNumber("/Elevator/Tuning/kS", ElevatorConstants.ELEVATOR_S);
    private LoggedNetworkNumber kG = new LoggedNetworkNumber("/Elevator/Tuning/kG", ElevatorConstants.ELEVATOR_G);
    private LoggedNetworkNumber kV = new LoggedNetworkNumber("/Elevator/Tuning/kV", ElevatorConstants.ELEVATOR_V);
    private LoggedNetworkNumber kA = new LoggedNetworkNumber("/Elevator/Tuning/kA", ElevatorConstants.ELEVATOR_A);

    /*
     * Control Loops/ PIDs
     */
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            ElevatorConstants.ELEVATOR_MAX_SPEED, ElevatorConstants.ELEVATOR_MAX_ACCEL);
    private ProfiledPIDController elevatorPID = new ProfiledPIDController(kP.get(), kI.get(),
            kD.get(), constraints);
    private ElevatorFeedforward feedforward = new ElevatorFeedforward(kS.get(), kG.get(),
            kV.get(), kA.get());

    private SparkMaxConfig elevatorConfig = new SparkMaxConfig();

    private ControlMode currentControlMode = ControlMode.OPEN_LOOP;
    private double targetPosition = 0.0; // inches
    private double targetVelocity = 0.0; // inches

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

    final LoggedNetworkNumber PIDOutputPub = new LoggedNetworkNumber("Elevator/PID Output (V)");
    final LoggedNetworkNumber FFOutputPub = new LoggedNetworkNumber("Elevator/FF Output (V)");
    final LoggedNetworkNumber TotalVoltagePub = new LoggedNetworkNumber("Elevator/Total Voltage (V)");

    final LoggedNetworkNumber VelPIDOutputPub = new LoggedNetworkNumber("Elevator/Velocity PID Output (V)");
    final LoggedNetworkNumber VelFFOutputPub = new LoggedNetworkNumber("Elevator/Velocity FF Output (V)");

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
        elevatorConfig.softLimit.forwardSoftLimit(ElevatorConstants.ELEVATOR_FW_LIMIT * ElevatorConstants.ELEVATOR_CONVERSION);
        elevatorConfig.softLimit.reverseSoftLimitEnabled(true);
        elevatorConfig.softLimit.reverseSoftLimit(ElevatorConstants.ELEVATOR_REVERSE_LIMIT * ElevatorConstants.ELEVATOR_CONVERSION);
        // Apply Config!
        elevatorMotor.configure(elevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        elevatorPID.setTolerance(0.1); // 0.1 inch tolerance
    }

    public void periodic() {
        elevatorPID.setPID(kP.get(), kI.get(), kD.get());
        feedforward = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

        elevatorPosPub.set(getElevatorPos());
        elevatorTargetPub.set(targetPosition);
        elevatorVelocityPub.set(getVelocity());
        elevatorVoltagePub.set(getVoltage());
        elevatorCurrentPub.set(getCurrent());
        elevatorTempPub.set(getTemperature());
        elevatorOnTargetPub.set(onTarget());

        elevatorControlLoop();
    }

    private void elevatorControlLoop() {
        switch (currentControlMode) {
            case POSITION -> {
                // Gather Signals
                double currentPos = getElevatorPos();
                double output = elevatorPID.calculate(currentPos, targetPosition);
                double velocity = elevatorPID.getSetpoint().velocity;
                double feedforwardOutput = feedforward.calculate(velocity);

                // Generated Voltage (Don't go above 12!!)
                double totalVoltage = MathUtil.clamp(output + feedforwardOutput, -12, 12);
                elevatorMotor.setVoltage(totalVoltage);
                // NT
                PIDOutputPub.set(output);
                FFOutputPub.set(feedforwardOutput);
                TotalVoltagePub.set(totalVoltage);
            }

            case VELOCITY -> {
                double currentVel = getVelocity();
                double velOutput = elevatorPID.calculate(currentVel, targetVelocity);
                double accel = elevatorPID.getSetpoint().velocity - currentVel;
                double velFeedforwardOutput = feedforward.calculate(
                        targetVelocity,
                        accel);

                double velocityVoltage = MathUtil.clamp(velOutput + velFeedforwardOutput, -12, 12);
                elevatorMotor.setVoltage(velocityVoltage);
                // NT
                TotalVoltagePub.set(velocityVoltage);
                VelPIDOutputPub.set(velOutput);
                VelFFOutputPub.set(velFeedforwardOutput);
            }

            case OPEN_LOOP -> {
                // Manual control mode — voltage set directly
            }
        }
    }

    public void setElevatorPos(Double target) {
        elevatorPID.reset(getElevatorPos()); // Fixes jolt at the beginning of the loop

        if (target >= ElevatorConstants.ELEVATOR_REVERSE_LIMIT
                && target <= ElevatorConstants.ELEVATOR_FW_LIMIT) {
            targetPosition = target;
            currentControlMode = ControlMode.POSITION;
        }

    }

    public void setRawVoltage(double voltage) {
        currentControlMode = ControlMode.OPEN_LOOP;
        elevatorMotor.setVoltage(voltage);
    }

    /*
     * Getters
     */

    public double getPIDTarget() {
        return targetPosition;
    }

    public double getElevatorPos() {
        return (elevatorMotor.getEncoder().getPosition() / ElevatorConstants.ELEVATOR_CONVERSION);
    }

    public double getVelocity() {
        return (elevatorMotor.getEncoder().getVelocity() / ElevatorConstants.ELEVATOR_CONVERSION);
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

    public boolean onTarget() {
        double error = Math.abs(targetPosition - getElevatorPos());
        return error < 0.1;
    }

    public boolean atPosition(double test) {
        double error = Math.abs(test - getElevatorPos());
        return error < 0.2;
    }

    /*
     * Commands
     */
    public Command setElevatorPosCommand(double position) {
        return runOnce(() -> setElevatorPos(position));
    }

    public Command stopCommand() {
        return runOnce(() -> setRawVoltage(0.0));
    }

    public void close() {
        elevatorMotor.close();
    }
}
