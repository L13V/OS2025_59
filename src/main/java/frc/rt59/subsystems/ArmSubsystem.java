package frc.rt59.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.rt59.Constants.ArmConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ArmSubsystem extends SubsystemBase {
    private enum ControlMode {
        OPEN_LOOP,
        POSITION,
        VELOCITY,
    }

    public enum ArmDirections {
        CW,
        CCW,
        NEAREST
    }

    // Hardware
    private final SparkMax armPivotMotor = new SparkMax(ArmConstants.ARM_CAN_ID, MotorType.kBrushless);
    private final RelativeEncoder armRelEncoder = armPivotMotor.getEncoder();
    private final AbsoluteEncoder armAbsEncoder = armPivotMotor.getAbsoluteEncoder();

    // Tunables
    // Tunable PID gains
    private LoggedNetworkNumber kP = new LoggedNetworkNumber("/Tuning/Arm-kP", ArmConstants.ARM_P);
    private LoggedNetworkNumber kI = new LoggedNetworkNumber("/Tuning/Arm-kI", ArmConstants.ARM_I);
    private LoggedNetworkNumber kD = new LoggedNetworkNumber("/Tuning/Arm-kD", ArmConstants.ARM_D);

    // Tunable feedforward
    private LoggedNetworkNumber kS = new LoggedNetworkNumber("/Tuning/Arm-kS", ArmConstants.ARM_S);
    private LoggedNetworkNumber kG = new LoggedNetworkNumber("/Tuning/Arm-kG", ArmConstants.ARM_G);
    private LoggedNetworkNumber kV = new LoggedNetworkNumber("/Tuning/Arm-kV", ArmConstants.ARM_V);
    private LoggedNetworkNumber kA = new LoggedNetworkNumber("/Tuning/Arm-kA", ArmConstants.ARM_A);

    // Target angle (for testing)
    private LoggedNetworkNumber targetAngleTunable = new LoggedNetworkNumber("/Tuning/Arm-TargetDegrees", 90.0);

    // Control
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            ArmConstants.ARM_MAX_SPEED, ArmConstants.ARM_MAX_ACCEL);
    private ProfiledPIDController armPID = new ProfiledPIDController(kP.get(), kI.get(),
            kD.get(), constraints);
    private ArmFeedforward feedforward = new ArmFeedforward(kS.get(), kG.get(),
            kV.get(), kA.get());

    private final SparkMaxConfig armConfig = new SparkMaxConfig();

    private ControlMode currentControlMode = ControlMode.OPEN_LOOP;
    private double targetPosition = 90.0; // degrees
    private double targetVelocity = 0.0;

    // Debug/test
    double testingTarget = 0.0;

    public ArmSubsystem() {
        // Motor config
        armConfig.idleMode(IdleMode.kBrake);
        armConfig.inverted(ArmConstants.ARM_INVERTED);
        armConfig.smartCurrentLimit(ArmConstants.ARM_CURRENT_LIMIT);
        armPivotMotor.configure(armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // Sync relative encoder with absolute (degrees -> rotations)
        armRelEncoder.setPosition(getArmAbsPosDegrees() / ArmConstants.ARM_CONVERSION);

        armPID.setTolerance(0.25); // 0.25 degree tolerance
    }

    @Override
    public void periodic() {
        armPID.setPID(kP.get(), kI.get(), kD.get());
        feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

        SmartDashboard.putNumber("Arm Absolute Encoder (deg)", getArmAbsPosDegrees());
        SmartDashboard.putNumber("Arm Relative Encoder (deg)", getArmRelPosDegrees());
        SmartDashboard.putNumber("Calculated Abosolue (deg)", getAngleWithinRotationDegrees());
        SmartDashboard.putNumber("Calculated Rotations", getRotationCount());
        SmartDashboard.putNumber("Arm Target (deg)", targetPosition);
        SmartDashboard.putNumber("Velocity (deg/s)", getVelocity());
        SmartDashboard.putNumber("Voltage (V)", getVoltage());
        SmartDashboard.putNumber("Current (A)", getCurrent());
        SmartDashboard.putNumber("Motor Temp (C)", getTemperature());
        SmartDashboard.putBoolean("OnTarget", onTarget());

        armControlLoop();
    }

    /** Main control loop */
    private void armControlLoop() {
        switch (currentControlMode) {
            case POSITION -> {
                // Gather Signals
                double currentPos = getArmRelPosDegrees(); // degrees
                double output = armPID.calculate(currentPos, targetPosition);
                double velocityDegPerSec = armPID.getSetpoint().velocity;

                // FeedForward
                double feedForwardAngle = Units.degreesToRadians(getAngleWithinRotationDegrees());
                double velocityRadPerSec = Units.degreesToRadians(velocityDegPerSec);
                double feedforwardOutput = feedforward.calculate(feedForwardAngle, velocityRadPerSec);

                // Generated Voltage (Don't go above 12!!)
                double totalVoltage = MathUtil.clamp(output + feedforwardOutput, -12, 12);
                armPivotMotor.setVoltage(totalVoltage);
                // armPivotMotor.setVoltage(output+feedforwardOutput);

                SmartDashboard.putNumber("PID Output", output);
                SmartDashboard.putNumber("FF Output", feedforwardOutput);
                SmartDashboard.putNumber("Total Voltage", totalVoltage);
            }

            case VELOCITY -> {
                double currentVel = getVelocity(); // degrees/sec
                double output = armPID.calculate(currentVel, targetVelocity);
                double accel = armPID.getSetpoint().velocity - currentVel;
                double angleRad = Units.degreesToRadians(getArmRelPosDegrees());
                double velFeedforward = feedforward.calculate(angleRad,
                        Units.degreesToRadians(targetVelocity),
                        Units.degreesToRadians(accel));

                double totalVoltage = MathUtil.clamp(output + velFeedforward, -12, 12);
                armPivotMotor.setVoltage(totalVoltage);

                SmartDashboard.putNumber("Vel PID Output", output);
                SmartDashboard.putNumber("Vel FF Output", velFeedforward);
                SmartDashboard.putNumber("Vel Total Voltage", totalVoltage);
            }

            case OPEN_LOOP -> {
                // Manual control mode — voltage set directly
            }
        }
    }

    /**
     * Finds the nearest rotation target in degrees (Checking for either a
     * counterclockwise, clockwise, or nearest solution is easier)
     * 
     * @param targetWithin Target position to reach (Within the current rotation,
     *                     ex: 760 degrees)
     * @param current      Current position (Relative, ex: 720 degrees)
     */
    private double findNearestPosition(double targetWithin, double current) {
        double error = targetWithin - current;
        if (error > 180)
            targetWithin -= 360;
        if (error < -180)
            targetWithin += 360;
        return targetWithin;
    }

    public boolean onTarget() {
        double error = Math.abs(targetPosition - getArmRelPosDegrees());
        return error < 1.0;
    }

    /**
     * Set a raw target angle in degrees
     */
    public void setRawAngle(double angleDegrees) {
        armPID.reset(getArmRelPosDegrees()); // Fixes jolt at the beginning of the loop
        targetPosition = angleDegrees;
        currentControlMode = ControlMode.POSITION;
    }

    public void useNTAngle() {
        armPID.reset(getArmRelPosDegrees()); // Fixes jolt at the beginning of the loop
        targetPosition = targetAngleTunable.get();
        currentControlMode = ControlMode.POSITION;
    }

    /** Set motor voltage directly */
    public void setRawVoltage(double voltage) {
        currentControlMode = ControlMode.OPEN_LOOP;
        armPivotMotor.setVoltage(voltage);
    }

    /*
     * Getters
     */

    public double getArmAbsPosDegrees() {
        return armAbsEncoder.getPosition();
    }

    public double getArmRelPosDegrees() {
        return armRelEncoder.getPosition() * ArmConstants.ARM_CONVERSION;
    }

    /**
     * Gets the current angle of the arm, within the range of 0-360 degrees
     * 
     * @return current angle
     * 
     */
    public double getAngleWithinRotationDegrees() {
        double angle = getArmRelPosDegrees() % 360.0;
        return (angle < 0) ? angle + 360 : angle;
    }

    public int getRotationCount() {
        return (int) ((getArmRelPosDegrees() - getAngleWithinRotationDegrees()) / 360.0);
    }

    /*
     * Motor Telemetry
     */

    public double getVelocity() {
        return armRelEncoder.getVelocity() * ArmConstants.ARM_CONVERSION / 60.0; // RPM → deg/s
    }

    public double getVoltage() {
        return armPivotMotor.getAppliedOutput() * armPivotMotor.getBusVoltage();
    }

    public double getCurrent() {
        return armPivotMotor.getOutputCurrent();
    }

    public double getTemperature() {
        return armPivotMotor.getMotorTemperature();
    }

    /*
     * Commands
     */
    public Command setAngleCommand(double angleDegrees) {
        return runOnce(() -> setRawAngle(angleDegrees));
    }

    public Command useSetAngleCommand() {
        return runOnce(() -> useNTAngle());
    }

    public Command stopCommand() {
        return runOnce(() -> setRawVoltage(0));
    }

    public void close() {
        armPivotMotor.close();
    }
}
