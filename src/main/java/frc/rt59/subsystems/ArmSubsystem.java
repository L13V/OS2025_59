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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.rt59.Constants.ArmConstants;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
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

    /*
     * Hardware
     */
    private final SparkMax armPivotMotor = new SparkMax(ArmConstants.ARM_CAN_ID, MotorType.kBrushless);
    private final RelativeEncoder armRelEncoder = armPivotMotor.getEncoder();
    private final AbsoluteEncoder armAbsEncoder = armPivotMotor.getAbsoluteEncoder();

    /*
     * Tunables/ Targets
     */
    // Tunable PID gains
    private LoggedNetworkNumber kP = new LoggedNetworkNumber("/Arm/Tuning/kP", ArmConstants.ARM_P);
    private LoggedNetworkNumber kI = new LoggedNetworkNumber("/Arm/Tuning/kI", ArmConstants.ARM_I);
    private LoggedNetworkNumber kD = new LoggedNetworkNumber("/Arm/Tuning/kD", ArmConstants.ARM_D);

    // Tunable feedforward
    private LoggedNetworkNumber kS = new LoggedNetworkNumber("/Arm/Tuning/kS", ArmConstants.ARM_S);
    private LoggedNetworkNumber kG = new LoggedNetworkNumber("/Arm/Tuning/kG", ArmConstants.ARM_G);
    private LoggedNetworkNumber kV = new LoggedNetworkNumber("/Arm/Tuning/kV", ArmConstants.ARM_V);
    private LoggedNetworkNumber kA = new LoggedNetworkNumber("/Arm/Tuning/kA", ArmConstants.ARM_A);

    // Target angle (for testing)
    private LoggedNetworkNumber targetAngleTunable = new LoggedNetworkNumber("/Arm/TargetDegrees", 90.0);

    // Control Loops/ PIDs
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

    /*
     * Debugging/Testing
     */
    double testingTarget = 0.0;

    // NT Publishers
    final LoggedNetworkNumber armAbsPosPub = new LoggedNetworkNumber("Arm/Absolute Encoder (deg)");
    final LoggedNetworkNumber armRelPosPub = new LoggedNetworkNumber("Arm/Relative Encoder (deg)");
    final LoggedNetworkNumber armCalcAbsPub = new LoggedNetworkNumber("Arm/Calculated Absolute (deg)");
    final LoggedNetworkNumber armCalcRotPub = new LoggedNetworkNumber("Arm/Calculated Rotations");
    final LoggedNetworkNumber armTargetPub = new LoggedNetworkNumber("Arm/Target (deg)");
    final LoggedNetworkNumber armVelocityPub = new LoggedNetworkNumber("Arm/Velocity (deg-s)");
    final LoggedNetworkNumber armVoltagePub = new LoggedNetworkNumber("Arm/Voltage (V)");
    final LoggedNetworkNumber armCurrentPub = new LoggedNetworkNumber("Arm/Current (A)");
    final LoggedNetworkNumber armTempPub = new LoggedNetworkNumber("Arm/Motor Temp (C)");
    final LoggedNetworkBoolean armOnTargetPub = new LoggedNetworkBoolean("Arm/OnTarget");

    final LoggedNetworkNumber PIDOutputPub = new LoggedNetworkNumber("Arm/PID Output (V)");
    final LoggedNetworkNumber FFOutputPub = new LoggedNetworkNumber("Arm/FF Output (V)");
    final LoggedNetworkNumber TotalVoltagePub = new LoggedNetworkNumber("Arm/Total Voltage (V)");

    final LoggedNetworkNumber VelPIDOutputPub = new LoggedNetworkNumber("Arm/Velocity PID Output (V)");
    final LoggedNetworkNumber VelFFOutputPub = new LoggedNetworkNumber("Arm/Velocity FF Output (V)");

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
        armPID.setPID(kP.get(), kI.get(), kD.get()); // Update PID
        feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

        armAbsPosPub.set(getArmAbsPosDegrees());
        armRelPosPub.set(getArmRelPosDegrees());
        armCalcAbsPub.set(getAngleWithinRotationDegrees());
        armCalcRotPub.set(getRotationCount());
        armTargetPub.set(targetPosition);
        armVelocityPub.set(getVelocity());
        armVoltagePub.set(getVoltage());
        armCurrentPub.set(getCurrent());
        armTempPub.set(getTemperature());
        armOnTargetPub.set(onTarget());
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
                // NT
                PIDOutputPub.set(output);
                FFOutputPub.set(feedforwardOutput);
                TotalVoltagePub.set(totalVoltage);
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
                // NT
                TotalVoltagePub.set(totalVoltage);
                VelPIDOutputPub.set(output);
                VelFFOutputPub.set(velFeedforward);
            }

            case OPEN_LOOP -> {
                // Manual control mode — voltage set directly
            }
        }
    }
    public boolean onTarget() {
        double error = Math.abs(targetPosition - getArmRelPosDegrees());
        return error < 0.25;
    }

    /**
     * Set a raw target angle in degrees
     */
    public void setRawAngle(double angleDegrees) {
        armPID.reset(getArmRelPosDegrees()); // Fixes jolt at the beginning of the loop
        targetPosition = angleDegrees;
        currentControlMode = ControlMode.POSITION;
    }

    public void setArmAngle(double inputAngle, ArmDirections direction) { // Input: 300
        final double initalarmAngle = getArmRelPosDegrees(); // 760
        final double currentRotationStartAngle = (initalarmAngle - (initalarmAngle % 360)); // 720
        final double TargetPosWithinRotation = currentRotationStartAngle + inputAngle; // 1020
        double calculatedTarget = TargetPosWithinRotation; // 1020

        armPID.reset(initalarmAngle); // Fixes jolt at the beginning of the loop
        switch (direction) {
            case NEAREST -> {
                double travelToNearest = calculatedTarget - initalarmAngle; // 1020-760=260
                if (travelToNearest > 180) {
                    calculatedTarget -= 360;
                } else if (travelToNearest < -180) {
                    calculatedTarget += 360;
                }
                targetPosition = calculatedTarget;
                currentControlMode = ControlMode.POSITION;
            }

            case CW -> {
                if (calculatedTarget < initalarmAngle+2) {
                    // Then it works!
                } else {
                    calculatedTarget -= 360;
                }
                targetPosition = calculatedTarget;
                currentControlMode = ControlMode.POSITION;
                
            }

            case CCW -> {
                if (calculatedTarget > initalarmAngle-2) {
                    // Then it works!
                } else {
                    calculatedTarget += 360;
                }
                targetPosition = calculatedTarget;
                currentControlMode = ControlMode.POSITION;
            }
        }

        // targetPosition = angleDegrees;
        // currentControlMode = ControlMode.POSITION;
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
    public Command setRawAngleCommand(double angleDegrees) {
        return runOnce(() -> setRawAngle(angleDegrees));
    }
    public Command setArmAngleCommand(double inputAngle, ArmDirections direction) {
        return runOnce(() -> setArmAngle(inputAngle, direction));
    }

    public Command useNTAngleCommand() {
        return runOnce(() -> useNTAngle());
    }

    public Command stopCommand() {
        return runOnce(() -> setRawVoltage(0));
    }

    public void close() {
        armPivotMotor.close();
    }
}
