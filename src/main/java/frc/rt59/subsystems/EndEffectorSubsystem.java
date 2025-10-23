package frc.rt59.subsystems;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.rt59.Constants.endEffectorConstants;

public class EndEffectorSubsystem extends SubsystemBase {
    /*
     * Hardware
     */
    // Motors
    private final TalonFX endEffectorMotor = new TalonFX(endEffectorConstants.ENDEFFECTOR_MOTOR_CAN_ID);
    private final TalonFXConfiguration endEffectorMotorConfig = new TalonFXConfiguration();
    // Sensors
    private final CANrange endeffectorCANRange = new CANrange(endEffectorConstants.CANRANGE_CAN_ID);
    private final CANrangeConfiguration endeffectorCANRangeConfig = new CANrangeConfiguration();

    final LoggedNetworkBoolean hasCoral = new LoggedNetworkBoolean("End Effector/ Has Coral");

    private final Orchestra orchestra = new Orchestra("espresso.chrp");


    public EndEffectorSubsystem() {

        // Motor Config
        endEffectorMotor.getConfigurator().refresh(endEffectorMotorConfig);
        endEffectorMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        endEffectorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Sensor config
        endeffectorCANRange.getConfigurator().refresh(endeffectorCANRangeConfig);
        endeffectorCANRangeConfig.ProximityParams.ProximityThreshold = endEffectorConstants.CORAL_DETECTION_THRESHOLD;
        endeffectorCANRange.getConfigurator().apply(endeffectorCANRangeConfig);
        orchestra.addInstrument(endEffectorMotor);
        // orchestra.play();
    }

    @Override
    public void periodic() {
        hasCoral.set(hasCoral());
    }

    /*
     * Setters
     */
    public void setPower(double power) {
        endEffectorMotor.set(power);
    }

    public void stop() {
        endEffectorMotor.stopMotor();
    }

    /*
     * Getters
     */
    public double getVelocity() {
        return endEffectorMotor.getVelocity().getValueAsDouble();
    }

    public double getCurrent() {
        return endEffectorMotor.getStatorCurrent().getValueAsDouble();
    }

    public double getTemperature() {
        return endEffectorMotor.getDeviceTemp().getValueAsDouble();
    }

    public boolean hasCoral() {
        return endeffectorCANRange.getIsDetected(true).getValue();
    }

    public void close() {
        endEffectorMotor.close();
    }
}
