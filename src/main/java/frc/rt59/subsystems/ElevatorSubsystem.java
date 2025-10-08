package frc.rt59.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.rt59.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    /*
     * Elevator Motor, Config, and ClosedLopp Controller
     */
    final SparkMax elevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_CAN_ID, MotorType.kBrushless);
    SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    SparkClosedLoopController elevatorPID = elevatorMotor.getClosedLoopController();

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
    public void moveToPos(Double target) {
        elevatorPID.setReference((target * ElevatorConstants.ELEVATOR_CONVERSION),ControlType.kPosition);
    }

}
