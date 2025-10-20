package frc.rt59.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.rt59.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
    /*
     * Hardware
     */
    // Motors
    private final SparkFlex leftIndexerMotor = new SparkFlex(IndexerConstants.LEFT_INDEXER_MOTOR_CAN_ID,
            MotorType.kBrushless);
    private final SparkFlex rightIndexerMotor = new SparkFlex(IndexerConstants.RIGHT_INDEXER_MOTOR_CAN_ID,
            MotorType.kBrushless);

    private SparkClosedLoopController leftClosedLoopController = leftIndexerMotor.getClosedLoopController();
    private SparkClosedLoopController rightClosedLoopController = rightIndexerMotor.getClosedLoopController();

    private SparkFlexConfig leftIndexerConfig = new SparkFlexConfig();
    private SparkFlexConfig rightIndexerConfig = new SparkFlexConfig();
    // Sensors
    private final CANrange indexerCANRange = new CANrange(IndexerConstants.INDEXER_RANGE_CAN_ID);
    private final CANrangeConfiguration indexerCANRangeConfig = new CANrangeConfiguration();

    final LoggedNetworkBoolean hasCoral = new LoggedNetworkBoolean("Indexer/Has Coral");

    public IndexerSubsystem() {
        /*
         * Motor Confifs
         */
        // Inverts
        leftIndexerConfig.inverted(false);
        rightIndexerConfig.inverted(true);
        // Current Limits
        leftIndexerConfig.smartCurrentLimit(IndexerConstants.INDEXER_CURRENT_LIMIT);
        rightIndexerConfig.smartCurrentLimit(IndexerConstants.INDEXER_CURRENT_LIMIT);
        // Idle
        leftIndexerConfig.idleMode(IdleMode.kCoast);
        rightIndexerConfig.idleMode(IdleMode.kCoast);
        // PIDs
        leftIndexerConfig.closedLoop.pid(IndexerConstants.INDEXER_P, IndexerConstants.INDEXER_I, IndexerConstants.INDEXER_D);
        rightIndexerConfig.closedLoop.pid(IndexerConstants.INDEXER_P, IndexerConstants.INDEXER_I, IndexerConstants.INDEXER_D);
        leftIndexerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        rightIndexerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        leftIndexerMotor.configure(leftIndexerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        rightIndexerMotor.configure(rightIndexerConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

        // CANRange
        indexerCANRangeConfig.ProximityParams.ProximityThreshold = IndexerConstants.INDEXER_CORAL_THRESHOLD;
        indexerCANRange.getConfigurator().apply(indexerCANRangeConfig);

    }

    public void periodic() {
        hasCoral.set(hasCoral());
    }

    /*
     * Setters
     */
    public void setPower(double power) {
        leftIndexerMotor.set(power);
        rightIndexerMotor.set(power);
    }

    public void setRpm(double rpm) {
        leftClosedLoopController.setReference(rpm,ControlType.kVelocity);
        rightClosedLoopController.setReference(rpm,ControlType.kVelocity);
    }

    public void setVoltage(double voltage) {
        leftIndexerMotor.setVoltage(voltage);
        rightIndexerMotor.setVoltage(-voltage);

    }

    public void stop() {
        leftIndexerMotor.stopMotor();
        rightIndexerMotor.stopMotor();
    }

    /*
     * MOTOR Getters
     */

    public double getLeftCurrent() {
        return leftIndexerMotor.getOutputCurrent();
    }

    public double getRightCurrent() {
        return rightIndexerMotor.getOutputCurrent();
    }

    public double getLeftTemp() {
        return leftIndexerMotor.getMotorTemperature();
    }

    public double getRightTemp() {
        return rightIndexerMotor.getMotorTemperature();
    }

    public double getLeftVelocity() {
        return leftIndexerMotor.getEncoder().getVelocity();
    }

    public double getRightVelocity() {
        return rightIndexerMotor.getEncoder().getVelocity();
    }
    /*
     * SENSOR Getters
     */

    public boolean hasCoral() {
        return indexerCANRange.getIsDetected().getValue();
    }

}
