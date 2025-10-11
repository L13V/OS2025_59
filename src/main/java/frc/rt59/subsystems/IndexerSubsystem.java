package frc.rt59.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.rt59.Constants.IndexerConstants;
import swervelib.encoders.CanAndMagSwerve;

public class IndexerSubsystem extends SubsystemBase {
    /*
     * Hardware
     */
    // Motors
    private final SparkFlex leftIndexerMotor = new SparkFlex(IndexerConstants.LEFT_INDEXER_MOTOR_CAN_ID,
            MotorType.kBrushless);
    private final SparkFlex rightIndexerMotor = new SparkFlex(IndexerConstants.RIGHT_INDEXER_MOTOR_CAN_ID,
            MotorType.kBrushless);
    private SparkFlexConfig leftIndexerConfig = new SparkFlexConfig();
    private SparkFlexConfig rightIndexerConfig = new SparkFlexConfig();
    // Sensors
    private final CANrange indexerCANRange = new CANrange(IndexerConstants.INDEXER_RANGE_CAN_ID);
    private final CANrangeConfiguration indexerCANRangeConfig = new CANrangeConfiguration();

    final LoggedNetworkBoolean hasCoral = new LoggedNetworkBoolean("Indexer/ Has Coral");


    public IndexerSubsystem() {
        // Motor Config
        leftIndexerConfig.inverted(false);
        rightIndexerConfig.inverted(true);
        leftIndexerConfig.smartCurrentLimit(IndexerConstants.INDEXER_CURRENT_LIMIT);
        rightIndexerConfig.smartCurrentLimit(IndexerConstants.INDEXER_CURRENT_LIMIT);
        leftIndexerConfig.idleMode(IdleMode.kCoast);
        rightIndexerConfig.idleMode(IdleMode.kCoast);

        leftIndexerMotor.configure(leftIndexerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        rightIndexerMotor.configure(rightIndexerConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

        // CANRangea
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

    public void close() {
        leftIndexerMotor.close();
        rightIndexerMotor.close();
    }

}
