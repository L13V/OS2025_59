package frc.rt59.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    public enum ArmDirections {
        CW,
        CCW,
        NEAREST
    }

    public ArmSubsystem() {
        final SparkMax armPivotMotor = new SparkMax(0, MotorType.kBrushless);
        SparkMaxConfig armConfig = new SparkMaxConfig();
        // armConfig.absoluteEncoder.
        AbsoluteEncoder armEncoder = armPivotMotor.getAbsoluteEncoder();
    }

}
