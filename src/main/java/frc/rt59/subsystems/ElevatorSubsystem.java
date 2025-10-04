package frc.rt59.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    public enum ArmDirections {
        CW,
        CCW,
        NEAREST
    }

    public ElevatorSubsystem() {
        final SparkMax elevatorMotor = new SparkMax(0, MotorType.kBrushless);
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        SparkClosedLoopController elevatorPID = elevatorMotor.getClosedLoopController();
    }

}
