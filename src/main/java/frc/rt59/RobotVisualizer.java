package frc.rt59;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import frc.rt59.subsystems.ArmSubsystem;
import frc.rt59.subsystems.ElevatorSubsystem;

public class RobotVisualizer extends SubsystemBase {
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final Supplier<Pose2d> robotPoseSupplier;

    public RobotVisualizer(ElevatorSubsystem elevator, ArmSubsystem arm, Supplier<Pose2d> robotPoseSupplier) {
        this.elevator = elevator;
        this.arm = arm;
        this.robotPoseSupplier = robotPoseSupplier;
    }

    @Override
    public void periodic() {
        Pose2d robotPose = robotPoseSupplier.get();
        double elevatorHeight = Units.inchesToMeters(elevator.getElevatorPos()); // inches → meters
        double armAngle = Units.degreesToRadians(arm.getAngleWithinRotationDegrees());

        Logger.recordOutput("Visualizer/RobotPose", robotPose);

        Pose3d elevatorPose = new Pose3d(0, 0, elevatorHeight, new Rotation3d());
        Pose3d armPose = new Pose3d(0, 0, elevatorHeight, new Rotation3d(0, armAngle, 0));

        Logger.recordOutput("Visualizer/ComponentPoses", new Pose3d[] { elevatorPose, armPose });
    }
}
