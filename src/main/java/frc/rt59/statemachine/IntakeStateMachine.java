// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.rt59.statemachine;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import frc.rt59.Constants.floorIntakeConstants;
import frc.rt59.commands.SetIntakeStateCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.rt59.subsystems.FloorIntakeSubsystem;
import frc.rt59.subsystems.IndexerSubsystem;
import frc.rt59.subsystems.EndEffectorSubsystem;

/** Add your docs here. */
public class IntakeStateMachine extends SubsystemBase {
    /**
     * State
     * 
     * @param angle
     *                      Angle for floor intake
     * @param nRollerPower
     *                      Floor intake roller power
     * @param cRollerPower
     *                      Floor intake roller power with coral
     * @param nIndexerPower
     *                      Indexer roller power
     * @param cIndexerPower
     *                      Indexer roller power with coral
     */
    public enum IntakeState {
        STARTING(130, 0.0, 0.0, 0.0, 0.0),
        STOW(120, 0.0, 0.0, 0.0, 0.0),
        DOWN(23, 0.7, 0.0, -1, 0.0),
        DOWN_DEAD(23, 0, 0, 0, 0),
        DOWN_OUTTAKE(23, -0.5, -0.5, 0.3, 0.3);

        public final double angle;
        public final double nRollerPower;
        public final double cRollerPower;
        public final double nIndexerPower;
        public final double cIndexerPower;

        IntakeState(double angle, double nRollerPower, double cRollerPower, double nIndexerPower,
                double cIndexerPower) {
            this.angle = angle;
            this.nRollerPower = nRollerPower;
            this.cRollerPower = cRollerPower;
            this.nIndexerPower = nIndexerPower;
            this.cIndexerPower = cIndexerPower;
        }
    }

    private final FloorIntakeSubsystem floorintake;
    private final IndexerSubsystem indexer;
    private final EndEffectorSubsystem endeffector;

    private IntakeState currentState = IntakeState.STARTING;
    private IntakeState targetState = IntakeState.STARTING;

    public boolean intakeStopped = false;

    final LoggedNetworkString currentStatePub = new LoggedNetworkString("Floor Intake/Current State");
    final LoggedNetworkString targetStatePub = new LoggedNetworkString("Floor Intake/Target State");
    final LoggedNetworkBoolean intakeDisabled = new LoggedNetworkBoolean("Intake Disabled");
    final LoggedNetworkNumber intakeSetpoint = new LoggedNetworkNumber("Intake Setpoint");
    final LoggedNetworkNumber intakePosition = new LoggedNetworkNumber("Intake Position");

    public IntakeStateMachine(FloorIntakeSubsystem floorintake, IndexerSubsystem indexer,
            EndEffectorSubsystem endeffector) {
        this.floorintake = floorintake;
        this.indexer = indexer;
        this.endeffector = endeffector;
    }

    public FloorIntakeSubsystem getFloorIntake() {
        return floorintake;
    }

    public IndexerSubsystem getIndexer() {
        return indexer;
    }

    public IntakeState getCurrentState() {
        return currentState;
    }

    public IntakeState getTargetState() {
        return targetState;
    }

    public void setTargetState(IntakeState target) {
        targetState = target;
    }

    public void confirmStateReached() {
        currentState = targetState;
    }

    public void periodic() {
        if (currentState == IntakeState.STARTING && targetState != IntakeState.STOW) {
            new SetIntakeStateCommand(this, floorintake, indexer, IntakeState.STOW).schedule();
        }

        if (targetState != IntakeState.DOWN_OUTTAKE && currentState != IntakeState.DOWN_OUTTAKE) {
            if (indexer.hasCoral() || endeffector.hasCoral()) {
                indexer.setPower(targetState.cIndexerPower);
                floorintake.setWheelPower(targetState.cRollerPower);
            } else if (!indexer.hasCoral() && !endeffector.hasCoral()) {
                indexer.setPower(targetState.nIndexerPower);
                floorintake.setWheelPower(targetState.nRollerPower);
            }
        }

        /*
         * Intake dead section
         */
        if (floorintake.getPivotPIDTarget() >= floorIntakeConstants.FLOOR_INTAKE_PIVOT_DEADZONE_MIN
                && floorintake.getPivotPIDTarget() <= floorIntakeConstants.FLOOR_INTAKE_PIVOT_DEADZONE_MAX
                && floorintake.getPivotAngle() >= floorIntakeConstants.FLOOR_INTAKE_PIVOT_DEADZONE_MIN
                && floorintake.getPivotAngle() <= floorIntakeConstants.FLOOR_INTAKE_PIVOT_DEADZONE_MAX
                && !intakeStopped) {
            floorintake.stopPivot();
            floorintake.pivotCoast();
            intakeStopped = true;

        }
        if ((floorintake.getPivotAngle() <= floorIntakeConstants.FLOOR_INTAKE_PIVOT_DEADZONE_MIN
                || floorintake.getPivotAngle() >= floorIntakeConstants.FLOOR_INTAKE_PIVOT_DEADZONE_MAX)
                && (floorintake.getPivotPIDTarget() <= floorIntakeConstants.FLOOR_INTAKE_PIVOT_DEADZONE_MIN
                        || floorintake.getPivotPIDTarget() >= floorIntakeConstants.FLOOR_INTAKE_PIVOT_DEADZONE_MAX)
                && intakeStopped) {
            intakeStopped = false;
            floorintake.pivotBrake();
        }

        targetStatePub.set(targetState.toString());
        currentStatePub.set(currentState.toString());
        intakeDisabled.set(intakeStopped);
        intakeSetpoint.set(floorintake.getPivotPIDTarget());
        intakePosition.set(floorintake.getPivotAngle());

    }

    public void initializeState() {
        // Ensure mechanisms start in a known safe position
        currentState = IntakeState.STOW;
        targetState = IntakeState.STOW;

        floorintake.setPivotAngle(IntakeState.STOW.angle);
        floorintake.setWheelPower(IntakeState.STOW.nRollerPower);
        indexer.setPower(IntakeState.STOW.nIndexerPower);
    }
}