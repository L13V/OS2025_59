// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.rt59.statemachine;

import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.rt59.subsystems.FloorIntakeSubsystem;
import frc.rt59.subsystems.IndexerSubsystem;

/** Add your docs here. */
public class FloorIntakeStateMachine extends SubsystemBase {
    /**
     * State
     * 
     * @param angle Angle for floor intake
     * @param nRollerPower Floor intake roller power
     * @param cRollerPower Floor intake roller power with coral
     * @param nIndexerRpm Indexer roller power
     * @param cIndexerRpm Indexer roller power with coral
     */
    public enum FloorIntakeState {
        STOW(90.0, 0.0, 0.0, 0.0, 0.0),
        DOWN(10.0, 0.5, 0.1, 5000.0, 100.0);

        public final double angle;
        public final double nRollerPower;
        public final double cRollerPower;
        public final double nIndexerRpm;
        public final double cIndexerRpm;

        FloorIntakeState(double angle, double nRollerPower, double cRollerPower, double nIndexerRpm, double cIndexerRpm) {
            this.angle = angle;
            this.nRollerPower = nRollerPower;
            this.cRollerPower = cRollerPower;
            this.nIndexerRpm = nIndexerRpm;
            this.cIndexerRpm = cIndexerRpm;
        }
    }

    private final FloorIntakeSubsystem floorintake;
    private final IndexerSubsystem indexer;

    private FloorIntakeState currentState = FloorIntakeState.STOW;
    private FloorIntakeState targetState = FloorIntakeState.STOW;
    final LoggedNetworkString currentStatePub = new LoggedNetworkString("Floor Intake/Current State");
    final LoggedNetworkString targetStatePub = new LoggedNetworkString("Floor Intake/Target State");

    public FloorIntakeStateMachine(FloorIntakeSubsystem floorintake, IndexerSubsystem indexer) {
        this.floorintake = floorintake;
        this.indexer = indexer;
    }

    public FloorIntakeSubsystem getFloorIntake() {
        return floorintake;
    }

    public IndexerSubsystem getIndexer() {
        return indexer;
    }

    public FloorIntakeState getCurrentState() {
        return currentState;
    }

    public FloorIntakeState getTargetState() {
        return targetState;
    }

    public void setTargetState(FloorIntakeState target) {
        targetState = target;
    }

    public void confirmStateReached() {
        currentState = targetState;
    }
}