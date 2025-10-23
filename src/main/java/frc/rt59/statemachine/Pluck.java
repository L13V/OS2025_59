// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.rt59.statemachine;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.rt59.statemachine.MainStateMachine.RobotState;
import frc.rt59.subsystems.ArmSubsystem;
import frc.rt59.subsystems.ElevatorSubsystem;
import frc.rt59.subsystems.EndEffectorSubsystem;
import frc.rt59.subsystems.IndexerSubsystem;
import frc.rt59.commands.SetMainStateCommand;

/** Add your docs here. */
public class Pluck extends SubsystemBase {

    private final MainStateMachine state;
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final IndexerSubsystem indexer;
    private final EndEffectorSubsystem endeffector;

    public Pluck(MainStateMachine state, ElevatorSubsystem elevator, ArmSubsystem arm, IndexerSubsystem indexer,
            EndEffectorSubsystem endeffector) {
        this.state = state;
        this.elevator = elevator;
        this.arm = arm;
        this.indexer = indexer;
        this.endeffector = endeffector;
    }

    public void periodic() {
        // Only trigger pluck if current state is already CORAL_STOW and target is also
        // CORAL_STOW
        if (state.getCurrentState() == RobotState.CORAL_STOW
                && state.getTargetState() != RobotState.PLUCK
                && indexer.hasCoral() && !endeffector.hasCoral()) {
            new SetMainStateCommand(state, elevator, arm, endeffector, RobotState.PLUCK).schedule();
        }

        // Only return to stow if we're currently in PLUCK or the target is PLUCK
        if ((state.getCurrentState() == RobotState.PLUCK || state.getTargetState() == RobotState.PLUCK)
                && endeffector.hasCoral() && state.getTargetState() != RobotState.CORAL_STOW) {

            new SetMainStateCommand(state, elevator, arm, endeffector, RobotState.CORAL_STOW).schedule();
        }
    }
}
