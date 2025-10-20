// SetStateCommand.java
package frc.rt59.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.rt59.statemachine.IntakeStateMachine.IntakeState;
import frc.rt59.statemachine.IntakeStateMachine;

import frc.rt59.subsystems.FloorIntakeSubsystem;
import frc.rt59.subsystems.IndexerSubsystem;

public class SetIntakeStateCommand extends Command {

    /*
     * STATE CHANGE STUFF
     */

    private final IntakeStateMachine intakeStateMachine;
    private final FloorIntakeSubsystem floorintake;
    private final IndexerSubsystem indexer;
    private final IntakeState targetState;

    public SetIntakeStateCommand(IntakeStateMachine intakeStateMachine, FloorIntakeSubsystem floorintake, IndexerSubsystem indexer,
            IntakeState targetState) {
        this.intakeStateMachine = intakeStateMachine;
        this.floorintake = floorintake;
        this.indexer = indexer;
        this.targetState = targetState;
        addRequirements(floorintake, indexer);
    }

    @Override
    public void initialize() {
        // Runs when command starts
        intakeStateMachine.setTargetState(targetState);
        floorintake.setPivotAngle(targetState.angle);
        floorintake.setWheelPower(targetState.nRollerPower);
        indexer.setPower(targetState.nIndexerPower);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return floorintake.atPivotPosition(targetState.angle);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            intakeStateMachine.confirmStateReached();
        }
    }
}
