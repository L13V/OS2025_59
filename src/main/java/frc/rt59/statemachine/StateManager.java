// StateManager.java
package frc.rt59.statemachine;

import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.rt59.commands.SetStateCommand;
import frc.rt59.subsystems.ArmSubsystem;
import frc.rt59.subsystems.ElevatorSubsystem;
import frc.rt59.subsystems.EndEffectorSubsystem;
import frc.rt59.subsystems.IndexerSubsystem;
import frc.rt59.subsystems.ArmSubsystem.ArmDirections;

public class StateManager extends SubsystemBase {

    // Enum that defines all possible robot states
    public enum RobotState {
        PLUCK(0, 96.0, ArmDirections.NEAREST),
        STARTING(0, 90.0, ArmDirections.NEAREST),
        STOW(5, 96.0, ArmDirections.NEAREST),
        L1(0, 90.0, ArmDirections.NEAREST),
        L2(15.0, 30.0, ArmDirections.NEAREST),
        L3(17.5, 200, ArmDirections.NEAREST),
        L4(15, 270, ArmDirections.NEAREST);

        // Each state stores its own parameters
        public final double targetElevatorHeight;
        public final double targetArmAngle;
        public final ArmDirections armDirection;

        // Constructor to set values for each state
        RobotState(double elevatorHeight, double armAngle, ArmDirections armDirection) {
            this.targetElevatorHeight = elevatorHeight;
            this.targetArmAngle = armAngle;
            this.armDirection = armDirection;
        }
    }

    // References to your subsystems
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final IndexerSubsystem indexer;
    private final EndEffectorSubsystem endeffector;

    // Track the current and target states
    private RobotState currentState = RobotState.STARTING;
    private RobotState targetState = RobotState.STARTING;
    final LoggedNetworkString currentStatePub = new LoggedNetworkString("State Machine/Current State");
    final LoggedNetworkString targetStatePub = new LoggedNetworkString("State Machine/Target State");

    // Constructor takes subsystem references (for convenience)
    public StateManager(ElevatorSubsystem elevator, ArmSubsystem arm, IndexerSubsystem indexer,
            EndEffectorSubsystem endeffector) {
        this.elevator = elevator;
        this.arm = arm;
        this.indexer = indexer;
        this.endeffector = endeffector;
    }

    // Accessors for subsystems
    public ElevatorSubsystem getElevator() {
        return elevator;
    }

    public ArmSubsystem getArm() {
        return arm;
    }

    // Getters and setters for state tracking
    public RobotState getCurrentState() {
        return currentState;
    }

    public RobotState getTargetState() {
        return targetState;
    }

    public void setTargetState(RobotState target) {
        targetState = target;
    }

    public void confirmStateReached() {
        currentState = targetState;
    }

    public void periodic() {
        // On first enable, stow robot
        if (currentState == RobotState.STARTING && targetState != RobotState.STOW) {
            new SetStateCommand(this, elevator, arm, RobotState.STOW).schedule();
        }
        // Check for coral in order to pluck
        if ((currentState == RobotState.STOW) && indexer.hasCoral() && !endeffector.hasCoral()
                && targetState != RobotState.PLUCK) {
            new SetStateCommand(this, elevator, arm, RobotState.PLUCK).schedule();
        }
        // Check for endeffector coral after pluck
        if ((currentState == RobotState.PLUCK) && !indexer.hasCoral() && endeffector.hasCoral()
                && targetState != RobotState.STOW) {
            new SetStateCommand(this, elevator, arm, RobotState.STOW).schedule();
        }

        switch (targetState) {
            case STOW -> {
                indexer.setRpm(2000);
                endeffector.setPower(0.05);
            }
            case PLUCK -> {
                indexer.stop();
                endeffector.setPower(0.2);
            }
            default -> {

            }
        }
        currentStatePub.set(currentState.name());
        targetStatePub.set(targetState.name());

    }
}
