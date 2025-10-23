// StateManager.java
package frc.rt59.statemachine;

import java.sql.Driver;

import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.rt59.commands.SetMainStateCommand;
import frc.rt59.subsystems.ArmSubsystem;
import frc.rt59.subsystems.ElevatorSubsystem;
import frc.rt59.subsystems.EndEffectorSubsystem;
import frc.rt59.subsystems.IndexerSubsystem;
import frc.rt59.subsystems.ArmSubsystem.ArmDirections;
import frc.rt59.Constants.endEffectorConstants;

public class MainStateMachine extends SubsystemBase {

    // Enum that defines all possible robot states
    public enum RobotState {
        STARTING(0, 90.0, ArmDirections.NEAREST, 0),
        CORAL_STOW(5, 90.0, ArmDirections.NEAREST, endEffectorConstants.IDLE_WITH_CORAL),
        AUTO_CORAL_STOW(5, 90.0, ArmDirections.NEAREST, endEffectorConstants.IDLE_WITH_CORAL),
        BALL_STOW(0, 270, ArmDirections.CCW, endEffectorConstants.IDLE_WITH_BALL),
        BALL_LOW_INTAKE(0, 180, ArmDirections.NEAREST, endEffectorConstants.INTAKE_BALL),
        BALL_HIGH_INTAKE(10, 180, ArmDirections.NEAREST, endEffectorConstants.INTAKE_BALL),
        PLUCK(0, 90.0, ArmDirections.NEAREST, endEffectorConstants.PLUCK_POWER),
        MANUAL_PLUCK(0, 90.0, ArmDirections.NEAREST, endEffectorConstants.PLUCK_POWER),
        // L1
        L1(8, 139.0, ArmDirections.NEAREST, endEffectorConstants.IDLE_WITH_CORAL),
        L1_SCORE(8, 139.0, ArmDirections.NEAREST, -0.32),
        // L2
        L2(16.5, 111.5, ArmDirections.NEAREST, endEffectorConstants.IDLE_WITH_CORAL),
        L2_SCORE(16.5, 137.0, ArmDirections.NEAREST, -0.2),
        // L3
        L3(1, 214, ArmDirections.NEAREST, endEffectorConstants.IDLE_WITH_CORAL),
        L3_SCORE(1, 194, ArmDirections.NEAREST, -0.1),
        // L4
        L4(26, 214.0, ArmDirections.NEAREST, endEffectorConstants.IDLE_WITH_CORAL),
        L4_SCORE(26, 191.0, ArmDirections.NEAREST, -0.175);

        // Each state stores its own parameters
        public final double targetElevatorHeight;
        public final double targetArmAngle;
        public final ArmDirections armDirection;
        public final double endEffectorPower;
        

        // Constructor to set values for each state
        RobotState(double elevatorHeight, double armAngle, ArmDirections armDirection, double endEffectorPower) {
            this.targetElevatorHeight = elevatorHeight;
            this.targetArmAngle = armAngle;
            this.armDirection = armDirection;
            this.endEffectorPower = endEffectorPower;
        }

        public RobotState getScoringState() {
            return switch (this) {
                case L1 -> L1_SCORE;
                case L2 -> L2_SCORE;
                case L3 -> L3_SCORE;
                case L4 -> L4_SCORE;
                default -> this;
            };
        }

        public RobotState getUnscoringState() {
            return switch (this) {
                case L1_SCORE -> L1;
                case L2_SCORE -> L2;
                case L3_SCORE -> L3;
                case L4_SCORE -> L4;
                default -> this;
            };
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
    public MainStateMachine(ElevatorSubsystem elevator, ArmSubsystem arm, IndexerSubsystem indexer,
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
        if (getTargetState() == RobotState.STARTING) {
            if (DriverStation.isTeleopEnabled()) {
                new SetMainStateCommand(this, elevator, arm, endeffector, RobotState.CORAL_STOW).schedule();
            }

            if (DriverStation.isAutonomousEnabled()) {
                new SetMainStateCommand(this, elevator, arm, endeffector, RobotState.AUTO_CORAL_STOW).schedule();
            }
        }

        if (getTargetState() == RobotState.AUTO_CORAL_STOW) {
            if (DriverStation.isTeleopEnabled()) {
                new SetMainStateCommand(this, elevator, arm, endeffector, RobotState.CORAL_STOW).schedule();
            }
        }
        currentStatePub.set(currentState.toString());
        targetStatePub.set(targetState.toString());

    }

    public void setToScoreState() {
        new SetMainStateCommand(this, elevator, arm, endeffector, getTargetState().getScoringState()).schedule();
    }

    public void setToUnscoreState() {
        new SetMainStateCommand(this, elevator, arm, endeffector, getTargetState().getUnscoringState()).schedule();
    }

    public void setEject(boolean eject) {
        if (eject) {
            endeffector.setPower(-0.2);
        } else {
            endeffector.setPower(targetState.endEffectorPower);
        }
    }

}
