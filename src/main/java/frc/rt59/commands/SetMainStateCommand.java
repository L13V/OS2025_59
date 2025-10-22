// SetStateCommand.java
package frc.rt59.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.rt59.statemachine.MainStateMachine;
import frc.rt59.statemachine.MainStateMachine.RobotState;
import frc.rt59.subsystems.ElevatorSubsystem;
import frc.rt59.subsystems.ArmSubsystem;
import frc.rt59.subsystems.EndEffectorSubsystem;
import frc.rt59.subsystems.ArmSubsystem.ArmDirections;

public class SetMainStateCommand extends Command {
    /*
     * OBSTACLES
     */
    private static class Obstacle {
        double startAngle;
        double endAngle;
        double minHeight;

        Obstacle(double startAngle, double endAngle, double minHeight) {
            this.startAngle = startAngle;
            this.endAngle = endAngle;
            this.minHeight = minHeight;
        }

        boolean intersects(double angle) {
            return angle >= startAngle && angle <= endAngle;
        }
    }

    // Define all obstacles
    private static final Obstacle[] OBSTACLES = {
            new Obstacle(93.5, 109.0, 5.25), // 59 plate
            new Obstacle(40.0, 82.5, 20.132) // floor intake
    };

    private double computeSafeThreshold(double initialAngle, double targetAngle) {
        double threshold = 0.0;
        for (Obstacle o : OBSTACLES) {
            boolean passesThrough = ((initialAngle < o.startAngle && targetAngle > o.endAngle) ||
                    (initialAngle > o.endAngle && targetAngle < o.startAngle) ||
                    (o.intersects(initialAngle) || o.intersects(targetAngle)));

            if (passesThrough) {
                threshold = Math.max(threshold, o.minHeight);
            }
        }
        return threshold;
    }

    /*
     * STATE CHANGE STUFF
     */

    private final MainStateMachine stateManager;
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final EndEffectorSubsystem endeffector;
    private final RobotState targetState;

    // Direction-specific elevator thresholds
    private static final double ELEVATOR_SAFE_THRESHOLD = 18;

    private double threshold = ELEVATOR_SAFE_THRESHOLD;

    // Cached target values
    private double targetElevatorPos;
    private double targetArmPos;
    private ArmDirections armDirection;

    // Internal state flags
    private boolean armCommanded = false;
    private boolean elevatorCommanded = false;
    private boolean phase1Started = false;

    public SetMainStateCommand(MainStateMachine stateManager, ElevatorSubsystem elevator, ArmSubsystem arm,
            EndEffectorSubsystem endeffector,
            RobotState targetState) {
        this.stateManager = stateManager;
        this.elevator = elevator;
        this.arm = arm;
        this.endeffector = endeffector;
        this.targetState = targetState;

        addRequirements(elevator, arm, endeffector);
    }

    public void initialize() {
        // Runs when command starts
        stateManager.setTargetState(targetState);
        armDirection = targetState.armDirection;

        // Arm
        double initalAbsArmAngle = arm.getAngleWithinRotationDegrees();
        targetArmPos = targetState.targetArmAngle;

        // Elevator
        targetElevatorPos = targetState.targetElevatorHeight;

        // Status Variables
        armCommanded = false;
        elevatorCommanded = false;
        phase1Started = false;

        // Figure out threshold based on target and inital arm angles
        threshold = computeSafeThreshold(initalAbsArmAngle, targetArmPos);

        endeffector.setPower(targetState.endEffectorPower); // Set Power once (So overrides work!!!)

    }

    public void execute() {
        double currentElevatorPos = elevator.getElevatorPos();
        double targetArmPos = targetState.targetArmAngle;

        boolean startAboveThreshold = currentElevatorPos >= threshold;
        boolean targetAboveThreshold = targetElevatorPos >= threshold;
        // --- CASE 1: Inherently safe (both above threshold) ---
        if (startAboveThreshold && targetAboveThreshold) {
            if (!elevatorCommanded) {
                elevator.setElevatorPos(targetElevatorPos);
                elevatorCommanded = true;
            }
            if (!armCommanded) {
                arm.setArmAngle(targetArmPos, armDirection);
                armCommanded = true;
            }
            return;
        }

        // --- CASE 2: Moving elevator up across threshold ---
        if (targetAboveThreshold && !startAboveThreshold) {
            if (!elevatorCommanded) {
                elevator.setElevatorPos(targetElevatorPos);
                elevatorCommanded = true;
            } else if (currentElevatorPos >= threshold && !armCommanded) {
                arm.setArmAngle(targetArmPos, armDirection);
                armCommanded = true;
            }
            return;
        }

        // --- CASE 3: Moving elevator down below threshold ---
        if (!targetAboveThreshold) {

            // Step 1: Always ensure elevator clears obstacles first
            if (!phase1Started) {
                // If we're going to pass through an obstacle, move elevator up to the threshold
                // first
                if (startAboveThreshold && currentElevatorPos < threshold) {
                    elevator.setElevatorPos(threshold);
                } else if (!startAboveThreshold) {
                    elevator.setElevatorPos(threshold);
                }
                phase1Started = true;
            }

            // Step 2: Once safe height reached, move arm to target
            if (!armCommanded && currentElevatorPos >= threshold - 0.1) {
                arm.setArmAngle(targetArmPos, armDirection);
                armCommanded = true;
            }

            // Step 3: After arm reaches target, lower elevator
            if (arm.atPosition(targetArmPos) && !elevatorCommanded) {
                elevator.setElevatorPos(targetElevatorPos);
                elevatorCommanded = true;
            }
        }

    }

    public boolean isFinished() {
        return elevator.atPosition(targetElevatorPos) && arm.atPosition(targetArmPos);
    }

    public void end(boolean interrupted) {
        if (!interrupted) {
            stateManager.confirmStateReached();
        }
    }
}
