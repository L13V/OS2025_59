// // StateManager.java
// package frc.rt59.statemachine;

// import frc.rt59.subsystems.ArmSubsystem.ArmDirections;
// /* 
//  * This file stores the states as well as targets associated with them. Also houses the variable for the current stage.
//  */
// public class StateManager {
//     public enum RobotState {
//         STOW(5.0, 10.0, ArmDirections.CCW),
//         L1(5.0, 10.0, ArmDirections.CCW),
//         L2(15.0, 25.0, ArmDirections.CCW),
//         L3(30.0, 40.0, ArmDirections.CW),
//         L4(45.0, 55.0, ArmDirections.CW);

//         public final double targetElevatorHeight;
//         public final double targetArmAngle;
//         public final ArmDirections armDirections;

//         RobotState(double targetElevatorHeight, double targetArmAngle, ArmDirections armDirection) {
//             this.targetElevatorHeight = targetElevatorHeight;
//             this.targetArmAngle = targetArmAngle;
//             this.armDirections = armDirection;
//         }
//     }

//     private RobotState currentState = RobotState.STOW;
//     private RobotState targetState = RobotState.STOW;

//     public RobotState getCurrentState() {
//         return currentState;
//     }

//     public RobotState getTargetState() {
//         return targetState;
//     }

//     public void setTargetState(RobotState target) {
//         targetState = target;
//     }

//     public void confirmStateReached() {
//         currentState = targetState;
//     }
// }
