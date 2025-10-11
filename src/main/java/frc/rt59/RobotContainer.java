// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.rt59;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.rt59.Constants.DriverControlConstants;
import frc.rt59.commands.SetStateCommand;
import frc.rt59.statemachine.StateManager;
import frc.rt59.statemachine.StateManager.RobotState;
import frc.rt59.subsystems.ArmSubsystem;
import frc.rt59.subsystems.ElevatorSubsystem;
import frc.rt59.subsystems.EndEffectorSubsystem;
import frc.rt59.subsystems.IndexerSubsystem;
import frc.rt59.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {
    /*
     * Controllers
     */
    final CommandXboxController driverXbox = new CommandXboxController(0);
    final CommandXboxController operatorXbox = new CommandXboxController(1);

    /*
     * Subsystems
     */
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));
    private final ArmSubsystem m_arm = new ArmSubsystem();
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    private final IndexerSubsystem m_indexer = new IndexerSubsystem();
    private final EndEffectorSubsystem m_endeffector = new EndEffectorSubsystem();
    private final StateManager m_statemanager = new StateManager(m_elevator, m_arm, m_indexer, m_endeffector);

    
    // private final SetStateCommand m_statemachine = new
    // SetStateCommand(m_statemanager, m_elevator, m_arm,RobotState.STOW);

    /**
     * Drive Code
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> driverXbox.getLeftY() * -1,
            () -> driverXbox.getLeftX() * -1)
            .withControllerRotationAxis(driverXbox::getRightX)
            .deadband(DriverControlConstants.DEADBAND)
            .scaleTranslation(0.8)
            .scaleRotation(-1)
            .allianceRelativeControl(true);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure bindings
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
        /*
         * PathPlanner Commands
         */
        // NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * Control Bindings
     */
    private void configureBindings() {
        /*
         * Setup Drive Code
         */
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

        /*
         * Standard Controls
         */
        driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
        driverXbox.back().whileTrue(drivebase.centerModulesCommand());
        // driverXbox.a().onTrue(m_arm.setArmAngleCommand(30, ArmDirections.NEAREST));
        // driverXbox.b().onTrue(m_arm.setArmAngleCommand(90, ArmDirections.NEAREST));
        // driverXbox.x().onTrue(m_arm.setArmAngleCommand(180, ArmDirections.NEAREST));
        // driverXbox.y().onTrue(m_arm.setArmAngleCommand(270, ArmDirections.NEAREST));

        // driverXbox.povUp().onTrue(m_elevator.setElevatorPosCommand(20));
        driverXbox.povDown().onTrue(m_elevator.setElevatorPosCommand(0));
        driverXbox.povLeft().onTrue(m_elevator.setElevatorPosCommand(10));
        driverXbox.povRight().onTrue(m_elevator.setElevatorPosCommand(16));

        driverXbox.a().onTrue(new SetStateCommand(m_statemanager, m_elevator, m_arm, RobotState.STOW));
        driverXbox.b().onTrue(new SetStateCommand(m_statemanager, m_elevator, m_arm, RobotState.L3));
        driverXbox.x().onTrue(new SetStateCommand(m_statemanager, m_elevator, m_arm, RobotState.L2));
        driverXbox.y().onTrue(new SetStateCommand(m_statemanager, m_elevator, m_arm, RobotState.L4));
    }

    /*
     * Autonomous
     */
    public Command getAutonomousCommand() {
        return drivebase.getAutonomousCommand("Forward");
        // return null;
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
