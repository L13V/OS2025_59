// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.rt59;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.rt59.Constants.DriverControlConstants;
import frc.rt59.commands.SetIntakeStateCommand;
import frc.rt59.commands.SetMainStateCommand;
import frc.rt59.statemachine.IntakeStateMachine;
import frc.rt59.statemachine.MainStateMachine;
import frc.rt59.statemachine.IntakeStateMachine.IntakeState;
import frc.rt59.statemachine.MainStateMachine.RobotState;
import frc.rt59.subsystems.ArmSubsystem;
import frc.rt59.subsystems.CandleSubsystem;
import frc.rt59.subsystems.ElevatorSubsystem;
import frc.rt59.subsystems.EndEffectorSubsystem;
import frc.rt59.subsystems.FloorIntakeSubsystem;
import frc.rt59.subsystems.IndexerSubsystem;
import frc.rt59.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import frc.rt59.statemachine.Pluck;

import com.pathplanner.lib.auto.NamedCommands;
import com.reduxrobotics.sensors.canandcolor.DigoutChannel.Index;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {

    boolean startupScheduled = false;
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
    private final FloorIntakeSubsystem m_floorintake = new FloorIntakeSubsystem();
    private final IndexerSubsystem m_indexer = new IndexerSubsystem();
    private final EndEffectorSubsystem m_endeffector = new EndEffectorSubsystem();
    private final MainStateMachine m_statemanager = new MainStateMachine(m_elevator, m_arm, m_indexer, m_endeffector);
    private final IntakeStateMachine m_intakestatemanager = new IntakeStateMachine(m_floorintake, m_indexer,
            m_endeffector);

    private final RobotVisualizer m_visualizer = new RobotVisualizer(m_elevator, m_arm, drivebase::getPose);
    private final CandleSubsystem m_leds = new CandleSubsystem();

    private final Pluck m_pluck = new Pluck(m_statemanager, m_elevator, m_arm, m_indexer, m_endeffector);

    // private final SetMainStateCommand m_statemachine = new
    // SetMainStateCommand(m_statemanager, m_elevator, m_arm,RobotState.STOW);

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
        // new SetMainStateCommand(m_statemanager, m_elevator, m_arm, m_endeffector,
        // RobotState.CORAL_STOW).schedule();
        // new SetIntakeStateCommand(m_intakestatemanager, m_floorintake, m_indexer,
        // IntakeState.STOW).schedule();
        // Configure bindings

        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
        /*
         * PathPlanner Commands
         */
        // L1
        NamedCommands.registerCommand("L1",
                new SetMainStateCommand(m_statemanager, m_elevator, m_arm, m_endeffector, RobotState.L1));
        NamedCommands.registerCommand("L1_Score",
                new SetMainStateCommand(m_statemanager, m_elevator, m_arm, m_endeffector, RobotState.L1_SCORE));
        // L2
        NamedCommands.registerCommand("L2",
                new SetMainStateCommand(m_statemanager, m_elevator, m_arm, m_endeffector, RobotState.L2));
        NamedCommands.registerCommand("L2_Score",
                new SetMainStateCommand(m_statemanager, m_elevator, m_arm, m_endeffector, RobotState.L2_SCORE));
        // L3
        NamedCommands.registerCommand("L3",
                new SetMainStateCommand(m_statemanager, m_elevator, m_arm, m_endeffector, RobotState.L3));
        NamedCommands.registerCommand("L3_Score",
                new SetMainStateCommand(m_statemanager, m_elevator, m_arm, m_endeffector, RobotState.L3_SCORE));
        // L4
        NamedCommands.registerCommand("L4",
                new SetMainStateCommand(m_statemanager, m_elevator, m_arm, m_endeffector, RobotState.L4));
        NamedCommands.registerCommand("L4_Score",
                new SetMainStateCommand(m_statemanager, m_elevator, m_arm, m_endeffector, RobotState.L4_SCORE));
        // Stow
        NamedCommands.registerCommand("Stow_Robot",
                new SetMainStateCommand(m_statemanager, m_elevator, m_arm, m_endeffector, RobotState.CORAL_STOW));
        // Floor Intake
        NamedCommands.registerCommand("Floor_Intake",
                new SetIntakeStateCommand(m_intakestatemanager, m_floorintake, m_indexer, IntakeState.DOWN));
        NamedCommands.registerCommand("Stow_Intake",
                new SetIntakeStateCommand(m_intakestatemanager, m_floorintake, m_indexer, IntakeState.STOW));

        // Wait For Coral
        NamedCommands.registerCommand("Wait_For_Coral",
                Commands.waitUntil(m_endeffector::hasCoral).withName("Wait_For_Coral"));
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
        /*
         * Floor Intake
         */
        // Floor Intake Down (Intaking)
        driverXbox.leftBumper()
                .onTrue(new SetIntakeStateCommand(m_intakestatemanager, m_floorintake, m_indexer, IntakeState.DOWN));
        // Floor Intake Down (Outtaking)
        driverXbox.rightBumper().onTrue(
                new SetIntakeStateCommand(m_intakestatemanager, m_floorintake, m_indexer, IntakeState.DOWN_OUTTAKE));
        driverXbox.rightBumper()
                .onFalse(new SetIntakeStateCommand(m_intakestatemanager, m_floorintake, m_indexer, IntakeState.DOWN));
        // Floor Intake Stow
        driverXbox.b()
                .onTrue(new SetIntakeStateCommand(m_intakestatemanager, m_floorintake, m_indexer, IntakeState.STOW));
        // Scoring
        driverXbox.rightTrigger().onTrue(new InstantCommand(() -> m_statemanager.setToScoreState()));
        // driverXbox.rightTrigger().onFalse(new InstantCommand(() ->
        // m_statemanager.setToUnscoreState()));
        operatorXbox.b().onTrue(new InstantCommand(() -> m_statemanager.setToScoreState()));
        operatorXbox.y().onTrue(new InstantCommand(() -> m_statemanager.setToUnscoreState()));
        // Eject
        driverXbox.y().onTrue(new InstantCommand(() -> m_statemanager.setEject(true)));
        driverXbox.y().onFalse(new InstantCommand(() -> m_statemanager.setEject(false)));

        operatorXbox.a().onTrue(new InstantCommand(() -> m_statemanager.setEject(true)));
        operatorXbox.a().onFalse(new InstantCommand(() -> m_statemanager.setEject(false)));
        // Stow
        operatorXbox.start().onTrue(
                new SetMainStateCommand(m_statemanager, m_elevator, m_arm, m_endeffector, RobotState.CORAL_STOW));

        // Levels
        operatorXbox.leftBumper()
                .onTrue(new SetMainStateCommand(m_statemanager, m_elevator, m_arm, m_endeffector, RobotState.L1));
        operatorXbox.leftTrigger()
                .onTrue(new SetMainStateCommand(m_statemanager, m_elevator, m_arm, m_endeffector, RobotState.L2));
        operatorXbox.rightBumper()
                .onTrue(new SetMainStateCommand(m_statemanager, m_elevator, m_arm, m_endeffector, RobotState.L3));
        operatorXbox.rightTrigger()
                .onTrue(new SetMainStateCommand(m_statemanager, m_elevator, m_arm, m_endeffector, RobotState.L4));

        operatorXbox.back()
                .and(() -> m_statemanager.getCurrentState() == RobotState.CORAL_STOW)
                .onTrue(
                        Commands.sequence(
                                // go to MANUAL_PLUCK
                                new SetMainStateCommand(m_statemanager, m_elevator, m_arm, m_endeffector,
                                        RobotState.MANUAL_PLUCK),
                                // wait 2 seconds
                                Commands.waitSeconds(2),
                                // then go back to STOW
                                new SetMainStateCommand(m_statemanager, m_elevator, m_arm, m_endeffector,
                                        RobotState.CORAL_STOW)));

        // the lion does not concern itself with comments

    }

    /*
     * Autonomous
     */
    public Command getAutonomousCommand() {
        return drivebase.getAutonomousCommand("W-L4-A4");
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }

}
