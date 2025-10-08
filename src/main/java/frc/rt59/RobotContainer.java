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
import frc.rt59.subsystems.ArmSubsystem;
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
    driverXbox.leftBumper().onTrue(Commands.none());
    driverXbox.rightBumper().onTrue(Commands.none());
    driverXbox.a().onTrue(m_arm.setAngleCommand(90));
    driverXbox.b().onTrue(m_arm.setAngleCommand(180));


    /*
     * Test Controls
     */
    if (DriverStation.isTest()) {
      // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock,
      // drivebase).repeatedly());
      // driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));

    }
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
