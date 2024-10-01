// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Here we initialize the subsystems and commands for the robot
  // Subsystems/commands which are called upon should, if possible, be from here

  // The parameter is where the swerve subsystem will load its configuration from
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));


  // Here, we create objects which are attached to different parts of the Xbox Controller (buttons, triggers, and joysticks)
  XboxController driverController = new XboxController(0); // The controller itself
  Trigger xButton = new JoystickButton(driverController, XboxController.Button.kX.value);

  LimelightSubsystem limelightSubsystem = new LimelightSubsystem(swerveSubsystem, driverController);


  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    // Configure the controller bindings
    configureBindings();
    double speed = 1;

    TeleopDrive xboxDrive = new TeleopDrive(swerveSubsystem, 
      () -> MathUtil.applyDeadband(driverController.getLeftX()*speed,   Constants.MOVEMENT_DEADBAND),
      () -> MathUtil.applyDeadband(driverController.getLeftY()*speed,   Constants.MOVEMENT_DEADBAND),
      () -> MathUtil.applyDeadband(-driverController.getRightX()*speed, Constants.ROTATION_DEADBAND),
      false, 0.5);

    swerveSubsystem.setDefaultCommand(xboxDrive);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    xButton.onTrue(Commands.runOnce(swerveSubsystem::zeroGyro));

    // Bind the D-Pad to angles on the robot 
    bindDpadToAngle(0);
    bindDpadToAngle(90);
    bindDpadToAngle(180);
    bindDpadToAngle(270);
  }

  private void bindDpadToAngle(int angle) {
    new POVButton(driverController, angle).onTrue(swerveSubsystem.simDriveCommand(()->0,()->0,()->angle));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Do nothing, for now no autonomous code has been written
    return new InstantCommand();
  }
}
