package frc.robot.commands;

// This command was imported from
// https://github.com/hannahL2323/AnkleBiter2024/blob/main/src/main/java/frc/robot/commands/TeleopDrive.java

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;
    
public class TeleopDrive extends Command {
    private final SwerveSubsystem  swerve;
    // Double supplier allows the program to call <Object>.getAsDouble() 
    private final DoubleSupplier   vX;
    private final DoubleSupplier   vY;
    private final DoubleSupplier   omega;
    private final Boolean          fieldRelative;
    private final SwerveController controller;
    private final double           speed;

    public TeleopDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega, Boolean fieldRelative, double speed) {

    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    this.fieldRelative = fieldRelative;
    this.controller = swerve.getSwerveController();
    this.speed = speed;
    
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    double xVelocity   = Math.pow(vX.getAsDouble(), 3) * speed;
    double yVelocity   = Math.pow(vY.getAsDouble(), 3) * speed;
    double angVelocity = Math.pow(omega.getAsDouble(), 3);
    SmartDashboard.putNumber("[SWERVE] VelocityX", xVelocity);
    SmartDashboard.putNumber("[SWERVE] VelocityY", yVelocity);
    SmartDashboard.putNumber("[SWERVE] Omega",     angVelocity);

    // Drive using raw values.
    swerve.drive(new Translation2d(xVelocity * Constants.MAX_SPEED, yVelocity * Constants.MAX_SPEED),
                 angVelocity * controller.config.maxAngularVelocity, fieldRelative);

  }

  // Never end the drive command
  @Override
  public boolean isFinished() {
    return false;
  }
}
