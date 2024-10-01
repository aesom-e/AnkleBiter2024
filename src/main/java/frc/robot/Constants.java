// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants {
  // Swerve constants
  public static final double ROBOT_MASS = (45) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  // Maximum speed of the robot, number chosen as it's the one chosen in the example, no other thinking
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
  public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);

  // Controller deadband, if the movement is less than this number, don't process it
  public static final double MOVEMENT_DEADBAND = 0.1;
  public static final double ROTATION_DEADBAND = 0.3;

}
