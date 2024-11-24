/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.Map;
import java.util.Optional;

public final class DriveCommands {

  public static Rotation2d ROTATE_180_DEGREES = Rotation2d.fromDegrees(180);

  @RobotPreferencesValue(column = 1, row = 1)
  public static final RobotPreferences.BooleanValue USE_ESTIMATED_POSE =
      new RobotPreferences.BooleanValue("AprilTag", "Use Estimated Pose", false);

  /**
   * Returns a command that resets the drivetrain orientation for teleop driving based on the
   * current alliance.
   *
   * @param subsystems The subsystems container.
   * @return A command to reset the drivetrain orientation.
   */
  public static Command resetOrientation(Subsystems subsystems) {
    SwerveSubsystem drivetrain = subsystems.drivetrain;

    return Commands.select(
        Map.of(
            Alliance.Blue,
            Commands.runOnce(() -> drivetrain.resetOrientation(new Rotation2d()), drivetrain),
            Alliance.Red,
            Commands.runOnce(
                () -> drivetrain.resetOrientation(Rotation2d.fromDegrees(180)), drivetrain)),
        () -> {
          Optional<Alliance> alliance = DriverStation.getAlliance();
          return alliance.orElse(Alliance.Blue);
        });
  }
}
