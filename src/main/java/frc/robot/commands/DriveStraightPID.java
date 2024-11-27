/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveStraightPID extends Command {
  private PIDController pid;
  private double distance;
  private SwerveSubsystem swerve;
  private double currentDistance;
  private double speed;
  private double kP = 0.1;
  private double kI = 0.0;
  private double kD = 0.0; // CHANGE THESE

  /** Creates a new DriveStraightPID. */
  public DriveStraightPID(double goalDistance, Subsystems subsystems) {
    swerve = subsystems.drivetrain;
    distance = goalDistance;
    pid = new PIDController(kP, kI, kD);
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.resetOrientation(new Rotation2d());
    swerve.resetPosition(new Pose2d());
    pid.setSetpoint(distance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentDistance = swerve.getPosition().getX();
    speed = pid.calculate(currentDistance);
    swerve.drive(speed, 0, 0, false);
    ;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
