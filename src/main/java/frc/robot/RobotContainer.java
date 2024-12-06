/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot;

import com.nrg948.preferences.RobotPreferencesLayout;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotConstants.OperatorConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveStraightPID;
import frc.robot.subsystems.Subsystems;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@RobotPreferencesLayout(groupName = "Preferences", column = 0, row = 0, width = 1, height = 1)
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Subsystems subsystems = new Subsystems();
  private final double distance = 3.0;

  // Robot autonomous must be initialized after the subsystems
  private final RobotAutonomous autonomous = new RobotAutonomous(subsystems);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.XboxControllerPort.DRIVER);
  private final CommandXboxController operatorController =
      new CommandXboxController(OperatorConstants.XboxControllerPort.MANIPULATOR);

  private Timer coastModeTimer = new Timer();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // !! ADD TO LAB

    DriverStation.silenceJoystickConnectionWarning(true);

    // subsystems.arm.setDefaultCommand(new ManualArmController(subsystems, operatorController));
    // subsystems.intake.setDefaultCommand(new IntakeUsingController(subsystems,
    // operatorController));
    // subsystems.shooter.setDefaultCommand(new ShootUsingController(subsystems.shooter,
    // operatorController));

    // Configure the trigger bindings
    configureBindings();

    initShuffleboard();
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

    driverController.start().onTrue(DriveCommands.resetOrientation(subsystems));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonomous.getAutonomousCommand(subsystems);
  }

  public void disabledInit() {
    coastModeTimer.restart();
    subsystems.drivetrain.disableAutoOrientation();
  }

  public void disabledPeriodic() {
    if (coastModeTimer.advanceIfElapsed(3)) {
      subsystems.drivetrain.setBrakeMode(false);
      coastModeTimer.stop();
    }
  }

  public void autonomousInit() {
    subsystems.drivetrain.setBrakeMode(true);
  }

  public void teleopInit() {
    subsystems.drivetrain.setBrakeMode(true);
    subsystems.drivetrain.disableAutoOrientation();
    CommandScheduler.getInstance().schedule(new DriveStraightPID(distance, subsystems));
  }

  public void periodic() {
    subsystems.periodic();
  }

  public void initShuffleboard() {
    subsystems.drivetrain.addShuffleboardTab();
    ShuffleboardTab tuning = Shuffleboard.getTab("Tuning");
    tuning.addNumber("CurrentDistance", () -> subsystems.drivetrain.getPosition().getX());
    tuning.addNumber("Goal Distance", () -> distance);
  }
}
