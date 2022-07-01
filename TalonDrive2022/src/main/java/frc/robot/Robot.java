// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveTeleop;
import frc.robot.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Constants
  public static final boolean kUsePoseEstimator = true;
  public static final boolean kUsePoseEstimatorInAuto = false;

  // Create Xbox controller for driver.
  private final XboxController driver_controller_ = new XboxController(0);

  // Initialize robot state.
  private final RobotState robot_state_ = new RobotState();

  // Create subsystems.
  private final Drivetrain drivetrain_ = new Drivetrain(robot_state_);


  // Create autonomous mode selector.
  private final SendableChooser<Command> auto_selector_ = new SendableChooser<>();
  private Command autonomous_command_ = null;

  // Keeps track of whether we need to clear buttons.
  private boolean clear_buttons_ = false;

  @Override
  public void robotInit() {
    // Disable LiveWindow telemetry.
    LiveWindow.disableAllTelemetry();

    // Silence joystick warnings in sim.
    if (RobotBase.isSimulation())
      DriverStation.silenceJoystickConnectionWarning(true);

    // Enable NetworkTables flush() at higher rate.
    setNetworkTablesFlushEnabled(true);

    // Reset robot state.
    robot_state_.resetPosition(new Pose2d());

    // Setup auto.
    //setupAuto();

    // Set default commands for subsystems:
    setDefaultCommands();

    // Setup teleop controls.
    setupTeleopControls();

    // Update alliance color every second (just in case we lose comms or power).
    addPeriodic(() -> robot_state_.setAlliance(DriverStation.getAlliance()), 1);

  }

  @Override
  public void disabledInit() {
    // Set coast mode on drivetrain, turret, and hood to make then easier to move.
    drivetrain_.setBrakeMode(false);
  }

  @Override
  public void autonomousInit() {
    // Set brake mode on turret and drivetrain.
    drivetrain_.setBrakeMode(true);

    // Set alliance color (guaranteed to be accurate here).
    DriverStation.Alliance alliance = DriverStation.getAlliance();
    robot_state_.setAlliance(alliance);

    // Start autonomous program.
    autonomous_command_ = auto_selector_.getSelected();
    if (autonomous_command_ != null) {
      autonomous_command_.schedule();
    }
  }

  @Override
  public void teleopInit() {
    // Set brake mode on drivetrain, turret, and hood.
    drivetrain_.setBrakeMode(true);

    // Cancel autonomous program.
    if (autonomous_command_ != null)
      autonomous_command_.cancel();
  }

  @Override
  public void robotPeriodic() {
    // Run command scheduler.
    CommandScheduler.getInstance().run();

    // Limit drivetrain output if scoring.
    //drivetrain_.limitOutput(score_hg_.isScheduled());

    // Update telemetry.
    //telemetry_.periodic();

    }

  /**
   * Sets default commands for each subsystem.
   */
  private void setDefaultCommands() {
    // Drivetrain:
    drivetrain_.setDefaultCommand(new DriveTeleop(drivetrain_, driver_controller_));

  }

  /**
   * Configures button / joystick bindings for teleop control (non-climb mode) if they are not
   * already configured in the respective subsystem default commands.
   */
  private void setupTeleopControls() {
   // X: drivetrain cheesy drive quick turn (in command)

    // Y: none

    // LS: drivetrain movement (in command)

    // RS: none

  }

}