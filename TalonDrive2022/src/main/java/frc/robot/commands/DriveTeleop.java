package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

public class DriveTeleop extends CommandBase {
  // Store references to drivetrain and Xbox controller.
  private final Drivetrain drivetrain_;
  private final XboxController controller_;

  // Constructor
  public DriveTeleop(Drivetrain drivetrain, XboxController controller) {
    // Assign member variables.
    drivetrain_ = drivetrain;
    controller_ = controller;

    // Require the drivetrain as part of this command.
    addRequirements(drivetrain_);
  }

  @Override
  public void execute() {
    // Get the forward-backward direction source. We multiply by -1 because pushing forward on
    // the joystick gives us negative values.
    double forward = -controller_.getLeftY();

    // Get the curvature direction source.
    double curvature = controller_.getLeftX();

    // Get the quick-turn source.
    boolean quick_turn = controller_.getXButton();

    // Compute the individual wheel percentages.
    DifferentialDrive.WheelSpeeds speeds = DifferentialDrive.curvatureDriveIK(
        forward, curvature, quick_turn);

    // Set the wheel percentages.
    drivetrain_.setPercent(0.25* speeds.left, 0.25* speeds.right);
  }
}
