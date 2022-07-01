/*// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.XboxController;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 
public class Robot extends TimedRobot {
  TalonSRX rightLeader = new TalonSRX(1);
  TalonSRX rightFollower = new TalonSRX(2);
  TalonSRX leftLeader = new TalonSRX(3);
  TalonSRX leftFollower = new TalonSRX(4);
  XboxController controller = new XboxController(0);


  @Override
  public void robotInit() {
    double forward = -controller_.getLeftY();
    double curvature = controller_.getLeftX();
    boolean quick_turn = controller_.getXButton();
    
    
  }

  @Override
  public void robotPeriodic() {
    rightLeader.set(ControlMode.PercentOutput, forward);
    leftLeader.set(ControlMode.PercentOutput, forward);
  }

  @Override
  public void teleopInit() {
    rightFollower.follow(rightLeader);
    leftFollower.follow(leftLeader);

  }

  @Override
  public void teleopPeriodic() {}

}
*/