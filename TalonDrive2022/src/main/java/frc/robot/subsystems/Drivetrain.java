package frc.robot.subsystems;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.RobotState;

public class Drivetrain extends SubsystemBase {
  // Robot State
  private final RobotState robot_state_;

  // Motor Controllers
  private final TalonSRX right_leader_;
  private final TalonSRX right_follower_;
  private final TalonSRX left_leader_;
  private final TalonSRX left_follower_;
  XboxController controller = new XboxController(0);
  
  // Control
  private final SimpleMotorFeedforward left_feedforward_;
  private final SimpleMotorFeedforward right_feedforward_;
  private double last_l_velocity_setpoint_ = 0;
  private double last_r_velocity_setpoint_ = 0;

  // Output Limit
  private boolean limit_output_ = false;

  // IO
  private OutputType output_type_ = OutputType.PERCENT;
  private final PeriodicIO io_ = new PeriodicIO();

  /**
   * Constructs an instance of the Drivetrain subsystem. Only one instance of this subsystem should
   * be created in the main Robot class and references to this instance should be passed around
   * the robot code.
   *
   * @param robot_state Reference to the global robot state instance.
   */
  public Drivetrain(RobotState robot_state) {
    // Store reference to robot state.
    robot_state_ = robot_state;

    // Initialize motor controllers.
    right_leader_ = new TalonSRX(2);
    right_leader_.setInverted(true);

    right_follower_ = new TalonSRX(1);
    right_follower_.follow(right_leader_);
    right_follower_.setInverted(2);

    left_leader_ = new TalonSRX(3);
    left_leader_.setInverted(false);

    left_follower_ = new TalonSRX(4);
    left_follower_.follow(left_leader_);

    // Initialize feedforward.
    left_feedforward_ = new SimpleMotorFeedforward(
        Constants.kLeftKs, Constants.kLeftKv, Constants.kLeftKa);
    right_feedforward_ = new SimpleMotorFeedforward(Constants.kRightKs, Constants.kRightKv,
        Constants.kRightKa);
  }

  /**
   * This method runs periodically every 20 ms. Here, all sensor values are read and all motor
   * outputs should be set.
   */
  @Override
  public void periodic() {
    // Read inputs.
    io_.l_leader_supply_current = left_leader_.getStatorCurrent();
    io_.l_follower_supply_current = left_follower_.getStatorCurrent();
    io_.r_leader_supply_current = right_leader_.getStatorCurrent();
    io_.r_follower_supply_current = right_follower_.getStatorCurrent();

    // Write outputs.
    switch (output_type_) {
      case PERCENT:
        right_leader_.set(ControlMode.PercentOutput, limit_output_ ?
            MathUtil.clamp(io_.r_demand, -Constants.kOutputLimit, Constants.kOutputLimit) :
            io_.r_demand);
        left_leader_.set(ControlMode.PercentOutput, limit_output_ ?
            MathUtil.clamp(io_.l_demand, -Constants.kOutputLimit, Constants.kOutputLimit) :
            io_.l_demand);
        //System.out.println("io_.r_demand " + io_.r_demand);
        //System.out.println("io_.l_demand " + io_.l_demand);
        //System.out.println("limit_output_ " + limit_output_);
        
        
        //System.out.println("Right Leader Stator Current: " + right_leader_.getStatorCurrent());
        //System.out.println("Left Leader Stator Current: " + left_leader_.getStatorCurrent());
        //System.out.println("Right Follower Stator Current: " + right_follower_.getStatorCurrent());
        //System.out.println("Left Follower Stator Current: " + left_follower_.getStatorCurrent());
        
        break;
      case VELOCITY:
        // Calculate feedforward value and add to built-in motor controller PID.
        double l_feedforward = left_feedforward_.calculate(io_.l_demand,
            (io_.l_demand - last_l_velocity_setpoint_) / 0.02);
        double r_feedforward = right_feedforward_.calculate(io_.r_demand,
            (io_.r_demand - last_r_velocity_setpoint_) / 0.02);

        // Store last velocity setpoints.
        last_l_velocity_setpoint_ = io_.l_demand;
        last_r_velocity_setpoint_ = io_.r_demand;
        break;
    }
  }


  /**
   * Sets brake mode on each of the drivetrain motors.
   *
   * @param value Whether brake mode should be enabled.
   */
  public void setBrakeMode(boolean value) {
    //IdleMode mode = value ? IdleMode.kBrake : IdleMode.kCoast;
    //left_leader_.setIdleMode(mode);
    //left_follower_.setIdleMode(mode);
    //right_leader_.setIdleMode(mode);
    //right_follower_.setIdleMode(mode);
  }

  /**
   * Sets the % output on the drivetrain.
   *
   * @param l The left % output in [-1, 1].
   * @param r The right % output in [-1, 1].
   */
  public void setPercent(double l, double r) {
    last_l_velocity_setpoint_ = 0;
    last_r_velocity_setpoint_ = 0;
    output_type_ = OutputType.PERCENT;
    io_.l_demand = l;
    io_.r_demand = r;
  }

  /**
   * Sets the desired drivetrain velocity for closed loop control.
   *
   * @param l Desired left velocity in m/s.
   * @param r Desired right velocity in m/s.
   */
  public void setVelocity(double l, double r) {
    output_type_ = OutputType.VELOCITY;
    io_.l_demand = l;
    io_.r_demand = r;
  }

  /**
   * Limits the output of the drivetrain to a constant value. This should be used when shooting
   * while moving to make it more accurate.
   *
   * @param value Whether to limit output or not.
   */
  public void limitOutput(boolean value) {
    limit_output_ = value;
  }

  /**
   * Returns the left position in meters.
   *
   * @return The left position in meters.
   */
  public double getLeftPosition() {
    return io_.l_position;
  }

  /**
   * Returns the right position in meters.
   *
   * @return The right position in meters.
   */
  public double getRightPosition() {
    return io_.r_position;
  }

  /**
   * Returns the left velocity in meters per second.
   *
   * @return The left velocity in meters per second.
   */
  public double getLeftVelocity() {
    return io_.l_velocity;
  }

  /**
   * Returns the right velocity in meters per second.
   *
   * @return The right velocity in meters per second.
   */
  public double getRightVelocity() {
    return io_.r_velocity;
  }

  enum OutputType {
    PERCENT, VELOCITY
  }

  public static class PeriodicIO {
    // Inputs
    double l_position;
    double r_position;
    double l_velocity;
    double r_velocity;
    Rotation2d angle = new Rotation2d();
    double angular_rate;

    double l_leader_supply_current;
    double l_follower_supply_current;
    double r_leader_supply_current;
    double r_follower_supply_current;

    // Outputs
    double l_demand;
    double r_demand;
  }

  public static class Constants {
    // Motor Controller IDs
    public static final int kLeftLeaderId = 1;
    public static final int kLeftFollowerId = 2;
    public static final int kRightLeaderId = 3;
    public static final int kRightFollowerId = 4;

    // Sensors
    public static final int kPigeonIMUId = 17;

    // Hardware
    public static double kGearRatio = 6.07;
    public static double kWheelRadius = 0.0508;
    public static double kTrackWidth = 0.759;
    public static double kMass = 65.0;
    public static double kMOI = 10.0;

    // Control
    public static double kLeftKs = 0.25438;
    public static double kLeftKv = 2.404;
    public static double kLeftKa = 0.17931;
    public static double kLeftKp = 0.00010892;
    public static double kRightKs = 0.23742;
    public static double kRightKv = 2.3732;
    public static double kRightKa = 0.14528;
    public static double kRightKp = 0.00010892;

    // Output Limit
    public static final double kOutputLimit = 0.3;
  }
}
