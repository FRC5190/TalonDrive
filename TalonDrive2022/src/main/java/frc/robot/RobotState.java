package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.Objects;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

public class RobotState {
  // Pose Estimator
  //private final PoseEstimator pose_estimator_;

  // Robot Speeds
  private ChassisSpeeds robot_speeds_ = new ChassisSpeeds(0, 0, 0);

  // Sensor Offsets
  private double l_encoder_offset_ = 0.0;
  private double r_encoder_offset_ = 0.0;
  private Rotation2d gyro_offset_ = new Rotation2d();

  private double l_encoder_ = 0.0;
  private double r_encoder_ = 0.0;
  private Rotation2d gyro_ = new Rotation2d();

  // Last Vision Pose
  private Pose2d last_vision_pose_ = new Pose2d();

  /**
   * Constructs a "robot state" instance. This keeps track of various states on the robot,
   * including robot pose, turret angle, and hood angle across time.
   */
  public RobotState() {
    // Initialize pose estimator.
    //pose_estimator_ = new PoseEstimator(new Rotation2d(), new Pose2d(),
    //    Constants.kEstimatorStateStdDevs, Constants.kEstimatorLocalStdDevs,
    //    Constants.kEstimatorVisionStdDevs);

  }

  /**
   * Updates the pose estimator with measurements from encoders and gyro.
   *
   * @param l_position       The measured left encoder position in meters.
   * @param r_position       The measured right encoder position in meters.
   * @param average_velocity The average velocity of the drivetrain in meters per second.
   * @param angle            The measured heading of the robot.
   */
  public void updateRobotPose(double l_position, double r_position,
                              double average_velocity, Rotation2d angle) {
    // Store encoder and gyro values.
    l_encoder_ = l_position;
    r_encoder_ = r_position;
    gyro_ = angle;
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param timestamp         The time of capture of the frame.
   * @param vision_robot_pose The robot pose as calculated from vision at the time of capture.
   */

  /**
   * Updates the robot speeds.
   *
   * @param speeds The current robot speeds, as measured by the encoders and gyro.
   */
  public void updateRobotSpeeds(ChassisSpeeds speeds) {
    // Update local variable.
    robot_speeds_ = speeds;
  }

  /**
   * Resets the position of the robot.
   *
   * @param pose The position to reset to.
   */
  public void resetPosition(Pose2d pose) {
    // Store offsets.
    l_encoder_offset_ = l_encoder_;
    r_encoder_offset_ = r_encoder_;
    gyro_offset_ = gyro_;
  }

  /**
   * Sets the alliance color of the robot.
   *
   * @param alliance The alliance color of the robot.
   */
  public void setAlliance(DriverStation.Alliance alliance) {
    alliance_ = Objects.requireNonNullElse(alliance, DriverStation.Alliance.Invalid);
  }

  /**
   * Returns the robot speeds at the current time.
   *
   * @return The robot speeds at the current time.
   */
  public ChassisSpeeds getRobotSpeeds() {
    return robot_speeds_;
  }

  public static class Constants {
    public static final double kErrorTolerance = 1.5;

    // Buffer
    public static final double kBufferLifetime = 1.25;

    // Fender Poses
    public static final Pose2d[] kFenderPoses = new Pose2d[]{
        new Pose2d(7.759, 2.862, Rotation2d.fromDegrees(249)),
        new Pose2d(6.983, 4.664, Rotation2d.fromDegrees(158)),
        new Pose2d(9.549, 3.663, Rotation2d.fromDegrees(340)),
        new Pose2d(8.736, 5.428, Rotation2d.fromDegrees(71))
    };
  }
}