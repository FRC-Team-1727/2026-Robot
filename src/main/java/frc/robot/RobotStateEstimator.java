package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotStateEstimator extends SubsystemBase {
  private CommandSwerveDrivetrain m_SwerveDriveTrain;
  private boolean doRejectUpdate = false;
  private int[] aprilTags = { 2, 5, 8, 9, 10, 11, 18, 21, 24, 25, 26, 27 };

  public RobotStateEstimator(CommandSwerveDrivetrain swerve) {
    m_SwerveDriveTrain = swerve;
    System.out.println("created");

    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", aprilTags);
  }

  @Override
  public void periodic() {
    doRejectUpdate = false;
    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    // LimelightHelpers.PoseEstimate mt1 =
    // LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if (mt1 == null) {
      return;
    }
    if (Math.abs(m_SwerveDriveTrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular
                                                                                                       // velocity is
                                                                                                       // greater than
                                                                                                       // 720 degrees
                                                                                                       // per second,
                                                                                                       // ignore vision
                                                                                                       // updates
    {
      doRejectUpdate = true;
    }
    if (mt1.tagCount == 0) {
      doRejectUpdate = true;
    }
    if (!doRejectUpdate) {
      m_SwerveDriveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
      m_SwerveDriveTrain.addVisionMeasurement(
          mt1.pose,
          mt1.timestampSeconds);
    }

    // System.out.println("dog");
    // System.out.println("distance : " +
    // FieldLayout.distanceFromAllianceWall(m_SwerveDriveTrain.getState().Pose.getX(),
    // false));
  }
}
