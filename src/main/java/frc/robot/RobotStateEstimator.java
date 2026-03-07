package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotStateEstimator extends SubsystemBase{
    private CommandSwerveDrivetrain m_SwerveDriveTrain;
    private boolean doRejectUpdate = false;
    public RobotStateEstimator(CommandSwerveDrivetrain swerve)
    {
        m_SwerveDriveTrain = swerve;
      System.out.println("created");
    }
    @Override
    public void periodic(){
      doRejectUpdate = false;
      LimelightHelpers.SetRobotOrientation("limelight", m_SwerveDriveTrain.getPigeon2().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      // LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      if(mt2 == null) {
        return;
      }
      if(Math.abs(m_SwerveDriveTrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        m_SwerveDriveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_SwerveDriveTrain.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
      
      //System.out.println("dog");
      //System.out.println("distance : " + FieldLayout.distanceFromAllianceWall(m_SwerveDriveTrain.getState().Pose.getX(), false));
    }
}
