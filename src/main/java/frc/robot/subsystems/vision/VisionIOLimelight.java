package frc.robot.subsystems.vision;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class VisionIOLimelight implements VisionIO {
  private final String name;

  public VisionIOLimelight(String limelightName) {
    this.name = limelightName;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = LimelightHelpers.getTV(name);

    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    if (estimate != null) {
      inputs.latestTimestamp = estimate.timestampSeconds;
      inputs.latestPose = estimate.pose;
      inputs.tagCount = estimate.tagCount;
      inputs.avgTagDist = estimate.avgTagDist;
      inputs.avgTagArea = estimate.avgTagArea;
    }
  }

  @Override
  public void setRobotOrientation(double yawDegrees, double yawRateDegPerSec) {
    LimelightHelpers.SetRobotOrientation(name, yawDegrees, yawRateDegPerSec, 0.0, 0.0, 0.0, 0.0);
  }
}
