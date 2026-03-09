package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO frontIO;
  private final VisionIO backIO;
  private final Drive drive;

  private final VisionIOInputsAutoLogged frontInputs = new VisionIOInputsAutoLogged();
  private final VisionIOInputsAutoLogged backInputs = new VisionIOInputsAutoLogged();

  public Vision(VisionIO frontIO, VisionIO backIO, Drive drive) {
    this.frontIO = frontIO;
    this.backIO = backIO;
    this.drive = drive;
  }

  @Override
  public void periodic() {
    // Supply gyro orientation for MegaTag2
    double yawDeg = drive.getRotation().getDegrees();
    double yawRateDegPerSec = Math.toDegrees(drive.getGyroYawVelocityRadPerSec());
    frontIO.setRobotOrientation(yawDeg, yawRateDegPerSec);
    backIO.setRobotOrientation(yawDeg, yawRateDegPerSec);

    // Update inputs
    frontIO.updateInputs(frontInputs);
    backIO.updateInputs(backInputs);
    Logger.processInputs("Vision/Front", frontInputs);
    Logger.processInputs("Vision/Back", backInputs);

    // Process each camera
    processCamera(frontInputs, "Front");
    processCamera(backInputs, "Back");
  }

  private void processCamera(VisionIOInputsAutoLogged inputs, String cameraName) {
    // Skip if no tags detected
    if (inputs.tagCount == 0) {
      return;
    }

    // Reject if robot is spinning too fast (MegaTag2 unreliable)
    if (Math.abs(drive.getGyroYawVelocityRadPerSec())
        > VisionConstants.MAX_ANGULAR_VELOCITY_RAD_PER_SEC) {
      return;
    }

    // Reject if pose is outside field bounds
    Pose2d pose = inputs.latestPose;
    if (pose.getX() < 0.0
        || pose.getX() > VisionConstants.FIELD_LENGTH_METERS
        || pose.getY() < 0.0
        || pose.getY() > VisionConstants.FIELD_WIDTH_METERS) {
      return;
    }

    // Calculate dynamic standard deviations
    double xyStdDev =
        VisionConstants.XY_STD_DEV_COEFFICIENT
            * (inputs.avgTagDist * inputs.avgTagDist)
            / inputs.tagCount;

    // Feed measurement to pose estimator
    drive.addVisionMeasurement(
        pose,
        inputs.latestTimestamp,
        VecBuilder.fill(xyStdDev, xyStdDev, VisionConstants.ROT_STD_DEV));

    Logger.recordOutput("Vision/" + cameraName + "/AcceptedPose", pose);
  }
}
