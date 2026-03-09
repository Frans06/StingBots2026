package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public double latestTimestamp = 0.0;
    public Pose2d latestPose = Pose2d.kZero;
    public int tagCount = 0;
    public double avgTagDist = 0.0;
    public double avgTagArea = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}

  /** Sets the robot orientation for MegaTag2. */
  public default void setRobotOrientation(double yawDegrees, double yawRateDegPerSec) {}
}
