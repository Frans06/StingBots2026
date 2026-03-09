// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final String CAN_BUS = "canivore";

  public static final class IntakeConstants {
    public static final int EXTENSION_MOTOR_ID = 9;
    public static final int ROLLER_MOTOR_ID = 10;

    // Extension MotionMagic
    public static final double EXTENSION_GEAR_RATIO = 1.0;
    public static final double EXTENSION_RETRACTED_POS = 0.0; // rotations
    public static final double EXTENSION_EXTENDED_POS = 17.5; // rotations
    public static final double EXTENSION_POSITION_TOLERANCE = 0.25; // rotations

    // Extension MotionMagic profile
    public static final double EXTENSION_CRUISE_VELOCITY = 40.0; // rot/s
    public static final double EXTENSION_ACCELERATION = 80.0; // rot/s^2
    public static final double EXTENSION_JERK = 800.0; // rot/s^3

    // Extension PID (Slot0)
    public static final double EXTENSION_KP = 24.0;
    public static final double EXTENSION_KI = 0.0;
    public static final double EXTENSION_KD = 0.1;
    public static final double EXTENSION_KS = 0.25;
    public static final double EXTENSION_KV = 0.12;
    public static final double EXTENSION_KA = 0.0;

    // Roller velocity
    public static final double ROLLER_INTAKE_VELOCITY = 40.0; // rot/s
    public static final double ROLLER_EJECT_VELOCITY = -20.0; // rot/s
    public static final double ROLLER_KP = 0.1;
    public static final double ROLLER_KI = 0.0;
    public static final double ROLLER_KD = 0.0;
    public static final double ROLLER_KS = 0.0;
    public static final double ROLLER_KV = 0.12;

    // Current limits
    public static final double EXTENSION_STATOR_LIMIT = 40.0;
    public static final double ROLLER_STATOR_LIMIT = 60.0;
  }

  public static final class IndexerConstants {
    public static final int FEED_MOTOR_ID = 11;
    public static final int LEFT_SHAFT_MOTOR_ID = 12;
    public static final int RIGHT_SHAFT_MOTOR_ID = 13;

    public static final double FEED_VELOCITY = 30.0; // rot/s
    public static final double SHAFT_VELOCITY = 40.0; // rot/s

    // PID for all three motors
    public static final double KP = 0.1;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KS = 0.0;
    public static final double KV = 0.12;

    public static final double STATOR_LIMIT = 40.0;
  }

  public static final class FeederConstants {
    public static final int KICKER_MOTOR_ID = 14;

    public static final double KICKER_VELOCITY = 60.0; // rot/s
    public static final double KP = 0.1;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KS = 0.0;
    public static final double KV = 0.12;

    public static final double STATOR_LIMIT = 60.0;
  }

  public static final class ShooterConstants {
    public static final int LEFT_HOOD_MOTOR_ID = 16;
    public static final int LEFT_FLYWHEEL_MOTOR_ID = 17;
    public static final int RIGHT_HOOD_MOTOR_ID = 18;
    public static final int RIGHT_FLYWHEEL_MOTOR_ID = 19;

    // Hood MotionMagic position: 0 rot = 0 deg, 6 rot = 45 deg
    public static final double HOOD_GEAR_RATIO = 1.0;
    public static final double HOOD_DOWN_POS = 0.0; // rotations
    public static final double HOOD_UP_POS = 6.0; // rotations (= 45 degrees)
    public static final double HOOD_POSITION_TOLERANCE = 0.15; // rotations

    // Hood MotionMagic profile
    public static final double HOOD_CRUISE_VELOCITY = 20.0; // rot/s
    public static final double HOOD_ACCELERATION = 40.0; // rot/s^2
    public static final double HOOD_JERK = 400.0; // rot/s^3

    // Hood PID
    public static final double HOOD_KP = 24.0;
    public static final double HOOD_KI = 0.0;
    public static final double HOOD_KD = 0.1;
    public static final double HOOD_KS = 0.25;
    public static final double HOOD_KV = 0.12;
    public static final double HOOD_KA = 0.0;

    // Flywheel MotionMagicVelocity
    public static final double FLYWHEEL_DEFAULT_VELOCITY = 80.0; // rot/s
    public static final double FLYWHEEL_VELOCITY_TOLERANCE = 2.0; // rot/s
    public static final double FLYWHEEL_KP = 0.3;
    public static final double FLYWHEEL_KI = 0.0;
    public static final double FLYWHEEL_KD = 0.0;
    public static final double FLYWHEEL_KS = 0.25;
    public static final double FLYWHEEL_KV = 0.12;
    public static final double FLYWHEEL_KA = 0.001;
    public static final double FLYWHEEL_ACCELERATION = 200.0; // rot/s^2

    // Current limits
    public static final double HOOD_STATOR_LIMIT = 40.0;
    public static final double FLYWHEEL_STATOR_LIMIT = 80.0;
  }
}
