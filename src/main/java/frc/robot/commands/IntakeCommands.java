package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {
  private IntakeCommands() {}

  /** Extend intake and run rollers while held. Retracts on release. */
  public static Command intakeGamePiece(Intake intake) {
    return Commands.startEnd(intake::extend, intake::retract, intake);
  }

  /** Retract the intake immediately. */
  public static Command retract(Intake intake) {
    return Commands.runOnce(intake::retract, intake);
  }
}
