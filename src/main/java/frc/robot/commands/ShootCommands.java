package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.RoutingMode;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class ShootCommands {
  private ShootCommands() {}

  /**
   * Full shoot sequence for a specific shooter.
   *
   * <p>Dependency chain: Shooter -> Feeder -> Indexer -> Intake
   *
   * <p>Phase 1: Spin up shooter + extend intake + route indexer (parallel, until shooter ready)
   * Phase 2: Fire - keep everything running + start feeder On release: stop all, retract intake
   */
  public static Command shootFull(
      Shooter shooter, Feeder feeder, Indexer indexer, Intake intake, RoutingMode routingMode) {

    return Commands.sequence(
            // Phase 1: Spin up + prepare (wait until shooter is ready)
            Commands.parallel(
                    Commands.run(shooter::spinUp, shooter),
                    Commands.runOnce(intake::extend, intake),
                    Commands.runOnce(() -> indexer.setRoutingMode(routingMode), indexer))
                .until(shooter::isReadyToFire),

            // Phase 2: Fire! Keep everything running + start feeder
            Commands.parallel(
                Commands.run(shooter::spinUp, shooter),
                Commands.run(intake::extend, intake),
                Commands.run(() -> indexer.setRoutingMode(routingMode), indexer),
                Commands.run(feeder::kick, feeder)))
        // Cleanup on end (button release or interruption)
        .finallyDo(
            () -> {
              shooter.stop();
              feeder.stop();
              indexer.stop();
              intake.retract();
            });
  }

  /** Shoot from the left shooter. Routes indexer LEFT_ONLY. */
  public static Command shootLeft(
      Shooter leftShooter, Feeder feeder, Indexer indexer, Intake intake) {
    return shootFull(leftShooter, feeder, indexer, intake, RoutingMode.LEFT_ONLY);
  }

  /** Shoot from the right shooter. Routes indexer RIGHT_ONLY. */
  public static Command shootRight(
      Shooter rightShooter, Feeder feeder, Indexer indexer, Intake intake) {
    return shootFull(rightShooter, feeder, indexer, intake, RoutingMode.RIGHT_ONLY);
  }
}
