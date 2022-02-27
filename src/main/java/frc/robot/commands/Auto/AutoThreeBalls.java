// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.commands.CollectCargoCommand;
import frc.robot.commands.ShootWhenReadyCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Swerve;
import static edu.wpi.first.wpilibj2.command.CommandGroupBase.*;

public class AutoThreeBalls extends AutonomousCommand {


  public AutoThreeBalls(
    Swerve swerve,
    Conveyor conveyor,
    Shooter shooter,
    Intake intake,
    Storage storage
  ) {
    super(Arrays.copyOfRange<Trajectory>(AutonomousCommand.allTrajectories, 0, 3), swerve, conveyor, shooter, intake, storage);

    resetOdometry();
    addCommands(
      new ShootWhenReadyCommand(conveyor, shooter).withTimeout(.85),

      sequence(
        getControllerCommand(trajectories[0], swerve, x_pid, y_pid, thetaController),
        getControllerCommand(trajectories[1], swerve, x_pid, y_pid, thetaController),
        getControllerCommand(trajectories[2], swerve, x_pid, y_pid, thetaController)
        )
        .raceWith(new CollectCargoCommand(intake, storage)),

      new InstantCommand(() -> swerve.drive(0, 0, 0, true)),

      new ShootWhenReadyCommand(conveyor, shooter).withTimeout(1.3)
    );
  }
}