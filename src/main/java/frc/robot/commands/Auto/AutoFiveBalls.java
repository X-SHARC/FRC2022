// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CollectCargoCommand;
import frc.robot.commands.ShootWhenReadyCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Swerve;
import static edu.wpi.first.wpilibj2.command.CommandGroupBase.*;

public class AutoFiveBalls extends AutonomousCommand {


  public AutoFiveBalls(
    Swerve swerve,
    Conveyor conveyor,
    Shooter shooter,
    Intake intake,
    Storage storage
  ) {
    super(AutonomousCommand.allTrajectories, swerve, conveyor, shooter, intake, storage);


    addCommands(
      new ShootWhenReadyCommand(conveyor, shooter, swerve).withTimeout(.85),

      sequence(
        getControllerCommand(trajectories[0], swerve, x_pid, y_pid, thetaController),
        getControllerCommand(trajectories[1], swerve, x_pid, y_pid, thetaController),
        getControllerCommand(trajectories[2], swerve, x_pid, y_pid, thetaController)
        )
        .raceWith(new CollectCargoCommand(intake, storage)),

      new InstantCommand(() -> swerve.drive(0, 0, 0, true)),

      new ShootWhenReadyCommand(conveyor, shooter, swerve).withTimeout(1.3),

      getControllerCommand(trajectories[3], swerve, x_pid, y_pid, thetaController)
        .raceWith(new CollectCargoCommand(intake, storage)),

      new InstantCommand(() -> swerve.drive(0, 0, 0, true)),
      
      new WaitCommand(.8),
      
      getControllerCommand(trajectories[4], swerve, x_pid, y_pid, thetaController),
      
      new InstantCommand(() -> swerve.drive(0, 0, 0, true)),

      new ShootWhenReadyCommand(conveyor, shooter, swerve).withTimeout(1.3)
    );
  }
}
