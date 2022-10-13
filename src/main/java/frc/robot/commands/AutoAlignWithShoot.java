
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Swerve.AutoAlign;
import frc.robot.commands.Swerve.AutoDistance;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAlignWithShoot extends SequentialCommandGroup {
  /** Creates a new AutoAlignWithShoot. */
  public AutoAlignWithShoot(Limelight ll, Swerve swerve, Conveyor conveyor, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoAlign(ll, swerve),
      new AutoDistance(swerve, ll),
      new AutoAlign(ll, swerve).withTimeout(0.4),
      new ShootWhenReadyCommand(conveyor, shooter, swerve)
      );
  }
}
