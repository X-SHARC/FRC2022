// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class AutoConveyor extends CommandBase {
  /** Creates a new AutoConveyor. */
  private Shooter shooter;
  private Conveyor conveyor;
  private int RPM;
  private boolean isShooterReady= false;
  public AutoConveyor(Shooter shooter, Conveyor conveyor, int RPM) {
    this.shooter = shooter;
    this.conveyor = conveyor;
    this.RPM = RPM;
    addRequirements(this.shooter, this.conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setRPM(RPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Button binding
    if(shooter.shooterPID.atSetpoint()) {
      conveyor.feedBall();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
