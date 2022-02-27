// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class ShootWhenReadyCommand extends CommandBase {

  private Conveyor conveyor;
  private Shooter shooter;
  private int rpm;

  /** Creates a new ShootWhenReadyCommand. */
  public ShootWhenReadyCommand(Conveyor conveyor, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyor, shooter);
    this.conveyor = conveyor;
    this.shooter = shooter;
    this.rpm = (int) SmartDashboard.getNumber("target shooter RPM", Constants.SHOOT_RPM);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conveyor.stop();
    shooter.setRPM(rpm);
    this.rpm = (int) SmartDashboard.getNumber("target shooter RPM", Constants.SHOOT_RPM);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    shooter.setRPM(rpm);
  
    if (shooter.shooterPID.atSetpoint()) {
      conveyor.feedBall();
    } else {
      conveyor.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.stop();
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
