// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class ShootWhenReadyCommand extends CommandBase {

  private Conveyor conveyor;
  private Shooter shooter;
  private Swerve swerve;
  private int rpm;
  private int iteration = 0;
  private SwerveModuleState desiredStates[] = {
    new SwerveModuleState(0.0011, Rotation2d.fromDegrees(-45)),
    new SwerveModuleState(0.0011, Rotation2d.fromDegrees(45)),
    new SwerveModuleState(0.0011, Rotation2d.fromDegrees(45)),
    new SwerveModuleState(0.0011, Rotation2d.fromDegrees(-45)),
  };

  /** Creates a new ShootWhenReadyCommand. */
  public ShootWhenReadyCommand(Conveyor conveyor, Shooter shooter, Swerve swerve) {
    SmartDashboard.putNumber("target shooter RPM", Constants.SHOOT_RPM);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyor, shooter, swerve);
    this.swerve = swerve;
    this.conveyor = conveyor;
    this.shooter = shooter;
    this.swerve = swerve;
    //this.rpm = (int) SmartDashboard.getNumber("target shooter RPM", Constants.SHOOT_RPM);
    this.rpm = 2800;
  
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conveyor.stop();
    shooter.setRPM(rpm);
    this.rpm = 2800;
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    shooter.setRPM(rpm);
    swerve.setModuleStates(desiredStates);
    
    if (shooter.shooterPID.atSetpoint()) {
      iteration++;
      if(iteration>5){
        //sw.setModuleStates(desiredStates);
        conveyor.feedBall();
      }
    } else {
      conveyor.stop();
      iteration = 0;
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
