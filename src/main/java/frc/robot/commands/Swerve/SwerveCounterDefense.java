// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class SwerveCounterDefense extends CommandBase {
  /** Creates a new SwerveCounterDefense. */
  Swerve swerve;
  XboxController driver;

  //Angle values can be changed
  private SwerveModuleState desiredStates[] = {
    new SwerveModuleState(0.7,Rotation2d.fromDegrees(45)),
    new SwerveModuleState(0.7,Rotation2d.fromDegrees(-45)),
    new SwerveModuleState(0.7,Rotation2d.fromDegrees(-45)),
    new SwerveModuleState(0.7,Rotation2d.fromDegrees(45)),
  };


  public SwerveCounterDefense(Swerve sw, XboxController driver) {
    this.swerve = sw;
    this.driver = driver;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(driver.getAButton()){
      swerve.setModuleStates(desiredStates);
    }
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
