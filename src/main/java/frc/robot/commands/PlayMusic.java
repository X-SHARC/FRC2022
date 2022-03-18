// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class PlayMusic extends CommandBase {
  /** Creates a new PlayMusic. */
  private Shooter shooter;
  private Orchestra orchestra;
  private final String FILE_PATH = "queen.chrp";
  public PlayMusic(Shooter s) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = s;
    this.orchestra = new Orchestra();
    orchestra.addInstrument(shooter.getShooterMasterMotor());
    SmartDashboard.putString("music load", orchestra.loadMusic(FILE_PATH).toString());
  }

  // Called when the command is initially schedule.
  @Override
  public void initialize() {
    
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("music play", orchestra.play().toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    orchestra.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
