// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Conveyor;

public class StorageCommand extends CommandBase {
  /** Creates a new StorageCommand. */
  Storage storage;
  Conveyor conveyor;
  XboxController operator;
  public StorageCommand(Storage storage, Conveyor conveyor, XboxController operator) {
    this.storage = storage;
    this.conveyor = conveyor;
    this.operator = operator;
    addRequirements(storage, conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(operator.getAButtonPressed()){
      conveyor.feedBall();
      storage.storageForward();
    }
    if (operator.getAButtonReleased()){
      conveyor.stop();
      storage.stop();
    }

    if(operator.getYButtonPressed()){
      storage.storageBackwards();
      conveyor.retractBall();
    }
    if(operator.getYButtonReleased()){
      storage.stop();
      conveyor.stop();
    }
    if(operator.getBButtonPressed()){
      conveyor.retractBall();
    }
    if(operator.getBButtonReleased()){
      conveyor.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.stop();
    storage.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
