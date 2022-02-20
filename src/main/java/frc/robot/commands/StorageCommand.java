// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class StorageCommand extends CommandBase {
  /** Creates a new StorageCommand. */
  Storage storage;
  Conveyor conveyor;
  Joystick operator;
  Intake intake;
  public StorageCommand(Storage storage, Conveyor conveyor, Joystick operator, Intake intake) {
    this.storage = storage;
    this.conveyor = conveyor;
    this.operator = operator;
    this.intake = intake;
    addRequirements(storage, conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(operator.getRawButtonPressed(2)){
      storage.storageForward();
      intake.runForward();
    }
    if (operator.getRawButtonReleased(2)){
      intake.stop();
      storage.stop();
    }

    if(operator.getRawButtonPressed(4)){
      storage.storageBackwards();
      conveyor.retractBall();
    }
    if(operator.getRawButtonReleased(4)){
      storage.stop();
      intake.stop();
    }
    if(operator.getRawButtonPressed(3)){
      conveyor.retractBall();
    }
    if(operator.getRawButtonReleased(3)){
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
