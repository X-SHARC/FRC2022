// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Storage;

public class CollectCargo extends CommandBase {
  /** Creates a new CollectCargo. */
  private Intake intake;
  private Storage storage;

  public CollectCargo(Intake intake, Storage storage) {
    this.intake = intake;
    this.storage = storage;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //   intake.extendIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runForward();
    storage.storageForward();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //intake.retractIntake();
    storage.stop();
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
