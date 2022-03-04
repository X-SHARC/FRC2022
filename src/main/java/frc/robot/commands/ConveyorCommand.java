// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class ConveyorCommand extends CommandBase {
  /** Creates a new ConveyorCommand. */
  XboxController operator;
  Conveyor conveyor;

  public ConveyorCommand(XboxController operator, Conveyor conveyor) {
    this.operator = operator;
    this.conveyor = conveyor;
    addRequirements(conveyor);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(operator.getLeftTriggerAxis())>0.4){
      conveyor.feedBall();
    }

    if(Math.abs(operator.getRightTriggerAxis())>0.4){
      conveyor.retractBall();
    }
    if(Math.abs(operator.getRightTriggerAxis())<0.4 && Math.abs(operator.getLeftTriggerAxis())<0.4){
      conveyor.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
