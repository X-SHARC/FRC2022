// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState.AlignmentState;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class AutoAlign extends CommandBase {
  /** Creates a new AutoAlign. */
  PIDController rotController = new PIDController(0.0758, 0, 0.00264);
  Timer timer = new Timer();
  Limelight LL;
  Swerve swerve;
  boolean isFinished = false;
  boolean temp = true;
  double atSetpointTime = 0;


  public AutoAlign(Limelight LL, Swerve swerve) {
    this.LL = LL;
    this.swerve = swerve;
    addRequirements(LL);
    rotController.setTolerance(1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = LL.getX();
    if(angle != 0.0) {
      RobotContainer.state.setAlignmentState(AlignmentState.ALIGNING);
      swerve.drive(0, 0, -Constants.Swerve.kMaxAngularSpeed * rotController.calculate(angle, 0), true);
    }
    else {
      RobotContainer.state.setAlignmentState(AlignmentState.FAIL);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(rotController.atSetpoint()) {
      if(temp){
        atSetpointTime = timer.get();
        temp = false;
      }
      else if((timer.get()-atSetpointTime) >0.3){
        timer.stop();
        RobotContainer.state.setAlignmentState(AlignmentState.SUCCESS);
        return true;
      }
    }
    else if(timer.get() > 1.2) {
      RobotContainer.state.setAlignmentState(AlignmentState.TIMEOUT);
      return true;
    }
    return false;
  }
}
