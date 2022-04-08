// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotState.DistanceState;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class AutoDistance extends CommandBase {
  Swerve swerve;
  Limelight LL;
  boolean temp = true;
  double atSetpointTime = 0;
  Timer timer = new Timer();
  PIDController distController = new PIDController(9.58, 0, 0);


  public AutoDistance(Swerve swerve, Limelight LL) {
    this.swerve = swerve;
    this.LL = LL;
    addRequirements(swerve, LL);
    distController.setTolerance(0.05);
    }

  @Override
  public void initialize() {
    timer.start();
    timer.reset();
  }

  @Override
  public void execute() {
    double distance = LL.getDistance();
    if(LL.getY() != 0.0) {
      RobotContainer.state.setDistanceState(DistanceState.ALIGNING);
      swerve.drive(-distController.calculate(distance, 1.36), 0, 0, false);
    }
    else {
      RobotContainer.state.setDistanceState(DistanceState.FAIL);
      swerve.drive(0, 0, 0, true);
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
    if(distController.atSetpoint()) {
      if(temp){
        atSetpointTime = timer.get();
        temp = false;
      }
      else if((timer.get()-atSetpointTime) >0.2){
        timer.stop();
        RobotContainer.state.setDistanceState(DistanceState.SUCCESS);
        return true;
      }
    }
    else if(timer.get() > 0.9
    ) {
      RobotContainer.state.setDistanceState(DistanceState.TIMEOUT);
      return true;
    }
    return false;
  }
}
