// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class AutoAlign extends CommandBase {
  /** Creates a new AutoAlign. */
  PIDController rotController = new PIDController(0.0798, 0, 0.00274);
  double targetAngle = 0;
  double currentAngle = 0;
  Limelight LL;
  Swerve swerve;

  public AutoAlign(Limelight LL, Swerve swerve) {
    this.LL = LL;
    this.swerve = swerve;
    addRequirements(LL);
    rotController.setTolerance(1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentAngle = swerve.getGyroDouble();
    targetAngle = LL.getX() + currentAngle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetAngle = LL.getX();
    targetAngle = LL.getX() + currentAngle;
    swerve.drive(0, 0, -Constants.Swerve.kMaxAngularSpeed * rotController.calculate(LL.getX(), 0), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotController.atSetpoint();
  }
}
