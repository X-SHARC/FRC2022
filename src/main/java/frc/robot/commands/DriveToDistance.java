// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve;

public class DriveToDistance extends SubsystemBase {
  /** Creates a new DriveToDistance. */
  Swerve swerveDrivetrain;

  double setpoint = 100;
  double output;

  double kp;
  PIDController distancePID = new PIDController(kp, 0, 0);
  public DriveToDistance(Swerve sw) {
    this.swerveDrivetrain = sw;
  }

  @Override
  public void periodic() {
    swerveDrivetrain.drive(0, output, 0, true);
  }
}
