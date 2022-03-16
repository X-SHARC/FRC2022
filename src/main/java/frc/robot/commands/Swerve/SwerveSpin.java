// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class SwerveSpin extends CommandBase {
  /** Creates a new SwerveSpin. */
  Swerve swerve;
  Intake intake;
  public SwerveSpin(Swerve sw, Intake intake) {
    this.swerve = sw;
    this.intake = intake;
    addRequirements(swerve,intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize(){
    if (intake.intakeSolenoid.get() == Value.kForward){
      intake.retractIntake();
    }

  }

  @Override
  public void execute() {
    swerve.drive(0, 0, 3, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
