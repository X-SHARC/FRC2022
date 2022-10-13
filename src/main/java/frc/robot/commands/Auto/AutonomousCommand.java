// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Swerve;

public class AutonomousCommand extends SequentialCommandGroup {

  public static Trajectory[] trajectories;
  public static Trajectory[] allTrajectories = {
    PathPlanner.loadPath("three1", 4, 3, false),
    PathPlanner.loadPath("three2", 4, 3, false),
    PathPlanner.loadPath("three3", 4, 3, false),
    PathPlanner.loadPath("threeplustwo1", 4, 3, false),
    PathPlanner.loadPath("threeplustwo2", 4, 3, false)
  };

  private Swerve swerve;
  private Conveyor conveyor;
  private Shooter shooter;
  private Intake intake;
  private Storage storage;


  protected static PIDController x_pid = new PIDController(0.87, 0, 0);
  protected static PIDController y_pid = new PIDController(8, 0, 0);
  protected static ProfiledPIDController thetaController =
    new ProfiledPIDController(10, 0, 0, Constants.Swerve.kThetaControllerConstraints);

  
  public AutonomousCommand(
    Trajectory[] trajectories,
    Swerve swerve,
    Conveyor conveyor,
    Shooter shooter,
    Intake intake,
    Storage storage
  ) {
    this.trajectories = allTrajectories;
    this.swerve = swerve;
    this.conveyor = conveyor;
    this.shooter = shooter;
    this.intake = intake;
    this.storage = storage;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.reset(swerve.getPose().getRotation().getRadians());
    
    SmartDashboard.putData("auto/theta_PID", thetaController);
    SmartDashboard.putData("auto/x_PID", x_pid);
    SmartDashboard.putData("auto/y_PID", y_pid);


  }

  public SwerveControllerCommand getTrajectoryControllerCommand(
    Trajectory trajectory
  ) {
    return new SwerveControllerCommand(
        trajectory,
        swerve::getPose, 
        Constants.Swerve.kinematics,
        x_pid,
        y_pid,
        thetaController,
        swerve::setClosedLoopStates,
        swerve
    );
  }

  protected void resetOdometry() {
    var initialPose = trajectories[0].getInitialPose();
    swerve.resetOdometry(
      new Pose2d(
        initialPose.getTranslation(),
        ((PathPlannerState) trajectories[0].getStates().get(0)).holonomicRotation
      )
    );
    swerve.resetFieldOrientation(trajectories[0].getInitialPose().getRotation());
  }

  protected Command getControllerCommand(Trajectory trajectory, Swerve swerve, PIDController x_pid, PIDController y_pid, ProfiledPIDController thetaController) {
    return new SwerveControllerCommand(
        trajectory,
        swerve::getPose, 
        Constants.Swerve.kinematics,
        x_pid,
        y_pid,
        thetaController,
        swerve::setClosedLoopStates,
        swerve
    );
}
}
