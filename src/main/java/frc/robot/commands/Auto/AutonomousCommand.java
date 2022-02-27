// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousCommand extends SequentialCommandGroup {

  public static Trajectory[] trajectories;
  public static Trajectory[] allTrajectories = {
    PathPlanner.loadPath("three1", 4, 3, false),
    PathPlanner.loadPath("three2", 4, 3, false),
    PathPlanner.loadPath("three3", 4, 3, false),
    PathPlanner.loadPath("threeplustwo1", 4, 3, false),
    PathPlanner.loadPath("threeplustwo2", 4, 3, false)
  };

  Swerve swerve;
  Conveyor conveyor;
  Shooter shooter;
  Intake intake;
  Storage storage;


  private static PIDController x_pid = new PIDController(0.87, 0, 0);
  private static PIDController y_pid = new PIDController(8, 0, 0);
  private static ProfiledPIDController thetaController =
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
    this.sverwe = sverwe;
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

  public static SwerveControllerCommand getTrajectoryControllerCommand(
    Trajectory trajectory,
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

  private void resetOdometry() {
    var initialPose = trajectories[0].getInitialPose();
    swerve.resetOdometry(
      new Pose2d(
        initialPose.getTranslation(),
        ((PathPlannerState) trajectories[0].getStates().get(0)).holonomicRotation
      )
    );
    swerve.resetFieldOrientation(trajectories[0].getInitialPose().getRotation());
  }
}
