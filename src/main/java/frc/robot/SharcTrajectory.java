// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.CollectCargoCommand;
import frc.robot.commands.ShootWhenReadyCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class SharcTrajectory {

    private Swerve swerve;
    private Alliance alliance;
    private Trajectory trajectory;
    private static double x_kp = 1.6;
    private static double y_kp = 1.9985;


    public ProfiledPIDController thetaController = new ProfiledPIDController((10), 0, 0, Constants.Swerve.kThetaControllerConstraints);


    public Trajectory getTrajectory() {
        return trajectory;
    }

    public SharcTrajectory(Swerve swerve, String autoMode) {
        this.swerve = swerve;
        this.alliance = DriverStation.getAlliance();
        switch (autoMode) {
            case "three":
                trajectory = PathPlanner.loadPath("ball31", 4, 2, false);
            case "five":
                trajectory = PathPlanner.loadPath("terminal", 4, 3.5);
            case "debug":
                trajectory = PathPlanner.loadPath("scurve", 2, 2);
            default:
                // XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
                // TODO: CHANGE THIS TO HAVE A PROPER DEFAULT
                // XXXXXXXXX VERY IMPORTANT XXXXXXXXXXXXXXXXX
                trajectory = PathPlanner.loadPath(autoMode, 4, 3);
            }
        }
        
    public Command getSwerveController() {
        var initialPose = trajectory.getInitialPose();
        swerve.resetOdometry(new Pose2d(initialPose.getTranslation(), ((PathPlannerState) trajectory.getStates().get(0)).holonomicRotation));
        swerve.addTrajectoryToField2d(trajectory);
        swerve.resetFieldOrientation(trajectory.getInitialPose().getRotation());
        
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        //thetaController.disableContinuousInput();
        
        
        thetaController.reset(swerve.getPose().getRotation().getRadians());

        PIDController x_pid = new PIDController(x_kp, 0, 0);
        PIDController y_pid = new PIDController(y_kp, 0, 0);
        
        
        SmartDashboard.putData("theta", thetaController);
        SmartDashboard.putData("x", x_pid);
        SmartDashboard.putData("y", y_pid);
        
        SwerveControllerCommand controller = new SwerveControllerCommand(
            trajectory,
            swerve::getPose, 
            Constants.Swerve.kinematics,
            x_pid,
            y_pid,
            thetaController,
            swerve::setClosedLoopStates,
            swerve
        );

        return controller.andThen(() -> swerve.drive(0, 0, 0, true));
    }


    public static Command getFiveBall(Swerve swerve, Conveyor conveyor, Shooter shooter, Intake intake, Storage storage) {
        PIDController x_pid = new PIDController(x_kp, 0, 0);
        PIDController y_pid = new PIDController(y_kp, 0, 0);
        
        ProfiledPIDController thetaController = new ProfiledPIDController(Math.PI, 0, 0, Constants.Swerve.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        //thetaController.disableContinuousInput();
        thetaController.reset(swerve.getPose().getRotation().getRadians());

        Trajectory[] trajectories = {
            PathPlanner.loadPath("three1", 6, 4.5, false),
            PathPlanner.loadPath("three2", 6, 4.5, false),
            PathPlanner.loadPath("three3", 6, 4.5, false),
            PathPlanner.loadPath("threeplustwo1", 6, 4.5, false),
            PathPlanner.loadPath("threeplustwo2", 6, 4.5, false)
        };
        
        for (int i = 0; i < trajectories.length; i++) {
            //SmartDashboard.putData("traj"+i, trajectories[i]);
        }

        var initialPose = trajectories[0].getInitialPose();
        swerve.resetOdometry(new Pose2d(initialPose.getTranslation(), ((PathPlannerState) trajectories[0].getStates().get(0)).holonomicRotation));
        swerve.resetFieldOrientation(trajectories[0].getInitialPose().getRotation());
        
        SmartDashboard.putData("theta", thetaController);
        SmartDashboard.putData("x", x_pid);
        SmartDashboard.putData("y", y_pid);

        CommandGroupBase commands = 
            (
                new ShootWhenReadyCommand(conveyor, shooter).withTimeout(.85)
                .andThen(() -> intake.extendIntake())
                .andThen(getControllerCommand(trajectories[0], swerve, x_pid, y_pid, thetaController))
                .andThen(getControllerCommand(trajectories[1], swerve, x_pid, y_pid, thetaController))
                .andThen(getControllerCommand(trajectories[2], swerve, x_pid, y_pid, thetaController))
                .andThen(() -> swerve.stopModules())
                .andThen((new ShootWhenReadyCommand(conveyor, shooter)).withTimeout(1.55))
                .andThen(getControllerCommand(trajectories[3], swerve, x_pid, y_pid, thetaController))
                .andThen(() -> swerve.stopModules())
                .andThen(new RunCommand(() -> {}).withTimeout(.4))
                .andThen(getControllerCommand(trajectories[4], swerve, x_pid, y_pid, thetaController))
                .andThen(() -> swerve.stopModules())
                .andThen((new ShootWhenReadyCommand(conveyor, shooter)).withTimeout(1.5))
            )
            .raceWith(new CollectCargoCommand(intake, storage))
        ;
        return commands;


    }

    public static Command getThreeBall(Swerve swerve, Conveyor conveyor, Shooter shooter, Intake intake, Storage storage) {
        PIDController x_pid = new PIDController(x_kp, 0, 0);
        PIDController y_pid = new PIDController(y_kp, 0, 0);
        
        ProfiledPIDController thetaController = new ProfiledPIDController(Math.PI, 0, 0, Constants.Swerve.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        //thetaController.disableContinuousInput();
        thetaController.reset(swerve.getPose().getRotation().getRadians());

        Trajectory[] trajectories = {
            PathPlanner.loadPath("three1", 4, 3, false),
            PathPlanner.loadPath("three2", 4, 3, false),
            PathPlanner.loadPath("three3", 4, 3, false)
        };
        
        for (int i = 0; i < trajectories.length; i++) {
            //SmartDashboard.putData("traj"+i, trajectories[i]);
        }

        var initialPose = trajectories[0].getInitialPose();
        swerve.resetOdometry(new Pose2d(initialPose.getTranslation(), ((PathPlannerState) trajectories[0].getStates().get(0)).holonomicRotation));
        swerve.resetFieldOrientation(trajectories[0].getInitialPose().getRotation());
        
        SmartDashboard.putData("theta", thetaController);
        SmartDashboard.putData("x", x_pid);
        SmartDashboard.putData("y", y_pid);

        CommandGroupBase commands = 
            (
                new ShootWhenReadyCommand(conveyor, shooter).withTimeout(.85)
                .andThen(() -> intake.extendIntake())
                .andThen(getControllerCommand(trajectories[0], swerve, x_pid, y_pid, thetaController))
                .andThen(getControllerCommand(trajectories[1], swerve, x_pid, y_pid, thetaController))
                .andThen(getControllerCommand(trajectories[2], swerve, x_pid, y_pid, thetaController))
                .andThen(() -> swerve.drive(0, 0, 0, true))
                .andThen((new ShootWhenReadyCommand(conveyor, shooter)).withTimeout(1.3))
            )
            .raceWith(new CollectCargoCommand(intake, storage))
        ;
        return commands;


    }

    public static Command getTwoBall(Swerve swerve, Conveyor conveyor, Shooter shooter, Intake intake, Storage storage) {
        PIDController x_pid = new PIDController(x_kp, 0, 0);
        PIDController y_pid = new PIDController(y_kp, 0, 0);
        
        ProfiledPIDController thetaController = new ProfiledPIDController(Math.PI, 0, 0, Constants.Swerve.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        //thetaController.disableContinuousInput();
        thetaController.reset(swerve.getPose().getRotation().getRadians());

        Trajectory[] trajectories = {
            PathPlanner.loadPath("twoA", 4, 3, false),
            PathPlanner.loadPath("twoB", 4, 3, false)
        };
        
        for (int i = 0; i < trajectories.length; i++) {
            //SmartDashboard.putData("traj"+i, trajectories[i]);
        }

        var initialPose = trajectories[0].getInitialPose();
        swerve.resetOdometry(new Pose2d(initialPose.getTranslation(), ((PathPlannerState) trajectories[0].getStates().get(0)).holonomicRotation));
        swerve.resetFieldOrientation(trajectories[0].getInitialPose().getRotation());
        
        SmartDashboard.putData("theta", thetaController);
        SmartDashboard.putData("x", x_pid);
        SmartDashboard.putData("y", y_pid);

        CommandGroupBase commands = 
            (
                new InstantCommand(() -> intake.extendIntake())
                .andThen(getControllerCommand(trajectories[0], swerve, x_pid, y_pid, thetaController))
                .andThen(getControllerCommand(trajectories[1], swerve, x_pid, y_pid, thetaController))
                .andThen(() -> swerve.drive(0, 0, 0, true))
                .andThen((new ShootWhenReadyCommand(conveyor, shooter)).withTimeout(1.3))
            )
            .raceWith(new CollectCargoCommand(intake, storage))
        ;
        return commands;


    }

    private static Command getControllerCommand(Trajectory trajectory, Swerve swerve, PIDController x_pid, PIDController y_pid, ProfiledPIDController thetaController) {
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
