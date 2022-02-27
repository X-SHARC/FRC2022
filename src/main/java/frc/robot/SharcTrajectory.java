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
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class SharcTrajectory {

    private Swerve swerve;
    private Alliance alliance;
    private Trajectory trajectory;
    private double kp = 1;


    public ProfiledPIDController thetaController = new ProfiledPIDController(Math.PI, 0, 0, Constants.Swerve.kThetaControllerConstraints);


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
        PIDController x_pid = new PIDController(kp, 0, 0);
        PIDController y_pid = new PIDController(kp, 0, 0);
        
        
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
}
