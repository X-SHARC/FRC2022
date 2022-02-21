// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class SharcTrajectory {

    private Swerve swerve;
    private Alliance alliance;
    private AutoMode auto;
    private PathPlannerTrajectory trajectory;
    private double kp;
    private double theta_kP;

    public enum AutoMode {
        THREE_BALL, FIVE_BALL, DEBUGGING
    }
    
    public SharcTrajectory(Swerve swerve, AutoMode autoMode) {
        this.swerve = swerve;
        this.alliance = DriverStation.getAlliance();
        this.auto = autoMode;
        switch (autoMode) {
            case THREE_BALL:
                trajectory = PathPlanner.loadPath("3Ball", 4, 3.5);
            case FIVE_BALL:
                trajectory = PathPlanner.loadPath("terminal", 4, 3.5);
            case DEBUGGING:
                trajectory = PathPlanner.loadPath("debugging", 4, 3.5);
            default:
                trajectory = PathPlanner.loadPath("debugging", 4, 3.5);
        }
    }

    public Command getSwerveController(){
        ProfiledPIDController thetaController = new ProfiledPIDController(theta_kP, 0, 0, Constants.Swerve.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        PPSwerveControllerCommand controller = new PPSwerveControllerCommand(
            trajectory,
            swerve::getPose,
            Constants.Swerve.kinematics,
            new PIDController(kp, 0, 0),
            new PIDController(kp, 0, 0),
            thetaController,
            swerve::setClosedLoopStates,
            swerve);

        return controller.andThen(() -> swerve.drive(0, 0, 0, false));
    }
}
