package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Swerve.SwerveDriveCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;


public class RobotContainer {
  //Subsystems
  Swerve swerveDrivetrain = new Swerve(true);
  Shooter shooter=new Shooter();

  //Joysticks
  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);

  //Commands
  SwerveDriveCommand driveCommand = new SwerveDriveCommand(swerveDrivetrain, driver);


  public RobotContainer() {
    configureButtonBindings();
  }


  private void configureButtonBindings() {
    swerveDrivetrain.setDefaultCommand(driveCommand);

    Button shooterButton = new JoystickButton(operator, 2).whileHeld(new RunCommand(()->shooter.pidShooter(5000), shooter));
    shooterButton.whenReleased(new RunCommand(()-> shooter.stop(), shooter));

  }



  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}

