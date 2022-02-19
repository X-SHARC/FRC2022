package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.StorageCommand;
import frc.robot.commands.Swerve.SwerveDriveCommand;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Swerve;


public class RobotContainer {
  //Subsystems
  Swerve swerveDrivetrain = new Swerve(true);
  Shooter shooter=new Shooter();
  Storage storage = new Storage();
  Conveyor conveyor = new Conveyor();
  Climb climb = new Climb();
  Intake intake = new Intake();


  //Joysticks
  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);

  //Commands
  SwerveDriveCommand driveCommand = new SwerveDriveCommand(swerveDrivetrain, driver);
  StorageCommand storageCommand = new StorageCommand(storage, conveyor, operator);


  public RobotContainer() {
    configureButtonBindings();
  }


  private void configureButtonBindings() {
    swerveDrivetrain.setDefaultCommand(driveCommand);
    storage.setDefaultCommand(storageCommand);

    Button shooterButton = new JoystickButton(operator, 2).whileHeld(new RunCommand(()->shooter.pidShooter(5000), shooter));
    shooterButton.whenReleased(new RunCommand(()-> shooter.stop(), shooter));

    Button intakeButton = new JoystickButton(operator, 5).whileHeld(new RunCommand(()-> intake.take_ball(), intake));
    intakeButton.whenReleased(new RunCommand(()-> intake.stop(), intake));

    Button intakeOutButton = new JoystickButton(operator, 6).whileHeld(new RunCommand(()-> intake.take_ball(), intake));
    intakeOutButton.whenReleased(new RunCommand(()-> intake.stop(), intake));
  }



  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}

