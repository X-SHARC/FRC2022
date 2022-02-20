package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.CollectCargo;
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
  Shooter shooter = new Shooter();
  Storage storage = new Storage();
  Conveyor conveyor = new Conveyor();
  Climb climb = new Climb();
  Intake intake = new Intake();

  Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);


  //Joysticks
  XboxController driver = new XboxController(0);
  Joystick operator = new Joystick(1);

  //Commands
  SwerveDriveCommand driveCommand = new SwerveDriveCommand(swerveDrivetrain, driver);
  StorageCommand storageCommand = new StorageCommand(storage, conveyor, operator, intake);
  CollectCargo collectCargo = new CollectCargo(intake, storage);
  ClimberCommand climberCommand = new ClimberCommand(climb, operator);


  public RobotContainer() {
    configureButtonBindings();
    compressor.enableDigital();
  }


  private void configureButtonBindings() {
    swerveDrivetrain.setDefaultCommand(driveCommand);
    storage.setDefaultCommand(storageCommand);
    climb.setDefaultCommand(climberCommand);

    Button shooterButton = new JoystickButton(operator, 1).whileHeld(new RunCommand(()->shooter.shootBall(), shooter));
    shooterButton.whenReleased(new RunCommand(()-> shooter.stop(), shooter));

    //Button intakeButton = new JoystickButton(operator, 5).whileHeld(new RunCommand(()-> intake.runForward(), intake));
    //intakeButton.whenReleased(new RunCommand(()-> intake.stop(), intake));

    Button intakeButton = new JoystickButton(operator, 5).whileHeld(collectCargo);

    Button intakeOutButton = new JoystickButton(operator, 6).whileHeld(new RunCommand(()-> intake.runBackwards(), intake));
    intakeOutButton.whenReleased(new RunCommand(()-> intake.stop(), intake));

    new JoystickButton(operator, 9).whenPressed(new RunCommand(()-> intake.extendIntake(), intake));
    new JoystickButton(operator, 10).whenPressed(new RunCommand(()-> intake.retractIntake(), intake));

    Button conveyorButton = new JoystickButton(operator, 2).whileHeld(new RunCommand(()-> conveyor.feedBall(), conveyor));
    conveyorButton.whenReleased(new RunCommand(()-> conveyor.stop(), conveyor));
    
  }


  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}

