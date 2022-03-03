package frc.robot;

import java.lang.invoke.ConstantCallSite;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.CollectCargoCommand;
import frc.robot.commands.RGBCommand;
import frc.robot.commands.ShootWhenReadyCommand;
import frc.robot.commands.StorageCommand;
import frc.robot.commands.ThePoPo;
import frc.robot.commands.Auto.AutoFiveBalls;
import frc.robot.commands.Auto.AutoThreeBalls;
import frc.robot.commands.Swerve.SwerveCounterDefense;
import frc.robot.commands.Swerve.SwerveDriveCommand;
import frc.robot.lib.drivers.WS2812Driver;
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

  WS2812Driver addressableLED = new WS2812Driver(0,15);

  Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);


  //Joysticks
  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);

  //Commands
  SwerveDriveCommand driveCommand = new SwerveDriveCommand(swerveDrivetrain, driver);
  StorageCommand storageCommand = new StorageCommand(storage, conveyor, intake);
  CollectCargoCommand collectCargoCommand = new CollectCargoCommand(intake, storage);
  ShootWhenReadyCommand shootWhenReadyCommand = new ShootWhenReadyCommand(conveyor, shooter, swerveDrivetrain);
  ClimberCommand climberCommand = new ClimberCommand(climb, operator);
  RGBCommand rgbCommand = new RGBCommand(addressableLED);
  ThePoPo arka_sokaklar = new ThePoPo(addressableLED);
  SwerveCounterDefense swerveCounterDefense = new SwerveCounterDefense(swerveDrivetrain, driver);

  SendableChooser<Command> autonomousChooser = new SendableChooser<>();

  public RobotContainer() {
    configureButtonBindings();
    compressor.enableDigital();
    addressableLED.toggleRGB();

    autonomousChooser.setDefaultOption("Do nothing", new PrintCommand("Doing nothing."));
    autonomousChooser.addOption("Three balls", new AutoThreeBalls(swerveDrivetrain, conveyor, shooter, intake, storage));
    autonomousChooser.addOption("Five balls", new AutoFiveBalls(swerveDrivetrain, conveyor, shooter, intake, storage));
    SmartDashboard.putData(autonomousChooser);
  }


  private void configureButtonBindings() {
    swerveDrivetrain.setDefaultCommand(driveCommand);
    addressableLED.setDefaultCommand(arka_sokaklar);
    climb.setDefaultCommand(climberCommand);

    Button resetOdometryButton = new JoystickButton(driver, 7);
    resetOdometryButton.whenPressed(new InstantCommand(() -> {
      swerveDrivetrain.resetOdometry(new Pose2d());
      swerveDrivetrain.resetFieldOrientation();
    }));



    // operator buttons

    Button[] triggers = {
      new Button(new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
          return Math.abs(operator.getRightTriggerAxis()) > 0.4;
        }
      }),
      new Button(new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
          return Math.abs(operator.getLeftTriggerAxis()) > 0.4;
        }
      }),
      new Button(new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
          return Math.abs(driver.getRightTriggerAxis()) > 0.4;
        }
      }),
      new Button(new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
          return Math.abs(driver.getRightTriggerAxis()) > 0.4;
        }
      })
    };

    triggers[0]
      .whileHeld(new RunCommand(()-> conveyor.feedBall(), conveyor))
      .whenReleased(new RunCommand(()-> conveyor.stop(), conveyor));
    
    triggers[1]
      .whileHeld(new RunCommand(() -> conveyor.retractBall(), conveyor))
      .whenReleased(new RunCommand(()-> conveyor.stop(), conveyor));
      
    triggers[2]
      .whileHeld(shootWhenReadyCommand);

    Button shooterButton =
      new JoystickButton(operator, 3)
      .whileHeld(new RunCommand(()->shooter.setRPM(2550), shooter))
      .whenReleased(new RunCommand(()-> shooter.stop(), shooter));

    //normally operator
    Button autoShootButton = new JoystickButton(driver, 4)
      .whileHeld(shootWhenReadyCommand);

    Button[] joystickPressed = {
      new JoystickButton(operator, 9),
      new JoystickButton(operator, 10)
    };
    

    Button[] intakeButtons = {
      new JoystickButton(driver, 6),
      new JoystickButton(operator, 6),
      new JoystickButton(operator, 5)
    };

    //intakeButtons[0]
      //.whileHeld(collectCargoCommand);

    intakeButtons[1]
      .whileHeld(collectCargoCommand);
    
    intakeButtons[2]
      .whileHeld(new RunCommand(()-> intake.runBackwards(), intake))
      .whenReleased(new RunCommand(()-> intake.stop(), intake));


      Button[] intakeExtensionButtons = {
        new JoystickButton(operator, 10),
        new JoystickButton(operator, 9)
      };

      intakeExtensionButtons[0]
        .whenPressed(new RunCommand(()-> intake.extendIntake(), intake));

      intakeExtensionButtons[1]
        .whenPressed(new RunCommand(()-> intake.retractIntake(), intake));

    Button popoButton = new JoystickButton(operator, 7);
    popoButton.whileHeld(arka_sokaklar);


    new JoystickButton(operator, 8).whileHeld(new RunCommand(()->addressableLED.turnOff(), addressableLED));
  }


  public Command getAutonomousCommand() {

    return autonomousChooser.getSelected();
  }
}

