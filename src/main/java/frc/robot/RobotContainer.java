package frc.robot;

import java.lang.invoke.ConstantCallSite;
import java.util.function.BooleanSupplier;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.cscore.UsbCamera;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoAlignWithShoot;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.CollectCargoCommand;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.PlayMusic;
import frc.robot.commands.RGBCommand;
import frc.robot.commands.SetLedState;
import frc.robot.commands.ShootWhenReadyCommand;
import frc.robot.commands.ThePoPo;
import frc.robot.commands.Swerve.AutoAlign;
import frc.robot.commands.Swerve.AutoDistance;
import frc.robot.commands.Swerve.SwerveAntiDefense;
import frc.robot.commands.Swerve.SwerveDriveCommand;
import frc.robot.commands.Swerve.SwerveSpin;
import frc.robot.lib.drivers.WS2812Driver;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Swerve;



public class RobotContainer {
  public static RobotState state = new RobotState();
  //Subsystems
  Swerve swerveDrivetrain = new Swerve(true);
  Shooter shooter = new Shooter();
  Storage storage = new Storage();
  Conveyor conveyor = new Conveyor();
  Climb climb = new Climb();
  Intake intake = new Intake();
  Limelight limelight = new Limelight();

  WS2812Driver addressableLED = new WS2812Driver(0,15);
  Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  //Joysticks
  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);

  //UsbCamera camera = new UsbCamera("Driver", 0);

  //Commands
  SwerveDriveCommand driveCommand = new SwerveDriveCommand(swerveDrivetrain, driver);
  CollectCargoCommand collectCargoCommand = new CollectCargoCommand(intake, storage);
  ShootWhenReadyCommand shootWhenReadyCommand = new ShootWhenReadyCommand(conveyor, shooter,swerveDrivetrain);
  ClimberCommand climberCommand = new ClimberCommand(climb, operator);
  RGBCommand rgbCommand = new RGBCommand(addressableLED);
  ThePoPo arka_sokaklar = new ThePoPo(addressableLED);
  ConveyorCommand conveyorCommand = new ConveyorCommand(operator, conveyor);
  SwerveAntiDefense swerveAntiDefence = new SwerveAntiDefense(swerveDrivetrain);
  SwerveSpin swerveSpin = new SwerveSpin(swerveDrivetrain,intake);
  PlayMusic playMusic = new PlayMusic(shooter);
  AutoAlign autoAlign = new AutoAlign(limelight, swerveDrivetrain);
  AutoAlignWithShoot alignWithShoot = new AutoAlignWithShoot(limelight, swerveDrivetrain, conveyor, shooter);
  AutoDistance autoDistance = new AutoDistance(swerveDrivetrain, limelight);
  SendableChooser<Command> autonomousChooser = new SendableChooser<>();
  SetLedState setLEDstate = new SetLedState(addressableLED);

  public RobotContainer() {
    //camera.setExposureAuto();
    //camera.setWhiteBalanceAuto();
    //camera.setFPS(30);
    //camera.setResolution(640, 480);
    configureButtonBindings();
    compressor.enableDigital();
    //addressableLED.toggleRGB();
    autonomousChooser.setDefaultOption("Do nothing", new PrintCommand("Doing nothing."));
    autonomousChooser.addOption("Two balls", SharcTrajectory.getTwoBall(swerveDrivetrain, conveyor, shooter, intake, storage, limelight));
    autonomousChooser.addOption("Three balls", SharcTrajectory.getThreeBall(swerveDrivetrain, conveyor, shooter, intake, storage));
    autonomousChooser.addOption("Five balls", SharcTrajectory.getFiveBall(swerveDrivetrain, conveyor, shooter, intake, storage));
    SmartDashboard.putData(autonomousChooser);
    
  }


  private void configureButtonBindings() {
    //new JoystickButton(operator, 4).toggleWhenPressed(playMusic);
    swerveDrivetrain.setDefaultCommand(driveCommand);
    addressableLED.setDefaultCommand(setLEDstate);
    climb.setDefaultCommand(climberCommand);
    conveyor.setDefaultCommand(conveyorCommand);

    /*Button resetOdometryButton = new JoystickButton(driver, 7);
    resetOdometryButton.whenPressed(new InstantCommand(() -> {
      swerveDrivetrain.resetOdometry(new Pose2d());
      swerveDrivetrain.resetFieldOrientation();
    }));*/



    // operator buttonsss
    //bok
    Button autoDistanceButton = new JoystickButton(driver, 1)
      .whileHeld(autoDistance);

    Button shooterButton =
      new JoystickButton(operator, 3)
      .whileHeld(new RunCommand(()->shooter.setRPM(2700), shooter))
      .whenReleased(new RunCommand(()-> shooter.stop(), shooter));

    //normally operator
    Button autoShootButton = new JoystickButton(driver, 6)
      .whileHeld(shootWhenReadyCommand);

    Button autoAlignAndShootButton = new JoystickButton(driver, 5)
      .whileHeld(alignWithShoot);

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
    new JoystickButton(driver, 2).whileHeld(swerveAntiDefence.alongWith(new RunCommand(()-> intake.retractIntake(), intake)));

    new JoystickButton(driver, 4).toggleWhenPressed(swerveSpin);

    Button storageButton = new JoystickButton(operator, 2).whileHeld(new RunCommand(()->storage.storageBackwards(), storage));
    storageButton.whenReleased(new RunCommand(()->storage.stop(), storage));


    //Button autoAlignButton = new JoystickButton(driver, 3).whileHeld(autoAlign);
    
  }


  public Command getAutonomousCommand() {

    return SharcTrajectory.getTwoBall(swerveDrivetrain, conveyor, shooter, intake, storage, limelight);
    //return new RunCommand(()->swerveDrivetrain.drive(0.5, 0.5, 0, true)).withTimeout(1.3);
  }
}

