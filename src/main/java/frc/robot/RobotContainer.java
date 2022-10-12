package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.RGBCommand;
import frc.robot.commands.SetLedState;
import frc.robot.commands.ThePoPo;
import frc.robot.commands.Swerve.SwerveDriveCommand;
import frc.robot.lib.drivers.WS2812Driver;
import frc.robot.subsystems.Swerve;



public class RobotContainer {
  public static RobotState state = new RobotState();
  //Subsystems
  Swerve swerveDrivetrain = new Swerve(true);
  
  WS2812Driver addressableLED = new WS2812Driver(0,15);
  Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  //Joysticks
  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);

  //UsbCamera camera = new UsbCamera("Driver", 0);

  //Commands
  SwerveDriveCommand driveCommand = new SwerveDriveCommand(swerveDrivetrain, driver);
  RGBCommand rgbCommand = new RGBCommand(addressableLED);
  ThePoPo arka_sokaklar = new ThePoPo(addressableLED);
  SendableChooser<Command> autonomousChooser = new SendableChooser<>();
  SetLedState setLEDstate = new SetLedState(addressableLED);

  public RobotContainer() {
    //camera.setExposureAuto();
    //camera.setWhiteBalanceAuto();
    //camera.setFPS(30);
    //camera.setResolution(640, 480);
    configureButtonBindings();
    //addressableLED.toggleRGB();
    
  }


  private void configureButtonBindings() {
    swerveDrivetrain.setDefaultCommand(driveCommand);
    
  }


  public Command getAutonomousCommand() {

    //return SharcTrajectory.getFiveBall(swerveDrivetrain, conveyor, shooter, intake, storage);
    return new RunCommand(()->swerveDrivetrain.drive(0.5, 0.5, 0, true)).withTimeout(1.3);
  }
}

