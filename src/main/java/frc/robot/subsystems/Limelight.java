// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.vision.VisionTarget;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private NetworkTable table;

  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;

  protected double x = 0;
  protected double y = 0;
  protected double area = 0;

  //create an enum named mode consisting of the following values: processing, off, on and blinking. All names must be capital
  public enum mode {
    PROCESSING, OFF, ON, BLINKING
  }

  VisionTarget target = new VisionTarget(2.64, 1.05, 58.8);

  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
  }

  public double getDistance(){
    return target.getDistance();
  }
  
  //write getters to get x y and area
  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public double getArea() {
    return area;
  }

  //write setter for camera mode of limelight

  public void setCamMode(mode mode) {
    // 0 for vision processing
    // 1 force off
    // 2 force blink
    // 3 force on
    //create a switch statement and set the mode to the correct value based on the previous comments respectively
    switch (mode) {
      case PROCESSING:
        table.getEntry("camMode").setNumber(0);
        break;
      case OFF:
        table.getEntry("camMode").setNumber(1);
        break;
      case BLINKING:
        table.getEntry("camMode").setNumber(2);
        break;
      case ON:
        table.getEntry("camMode").setNumber(3);
        break;
    }
  }



  @Override
  public void periodic() {
    // read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    target.update(x, y);
    area = ta.getDouble(0.0);

    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Limelight Distance", getDistance());
    SmartDashboard.putNumber("angle", 
      Math.toDegrees(
        Math.atan( 
         ( (2.64-1.05) - 1.36*Math.tan(Math.toRadians(y) ) ) /
         ( 1.36 + (2.64-1.05) * Math.tan(Math.toRadians(y) ))
       )
      )
    );
    
  }
}
