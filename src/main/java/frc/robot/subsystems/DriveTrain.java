// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveTrain extends SubsystemBase {

  //left and right talon motors
  private WPI_TalonSRX left = new WPI_TalonSRX(Constants.leftPort);
  private WPI_TalonSRX right = new WPI_TalonSRX(Constants.rightPort);
  private double ticksToMeters = (127.0/10581.0)/100.0;

  //navX (gyro--returns angle)
  private AHRS gyro = new AHRS(SPI.Port.kMXP);

  /** Creates a new DriveTrain. This is the DriveTrain constructor*/
  public DriveTrain() {
    //configuring talons to default settings
    left.configFactoryDefault();
    right.configFactoryDefault();

    //depending on your robot, this could change. Figure out in testing
    left.setInverted(false);
    right.setInverted(true);
    
    //configuring encoders to relative mode
    left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  }


  //two-stick drive
  public void tankDrive(double lPower, double rPower){
      //setting inputted power to talons
      left.set(ControlMode.PercentOutput, lPower);
      right.set(ControlMode.PercentOutput, rPower);
  }
  public double getPos(){
    return((left.getSelectedSensorPosition())+right.getSelectedSensorPosition()/2*ticksToMeters);
  }
  public void resetEncoders(){
    left.setSelectedSensorPosition(0);
    right.setSelectedSensorPosition(0);
  }

  //using encoders to get distance travelled in ticks (raw sensor units)
  public double getDistanceTicks(){
       double rightTicks = right.getSelectedSensorPosition();
       double leftTicks = left.getSelectedSensorPosition();

       return (rightTicks + leftTicks) / 2;
  }

  public double getAngle(){
    //returns angle about Z axis from -180 to 180
    //neg left, pos right
    System.out.println("ANGLE" + gyro.getAngle());
    return gyro.getAngle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tankDrive(RobotContainer.getJoy().getY(), RobotContainer.getJoy1().getY());

    //look for this message in the RioLog and in Driverstation!
    System.out.println("HELLO!! RUNNING TANK DRIVE :)");

    //look for encoder raw sensor units & angle turned in SmartDashboard
    SmartDashboard.putNumber("Distance In Raw Units", getDistanceTicks());
    SmartDashboard.putNumber("Angle turned", getAngle());
  }
}
