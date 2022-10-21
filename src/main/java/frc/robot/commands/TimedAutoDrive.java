// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TimedAutoDrive extends CommandBase {
  private final Timer m_timer;


  /** Creates a new TimedDrive. */
  public TimedAutoDrive() {
    m_timer =  new Timer();
    m_timer.reset();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.getDrive());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_timer.get() < 10.0){
      RobotContainer.getDrive().tankDrive(0.3, 0.3);
    }
    else if(m_timer.get() < 15.0){
      RobotContainer.getDrive().tankDrive(-0.3, 0.3);
    }
    else if(m_timer.get() < 15.0){
      RobotContainer.getDrive().tankDrive(-0.3, -0.3);
    }
    else if(m_timer.get() < 25.0){
      RobotContainer.getDrive().tankDrive(0.3, 0.3);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.getDrive().tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return(m_timer.get() > 25.0);
  }
}
