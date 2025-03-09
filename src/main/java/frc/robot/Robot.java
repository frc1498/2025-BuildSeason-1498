// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public boolean hasDeterminedAlliance = false;



  public Robot() {
    m_robotContainer = new RobotContainer();

  }

  @Override
  public void robotInit() {
    //Port Forwarding for the limelight.
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, "limelight.local", port);       
    }
    //Get the deploy directory for Elastic.
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    RobotController.setBrownoutVoltage(4.5);
    
    SmartDashboard.putData(CommandScheduler.getInstance());

    //setup the camera
    
    UsbCamera camera = CameraServer.startAutomaticCapture(0);
    camera.setResolution(640, 480);


  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {
    
 

  }

  @Override
  public void disabledPeriodic() {

    //Constantly check for the current alliance.
    //This 'latches' once it has been retrieved.
    if (!hasDeterminedAlliance && DriverStation.isDSAttached()) {
      switch (DriverStation.getAlliance().get()) {
      case Blue:
        m_robotContainer.allianceColor = Alliance.Blue;
      break;
      case Red:
        m_robotContainer.allianceColor = Alliance.Red;
      break;
      }

      //After determining the alliance, set a flag so any trigger is only run once.
      hasDeterminedAlliance = true;
      m_robotContainer.hasDeterminedAlliance = true;

      };


      }


  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
/*
    m_robotContainer.endEffectorCommand.wrist.stop();
    m_robotContainer.endEffectorCommand.wrist.wristCoralStow();
    m_robotContainer.endEffectorCommand.arm.armCoralStow();
    m_robotContainer.endEffectorCommand.elevator.elevatorCoralStow();
    m_robotContainer.intake.intakeRaised();
    m_robotContainer.intake.rollerStop();
    m_robotContainer.climber.toClimberStow();
*/


  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
