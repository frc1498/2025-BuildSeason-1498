// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

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
    
    SmartDashboard.putData(CommandScheduler.getInstance());
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
    /*
    m_robotContainer.endEffectorCommand.wrist.wristSpin.setControl(m_robotContainer.endEffectorCommand.wrist.spinControl.withVelocity(0));
    m_robotContainer.endEffectorCommand.wrist.wristRotate.setControl(m_robotContainer.endEffectorCommand.wrist.rotateDutyCycleControl.withOutput(0));
 
    m_robotContainer.endEffectorCommand.arm.armRotate.setControl(m_robotContainer.endEffectorCommand.arm.rotateDutyCycleControl.withOutput(0));
 
    m_robotContainer.endEffectorCommand.elevator.elevatorDriveFront.setControl(m_robotContainer.endEffectorCommand.elevator.rotateDutyCycleControl.withOutput(0));

    m_robotContainer.intake.rotateMotor.setControl(m_robotContainer.intake.rotateDutyCycleControl.withOutput(0));
    m_robotContainer.intake.spinMotor.setControl(m_robotContainer.intake.spinMotorMode.withVelocity(0));
   
    m_robotContainer.climber.climberRotate.setControl(m_robotContainer.climber.rotateDutyCycleControl.withOutput(0));
    */

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

    m_robotContainer.endEffectorCommand.wrist.stop();
    m_robotContainer.endEffectorCommand.wrist.wristCoralStow();
    m_robotContainer.endEffectorCommand.arm.armCoralStow();
    m_robotContainer.endEffectorCommand.elevator.elevatorCoralStow();
    m_robotContainer.intake.intakeRaised();
    m_robotContainer.intake.rollerStop();
    m_robotContainer.climber.toClimberStow();

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
