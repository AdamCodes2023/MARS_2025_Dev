// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.commands.IntakePositionUp;
import frc.robot.subsystems.lights.Lights;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  //private final IntakePositionUp defaultPneumaticState;

  private final RobotContainer m_robotContainer;

  private final PowerDistribution powerDistributionHub;

  public Robot() {
    Lights.createShuffleboard();
    m_robotContainer = new RobotContainer();
    //defaultPneumaticState = new IntakePositionUp(m_robotContainer.pneumaticControl);
    //CameraServer.startAutomaticCapture();
    powerDistributionHub = new PowerDistribution(4, ModuleType.kRev);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    Lights.inAutonomous();
    if (!Lights.inAuto) {
      //if (Lights.notUsed) {
      Lights.getAllianceLights();
      //}
    } else {
      Lights.turnMars();
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    //defaultPneumaticState.end(false);
  }

  @Override
  public void disabledExit() {
    powerDistributionHub.setSwitchableChannel(true);
  }

  @Override
  public void autonomousInit() {
    Lights.turnMars();

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

    //Lights.notUsed = true;
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