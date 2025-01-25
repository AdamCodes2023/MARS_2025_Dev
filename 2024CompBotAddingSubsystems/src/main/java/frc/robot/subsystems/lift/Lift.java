// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {
  private TalonFX leftClimber, rightClimber;

  private final DutyCycleOut leftOut = new DutyCycleOut(0);
  private final DutyCycleOut rightOut = new DutyCycleOut(0);

  /** Creates a new Lift. */
  public Lift() {
    leftClimber = new TalonFX(30);

    rightClimber = new TalonFX(32);

    /* Configure the devices */
    var leftConfiguration = new TalonFXConfiguration();
    var rightConfiguration = new TalonFXConfiguration();

    /* User can optionally change the configs or leave it alone to perform a factory default */
    leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    leftClimber.getConfigurator().apply(leftConfiguration);

    rightClimber.getConfigurator().apply(rightConfiguration);

    leftClimber.setNeutralMode(NeutralModeValue.Brake);
    rightClimber.setNeutralMode(NeutralModeValue.Brake);

    leftClimber.setSafetyEnabled(true);
    rightClimber.setSafetyEnabled(true);
    
    createShuffleboard();
  }

  public void createShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("LIFT");
    tab.add("LIFT", this);
    tab.addNumber("OutputSpeed", this::getOutputSpeed);
    //tab.addBoolean("LeftLowLimitSwitch", this::atLeftLowLimit);
    //tab.addBoolean("RightLowLimitSwitch", this::atRightLowLimit);
    //tab.addBoolean("LeftHighLimitSwitch", this::atLeftHighLimit);
    //tab.addBoolean("RightHighLimitSwitch", this::atRightHighLimit);
  }

  public double getOutputSpeed() {
    return leftOut.Output;
  }

  public void runClimbers(double speed) {
    leftOut.Output = speed;
    rightOut.Output = speed;

    leftClimber.setControl(leftOut);
    rightClimber.setControl(rightOut);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
