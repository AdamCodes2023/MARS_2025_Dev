// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {
  private TalonFX leftClimber, rightClimber;
  private TalonFXConfiguration leftConfiguration, rightConfiguration;

  private final DutyCycleOut leftOut = new DutyCycleOut(0);
  private final DutyCycleOut rightOut = new DutyCycleOut(0);

  /** Creates a new Lift. */
  public Lift() {
    leftClimber = new TalonFX(30);

    rightClimber = new TalonFX(32);

    /* Configure the devices */
    leftConfiguration = new TalonFXConfiguration();
    rightConfiguration = new TalonFXConfiguration();

    /* User can optionally change the configs or leave it alone to perform a factory default */
    leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    leftConfiguration.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
    leftConfiguration.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
    leftConfiguration.HardwareLimitSwitch.ForwardLimitEnable = true;
    rightConfiguration.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
    rightConfiguration.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
    rightConfiguration.HardwareLimitSwitch.ForwardLimitEnable = true;

    leftConfiguration.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    leftConfiguration.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    leftConfiguration.HardwareLimitSwitch.ReverseLimitEnable = true;
    rightConfiguration.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    rightConfiguration.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    rightConfiguration.HardwareLimitSwitch.ReverseLimitEnable = true;

    leftClimber.getConfigurator().apply(leftConfiguration);

    rightClimber.getConfigurator().apply(rightConfiguration);

    leftClimber.setNeutralMode(NeutralModeValue.Brake);
    rightClimber.setNeutralMode(NeutralModeValue.Brake);

    leftClimber.setSafetyEnabled(false);
    rightClimber.setSafetyEnabled(false);
    
    createShuffleboard();
  }

  public void createShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("LIFT");
    tab.add("LIFT", this);
    tab.addNumber("OutputSpeed", this::getOutputSpeed);
    tab.addBoolean("LeftLowLimitSwitch", this::atLeftLowLimit);
    tab.addBoolean("RightLowLimitSwitch", this::atRightLowLimit);
    tab.addBoolean("LeftHighLimitSwitch", this::atLeftHighLimit);
    tab.addBoolean("RightHighLimitSwitch", this::atRightHighLimit);
  }

  public boolean atLeftLowLimit() {
    double val = leftClimber.getForwardLimit().getValueAsDouble();
    if (val != 1.0) {
        return true;
    } else {
        return false;
    }
  }

  public boolean atRightLowLimit() {
    double val = rightClimber.getForwardLimit().getValueAsDouble();
    if (val != 1.0) {
        return true;
    } else {
        return false;
    }
  }

  public boolean atLeftHighLimit() {
    double val = leftClimber.getReverseLimit().getValueAsDouble();
    if (val != 1.0) {
        return true;
    } else {
        return false;
    }
  }

  public boolean atRightHighLimit() {
    double val = rightClimber.getReverseLimit().getValueAsDouble();
    if (val != 1.0) {
        return true;
    } else {
        return false;
    }
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
