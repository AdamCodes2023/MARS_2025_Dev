// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.lights.Lights;

public class Climber extends SubsystemBase {
  private final NeutralOut stop;
  private final DutyCycleOut climberOut;
  private final TalonFX climberMotor;
  private final TalonFXConfiguration climberConfiguration;
  private final CurrentLimitsConfigs climberCurrentConfiguration;
  private final DigitalInput leftAttachment;
  private final DigitalInput rightAttachment;
  private final DigitalInput bottomLimit;
  private final DigitalInput topLimit;

  private boolean looking;

  /** Creates a new Climber. */
  public Climber() {
    stop = new NeutralOut();
    climberOut = new DutyCycleOut(0.0);

    climberMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_CANID.getValue());
    climberConfiguration = new TalonFXConfiguration();

    /* User can optionally change the configs or leave it alone to perform a factory default */
    climberConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    climberConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    climberCurrentConfiguration = new CurrentLimitsConfigs();
    climberCurrentConfiguration.withSupplyCurrentLowerLimit(Amps.of(50)) // Default limit of 50 A
      .withSupplyCurrentLimit(Amps.of(20)) // Reduce the limit to 20 A if we've limited to 50 A...
      .withSupplyCurrentLowerTime(Seconds.of(1.0)) // ...for at least 1 second
      .withSupplyCurrentLimitEnable(true); // And enable it

    climberCurrentConfiguration.withStatorCurrentLimit(Amps.of(120)) // Limit stator current to 120 A
      .withStatorCurrentLimitEnable(true); // And enable it

    climberConfiguration.CurrentLimits = climberCurrentConfiguration;

    climberMotor.getConfigurator().apply(climberConfiguration);

    climberMotor.setSafetyEnabled(false);

    leftAttachment = new DigitalInput(ClimberConstants.LEFT_ATTACHMENT_CHANNEL.getValue());
    rightAttachment = new DigitalInput(ClimberConstants.RIGHT_ATTACHMENT_CHANNEL.getValue());
    bottomLimit = new DigitalInput(ClimberConstants.BOTTOM_LIMIT_CHANNEL.getValue());
    topLimit = new DigitalInput(ClimberConstants.TOP_LIMIT_CHANNEL.getValue());

    looking = true;

    createShuffleboard();
  }

  private void createShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("CLIMBER");
    tab.add("ClimberSubsystem", this);
    tab.addNumber("Output", this::getClimberOutput);
    tab.addNumber("Current", this::getClimberCurrent);
    tab.addBoolean("LeftAttachment", this::getLeftAttachment);
    tab.addBoolean("RightAttachment", this::getRightAttachment);
    tab.addBoolean("BottomLimit", this::getBottomLimit);
    tab.addBoolean("TopLimit", this::getTopLimit);
  }

  private double getClimberOutput() {
    return climberOut.Output;
  }

  private double getClimberCurrent() {
    return climberMotor.getSupplyCurrent().getValueAsDouble();
    //return climberMotor.getStatorCurrent().getValueAsDouble();
  }
  
  private boolean getLeftAttachment() {
    return leftAttachment.get();
  }

  private boolean getRightAttachment() {
    return rightAttachment.get();
  }

  private boolean getBottomLimit() {
    return bottomLimit.get();
  }

  private boolean getTopLimit() {
    return topLimit.get();
  }

  public void stop() {
    climberOut.Output = 0.0;
    climberMotor.setControl(stop);
  }

  public void run(double speed) {
    climberOut.Output = speed;
    climberMotor.setControl(climberOut);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (getBottomLimit() || getTopLimit()) {
      if (looking) {
        stop();
        looking = false;
      }
    } else {
      looking = true;
    }

    if (getLeftAttachment() || getRightAttachment()) {
      Lights.climbMode = true;
      if (getLeftAttachment()) {
        Lights.turnClimberLeftAttachment();
      }
      if (getRightAttachment()) {
        Lights.turnClimberRightAttachment();
      }
    } else {
      Lights.climbMode = false;
      //Lights.turnOffElevator();
    }
  }
}
