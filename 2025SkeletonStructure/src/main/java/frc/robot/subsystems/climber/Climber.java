// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

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

public class Climber extends SubsystemBase {
  private final NeutralOut stop;
  private final DutyCycleOut climberOut;
  private final TalonFX climberMotor;
  private final TalonFXConfiguration climberConfiguration;
  private final DigitalInput leftAttachment;
  private final DigitalInput rightAttachment;
  private final DigitalInput bottomLimit;
  private final DigitalInput topLimit;

  private boolean looking;

  /** Creates a new Climber. */
  public Climber() {
    stop = new NeutralOut();
    climberOut = new DutyCycleOut(0.0);

    climberMotor = new TalonFX(99);
    climberConfiguration = new TalonFXConfiguration();

    /* User can optionally change the configs or leave it alone to perform a factory default */
    climberConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    climberConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    climberMotor.getConfigurator().apply(climberConfiguration);

    climberMotor.setSafetyEnabled(false);

    leftAttachment = new DigitalInput(99);
    rightAttachment = new DigitalInput(99);
    bottomLimit = new DigitalInput(99);
    topLimit = new DigitalInput(99);

    looking = true;

    createShuffleboard();
  }

  private void createShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("CLIMBER");
    tab.add("ClimberSubsystem", this);
    tab.addNumber("Output", this::getClimberOutput);
    tab.addBoolean("LeftAttachment", this::getLeftAttachment);
    tab.addBoolean("RightAttachment", this::getRightAttachment);
    tab.addBoolean("BottomLimit", this::getBottomLimit);
    tab.addBoolean("TopLimit", this::getTopLimit);
  }

  private double getClimberOutput() {
    return climberOut.Output;
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
  }
}
