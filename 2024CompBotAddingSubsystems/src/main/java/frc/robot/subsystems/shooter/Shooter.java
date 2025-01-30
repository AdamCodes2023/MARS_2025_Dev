// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private TalonFX leftShooter, rightShooter;
  private TalonFXConfiguration leftConfiguration, rightConfiguration;
  private double speedCorrector, motorSpeed;

  /* Start at velocity 0, use slot 0 */
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
  /* Keep a neutral out so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  /** Creates a new Shooter. */
  public Shooter() {
    speedCorrector = 1.0;
    motorSpeed = 0.0;

    leftShooter = new TalonFX(42);
    rightShooter = new TalonFX(41);

    /* Factory Default all hardware to prevent unexpected behaviour */
    leftConfiguration = new TalonFXConfiguration();
    rightConfiguration = new TalonFXConfiguration();

    /**
     * Set based on what direction you want forward/positive to be. This does not affect sensor
     * phase.
     */
    leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    leftConfiguration.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    leftConfiguration.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    leftConfiguration.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    leftConfiguration.Slot0.kI = 0; // No output for integrated error
    leftConfiguration.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    leftConfiguration.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    rightConfiguration.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    rightConfiguration.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    rightConfiguration.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    rightConfiguration.Slot0.kI = 0; // No output for integrated error
    rightConfiguration.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    rightConfiguration.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

    leftShooter.getConfigurator().apply(leftConfiguration);
    rightShooter.getConfigurator().apply(rightConfiguration);

    leftShooter.setNeutralMode(NeutralModeValue.Coast);
    rightShooter.setNeutralMode(NeutralModeValue.Coast);

    leftShooter.setSafetyEnabled(false);
    rightShooter.setSafetyEnabled(false);

    createShuffleboard();
  }

  public void createShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("SHOOTER");
    tab.add("SHOOTER", this);
    tab.addNumber("SHOOTER_SPEED", this::getShooterSpeed);
    tab.addNumber("SPEED_CORRECTOR", this::getSpeedCorrector);
  }

  public double getShooterSpeed() {
    return motorSpeed;
  }

  public double getSpeedCorrector() {
    return speedCorrector;
  }

  public void increaseSpeedCorrector() {
    speedCorrector += 0.1;
  }

  public void decreaseSpeedCorrector() {
    speedCorrector -= 0.1;
  }

  public void runShooter(double speed) {
    motorSpeed = speed;
    if (speed == 0) {
      leftShooter.setControl(m_brake);
      rightShooter.setControl(m_brake);
    } else {
      leftShooter.setControl(velocityVoltage.withVelocity(-speed * speedCorrector));
      rightShooter.setControl(velocityVoltage.withVelocity(speed * speedCorrector));
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}