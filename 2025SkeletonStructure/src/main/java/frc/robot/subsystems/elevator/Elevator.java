// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

//import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final NeutralOut stop;
  private final MotionMagicVoltage mmReq;
  private final TalonFX elevatorMotor;
  private final CANcoder elevatorEncoder;
  private final TalonFXConfiguration elevatorConfiguration;
  private final CANcoderConfiguration encoderConfiguration;
  private final DigitalInput hardStopBottom;
  private final DigitalInput hardStopTop; 

  private boolean looking;

  /** Creates a new Elevator. */
  public Elevator() {
    /** Variable Initialization */
    stop = new NeutralOut();
    mmReq = new MotionMagicVoltage(0);

    elevatorMotor = new TalonFX(ElevatorConstants.ELEVATOR_MOTOR_CANID.getValue());
    elevatorEncoder = new CANcoder(ElevatorConstants.ELEVATOR_CANCODER_CANID.getValue());
    elevatorConfiguration = new TalonFXConfiguration();
    encoderConfiguration = new CANcoderConfiguration();

    /** Motor Output Configs */
    elevatorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    elevatorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    /** Feedback Configs */
    FeedbackConfigs feedback = elevatorConfiguration.Feedback;
    feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    feedback.FeedbackRemoteSensorID = elevatorEncoder.getDeviceID();

    /** MotionMagic Configs */
    MotionMagicConfigs motionMagic = elevatorConfiguration.MotionMagic;
    motionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5)) // Reach 5 (mechanism) rotations per second cruise.
               .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max velocity.
               .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100)); // Take approximately 0.1 seconds to reach max acceleration.

    /** PID Configs */
    Slot0Configs slot0 = elevatorConfiguration.Slot0;
    slot0.kS = 0.25; // Add 0.25 V output to overcome static friction.
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output.
    slot0.kA = 0.01; // An acceleration of 1 rps/s requries 0.01 V output.
    slot0.kP = 60.0; // A position error of 0.2 rotations results in 12 V output.
    slot0.kI = 0.0; // No ouptut for integrated error.
    slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output.

    /** Absolute Encoder Configs */
    //encoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

    elevatorMotor.getConfigurator().apply(elevatorConfiguration);
    elevatorEncoder.getConfigurator().apply(encoderConfiguration);

    elevatorMotor.setSafetyEnabled(false);

    hardStopBottom = new DigitalInput(ElevatorConstants.HARD_STOP_BOTTOM_CHANNEL.getValue());
    hardStopTop = new DigitalInput(ElevatorConstants.HARD_STOP_TOP_CHANNEL.getValue());

    looking = true;

    createShuffleboard();
  }

  private void createShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("ELEVATOR");
    tab.add("ElevatorSubsystem", this);
    tab.addNumber("Position", this::getElevatorPosition);
    tab.addNumber("OutputCurrent", this::getElevatorCurrent);
    tab.addBoolean("HardStopBottom", this::getHardStopBottom);
    tab.addBoolean("HardStopTop", this::getHardStopTop);
  }

  private double getElevatorPosition() {
    return elevatorEncoder.getAbsolutePosition().getValueAsDouble();
  }

  private double getElevatorCurrent() {
    return elevatorMotor.getTorqueCurrent().getValueAsDouble();
  }

  private boolean getHardStopBottom() {
    return !hardStopBottom.get();
  }

  private boolean getHardStopTop() {
    return !hardStopTop.get();
  }

  public void stopMotor() {
    elevatorMotor.setControl(stop);
  }

  public void goToPos1() {
    elevatorMotor.setControl(mmReq.withPosition(0.1).withSlot(0));
    //elevatorMotor.setPosition(Rotations.of(1));
  }

  public boolean atPos1() {
    double currentPos = getElevatorPosition();
    return currentPos > 0.08 && currentPos < 0.12;
  }

  public void goToPos2() {
    elevatorMotor.setControl(mmReq.withPosition(0.2).withSlot(0));
  }

  public boolean atPos2() {
    double currentPos = getElevatorPosition();
    return currentPos > 0.18 && currentPos < 0.22;
  }

  public void goToPos3() {
    elevatorMotor.setControl(mmReq.withPosition(0.3).withSlot(0));
  }

  public boolean atPos3() {
    double currentPos = getElevatorPosition();
    return currentPos > 0.28 && currentPos < 0.32;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (getHardStopBottom() || getHardStopTop()) {
      if (looking) {
        stopMotor();
        looking = false;
      }
    } else {
      looking = true;
    }
  }
}