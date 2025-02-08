// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final SparkMax verticalIntake;
  private final SparkMax horizontalIntake;

  private final DigitalInput irProxLeft;
  private final DigitalInput irProxCenter;
  private final DigitalInput irProxRight;

  private double verticalSpeed;
  private double horizontalSpeed;

  /** Creates a new Intake. */
  public Intake() {
    verticalIntake = new SparkMax(IntakeConstants.VERTICAL_INTAKE_MOTOR_CANID.getValue(), MotorType.kBrushless);
    horizontalIntake = new SparkMax(IntakeConstants.HORIZONTAL_INTAKE_MOTOR_CANID.getValue(), MotorType.kBrushless);

    irProxLeft = new DigitalInput(IntakeConstants.IR_PROX_LEFT_CHANNEL.getValue());
    irProxCenter = new DigitalInput(IntakeConstants.IR_PROX_CENTER_CHANNEL.getValue());
    irProxRight = new DigitalInput(IntakeConstants.IR_PROX_RIGHT_CHANNEL.getValue());

    verticalSpeed = 0.0;
    horizontalSpeed = 0.0;

    createShuffleboard();
  }

  private void createShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("INTAKE");
    tab.add("IntakeSubsystem", this);
    tab.addNumber("VerticalIntakeSpeed", this::getVerticalIntakeSpeed);
    tab.addNumber("HorizontalIntakeSpeed", this::getHorizontalIntakeSpeed);
    tab.addBoolean("IR_ProxLeft", this::getIRProxLeft);
    tab.addBoolean("IR_ProxCenter", this::getIRProxCenter);
    tab.addBoolean("IR_ProxRight", this::getIRProxRight);
    tab.addBoolean("IntakeHasGamePiece", this::hasGamePiece);
  }

  private double getVerticalIntakeSpeed() {
    return verticalSpeed;
  }

  private double getHorizontalIntakeSpeed() {
    return horizontalSpeed;
  }

  private boolean getIRProxLeft() {
    return irProxLeft.get();
  }

  private boolean getIRProxCenter() {
    return irProxCenter.get();
  }

  private boolean getIRProxRight() {
    return irProxRight.get();
  }

  private boolean hasGamePiece() {
    return getIRProxLeft() || getIRProxCenter() || getIRProxRight();
  }

  public void runVertical(double speed) {
    verticalSpeed = speed;
    verticalIntake.set(speed);
  }

  public void stopVertical() {
    verticalSpeed = 0.0;
    verticalIntake.set(0.0);
  }

  public void runHorizontal(double speed) {
    horizontalSpeed = speed;
    horizontalIntake.set(speed);
  }

  public void stopHorizontal() {
    horizontalSpeed = 0.0;
    horizontalIntake.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
