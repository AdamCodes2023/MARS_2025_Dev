// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.lights.Lights;

public class Intake extends SubsystemBase {
  private final SparkMax verticalIntake;
  private final SparkMaxConfig verticalIntakeConfig;
  private final SparkMax horizontalIntake;
  private final SparkMaxConfig horizontalIntakeConfig;

  private final DigitalInput irProxLeft;
  private final DigitalInput irProxCenter;
  private final DigitalInput irProxRight;

  private double verticalSpeed;
  private double horizontalSpeed;

  private boolean toggleGamePieceLights;
  private boolean toggleVerticalLights;
  private boolean toggleHorizontalLights;

  /** Creates a new Intake. */
  public Intake() {
    verticalIntake = new SparkMax(IntakeConstants.VERTICAL_INTAKE_MOTOR_CANID.getValue(), MotorType.kBrushless);
    horizontalIntake = new SparkMax(IntakeConstants.HORIZONTAL_INTAKE_MOTOR_CANID.getValue(), MotorType.kBrushless);

    verticalIntakeConfig = new SparkMaxConfig();
    verticalIntakeConfig.smartCurrentLimit(20);
    verticalIntakeConfig.secondaryCurrentLimit(25);
    verticalIntakeConfig.idleMode(IdleMode.kCoast);

    horizontalIntakeConfig = new SparkMaxConfig();
    horizontalIntakeConfig.smartCurrentLimit(20);
    horizontalIntakeConfig.secondaryCurrentLimit(25);
    horizontalIntakeConfig.idleMode(IdleMode.kCoast);

    verticalIntake.configure(verticalIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    horizontalIntake.configure(horizontalIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    irProxLeft = new DigitalInput(IntakeConstants.IR_PROX_LEFT_CHANNEL.getValue());
    irProxCenter = new DigitalInput(IntakeConstants.IR_PROX_CENTER_CHANNEL.getValue());
    irProxRight = new DigitalInput(IntakeConstants.IR_PROX_RIGHT_CHANNEL.getValue());

    verticalSpeed = 0.0;
    horizontalSpeed = 0.0;

    toggleGamePieceLights = true;
    toggleVerticalLights = true;
    toggleHorizontalLights = true;

    createShuffleboard();
  }

  private void createShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("INTAKE");
    tab.add("IntakeSubsystem", this);
    tab.addNumber("VerticalIntakeSpeed", this::getVerticalIntakeSpeed);
    tab.addNumber("VerticalIntakeCurrent", this::getVerticalIntakeCurrent);
    tab.addNumber("HorizontalIntakeSpeed", this::getHorizontalIntakeSpeed);
    tab.addNumber("HorizontalIntakeCurrent", this::getHorizontalIntakeCurrent);
    tab.addBoolean("IR_ProxLeft", this::getIRProxLeft);
    tab.addBoolean("IR_ProxCenter", this::getIRProxCenter);
    tab.addBoolean("IR_ProxRight", this::getIRProxRight);
    tab.addBoolean("IntakeHasGamePiece", this::hasGamePiece);
  }

  private double getVerticalIntakeSpeed() {
    return verticalSpeed;
  }

  private double getVerticalIntakeCurrent() {
    return verticalIntake.getOutputCurrent();
  }

  private double getHorizontalIntakeSpeed() {
    return horizontalSpeed;
  }

  private double getHorizontalIntakeCurrent() {
    return horizontalIntake.getOutputCurrent();
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

  public boolean hasGamePiece() {
    return getIRProxLeft() || getIRProxCenter() || getIRProxRight();
  }

  public void runVertical(double speed) {
    verticalSpeed = -speed;
    verticalIntake.set(-speed);
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
    if (hasGamePiece() && toggleGamePieceLights) {
      Lights.turnIntakeHasGamePiece();
      toggleGamePieceLights = false;
    }

    // POSSIBLY UNNECESSARY
    if (!hasGamePiece() && !toggleGamePieceLights) {
      Lights.turnOffIntake();
      toggleGamePieceLights = true;
    }
    //

    if (!hasGamePiece() && getVerticalIntakeSpeed() != 0.0) {
      Lights.turnIntakeVerticalRunning();
      toggleVerticalLights = false;
    }

    if (getVerticalIntakeSpeed() == 0.0 && !toggleVerticalLights) {
      toggleVerticalLights = true;
      if (!hasGamePiece()) {
        Lights.turnOffIntake();
      }
    }

    if (!hasGamePiece() && getHorizontalIntakeSpeed() != 0.0) {
      Lights.turnIntakeHorizontalRunning();
      toggleHorizontalLights = false;
    }

    if (getHorizontalIntakeSpeed() == 0.0 && !toggleHorizontalLights) {
      toggleHorizontalLights = true;
      if (!hasGamePiece()) {
        Lights.turnOffIntake();
      }
    }
  }
}
