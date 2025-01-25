// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.index;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Index extends SubsystemBase {
  private TalonSRX transitionBelt;

  /** Creates a new Index. */
  public Index() {
    transitionBelt = new TalonSRX(31);
    transitionBelt.configFactoryDefault();
    transitionBelt.setNeutralMode(NeutralMode.Coast);

    createShuffleboard();
  }

  public void createShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("INDEX");
    tab.add("INDEX", this);
  }

  public void stopIndex() {
    transitionBelt.set(TalonSRXControlMode.PercentOutput, 0.0);
  }

  public void runIndex(double speed) {
    transitionBelt.set(TalonSRXControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}