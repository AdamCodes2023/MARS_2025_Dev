// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticControl extends SubsystemBase {
  private final PneumaticHub pneumaticHub;
  private final Compressor compressor;

  // Solenoid corresponds to a single solenoid.
  private final Solenoid articulation;
  private final Solenoid grip;
 
  /** Creates a new Pneumatics. */
  public PneumaticControl() {
    pneumaticHub = new PneumaticHub(5);
    compressor = new Compressor(5, PneumaticsModuleType.REVPH);

    // In this case, it's connected to channel 0 of a PH with the default CAN ID.
    articulation = new Solenoid(5, PneumaticsModuleType.REVPH, 99);
    grip = new Solenoid(5, PneumaticsModuleType.REVPH, 99);

    createShuffleboard();
  }

  private void createShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("PNEUMATICS");
    tab.add("PneumaticsSubsystem", this);
    tab.addNumber("Current(AMPS)", this::getCurrent);
    tab.addBoolean("CompressorEnabled", this::getCompressorEnabled);
    tab.addBoolean("PressureSwitchValue", this::getPressureSwitchValue);
  }

  private double getCurrent() {
    return compressor.getCurrent();
  }

  private boolean getCompressorEnabled() {
    return compressor.isEnabled();
  }

  private boolean getPressureSwitchValue() {
    return !compressor.getPressureSwitchValue();
  }

  public void articulationOn() {
    articulation.set(true);
  }

  public void articulationOff() {
    articulation.set(false);
  }

  public void gripOn() {
    grip.set(true);
  }

  public void gripOff() {
    grip.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
