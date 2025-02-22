// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticControl extends SubsystemBase {
  private final PneumaticHub pneumaticHub;
  private final Compressor compressor;

  // Solenoid corresponds to a single solenoid.
  //private final Solenoid articulation;
  private final DoubleSolenoid articulation;
  private final Solenoid grip;

  private boolean articulationForwardState;
  private boolean articulationReverseState;
  private boolean articulationOffState;
  private boolean gripState;
 
  /** Creates a new Pneumatics. */
  public PneumaticControl() {
    pneumaticHub = new PneumaticHub(PneumaticControlConstants.REV_PNEUMATIC_HUB_CANID.getValue());
    
    compressor = new Compressor(PneumaticControlConstants.REV_PNEUMATIC_HUB_CANID.getValue(),
                                PneumaticsModuleType.REVPH
                                );

    // In this case, it's connected to channel 0 of a PH with the default CAN ID.
    /*
    articulation = new Solenoid(PneumaticControlConstants.REV_PNEUMATIC_HUB_CANID.getValue(),
                                PneumaticsModuleType.REVPH,
                                PneumaticControlConstants.ARTICULATION_SOLENOID_CHANNEL.getValue()
                                );
    */
    articulation = new DoubleSolenoid(PneumaticControlConstants.REV_PNEUMATIC_HUB_CANID.getValue(),
                                      PneumaticsModuleType.REVPH,
                                      PneumaticControlConstants.ARTICULATION_SOLENOID_FORWARD_CHANNEL.getValue(),
                                      PneumaticControlConstants.ARTICULATION_SOLENOID_REVERSE_CHANNEL.getValue()
                                      );

    grip = new Solenoid(PneumaticControlConstants.REV_PNEUMATIC_HUB_CANID.getValue(),
                        PneumaticsModuleType.REVPH,
                        PneumaticControlConstants.GRIP_SOLENOID_CHANNEL.getValue()
                        );

    articulationForwardState = false;
    articulationReverseState = false;
    articulationOffState = true;

    gripState = false;

    createShuffleboard();
  }

  private void createShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("PNEUMATICS");
    tab.add("PneumaticsSubsystem", this);
    tab.addNumber("Current", this::getCurrent);
    tab.addBoolean("CompressorEnabled", this::getCompressorEnabled);
    tab.addBoolean("PressureSwitchValue", this::getPressureSwitchValue);
    tab.addBoolean("ArticulationFORWARD", this::getArticulationForward);
    tab.addBoolean("ArticulationREVERSE", this::getArticulationReverse);
    tab.addBoolean("ArticulationOFF", this::getArticulationOff);
    tab.addBoolean("GripON", this::getGripOn);
    tab.addBoolean("GripOFF", this::getGripOff);
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

  public void articulationForward() {
    articulation.set(Value.kForward);
    articulationForwardState = true;
    articulationReverseState = false;
    articulationOffState = false;
  }

  public void articulationReverse() {
    articulation.set(Value.kReverse);
    articulationForwardState = false;
    articulationReverseState = true;
    articulationOffState = false;
  }

  public void articulationOff() {
    articulation.set(Value.kOff);
    articulationForwardState = false;
    articulationReverseState = false;
    articulationOffState = true;
  }

  public void gripOn() {
    grip.set(true);
    gripState = true;
  }

  public void gripOff() {
    grip.set(false);
    gripState = false;
  }

  private boolean getArticulationForward() {
    return articulationForwardState;
  }

  private boolean getArticulationReverse() {
    return articulationReverseState;
  }

  private boolean getArticulationOff() {
    return articulationOffState;
  }

  private boolean getGripOn() {
    return gripState;
  }

  private boolean getGripOff() {
    return !gripState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
