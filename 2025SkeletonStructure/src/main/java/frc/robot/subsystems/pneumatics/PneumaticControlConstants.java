// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pneumatics;

/** Add your docs here. */
public enum PneumaticControlConstants {
    REV_PNEUMATIC_HUB_CANID(5),
    ARTICULATION_SOLENOID_FORWARD_CHANNEL(0),
    ARTICULATION_SOLENOID_REVERSE_CHANNEL(1),
    GRIP_SOLENOID_CHANNEL(2);
    

    private int value = 0;

    public int getValue() {
        return value;
    }

    private PneumaticControlConstants(int assignedValue) {
        value = assignedValue;
    }
}