// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

/** Add your docs here. */
public enum ClimberConstants {
    CLIMBER_MOTOR_CANID(31),
    LEFT_ATTACHMENT_CHANNEL(9),
    RIGHT_ATTACHMENT_CHANNEL(8);
    //BOTTOM_LIMIT_CHANNEL(2),
    //TOP_LIMIT_CHANNEL(3);

    private int value = 0;

    public int getValue() {
        return value;
    }

    private ClimberConstants(int assignedValue) {
        value = assignedValue;
    }
}
