// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

/** Add your docs here. */
public enum ElevatorConstants {
    ELEVATOR_MOTOR_CANID(31),
    ELEVATOR_CANCODER_CANID(40),
    HARD_STOP_BOTTOM_CHANNEL(4),
    HARD_STOP_TOP_CHANNEL(5);

    private int value = 0;

    public int getValue() {
        return value;
    }

    private ElevatorConstants(int assignedValue) {
        value = assignedValue;
    }
}