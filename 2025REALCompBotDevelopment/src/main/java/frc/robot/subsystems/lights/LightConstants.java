// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights;

/** Add your docs here. */
public enum LightConstants {
    DRIVETRAIN_ARDUINO_DEVICE_ADDRESS(4),
    ELEVATOR_ARDUINO_DEVICE_ADDRESS(5),
    INTAKE_ARDUINO_DEVICE_ADDRESS(6),
    ARDUINO_REGISTER_ADDRESS(0),
    OFF_LIGHT_COMMAND(0),
    BLUE_LIGHT_COMMAND(1),
    RED_LIGHT_COMMAND(2),
    MARS_LIGHT_COMMAND(3),
    LEVEL_ZERO_ELEVATOR_LIGHT_COMMAND(1),
    LEVEL_ONE_ELEVATOR_LIGHT_COMMAND(2),
    LEVEL_TWO_ELEVATOR_LIGHT_COMMAND(3),
    LEVEL_THREE_ELEVATOR_LIGHT_COMMAND(4),
    LEFT_ATTACHMENT_ELEVATOR_LIGHT_COMMAND(5),
    RIGHT_ATTACHMENT_ELEVATOR_LIGHT_COMMAND(6),
    VERTICAL_RUNNING_INTAKE_LIGHT_COMMAND(1),
    HORIZONTAL_RUNNING_INTAKE_LIGHT_COMMAND(2),
    GAME_PIECE_INTAKE_LIGHT_COMMAND(3);
    

    private int value = 0;

    public int getValue() {
        return value;
    }

    private LightConstants(int assignedValue) {
        value = assignedValue;
    }
}