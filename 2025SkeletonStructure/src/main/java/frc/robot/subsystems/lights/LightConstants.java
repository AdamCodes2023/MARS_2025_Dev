// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights;

/** Add your docs here. */
public enum LightConstants {
    ARDUINO_DEVICE_ADDRESS(4),
    ARDUINO_REGISTER_ADDRESS(0),
    OFF_LIGHT_COMMAND(0),
    BLUE_LIGHT_COMMAND(1),
    RED_LIGHT_COMMAND(2),
    SHOOT_LIGHT_COMMAND(3),
    MARS_LIGHT_COMMAND(4),
    BLUE_INTAKE_LIGHT_COMMAND(5),
    RED_INTAKE_LIGHT_COMMAND(6);

    private int value = 0;

    public int getValue() {
        return value;
    }

    private LightConstants(int assignedValue) {
        value = assignedValue;
    }
}