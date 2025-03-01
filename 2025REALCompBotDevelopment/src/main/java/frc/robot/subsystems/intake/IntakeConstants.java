// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

/** Add your docs here. */
public enum IntakeConstants {
    VERTICAL_INTAKE_MOTOR_CANID(33),
    HORIZONTAL_INTAKE_MOTOR_CANID(32),
    IR_PROX_LEFT_CHANNEL(4),
    IR_PROX_CENTER_CHANNEL(5),
    IR_PROX_RIGHT_CHANNEL(6);

    private int value = 0;

    public int getValue() {
        return value;
    }

    private IntakeConstants(int assignedValue) {
        value = assignedValue;
    }
}