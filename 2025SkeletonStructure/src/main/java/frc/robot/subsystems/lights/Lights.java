// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.hal.I2CJNI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import java.nio.ByteBuffer;
import java.util.Optional;

public class Lights extends SubsystemBase {
  private static I2C arduino = new I2C(Port.kMXP, LightConstants.ARDUINO_DEVICE_ADDRESS.getValue());
  // private byte arduinoAddress = 4;
  // private byte bytesToSend = 1;
  private static boolean success = true;
  private static int currentLightCommand = -1;

  public static boolean notUsed = true;
  public static boolean inAuto = false;

  /** Creates a new Lights. */
  /*
  public Lights() {
    //I2CJNI.i2CInitialize(1);

    arduino = new I2C(Port.kMXP, 4);
    if (SmartDashboard.getBoolean("FMSInfo/IsRedAlliance", true)) {
      turnRed();
    } else {
      turnBlue();
    }

    //createShuffleboard();
  }
  */

  public static void createShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("LIGHTS");
    // tab.add("LIGHTS", this);
    tab.addBoolean("I2C SEND SUCCESS", () -> getSendSuccess());
    tab.addNumber("LIGHT COMMAND", () -> getCurrentLightCommand());
    //SmartDashboard.putNumber("LIGHT COMMAND", -1);
  }

  private static boolean getSendSuccess() {
    return !success;
  }

  private static int getCurrentLightCommand() {
    return currentLightCommand;
  }

  public static void getAllianceLights() {
    /*
    if (SmartDashboard.getBoolean("FMSInfo/IsRedAlliance", false)) {
      turnRed();
    } else {
      turnBlue();
    }
    */

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        turnRed();
        notUsed = true;
      }
      if (ally.get() == Alliance.Blue) {
        turnBlue();
        notUsed = true;
      }
    } else {
      turnOff();
    }
  }

  public static void getAllianceIntakeLights() {
    /*
    if (SmartDashboard.getBoolean("FMSInfo/IsRedAlliance", false)) {
      turnIntakeRed();
    } else {
      turnIntakeBlue();
    }
    */

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        turnIntakeRed();
      }
      if (ally.get() == Alliance.Blue) {
        turnIntakeBlue();
      }
    } else {
      turnOff();
    }
  }

  /*
  public static void getAmplifierLights() {
    String ampMessage = DriverStation.getGameSpecificMessage();
    if (ampMessage.compareToIgnoreCase("AMPLIFIED") == 0) {
      turnMars();
    }
  }
  */

  public static void turnOff() {
    /*
    byte[] dataBytes = {0};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */

    success = arduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(),
                            LightConstants.OFF_LIGHT_COMMAND.getValue()
                            );

    notUsed = true;
    currentLightCommand = LightConstants.OFF_LIGHT_COMMAND.getValue();
    //SmartDashboard.putNumber("LIGHT COMMAND", LightConstants.OFF_LIGHT_COMMAND.getValue());
  }

  public static void turnRed() {
    /*
    byte[] dataBytes = {2};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */

    success = arduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(),
                            LightConstants.RED_LIGHT_COMMAND.getValue()
                            );

    notUsed = false;
    currentLightCommand = LightConstants.RED_LIGHT_COMMAND.getValue();
    //SmartDashboard.putNumber("LIGHT COMMAND", LightConstants.RED_LIGHT_COMMAND.getValue());
  }

  public static void turnBlue() {
    /*
    byte[] dataBytes = {1};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */

    success = arduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(),
                            LightConstants.BLUE_LIGHT_COMMAND.getValue()
                            );

    notUsed = false;
    currentLightCommand = LightConstants.BLUE_LIGHT_COMMAND.getValue();
    //SmartDashboard.putNumber("LIGHT COMMAND", LightConstants.BLUE_LIGHT_COMMAND.getValue());
  }

  public static void turnIntakeRed() {
    /*
    byte[] dataBytes = {6};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */

    success = arduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(),
                            LightConstants.RED_INTAKE_LIGHT_COMMAND.getValue()
                            );

    notUsed = false;
    currentLightCommand = LightConstants.RED_INTAKE_LIGHT_COMMAND.getValue();
    //SmartDashboard.putNumber("LIGHT COMMAND", LightConstants.RED_INTAKE_LIGHT_COMMAND.getValue());
  }

  public static void turnIntakeBlue() {
    /*
    byte[] dataBytes = {5};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */

    success = arduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(),
                            LightConstants.BLUE_INTAKE_LIGHT_COMMAND.getValue()
                            );

    notUsed = false;
    currentLightCommand = LightConstants.BLUE_INTAKE_LIGHT_COMMAND.getValue();
    //SmartDashboard.putNumber("LIGHT COMMAND", LightConstants.BLUE_INTAKE_LIGHT_COMMAND.getValue());
  }

  public static void turnShoot() {
    /*
    byte[] dataBytes = {3};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */

    success = arduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(),
                            LightConstants.SHOOT_LIGHT_COMMAND.getValue()
                            );

    notUsed = false;
    currentLightCommand = LightConstants.SHOOT_LIGHT_COMMAND.getValue();
    //SmartDashboard.putNumber("LIGHT COMMAND", LightConstants.SHOOT_LIGHT_COMMAND.getValue());
  }

  public static void turnMars() {
    /*
    byte[] dataBytes = {4};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */

    success = arduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(),
                            LightConstants.MARS_LIGHT_COMMAND.getValue()
                            );

    notUsed = false;
    currentLightCommand = LightConstants.MARS_LIGHT_COMMAND.getValue();
    //SmartDashboard.putNumber("LIGHT COMMAND", LightConstants.MARS_LIGHT_COMMAND.getValue());
  }

  public static void inAutonomous() {
    //inAuto = DriverStation.isAutonomous();
    inAuto = DriverStation.isAutonomousEnabled();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}