
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import java.nio.ByteBuffer;
import java.util.Optional;

public class Lights extends SubsystemBase {
  private static I2C arduino = new I2C(Port.kMXP, 4);
  // private byte arduinoAddress = 4;
  // private byte bytesToSend = 1;
  private static boolean success = true;
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
    SmartDashboard.putNumber("LIGHT COMMAND", -1);
  }

  public static boolean getSendSuccess() {
    return !success;
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
    success = arduino.write(0, 0);
    notUsed = true;
    SmartDashboard.putNumber("LIGHT COMMAND", 0);
  }

  public static void turnRed() {
    /*
    byte[] dataBytes = {2};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */
    success = arduino.write(0, 2);
    notUsed = false;
    SmartDashboard.putNumber("LIGHT COMMAND", 2);
  }

  public static void turnBlue() {
    /*
    byte[] dataBytes = {1};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */
    success = arduino.write(0, 1);
    notUsed = false;
    SmartDashboard.putNumber("LIGHT COMMAND", 1);
  }

  public static void turnIntakeRed() {
    /*
    byte[] dataBytes = {6};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */
    success = arduino.write(0, 6);
    notUsed = false;
    SmartDashboard.putNumber("LIGHT COMMAND", 6);
  }

  public static void turnIntakeBlue() {
    /*
    byte[] dataBytes = {5};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */
    success = arduino.write(0, 5);
    notUsed = false;
    SmartDashboard.putNumber("LIGHT COMMAND", 5);
  }

  public static void turnShoot() {
    /*
    byte[] dataBytes = {3};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */
    success = arduino.write(0, 3);
    notUsed = false;
    SmartDashboard.putNumber("LIGHT COMMAND", 3);
  }

  public static void turnMars() {
    /*
    byte[] dataBytes = {4};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */
    success = arduino.write(0, 4);
    notUsed = false;
    SmartDashboard.putNumber("LIGHT COMMAND", 4);
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
