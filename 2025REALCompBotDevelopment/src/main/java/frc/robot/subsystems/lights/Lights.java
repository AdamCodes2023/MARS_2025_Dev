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
  private static I2C drivetrainArduino = new I2C(Port.kMXP, LightConstants.DRIVETRAIN_ARDUINO_DEVICE_ADDRESS.getValue());
  private static I2C elevatorArduino = new I2C(Port.kMXP, LightConstants.ELEVATOR_ARDUINO_DEVICE_ADDRESS.getValue());
  private static I2C intakeArduino = new I2C(Port.kMXP, LightConstants.INTAKE_ARDUINO_DEVICE_ADDRESS.getValue());
  
  // private byte arduinoAddress = 4;
  // private byte bytesToSend = 1;

  private static boolean drivetrainSendSuccess = true;
  private static boolean elevatorSendSuccess = true;
  private static boolean intakeSendSuccess = true;

  private static int currentDrivetrainLightCommand = -1;
  private static int currentElevatorLightCommand = -1;
  private static int currentIntakeLightCommand = -1;

  //public static boolean notUsed = true;
  public static boolean inAuto = false;
  public static boolean climbMode = false;

  /** Creates a new Lights. */
  /*
  public Lights() {
    //createShuffleboard();
  }
  */

  public static void createShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("LIGHTS");
    tab.addBoolean("I2C DRIVETRAIN SEND SUCCESS", () -> getDrivetrainSendSuccess());
    tab.addBoolean("I2C ELEVATOR SEND SUCCESS", () -> getElevatorSendSuccess());
    tab.addBoolean("I2C INTAKE SEND SUCCESS", () -> getIntakeSendSuccess());
    tab.addNumber("DRIVETRAIN LIGHT COMMAND", () -> getCurrentDrivetrainLightCommand());
    tab.addNumber("ELEVATOR LIGHT COMMAND", () -> getCurrentElevatorLightCommand());
    tab.addNumber("INTAKE LIGHT COMMAND", () -> getCurrentIntakeLightCommand());

    turnOffDrivetrain();
    turnOffElevator();
    turnOffIntake();
  }

  private static boolean getDrivetrainSendSuccess() {
    return !drivetrainSendSuccess;
  }

  private static boolean getElevatorSendSuccess() {
    return !elevatorSendSuccess;
  }

  private static boolean getIntakeSendSuccess() {
    return !intakeSendSuccess;
  }

  private static int getCurrentDrivetrainLightCommand() {
    return currentDrivetrainLightCommand;
  }

  private static int getCurrentElevatorLightCommand() {
    return currentElevatorLightCommand;
  }

  private static int getCurrentIntakeLightCommand() {
    return currentIntakeLightCommand;
  }

  public static void getAllianceLights() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        turnRed();
        //notUsed = true;
      }
      if (ally.get() == Alliance.Blue) {
        turnBlue();
        //notUsed = true;
      }
    } else {
      turnOffDrivetrain();
    }
  }

  public static void inAutonomous() {
    //inAuto = DriverStation.isAutonomous();
    inAuto = DriverStation.isAutonomousEnabled();
  }

  /*
  public static void getAmplifierLights() {
    String ampMessage = DriverStation.getGameSpecificMessage();
    if (ampMessage.compareToIgnoreCase("AMPLIFIED") == 0) {
      turnMars();
    }
  }
  */

  public static void turnOffDrivetrain() {
    /*
    byte[] dataBytes = {0};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */

    drivetrainSendSuccess = drivetrainArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(),
                            LightConstants.OFF_LIGHT_COMMAND.getValue()
                            );

    //notUsed = true;
    currentDrivetrainLightCommand = LightConstants.OFF_LIGHT_COMMAND.getValue();
  }

  public static void turnRed() {
    /*
    byte[] dataBytes = {2};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */

    drivetrainSendSuccess = drivetrainArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(),
                            LightConstants.RED_LIGHT_COMMAND.getValue()
                            );

    //notUsed = false;
    currentDrivetrainLightCommand = LightConstants.RED_LIGHT_COMMAND.getValue();
  }

  public static void turnBlue() {
    /*
    byte[] dataBytes = {1};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */

    drivetrainSendSuccess = drivetrainArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(),
                            LightConstants.BLUE_LIGHT_COMMAND.getValue()
                            );

    //notUsed = false;
    currentDrivetrainLightCommand = LightConstants.BLUE_LIGHT_COMMAND.getValue();
  }

  public static void turnMars() {
    /*
    byte[] dataBytes = {4};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */

    drivetrainSendSuccess = drivetrainArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(),
                            LightConstants.MARS_LIGHT_COMMAND.getValue()
                            );

    //notUsed = false;
    currentDrivetrainLightCommand = LightConstants.MARS_LIGHT_COMMAND.getValue();
  }

  public static void turnOffElevator() {
    /*
    byte[] dataBytes = {0};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */

    elevatorSendSuccess = elevatorArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(),
                            LightConstants.OFF_LIGHT_COMMAND.getValue()
                            );

    //notUsed = true;
    currentElevatorLightCommand = LightConstants.OFF_LIGHT_COMMAND.getValue();
  }

  public static void turnElevatorLevelZero() {
    /*
    byte[] dataBytes = {0};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */

    elevatorSendSuccess = elevatorArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(),
                            LightConstants.LEVEL_ZERO_ELEVATOR_LIGHT_COMMAND.getValue()
                            );

    //notUsed = true;
    currentElevatorLightCommand = LightConstants.LEVEL_ZERO_ELEVATOR_LIGHT_COMMAND.getValue();
  }

  public static void turnElevatorLevelOne() {
    /*
    byte[] dataBytes = {0};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */

    elevatorSendSuccess = elevatorArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(),
                            LightConstants.LEVEL_ONE_ELEVATOR_LIGHT_COMMAND.getValue()
                            );

    //notUsed = true;
    currentElevatorLightCommand = LightConstants.LEVEL_ONE_ELEVATOR_LIGHT_COMMAND.getValue();
  }

  public static void turnElevatorLevelTwo() {
    /*
    byte[] dataBytes = {0};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */

    elevatorSendSuccess = elevatorArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(),
                            LightConstants.LEVEL_TWO_ELEVATOR_LIGHT_COMMAND.getValue()
                            );

    //notUsed = true;
    currentElevatorLightCommand = LightConstants.LEVEL_TWO_ELEVATOR_LIGHT_COMMAND.getValue();
  }

  public static void turnElevatorLevelThree() {
    /*
    byte[] dataBytes = {0};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */

    elevatorSendSuccess = elevatorArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(),
                            LightConstants.LEVEL_THREE_ELEVATOR_LIGHT_COMMAND.getValue()
                            );

    //notUsed = true;
    currentElevatorLightCommand = LightConstants.LEVEL_THREE_ELEVATOR_LIGHT_COMMAND.getValue();
  }

  public static void turnElevatorLevelFour() {
    /*
    byte[] dataBytes = {0};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */

    elevatorSendSuccess = elevatorArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(),
                            LightConstants.LEVEL_FOUR_ELEVATOR_LIGHT_COMMAND.getValue()
                            );

    //notUsed = true;
    currentElevatorLightCommand = LightConstants.LEVEL_FOUR_ELEVATOR_LIGHT_COMMAND.getValue();
  }

  public static void turnClimberAttachment() {
    /*
    byte[] dataBytes = {0};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */

    elevatorSendSuccess = elevatorArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(),
                            LightConstants.ATTACHMENT_ELEVATOR_LIGHT_COMMAND.getValue()
                            );

    //notUsed = true;
    currentElevatorLightCommand = LightConstants.ATTACHMENT_ELEVATOR_LIGHT_COMMAND.getValue();
  }

  public static void turnOffIntake() {
    /*
    byte[] dataBytes = {0};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */

    intakeSendSuccess = intakeArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(),
                            LightConstants.OFF_LIGHT_COMMAND.getValue()
                            );

    //notUsed = true;
    currentIntakeLightCommand = LightConstants.OFF_LIGHT_COMMAND.getValue();
  }

  public static void turnIntakeVerticalRunning() {
    /*
    byte[] dataBytes = {6};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */

    intakeSendSuccess = intakeArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(),
                            LightConstants.VERTICAL_RUNNING_INTAKE_LIGHT_COMMAND.getValue()
                            );

    //notUsed = false;
    currentIntakeLightCommand = LightConstants.VERTICAL_RUNNING_INTAKE_LIGHT_COMMAND.getValue();
  }

  public static void turnEjectVerticalRunning() {
    /*
    byte[] dataBytes = {6};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */

    intakeSendSuccess = intakeArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(),
                            LightConstants.VERTICAL_RUNNING_EJECT_LIGHT_COMMAND.getValue()
                            );

    //notUsed = false;
    currentIntakeLightCommand = LightConstants.VERTICAL_RUNNING_EJECT_LIGHT_COMMAND.getValue();
  }

  public static void turnIntakeHasGamePiece() {
    /*
    byte[] dataBytes = {3};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */

    intakeSendSuccess = intakeArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(),
                            LightConstants.GAME_PIECE_INTAKE_LIGHT_COMMAND.getValue()
                            );

    //notUsed = false;
    currentIntakeLightCommand = LightConstants.GAME_PIECE_INTAKE_LIGHT_COMMAND.getValue();
  }

  public static void turnIntakeHorizontalRunning() {
    /*
    byte[] dataBytes = {5};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    success = I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
    */

    intakeSendSuccess = intakeArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(),
                            LightConstants.HORIZONTAL_RUNNING_INTAKE_LIGHT_COMMAND.getValue()
                            );

    //notUsed = false;
    currentIntakeLightCommand = LightConstants.HORIZONTAL_RUNNING_INTAKE_LIGHT_COMMAND.getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}