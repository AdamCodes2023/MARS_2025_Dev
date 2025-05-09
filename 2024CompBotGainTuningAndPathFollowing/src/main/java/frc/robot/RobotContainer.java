// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class RobotContainer {
    private CommandJoystick driveJoystick;
    private CommandJoystick rotateJoystick;
    private Trigger[] driveJoystickButtons;
    private Trigger[] rotateJoystickButtons;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    public RobotContainer() {
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);

        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        driveJoystick = new CommandJoystick(0);
        rotateJoystick = new CommandJoystick(1);

        // buttons use 1-based indexing such that the index matches the button number; leave index 0 set
        // to null
        this.driveJoystickButtons = new Trigger[13];
        this.rotateJoystickButtons = new Trigger[13];

        for (int i = 1; i < driveJoystickButtons.length; i++) {
            driveJoystickButtons[i] = driveJoystick.button(i);
            rotateJoystickButtons[i] = rotateJoystick.button(i);
        }
        configureBindings();
    }

    private void configureBindings() {
        Trigger db1 = driveJoystickButtons[1];
        Trigger db2 = driveJoystickButtons[2];
        Trigger db3 = driveJoystickButtons[3];
        Trigger quasistaticSysIdForwardButton = driveJoystickButtons[4];
        Trigger quasistaticSysIdReverseButton = driveJoystickButtons[5];
        Trigger dynamicSysIdForwardButton = driveJoystickButtons[6];
        Trigger dynamicSysIdReverseButton = driveJoystickButtons[7];
        Trigger db8 = driveJoystickButtons[8];
        Trigger gyroResetButton = driveJoystickButtons[9];
        Trigger brakeButton = driveJoystickButtons[10];
        Trigger xStanceButton = driveJoystickButtons[11];
        
        Trigger rb1 = driveJoystickButtons[1];
        Trigger rb2 = driveJoystickButtons[2];
        Trigger rb3 = driveJoystickButtons[3];
        Trigger rb4 = driveJoystickButtons[4];
        Trigger rb5 = driveJoystickButtons[5];
        Trigger rb6 = driveJoystickButtons[6];
        Trigger rb7 = driveJoystickButtons[7];
        Trigger rb8 = driveJoystickButtons[8];
        Trigger rb9 = driveJoystickButtons[9];
        Trigger rb10 = driveJoystickButtons[10];
        Trigger rb11 = driveJoystickButtons[11];
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveJoystick.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveJoystick.getX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-rotateJoystick.getX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        brakeButton.whileTrue(drivetrain.applyRequest(() -> brake));
        xStanceButton.whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driveJoystick.getY(), -driveJoystick.getX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        quasistaticSysIdForwardButton.whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        quasistaticSysIdReverseButton.whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        dynamicSysIdForwardButton.whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        dynamicSysIdReverseButton.whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        

        // reset the field-centric heading on left bumper press
        gyroResetButton.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.selectedCommand();
        //return Commands.print("No autonomous command configured");
    }
}
