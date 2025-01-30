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
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.ShootAmp;
import frc.robot.commands.ShootSpeaker;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.lift.Lift;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter_angle.ShooterAngle;


public class RobotContainer {
    private CommandJoystick driveJoystick;
    private CommandJoystick rotateJoystick;
    private CommandJoystick extraJoystick;
    private Trigger[] driveJoystickButtons;
    private Trigger[] rotateJoystickButtons;
    private Trigger[] extraJoystickButtons;

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

    private final Index index;
    private final Intake intake;
    private final Lift lift;
    private final Shooter shooter;
    private final ShooterAngle shooterAngle;

    public RobotContainer() {
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);

        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        driveJoystick = new CommandJoystick(0);
        rotateJoystick = new CommandJoystick(1);
        extraJoystick = new CommandJoystick(2);

        // buttons use 1-based indexing such that the index matches the button number; leave index 0 set
        // to null
        driveJoystickButtons = new Trigger[13];
        rotateJoystickButtons = new Trigger[13];
        extraJoystickButtons = new Trigger[13];

        for (int i = 1; i < driveJoystickButtons.length; i++) {
            driveJoystickButtons[i] = driveJoystick.button(i);
            rotateJoystickButtons[i] = rotateJoystick.button(i);
            extraJoystickButtons[i] = extraJoystick.button(i);
        }

        rotateJoystickButtons[12] = rotateJoystick.button(12);

        index = new Index();
        intake = new Intake();
        lift = new Lift();
        shooter = new Shooter();
        shooterAngle = new ShooterAngle();

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
        
        Trigger shootAmpButton = rotateJoystickButtons[1];
        Trigger shootSpeakerButton = rotateJoystickButtons[2];
        Trigger shootSpeakerMidButton = rotateJoystickButtons[3];
        Trigger shootSpeakerFarButton = rotateJoystickButtons[4];
        Trigger intakeButton = rotateJoystickButtons[5];
        Trigger outakeButton = rotateJoystickButtons[6];
        Trigger positiveAdjustAngleButton = rotateJoystickButtons[7];
        Trigger positiveAdjustSpeedButton = rotateJoystickButtons[8];
        Trigger negativeAdjustSpeedButton = rotateJoystickButtons[9];
        Trigger shootReverseButton = rotateJoystickButtons[10];
        Trigger liftUpButton = rotateJoystickButtons[11];
        Trigger liftDownButton = rotateJoystickButtons[12];

        Trigger shootForwardButton = extraJoystickButtons[1];
        Trigger negativeAdjustAngleButton = extraJoystickButtons[2];
        Trigger ex3 = extraJoystickButtons[3];
        Trigger ex4 = extraJoystickButtons[4];
        Trigger ex5 = extraJoystickButtons[5];
        Trigger ex6 = extraJoystickButtons[6];
        Trigger ex7 = extraJoystickButtons[7];
        Trigger ex8 = extraJoystickButtons[8];
        Trigger ex9 = extraJoystickButtons[9];
        Trigger ex10 = extraJoystickButtons[10];
        Trigger ex11 = extraJoystickButtons[11];
        //Trigger ex12 = extraJoystickButtons[12];
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

        RepeatCommand repeatShootLights = new RepeatCommand(Commands.runOnce(Lights::turnShoot));

        Command intakeInCommand = new IntakeIn(index, intake, shooterAngle);
        Command intakeOutCommand = new IntakeOut(index, intake, shooterAngle);
        RepeatCommand repeatIn = new RepeatCommand(Commands.runOnce(() -> intakeInCommand.execute()));
        RepeatCommand repeatOut = new RepeatCommand(Commands.runOnce(() -> intakeOutCommand.execute()));
        // INTAKE TESTING
        intakeButton.onTrue(repeatIn);
        intakeButton.onFalse(Commands.runOnce(() -> repeatIn.cancel()));
        intakeButton.onFalse(Commands.runOnce(() -> intakeInCommand.end(true)));

        outakeButton.onTrue(repeatOut);
        outakeButton.onFalse(Commands.runOnce(() -> repeatOut.cancel()));
        outakeButton.onFalse(Commands.runOnce(() -> intakeOutCommand.end(true)));

        // SHOOTER TESTING
        Command shootAmpCommand = new ShootAmp(index, shooterAngle, shooter);
        RepeatCommand repeatShootAmp =
            new RepeatCommand(Commands.runOnce(() -> shootAmpCommand.execute()));
        shootAmpButton.onTrue(repeatShootAmp);
        shootAmpButton.onFalse(Commands.runOnce(() -> repeatShootAmp.cancel()));
        shootAmpButton.onFalse(Commands.runOnce(() -> shootAmpCommand.end(true)));
        shootAmpButton
            .onFalse(
                new FunctionalCommand(
                    shooterAngle::stopMotor,
                    shooterAngle::intakePosition,
                    shooterAngle::stopMotorWithInterrupt,
                    shooterAngle::atIntakeAlignment));
        shootAmpButton.onTrue(repeatShootLights);
        shootAmpButton.onFalse(Commands.runOnce(() -> repeatShootLights.cancel()));
        shootAmpButton.onFalse(Commands.runOnce(Lights::turnOff));

        Command shootSpeakerCommand = new ShootSpeaker(index, shooterAngle, shooter, 179.6, 15000);
        RepeatCommand repeatShootSpeaker =
            new RepeatCommand(Commands.runOnce(() -> shootSpeakerCommand.execute()));
        shootSpeakerButton.onTrue(repeatShootSpeaker);
        shootSpeakerButton.onFalse(Commands.runOnce(() -> repeatShootSpeaker.cancel()));
        shootSpeakerButton.onFalse(Commands.runOnce(() -> shootSpeakerCommand.end(true)));
        shootSpeakerButton
            .onFalse(
                new FunctionalCommand(
                    shooterAngle::stopMotor,
                    shooterAngle::intakePosition,
                    shooterAngle::stopMotorWithInterrupt,
                    shooterAngle::atIntakeAlignment));
        shootSpeakerButton.onTrue(repeatShootLights);
        shootSpeakerButton.onFalse(Commands.runOnce(() -> repeatShootLights.cancel()));
        shootSpeakerButton.onFalse(Commands.runOnce(Lights::turnOff));

        Command shootSpeakerMidCommand = new ShootSpeaker(index, shooterAngle, shooter, 198.1, 30000);
        RepeatCommand repeatShootSpeakerMid =
            new RepeatCommand(Commands.runOnce(() -> shootSpeakerMidCommand.execute()));
        shootSpeakerMidButton.onTrue(repeatShootSpeakerMid);
        shootSpeakerMidButton.onFalse(Commands.runOnce(() -> repeatShootSpeakerMid.cancel()));
        shootSpeakerMidButton.onFalse(Commands.runOnce(() -> shootSpeakerMidCommand.end(true)));
        shootSpeakerMidButton
            .onFalse(
                new FunctionalCommand(
                    shooterAngle::stopMotor,
                    shooterAngle::intakePosition,
                    shooterAngle::stopMotorWithInterrupt,
                    shooterAngle::atIntakeAlignment));
        shootSpeakerMidButton.onTrue(repeatShootLights);
        shootSpeakerMidButton.onFalse(Commands.runOnce(() -> repeatShootLights.cancel()));
        shootSpeakerMidButton.onFalse(Commands.runOnce(Lights::turnOff));

        Command shootSpeakerFarCommand = new ShootSpeaker(index, shooterAngle, shooter, 219.1, 30000);
        RepeatCommand repeatShootSpeakerFar =
            new RepeatCommand(Commands.runOnce(() -> shootSpeakerFarCommand.execute()));
        shootSpeakerFarButton.onTrue(repeatShootSpeakerFar);
        shootSpeakerFarButton.onFalse(Commands.runOnce(() -> repeatShootSpeakerFar.cancel()));
        shootSpeakerFarButton.onFalse(Commands.runOnce(() -> shootSpeakerFarCommand.end(true)));
        shootSpeakerFarButton
            .onFalse(
                new FunctionalCommand(
                    shooterAngle::stopMotor,
                    shooterAngle::intakePosition,
                    shooterAngle::stopMotorWithInterrupt,
                    shooterAngle::atIntakeAlignment));
        shootSpeakerFarButton.onTrue(repeatShootLights);
        shootSpeakerFarButton.onFalse(Commands.runOnce(() -> repeatShootLights.cancel()));
        shootSpeakerFarButton.onFalse(Commands.runOnce(Lights::turnOff));
    

        shootForwardButton.onTrue(Commands.runOnce(() -> shooter.runShooter(10000.0)));
        shootForwardButton.onFalse(Commands.runOnce(() -> shooter.runShooter(0.0)));
        shootForwardButton.onTrue(Commands.runOnce(() -> index.runIndex(-0.7)));
        shootForwardButton.onFalse(Commands.runOnce(() -> index.stopIndex()));
        shootForwardButton.onTrue(repeatShootLights);
        shootForwardButton.onFalse(Commands.runOnce(() -> repeatShootLights.cancel()));
        shootForwardButton.onFalse(Commands.runOnce(Lights::turnOff));

        shootReverseButton.onTrue(Commands.runOnce(() -> shooter.runShooter(-10000.0)));
        shootReverseButton.onFalse(Commands.runOnce(() -> shooter.runShooter(0.0)));
        shootReverseButton.onTrue(Commands.runOnce(() -> index.runIndex(0.7)));
        shootReverseButton.onFalse(Commands.runOnce(() -> index.stopIndex()));
        shootReverseButton.onTrue(repeatShootLights);
        shootReverseButton.onFalse(Commands.runOnce(() -> repeatShootLights.cancel()));
        shootReverseButton.onFalse(Commands.runOnce(Lights::turnOff));


        //ADJUSTMENT SWITCHES
        positiveAdjustSpeedButton.onTrue(Commands.runOnce(() -> shooter.increaseSpeedCorrector()));

        negativeAdjustSpeedButton.onTrue(Commands.runOnce(() -> shooter.decreaseSpeedCorrector()));

        positiveAdjustAngleButton.onTrue(Commands.runOnce(() -> shooterAngle.increaseAngleCorrector()));

        negativeAdjustAngleButton.onTrue(Commands.runOnce(() -> shooterAngle.decreaseAngleCorrector()));

        // LIFT TESTING
        liftDownButton.onTrue(Commands.runOnce(() -> lift.runClimbers(1.0)));
        liftDownButton.onFalse(Commands.runOnce(() -> lift.runClimbers(0.0)));

        liftUpButton.onTrue(Commands.runOnce(() -> lift.runClimbers(-1.0)));
        liftUpButton.onFalse(Commands.runOnce(() -> lift.runClimbers(0.0)));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.selectedCommand();
        //return Commands.print("No autonomous command configured");
    }
}
