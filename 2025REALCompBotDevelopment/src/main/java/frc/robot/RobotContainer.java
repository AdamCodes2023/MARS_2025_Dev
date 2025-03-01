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
import frc.robot.commands.CloseJaw;
import frc.robot.commands.ElevatorToAlgaeLevelOne;
import frc.robot.commands.ElevatorToAlgaeLevelTwo;
import frc.robot.commands.ElevatorToFeederStation;
import frc.robot.commands.ElevatorToGround;
import frc.robot.commands.ElevatorToScoreBase;
import frc.robot.commands.ElevatorToScoreLevelOne;
import frc.robot.commands.ElevatorToScoreLevelTwo;
import frc.robot.commands.IntakePositionDown;
import frc.robot.commands.IntakePositionUp;
import frc.robot.commands.OpenJaw;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pneumatics.PneumaticControl;


public class RobotContainer {
    private CommandJoystick driveJoystick;
    private CommandJoystick rotateJoystick;
    private CommandJoystick extraJoystick;
    private Trigger[] driveJoystickButtons;
    private Trigger[] rotateJoystickButtons;
    private Trigger[] extraJoystickButtons;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private double turboDriveMultiplier = 1.5;
    private double slowDriveMultiplier = 0.5;

    private double verticalIntakeSpeed = 1.0;
    private double horizontalIntakeSpeed = 1.0;
    private double climberSpeed = 0.5;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final Climber climber = new Climber();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Elevator elevator = new Elevator();
    public final Intake intake = new Intake();
    //LIGHTS
    public final PneumaticControl pneumaticControl = new PneumaticControl();

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    public RobotContainer() {
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory, climber, elevator, intake, pneumaticControl);

        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        autoChooser.addRoutine("SimpleMultiPath", autoRoutines::simpleMultiPathAuto);
        autoChooser.addRoutine("ScoreCoralFar", autoRoutines::scoreCoralFarAuto);
        autoChooser.addRoutine("ScoreCoralMiddle", autoRoutines::scoreCoralMiddleAuto);
        autoChooser.addRoutine("ScoreCoralClose", autoRoutines::scoreCoralCloseAuto);
        autoChooser.addRoutine("ScoreTwoCoralFar", autoRoutines::scoreTwoCoralFarAuto);
        autoChooser.addRoutine("ScoreTwoCoralMiddle", autoRoutines::scoreTwoCoralMiddleAuto);
        autoChooser.addRoutine("ScoreTwoCoralClose", autoRoutines::scoreTwoCoralCloseAuto);
        autoChooser.addRoutine("ScoreThreeCoralFar", autoRoutines::scoreThreeCoralFarAuto);
        autoChooser.addRoutine("ScoreThreeCoralMiddle", autoRoutines::scoreThreeCoralMiddleAuto);
        autoChooser.addRoutine("ScoreThreeCoralClose", autoRoutines::scoreThreeCoralCloseAuto);
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
        configureBindings();
    }

    private void configureBindings() {
        Trigger turboDriveButton = driveJoystickButtons[1];
        Trigger slowDriveButton = driveJoystickButtons[2];
        Trigger db3 = driveJoystickButtons[3];
        //Trigger quasistaticSysIdForwardButton = driveJoystickButtons[4];
        //Trigger quasistaticSysIdReverseButton = driveJoystickButtons[5];
        //Trigger dynamicSysIdForwardButton = driveJoystickButtons[6];
        //Trigger dynamicSysIdReverseButton = driveJoystickButtons[7];
        Trigger climberUpButton = driveJoystickButtons[4];
        Trigger climberDownButton = driveJoystickButtons[5];
        Trigger db6 = driveJoystickButtons[6];
        Trigger db7 = driveJoystickButtons[7];
        Trigger db8 = driveJoystickButtons[8];
        Trigger gyroResetButton = driveJoystickButtons[9];
        Trigger brakeButton = driveJoystickButtons[10];
        Trigger xStanceButton = driveJoystickButtons[11];
        
        Trigger groundPosElevatorButton = rotateJoystickButtons[1];
        Trigger scoreLevelOneElevatorButton = rotateJoystickButtons[2];
        Trigger scoreLevelTwoElevatorButton = rotateJoystickButtons[3];
        Trigger removeAlgaeLevelTwoElevatorButton = rotateJoystickButtons[4];
        Trigger feederStationElevatorButton = rotateJoystickButtons[5];
        //Trigger scoreBaseElevatorButton = rotateJoystickButtons[2];
        //Trigger removeAlgaeLevelOneElevatorButton = rotateJoystickButtons[6];
        Trigger manualElevatorAdjustmentUpButton = rotateJoystickButtons[6];
        Trigger manualElevatorAdjustmentDownButton = rotateJoystickButtons[7];
        Trigger verticalIntakeInButton = rotateJoystickButtons[8];
        Trigger verticalIntakeOutButton = rotateJoystickButtons[9];
        Trigger horizontalIntakeRightButton = rotateJoystickButtons[10];
        Trigger horizontalIntakeLeftButton = rotateJoystickButtons[11];

        Trigger jawPneumaticsButton = extraJoystickButtons[1];
        Trigger intakePositionPneumaticsButton = extraJoystickButtons[2];
        Trigger ex3 = extraJoystickButtons[3];
        Trigger ex4 = extraJoystickButtons[4];
        Trigger ex5 = extraJoystickButtons[5];
        Trigger ex6 = extraJoystickButtons[6];
        Trigger ex7 = extraJoystickButtons[7];
        Trigger ex8 = extraJoystickButtons[8];
        Trigger ex9 = extraJoystickButtons[9];
        Trigger ex10 = extraJoystickButtons[10];
        Trigger ex11 = extraJoystickButtons[11];

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

        // Default Pneumatic Command
        //pneumaticControl.setDefaultCommand(new IntakePositionUp(pneumaticControl));

        turboDriveButton.whileTrue(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveJoystick.getY() * MaxSpeed * turboDriveMultiplier) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveJoystick.getX() * MaxSpeed * turboDriveMultiplier) // Drive left with negative X (left)
                    .withRotationalRate(-rotateJoystick.getX() * MaxAngularRate * turboDriveMultiplier) // Drive counterclockwise with negative X (left)
            )
        );

        slowDriveButton.whileTrue(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveJoystick.getY() * MaxSpeed * slowDriveMultiplier) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveJoystick.getX() * MaxSpeed * slowDriveMultiplier) // Drive left with negative X (left)
                    .withRotationalRate(-rotateJoystick.getX() * MaxAngularRate * slowDriveMultiplier) // Drive counterclockwise with negative X (left)
            )
        );

        brakeButton.whileTrue(drivetrain.applyRequest(() -> brake));
        xStanceButton.whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driveJoystick.getY(), -driveJoystick.getX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //quasistaticSysIdForwardButton.whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //quasistaticSysIdReverseButton.whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        //dynamicSysIdForwardButton.whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //dynamicSysIdReverseButton.whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        

        // reset the field-centric heading on button press
        gyroResetButton.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        groundPosElevatorButton.onTrue(new ElevatorToGround(elevator));
        //scoreBaseElevatorButton.onTrue(new ElevatorToScoreBase(elevator));
        scoreLevelOneElevatorButton.onTrue(new ElevatorToScoreLevelOne(elevator));
        scoreLevelTwoElevatorButton.onTrue(new ElevatorToScoreLevelTwo(elevator));
        feederStationElevatorButton.onTrue(new ElevatorToFeederStation(elevator));
        //removeAlgaeLevelOneElevatorButton.onTrue(new ElevatorToAlgaeLevelOne(elevator));
        removeAlgaeLevelTwoElevatorButton.onTrue(new ElevatorToAlgaeLevelTwo(elevator));
        manualElevatorAdjustmentUpButton.onTrue(Commands.runOnce(() -> elevator.manualElevatorAdjustmentUp()));
        manualElevatorAdjustmentUpButton.onFalse(Commands.runOnce(() -> elevator.stopMotor()));
        manualElevatorAdjustmentDownButton.onTrue(Commands.runOnce(() -> elevator.manualElevatorAdjustmentDown()));
        manualElevatorAdjustmentDownButton.onFalse(Commands.runOnce(() -> elevator.stopMotor()));

        verticalIntakeInButton.onTrue(Commands.runOnce(() -> intake.runVertical(verticalIntakeSpeed)));
        verticalIntakeInButton.onFalse(Commands.runOnce(() -> intake.stopVertical()));
        verticalIntakeOutButton.onTrue(Commands.runOnce(() -> intake.runVertical(-verticalIntakeSpeed)));
        verticalIntakeOutButton.onFalse(Commands.runOnce(() -> intake.stopVertical()));
        horizontalIntakeRightButton.onTrue(Commands.runOnce(() -> intake.runHorizontal(horizontalIntakeSpeed)));
        horizontalIntakeRightButton.onFalse(Commands.runOnce(() -> intake.stopHorizontal()));
        horizontalIntakeLeftButton.onTrue(Commands.runOnce(() -> intake.runHorizontal(-horizontalIntakeSpeed)));
        horizontalIntakeLeftButton.onFalse(Commands.runOnce(() -> intake.stopHorizontal()));

        jawPneumaticsButton.onTrue(new OpenJaw(pneumaticControl));
        jawPneumaticsButton.onFalse(new CloseJaw(pneumaticControl));
        intakePositionPneumaticsButton.onTrue(new IntakePositionDown(pneumaticControl));
        intakePositionPneumaticsButton.onFalse(new IntakePositionUp(pneumaticControl));

        climberUpButton.onTrue(Commands.runOnce(() -> climber.run(climberSpeed)));
        climberUpButton.onFalse(Commands.runOnce(() -> climber.stop()));
        climberDownButton.onTrue(Commands.runOnce(() -> climber.run(-climberSpeed)));
        climberDownButton.onFalse(Commands.runOnce(() -> climber.stop()));
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.selectedCommand();
        
        //return Commands.print("No autonomous command configured");
    }
}
