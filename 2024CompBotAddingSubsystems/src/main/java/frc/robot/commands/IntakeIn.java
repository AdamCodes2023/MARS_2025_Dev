// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter_angle.ShooterAngle;

public class IntakeIn extends Command {
  private final Index index;
  private final Intake intake;
  private final ShooterAngle shang;
  private boolean triggered, stage1, stage2, firstTrigger;

  /** Creates a new IntakeIn. */
  public IntakeIn(Index index, Intake intake, ShooterAngle shang) {
    this.index = index;
    this.intake = intake;
    this.shang = shang;
    triggered = false;
    stage1 = true;
    stage2 = false;
    firstTrigger = true;
    addRequirements(index, intake, shang);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (stage1) {
      if (firstTrigger) {
        shang.intakePosition();
        firstTrigger = false;
      }
      if (shang.atIntakeAlignment()) {
        stage1 = false;
        stage2 = true;
      }
    }

    if (stage2) {
      if (!intake.hasGamePiece() && !triggered) {
        index.runIndex(-0.5);
        intake.runIntake(-1.0);
      } else {
        index.stopIndex();
        intake.stopIntake();
        triggered = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    index.stopIndex();
    intake.stopIntake();
    if (intake.hasGamePiece()) {
      shang.stopMotor();
    }
    triggered = false;
    stage1 = true;
    stage2 = false;
    firstTrigger = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}