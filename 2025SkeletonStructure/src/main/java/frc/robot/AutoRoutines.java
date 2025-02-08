
package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.commands.EjectGamePieceVertical;
import frc.robot.commands.ElevatorToPosTwo;
import frc.robot.commands.IntakeGamePiece;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pneumatics.PneumaticControl;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory, Climber climber, Elevator elevator, Intake intake, PneumaticControl pneumaticControl) {
        m_factory = factory;
        m_factory.bind("EleUp", new ElevatorToPosTwo(elevator));
        m_factory.bind("EjectVert", new EjectGamePieceVertical(intake));
        m_factory.bind("Intake", new IntakeGamePiece(intake));
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("SimplePath Auto");
        final AutoTrajectory simplePath = routine.trajectory("SimplePath");

        routine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        return routine;
    }

    public AutoRoutine scoreCoralFarAuto() {
        final AutoRoutine routine = m_factory.newRoutine("ScoreCoralFar Auto");
        final AutoTrajectory scoreCoralFar = routine.trajectory("ScoreCoralFar");

        routine.active().onTrue(
            scoreCoralFar.resetOdometry()
                .andThen(scoreCoralFar.cmd())
        );
        return routine;
    }
}
