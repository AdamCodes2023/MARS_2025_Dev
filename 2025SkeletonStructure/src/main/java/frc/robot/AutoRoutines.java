
package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.commands.ElevatorToPosTwo;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory, Elevator elevator, Climber climber) {
        m_factory = factory;
        m_factory.bind("EleUp", new ElevatorToPosTwo(elevator));
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
