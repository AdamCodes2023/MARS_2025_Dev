package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.EjectGamePieceVertical;
import frc.robot.commands.ElevatorToFeederStation;
import frc.robot.commands.ElevatorToScoreBase;
import frc.robot.commands.IntakeGamePiece;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pneumatics.PneumaticControl;

public class AutoRoutines {
    private final AutoFactory m_factory;
    private final Climber m_climber;
    private final Elevator m_elevator;
    private final Intake m_intake;
    private final PneumaticControl m_pneumaticControl;

    public AutoRoutines(AutoFactory factory, Climber climber, Elevator elevator, Intake intake, PneumaticControl pneumaticControl) {
        m_factory = factory;
        m_climber = climber;
        m_elevator = elevator;
        m_intake = intake;
        m_pneumaticControl = pneumaticControl;

        m_factory.bind("EleUpScoreBase", new ElevatorToScoreBase(m_elevator));
        m_factory.bind("EleUpIntake", new ElevatorToFeederStation(m_elevator));
        //m_factory.bind("EjectVert", new EjectGamePieceVertical(m_intake));
        //m_factory.bind("Intake", new IntakeGamePiece(m_intake));
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

    public AutoRoutine simpleMultiPathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("SimpleMultiPath Auto");
        final AutoTrajectory scoreCoralMiddle = routine.trajectory("ScoreCoralMiddle");
        final AutoTrajectory goToFeederStationFromSideTwo = routine.trajectory("GetToFeederStationFromSideTwo");
        final AutoTrajectory scoreCoralSideThreeFromFeederStation = routine.trajectory("ScoreCoralSideThreeFromFeederStation");
        final AutoTrajectory goToFeederStationFromSideThree = routine.trajectory("GetToFeederStationFromSideThree");

        routine.active().onTrue(
            scoreCoralMiddle.resetOdometry()
                .andThen(scoreCoralMiddle.cmd())
                    .andThen(goToFeederStationFromSideTwo.cmd())
                        .andThen(scoreCoralSideThreeFromFeederStation.cmd())
                            .andThen(goToFeederStationFromSideThree.cmd())
                                .andThen(scoreCoralSideThreeFromFeederStation.cmd())
        );
        return routine;
    }

    public AutoRoutine scoreCoralFarAuto() {
        final AutoRoutine routine = m_factory.newRoutine("ScoreCoralFar Auto");
        final AutoTrajectory scoreCoralFar = routine.trajectory("ScoreCoralFar");

        routine.active().onTrue(
            scoreCoralFar.resetOdometry()
                .andThen(scoreCoralFar.cmd())
                    .andThen(new WaitCommand(0.5))
                        .andThen(new EjectGamePieceVertical(m_intake))
        );
        return routine;
    }

    public AutoRoutine scoreCoralMiddleAuto() {
        final AutoRoutine routine = m_factory.newRoutine("ScoreCoralMiddle Auto");
        final AutoTrajectory scoreCoralMiddle = routine.trajectory("ScoreCoralMiddle");

        routine.active().onTrue(
            scoreCoralMiddle.resetOdometry()
                .andThen(scoreCoralMiddle.cmd())
                    .andThen(new WaitCommand(0.5))
                        .andThen(new EjectGamePieceVertical(m_intake))
        );
        return routine;
    }

    public AutoRoutine scoreCoralCloseAuto() {
        final AutoRoutine routine = m_factory.newRoutine("ScoreCoralClose Auto");
        final AutoTrajectory scoreCoralClose = routine.trajectory("ScoreCoralClose");

        routine.active().onTrue(
            scoreCoralClose.resetOdometry()
                .andThen(scoreCoralClose.cmd())
                    .andThen(new WaitCommand(0.5))
                        .andThen(new EjectGamePieceVertical(m_intake))
        );
        return routine;
    }

    public AutoRoutine scoreCoralSideOneCloseAuto() {
        final AutoRoutine routine = m_factory.newRoutine("ScoreCoralSideOneClose Auto");
        final AutoTrajectory scoreCoralSideOneClose = routine.trajectory("ScoreCoralSideOneClose");

        routine.active().onTrue(
            scoreCoralSideOneClose.resetOdometry()
                .andThen(scoreCoralSideOneClose.cmd())
                    .andThen(new WaitCommand(0.5))
                        .andThen(new EjectGamePieceVertical(m_intake))
        );
        return routine;
    }

    public AutoRoutine scoreTwoCoralFarAuto() {
        final AutoRoutine routine = m_factory.newRoutine("ScoreTwoCoralFar Auto");
        final AutoTrajectory scoreCoralFar = routine.trajectory("ScoreCoralFar");
        final AutoTrajectory goToFeederStationFromSideTwo = routine.trajectory("GetToFeederStationFromSideTwo");
        final AutoTrajectory scoreCoralSideThreeFromFeederStation = routine.trajectory("ScoreCoralSideThreeFromFeederStation");

        routine.active().onTrue(
            scoreCoralFar.resetOdometry()
                .andThen(scoreCoralFar.cmd())
                    .andThen(new WaitCommand(0.5))
                        .andThen(new EjectGamePieceVertical(m_intake))
                            .andThen(new WaitCommand(0.2))
                                .andThen(goToFeederStationFromSideTwo.cmd())
                                    .andThen(new WaitCommand(0.5))
                                        .andThen(new IntakeGamePiece(m_intake))
                                            .andThen(new WaitCommand(0.2))
                                                .andThen(scoreCoralSideThreeFromFeederStation.cmd())
                                                    .andThen(new WaitCommand(0.5))
                                                        .andThen(new EjectGamePieceVertical(m_intake))
        );
        return routine;
    }

    public AutoRoutine scoreTwoCoralMiddleAuto() {
        final AutoRoutine routine = m_factory.newRoutine("ScoreTwoCoralMiddle Auto");
        final AutoTrajectory scoreCoralMiddle = routine.trajectory("ScoreCoralMiddle");
        final AutoTrajectory goToFeederStationFromSideTwo = routine.trajectory("GetToFeederStationFromSideTwo");
        final AutoTrajectory scoreCoralSideThreeFromFeederStation = routine.trajectory("ScoreCoralSideThreeFromFeederStation");

        routine.active().onTrue(
            scoreCoralMiddle.resetOdometry()
                .andThen(scoreCoralMiddle.cmd())
                    .andThen(new WaitCommand(0.5))
                        .andThen(new EjectGamePieceVertical(m_intake))
                            .andThen(new WaitCommand(0.2))
                                .andThen(goToFeederStationFromSideTwo.cmd())
                                    .andThen(new WaitCommand(0.5))
                                        .andThen(new IntakeGamePiece(m_intake))
                                            .andThen(new WaitCommand(0.2))
                                                .andThen(scoreCoralSideThreeFromFeederStation.cmd())
                                                    .andThen(new WaitCommand(0.5))
                                                        .andThen(new EjectGamePieceVertical(m_intake))
        );
        return routine;
    }

    public AutoRoutine scoreTwoCoralCloseAuto() {
        final AutoRoutine routine = m_factory.newRoutine("ScoreTwoCoralClose Auto");
        final AutoTrajectory scoreCoralClose = routine.trajectory("ScoreCoralClose");
        final AutoTrajectory goToFeederStationFromSideTwo = routine.trajectory("GetToFeederStationFromSideTwo");
        final AutoTrajectory scoreCoralSideThreeFromFeederStation = routine.trajectory("ScoreCoralSideThreeFromFeederStation");

        routine.active().onTrue(
            scoreCoralClose.resetOdometry()
                .andThen(scoreCoralClose.cmd())
                    .andThen(new WaitCommand(0.5))
                        .andThen(new EjectGamePieceVertical(m_intake))
                            .andThen(new WaitCommand(0.2))
                                .andThen(goToFeederStationFromSideTwo.cmd())
                                    .andThen(new WaitCommand(0.5))
                                        .andThen(new IntakeGamePiece(m_intake))
                                            .andThen(new WaitCommand(0.2))
                                                .andThen(scoreCoralSideThreeFromFeederStation.cmd())
                                                    .andThen(new WaitCommand(0.5))
                                                        .andThen(new EjectGamePieceVertical(m_intake))
        );
        return routine;
    }

    public AutoRoutine scoreThreeCoralFarAuto() {
        final AutoRoutine routine = m_factory.newRoutine("ScoreThreeCoralFar Auto");
        final AutoTrajectory scoreCoralFar = routine.trajectory("ScoreCoralFar");
        final AutoTrajectory goToFeederStationFromSideTwo = routine.trajectory("GetToFeederStationFromSideTwo");
        final AutoTrajectory scoreCoralSideThreeFromFeederStation = routine.trajectory("ScoreCoralSideThreeFromFeederStation");
        final AutoTrajectory goToFeederStationFromSideThree = routine.trajectory("GetToFeederStationFromSideThree");

        routine.active().onTrue(
            scoreCoralFar.resetOdometry()
                .andThen(scoreCoralFar.cmd())
                    .andThen(new WaitCommand(0.5))
                        .andThen(new EjectGamePieceVertical(m_intake))
                            .andThen(new WaitCommand(0.2))
                                .andThen(goToFeederStationFromSideTwo.cmd())
                                    .andThen(new WaitCommand(0.5))
                                        .andThen(new IntakeGamePiece(m_intake))
                                            .andThen(new WaitCommand(0.2))
                                                .andThen(scoreCoralSideThreeFromFeederStation.cmd())
                                                    .andThen(new WaitCommand(0.5))
                                                        .andThen(new EjectGamePieceVertical(m_intake))
                                                            .andThen(new WaitCommand(0.2))
                                                                .andThen(goToFeederStationFromSideThree.cmd())
                                                                    .andThen(new WaitCommand(0.5))
                                                                        .andThen(new IntakeGamePiece(m_intake))
                                                                            .andThen(new WaitCommand(0.2))
                                                                                .andThen(scoreCoralSideThreeFromFeederStation.cmd())
                                                                                    .andThen(new WaitCommand(0.5))
                                                                                        .andThen(new EjectGamePieceVertical(m_intake))
        );
        return routine;
    }

    public AutoRoutine scoreThreeCoralMiddleAuto() {
        final AutoRoutine routine = m_factory.newRoutine("ScoreThreeCoralMiddle Auto");
        final AutoTrajectory scoreCoralMiddle = routine.trajectory("ScoreCoralMiddle");
        final AutoTrajectory goToFeederStationFromSideTwo = routine.trajectory("GetToFeederStationFromSideTwo");
        final AutoTrajectory scoreCoralSideThreeFromFeederStation = routine.trajectory("ScoreCoralSideThreeFromFeederStation");
        final AutoTrajectory goToFeederStationFromSideThree = routine.trajectory("GetToFeederStationFromSideThree");

        routine.active().onTrue(
            scoreCoralMiddle.resetOdometry()
                .andThen(scoreCoralMiddle.cmd())
                    .andThen(new WaitCommand(0.5))
                        .andThen(new EjectGamePieceVertical(m_intake))
                            .andThen(new WaitCommand(0.2))
                                .andThen(goToFeederStationFromSideTwo.cmd())
                                    .andThen(new WaitCommand(0.5))
                                        .andThen(new IntakeGamePiece(m_intake))
                                            .andThen(new WaitCommand(0.2))
                                                .andThen(scoreCoralSideThreeFromFeederStation.cmd())
                                                    .andThen(new WaitCommand(0.5))
                                                        .andThen(new EjectGamePieceVertical(m_intake))
                                                            .andThen(new WaitCommand(0.2))
                                                                .andThen(goToFeederStationFromSideThree.cmd())
                                                                    .andThen(new WaitCommand(0.5))
                                                                        .andThen(new IntakeGamePiece(m_intake))
                                                                            .andThen(new WaitCommand(0.2))
                                                                                .andThen(scoreCoralSideThreeFromFeederStation.cmd())
                                                                                    .andThen(new WaitCommand(0.5))
                                                                                        .andThen(new EjectGamePieceVertical(m_intake))
        );
        return routine;
    }

    public AutoRoutine scoreThreeCoralCloseAuto() {
        final AutoRoutine routine = m_factory.newRoutine("ScoreThreeCoralClose Auto");
        final AutoTrajectory scoreCoralClose = routine.trajectory("ScoreCoralClose");
        final AutoTrajectory goToFeederStationFromSideTwo = routine.trajectory("GetToFeederStationFromSideTwo");
        final AutoTrajectory scoreCoralSideThreeFromFeederStation = routine.trajectory("ScoreCoralSideThreeFromFeederStation");
        final AutoTrajectory goToFeederStationFromSideThree = routine.trajectory("GetToFeederStationFromSideThree");

        routine.active().onTrue(
            scoreCoralClose.resetOdometry()
                .andThen(scoreCoralClose.cmd())
                    .andThen(new WaitCommand(0.5))
                        .andThen(new EjectGamePieceVertical(m_intake))
                            .andThen(new WaitCommand(0.2))
                                .andThen(goToFeederStationFromSideTwo.cmd())
                                    .andThen(new WaitCommand(0.5))
                                        .andThen(new IntakeGamePiece(m_intake))
                                            .andThen(new WaitCommand(0.2))
                                                .andThen(scoreCoralSideThreeFromFeederStation.cmd())
                                                    .andThen(new WaitCommand(0.5))
                                                        .andThen(new EjectGamePieceVertical(m_intake))
                                                            .andThen(new WaitCommand(0.2))
                                                                .andThen(goToFeederStationFromSideThree.cmd())
                                                                    .andThen(new WaitCommand(0.5))
                                                                        .andThen(new IntakeGamePiece(m_intake))
                                                                            .andThen(new WaitCommand(0.2))
                                                                                .andThen(scoreCoralSideThreeFromFeederStation.cmd())
                                                                                    .andThen(new WaitCommand(0.5))
                                                                                        .andThen(new EjectGamePieceVertical(m_intake))
        );
        return routine;
    }
}
