package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Field.Branch;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Ejector;
import frc.robot.subsystems.Elevator;

public class Auto {
    
    private final CommandSwerveDrivetrain drivetrain;
    private final Ejector ejector;
    private final Elevator elevator;
    private final BooleanSupplier isRedAlliance;
    private final Pose2d kBlueCenter = new Pose2d(new Translation2d(7.2, 4), new Rotation2d(Math.PI));
    private final Pose2d kBlueLeft = new Pose2d(new Translation2d(7.2, 5.5), new Rotation2d(Math.PI));
    private final Pose2d kBlueRight = new Pose2d(new Translation2d(7.2, 2.5), new Rotation2d(Math.PI));
    private final Pose2d kRedCenter = new Pose2d(new Translation2d(10.2, 4), new Rotation2d());
    private final Pose2d kRedLeft = new Pose2d(new Translation2d(10.2, 2.5), new Rotation2d());
    private final Pose2d kRedRight = new Pose2d(new Translation2d(10.2, 5.5), new Rotation2d());

    public Auto(CommandSwerveDrivetrain drivetrain, Ejector ejector, Elevator elevator, BooleanSupplier isRedAlliance) {
        this.drivetrain = drivetrain;
        this.ejector = ejector;
        this.elevator = elevator;
        this.isRedAlliance = isRedAlliance;
    }

    public Command centerOnePiece(){
        return Commands.sequence(
            new WaitCommand(0.5),
            new InstantCommand(() -> drivetrain.resetPose(isRedAlliance.getAsBoolean() ? kRedCenter : kBlueCenter)),
            drivetrain.fieldCentricFacingAngle(
                () -> -0.2,
                () -> 0,
                () -> new Rotation2d(Degrees.of(180).in(Radians))).withTimeout(1),
            drivetrain.stop().withTimeout(0.75),
            drivetrain.goToPose(() -> Constants.Field.getWaypoint(isRedAlliance.getAsBoolean() ? 10 : 21, Branch.kLeft), isRedAlliance),
            Commands.deadline(
                scoreCoral(() -> Constants.ElevatorConstants.kL4), 
                drivetrain.brake()
            )
        );
    }

    public Command leftOnePiece(){
        return Commands.sequence(
            new WaitCommand(0.5),
            new InstantCommand(() -> drivetrain.resetPose(isRedAlliance.getAsBoolean() ? kRedLeft : kBlueLeft)),
            drivetrain.fieldCentricFacingAngle(
                () -> -0.2,
                () -> 0,
                () -> new Rotation2d(Degrees.of(180).in(Radians))).withTimeout(2.25),
                drivetrain.fieldCentricFacingAngle(
                    () -> 0,
                    () -> 0,
                    () -> new Rotation2d(Degrees.of(240).in(Radians))).withTimeout(1),
            drivetrain.goToPose(() -> Constants.Field.getWaypoint(isRedAlliance.getAsBoolean() ? 11 : 20, Branch.kLeft), isRedAlliance),
            Commands.deadline(
                scoreCoral(() -> Constants.ElevatorConstants.kL4), 
                drivetrain.brake()
            )
        );
    }

    public Command rightOnePiece(){
        return Commands.sequence(
            new WaitCommand(0.5),
            new InstantCommand(() -> drivetrain.resetPose(isRedAlliance.getAsBoolean() ? kRedRight : kBlueRight)),
            drivetrain.fieldCentricFacingAngle(
                () -> -0.2,
                () -> 0,
                () -> new Rotation2d(Degrees.of(180).in(Radians))).withTimeout(2.25),
                drivetrain.fieldCentricFacingAngle(
                    () -> 0,
                    () -> 0,
                    () -> new Rotation2d(Degrees.of(120).in(Radians))).withTimeout(1),
            drivetrain.goToPose(() -> Constants.Field.getWaypoint(isRedAlliance.getAsBoolean() ? 9 :22, Branch.kLeft), isRedAlliance),
            Commands.deadline(
                scoreCoral(() -> Constants.ElevatorConstants.kL4), 
                drivetrain.brake()
            )
        );
    }

    /**
     * Creates a command sequence to load coral.
     * @return Runnable command.
     */
    public Command loadCoral() {
        return Commands
            .parallel(
                elevator.goToSetpoint(() -> Constants.ElevatorConstants.kLoad),
                ejector.intake())
            .until(() -> ejector.hasCoral());
    }

    public Command reverseCoralAtLoadHeight() {
        return Commands
            .parallel(
                elevator.goToSetpoint(() -> Constants.ElevatorConstants.kLoad),
                ejector.reverse()
            );
    }

    /**
     * Creates a command sequence to score coral.
     * @return Runnable command.
     */
    public Command scoreCoral(DoubleSupplier setPoint) {
        return Commands.sequence(
            elevator.goToSetpoint(setPoint).until(() -> elevator.atSetpoint()),
            Commands.parallel(
                elevator.goToSetpoint(setPoint),
                ejector.eject()
            ).withTimeout(1)
        ).withName("Score Coral");
    }
}
