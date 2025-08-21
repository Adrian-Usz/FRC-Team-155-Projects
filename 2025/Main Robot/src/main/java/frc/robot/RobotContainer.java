// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Field.Branch;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Ejector;
import frc.robot.subsystems.Elevator;

public class RobotContainer {

    private final double joystickDeadband = 0.1;
    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
    private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(3);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Elevator elevator = new Elevator();
    private final Ejector ejector = new Ejector();
    private final Blinkin blinkin = new Blinkin();
    private final Auto auto = new Auto(drivetrain, ejector, elevator, () -> isRedAlliance());
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    private boolean isRedAlliance = false;
    public boolean allianceIsSet = false;

    public RobotContainer() {
        
        // Send autonomous options to dashboard.
        autoChooser.setDefaultOption("Default: Nothing", null);
        autoChooser.addOption("Center", auto.centerOnePiece());
        autoChooser.addOption("Left", auto.leftOnePiece());
        autoChooser.addOption("Right", auto.rightOnePiece());
        SmartDashboard.putData("Auto Choices", autoChooser);

        // Run initial calculations for all scoring waypoints
        Constants.Field.calculateScoringWaypoints();

        // Set default commands for all subsystems
        drivetrain.setDefaultCommand(Commands.sequence(
            new InstantCommand(() -> drivetrain.updateHeadingCorrection()),
            drivetrain.fieldCentricFacingAngle(
                () -> squareInput(getVelocityX()),
                () -> squareInput(getVelocityY()),
                () -> drivetrain.getHeadingCorrectionAngle()
        )));

        ejector.setDefaultCommand(ejector.stop());
        elevator.setDefaultCommand(elevator.stow());
        blinkin.setDefaultCommand(blinkin.off());

        // Start telemetry logging
        drivetrain.registerTelemetry(logger::telemeterize);

        configureBindings();
    }

    private void configureBindings() {

        /* ====== MAIN DRIVER BINDINGS ===== */

        // "Zero" the gyro towards the current facing direction
        controller.a().onTrue(Commands.runOnce(() -> {
            drivetrain.seedFieldCentric();
            drivetrain.updateHeadingCorrection();
        },  drivetrain));

        // Use a fully manual drive whenever the right thumbstick isn't 0, meaning the driver is trying
        // to manually rotate the robot. This command is not interruptable.
        new Trigger(() -> getVelocityRotation() != 0).whileTrue(drivetrain.manualDrive(
            () -> squareInput(getVelocityX()),
            () -> squareInput(getVelocityY()),
            () -> squareInput(getVelocityRotation())
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

        // When we have coral, are close enough to the reef, and are in teleop, update the default
        // drive command to face the reef while the driver maintains translation control. If any of those
        // conditions are no longer true, reset default command to standard drive with heading correction.
        new Trigger(() -> ejector.hasCoral())
            .and(new Trigger(() -> drivetrain.getDistanceFromReefCenter(() -> isRedAlliance) < Feet.of(10).in(Meters)))
            .and(() -> DriverStation.isTeleop())
            .whileTrue(drivetrain.fieldCentricFacingAngle(
                () -> squareInput(getVelocityX()),
                () -> squareInput(getVelocityY()),
                () -> drivetrain.getHeadingTowardsReef()));
            

        // Command the robot to face its back side towards the right loading station. The driver
        // still has full translation control.
        controller.rightTrigger().whileTrue(drivetrain.fieldCentricFacingAngle(
            () -> squareInput(getVelocityX()),
            () -> squareInput(getVelocityY()),
            () -> Constants.Field.kRightLoadStationHeading));

        controller.leftTrigger().whileTrue(drivetrain.fieldCentricFacingAngle(
            () -> squareInput(getVelocityX()),
            () -> squareInput(getVelocityY()),
            () -> Constants.Field.kLeftLoadStationHeading));

        controller.rightBumper().and(() -> (OperatorBoard.getScoringWaypoint(() -> isRedAlliance) != null)).whileTrue(
            drivetrain.goToPose(
                () -> OperatorBoard.getScoringWaypoint(() -> isRedAlliance),
                () -> isRedAlliance
            ).andThen(drivetrain.brake()));

        /* ===== OPERATOR BINDINGS ===== */
        OperatorBoard.doTheThing().and(OperatorBoard.leftStation().or(OperatorBoard.rightStation())).whileTrue(
            auto.loadCoral()
        );

        OperatorBoard.doTheThing().and(OperatorBoard.leftStation().and(OperatorBoard.rightStation())).whileTrue(
            auto.reverseCoralAtLoadHeight().withTimeout(1).andThen(auto.loadCoral())
        );

        OperatorBoard.doTheThing().and(() -> (OperatorBoard.getScoringWaypoint(() -> isRedAlliance) != null)).whileTrue(
            auto.scoreCoral(() -> OperatorBoard.getElevatorSetpoint())
        );

        OperatorBoard.elevatorBrake().whileTrue(elevator.stop().withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        
        /* ===== LED PATTERN TRIGGERS ===== */
        new Trigger(() -> ejector.hasCoral()).onTrue(blinkin.taskComplete());
        new Trigger(() -> drivetrain.atPose()).whileTrue(blinkin.taskComplete());
        new Trigger(() -> Math.abs(elevator.getHeight() - Constants.ElevatorConstants.kLoad) < 0.1)
            .and(OperatorBoard.leftStation().or(OperatorBoard.rightStation()))
            .whileTrue(blinkin.elevatorAtLoadHeight());

        // Keep this here for testing purposes
        operatorController.rightTrigger().whileTrue(elevator.manualDrive(
            () -> -MathUtil.applyDeadband(operatorController.getRightY(), 0.075)));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Inverts and applies a deadband to the main driver controller's left Y axis.
     * @return Joystick value in range -1 to 1
     */
    public double getVelocityX() {
        return -MathUtil.applyDeadband(controller.getLeftY(), joystickDeadband);
    }

    /**
     * Inverts and applies a deadband to the main driver controller's left X axis.
     * @return Joystick value in range -1 to 1
     */
    public double getVelocityY() {
        return -MathUtil.applyDeadband(controller.getLeftX(), joystickDeadband);
    }

    /**
     * Inverts and applies a deadband to the main driver controller's right X axis.
     * @return Joystick value in range -1 to 1
     */
    public double getVelocityRotation() {
        return -MathUtil.applyDeadband(controller.getRightX(), joystickDeadband);
    }

    /**
     * Squares an input number, preserving original sign.
     * @param input Number to square
     * @return Squared value with preserved sign.
     */
    public double squareInput(double input) {
        if (input  < 0) {
            return -Math.pow(input, 2);
        }
        return Math.pow(input, 2);
    }

    /**
     * Method to poll driverstation for alliance color. If color is received, isRedAlliance
     * boolean is updated, and allainceIsSet boolean is updated to prevent further
     * driverstation calls.
     */
    public void getAllianceColor(){
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                isRedAlliance = true;
            }
            allianceIsSet = true;
        }
    }

    /**
     * Getter method for isRedAlliance boolean.
     * @return True if received driver station alliance is Red, false if Blue.
     */
    public boolean isRedAlliance() {
        return isRedAlliance;
    }

    /**
     * Wrapper for the drivetrain's updateMaxSpeed method to be callable
     * in robotPeriodic in Robot.java
     */
    public void updateDrivetrainSpeed() {
        drivetrain.updateMaxSpeed(elevator::getHeight);
    }
}