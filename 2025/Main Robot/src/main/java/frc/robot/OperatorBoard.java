package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Field.Branch;

public class OperatorBoard {
    private static final GenericHID leftPico = new GenericHID(1);
    private static final GenericHID rightPico = new GenericHID(2);


    public static Trigger leftStation(){
        return new Trigger(() -> leftPico.getRawButton(13));

    }

    public static Trigger rightStation(){
        return new Trigger(() -> leftPico.getRawButton(14));
    }

    public static Pose2d getScoringWaypoint(BooleanSupplier isRedAlliance){

        if (leftPico.getRawButton(1)) {
            return Constants.Field.getWaypoint(
                !isRedAlliance.getAsBoolean() ? 21 : 10,
                Branch.kRight);
        }
        
        if (leftPico.getRawButton(2)) {
            return Constants.Field.getWaypoint(
                !isRedAlliance.getAsBoolean() ? 21 : 10,
                Branch.kLeft);
        }

        if (leftPico.getRawButton(3)){
            return Constants.Field.getWaypoint(
               !isRedAlliance.getAsBoolean() ? 22 : 9,
               Branch.kRight);
        }

        if (leftPico.getRawButton(4)){
            return Constants.Field.getWaypoint(
                !isRedAlliance.getAsBoolean() ? 22 : 9, 
                Branch.kLeft);
        }

        if (leftPico.getRawButton(5)) {
            return Constants.Field.getWaypoint(
                !isRedAlliance.getAsBoolean() ? 17 :8, 
                Branch.kRight);
        }

        if (leftPico.getRawButton(6)) {
            return Constants.Field.getWaypoint(
                !isRedAlliance.getAsBoolean() ? 17 :8, 
                Branch.kLeft);
        }
        
        if (leftPico.getRawButton(7)) {
            return Constants.Field.getWaypoint(
                !isRedAlliance.getAsBoolean() ? 18 : 7,
                Branch.kRight);
        }

        if (leftPico.getRawButton(8)) {
            return Constants.Field.getWaypoint(
                !isRedAlliance.getAsBoolean() ? 18 :7, 
                Branch.kLeft);
        }

        if (leftPico.getRawButton(9)) {
            return Constants.Field.getWaypoint(
                !isRedAlliance.getAsBoolean()  ? 19 : 6, 
                Branch.kRight);
        }

        if (leftPico.getRawButton(10)) {
            return Constants.Field.getWaypoint(
                !isRedAlliance.getAsBoolean() ? 19 :6, 
                Branch.kLeft);
        }

        if (leftPico.getRawButton(11)) {
            return Constants.Field.getWaypoint(
                !isRedAlliance.getAsBoolean() ? 20 : 11, 
                Branch.kRight);
        }

        if (leftPico.getRawButton(12)) {
            return Constants.Field.getWaypoint(
                !isRedAlliance.getAsBoolean() ? 20 : 11, 
                Branch.kLeft);
        }

        return null;
    }


    public static Trigger elevatorBrake(){
        return new Trigger(() -> rightPico.getRawButton(6));
    }

    public static Trigger clawsUp(){
        return new Trigger(() -> rightPico.getRawButton(9));
    }

    public static Trigger clawsDown(){
        return new Trigger(() -> rightPico.getRawButton(8));
    }

    public static Trigger doTheThing(){
        return new Trigger(() -> rightPico.getRawButton(7));
    }

    public static double getElevatorSetpoint(){
        if (rightPico.getRawButton(5))
            return Constants.ElevatorConstants.kStow;

        if (rightPico.getRawButton(1))
            return Constants.ElevatorConstants.kL4;
        
        if (rightPico.getRawButton(2)) {
            return Constants.ElevatorConstants.kL3;
        }

        if (rightPico.getRawButton(3)) 
            return Constants.ElevatorConstants.kL2;

        if (rightPico.getRawButton(4))
            return Constants.ElevatorConstants.kL1;

        return Constants.ElevatorConstants.kStow;
    } 
}
