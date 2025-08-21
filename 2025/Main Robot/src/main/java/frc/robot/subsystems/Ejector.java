package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.LaserCan;

import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Ejector extends SubsystemBase {
    private final SparkMax topMotor = new SparkMax(Constants.RobotMap.kEjectorMotorA, MotorType.kBrushless);
    private final SparkMax bottomMotor = new SparkMax(Constants.RobotMap.kEjectorMotorB, MotorType.kBrushless);

    private final SparkMaxConfig normalConfig = new SparkMaxConfig();
    private final SparkMaxConfig invertedConfig = new SparkMaxConfig();

    private final LaserCan coralDetector = new LaserCan(Constants.RobotMap.kCoralDetector);
    private final Alert coralDetectorAlert = new Alert("Coral Detector not found", AlertType.kError);

    public Ejector() {
        normalConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .inverted(false);

        invertedConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .inverted(true);
        
        topMotor.configure(
            normalConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        bottomMotor.configure(
            invertedConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    public boolean hasCoral() {
        if (coralDetector.getMeasurement() != null) {
            coralDetectorAlert.set(false);
            return coralDetector.getMeasurement().distance_mm < 50.8;
        }
        
        coralDetectorAlert.set(true);
        return false;
    }

    private void setMotorSpeeds(double topMotorSpeed,double bottomMotorSpeed) {
        topMotor.set(topMotorSpeed);
        bottomMotor.set(bottomMotorSpeed);
    }

    public Command stop(){
        return this.runOnce(() -> {
            setMotorSpeeds(0, 0);
        }).withName("Stop");
    }

    public Command eject(){
       return this.run(() -> {
            setMotorSpeeds(
                Constants.EjectorConstants.kEjectionSpeed,
                Constants.EjectorConstants.kEjectionSpeed
            );
       }).withName("Eject");
    }

    public Command reverse() {
        return this.run(() -> {
            setMotorSpeeds(
                -Constants.EjectorConstants.kIntakeSpeed,
                -Constants.EjectorConstants.kIntakeSpeed);
        });
    }

    public Command intake(){
        //Until front bean break, run moters 
        // True if blocked
        return this.run(() -> {
            setMotorSpeeds(
                Constants.EjectorConstants.kIntakeSpeed, 
                Constants.EjectorConstants.kIntakeSpeed
            );
        }).until(
            () -> hasCoral()
        ).withName("Intake");
    }

    public void periodic() {
        // SmartDashboard.putBoolean("Coral is loaded", hasCoral());
        SmartDashboard.putNumber("Ejector Top Motor", topMotor.get());
        SmartDashboard.putNumber("Ejector Bottom Motor", bottomMotor.get());
        SmartDashboard.putData(this);

    }
}
