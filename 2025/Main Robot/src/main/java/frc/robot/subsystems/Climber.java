package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase{
    private final SparkMax motorA = new SparkMax(Constants.RobotMap.kClimberMotorA, MotorType.kBrushless);
    private final SparkMax motorB = new SparkMax(Constants.RobotMap.kClimberMotorB, MotorType.kBrushless);

    private final SparkMaxConfig normalConfig = new SparkMaxConfig();
    private final SparkMaxConfig invertedConfig = new SparkMaxConfig();

    private final DutyCycleEncoder encoderA = new DutyCycleEncoder(Constants.RobotMap.kClimberEncoderA);
    private final DutyCycleEncoder encoderB = new DutyCycleEncoder(Constants.RobotMap.kClimberEncoderB);

    public Climber(){
        
        normalConfig
            .smartCurrentLimit(40)  
            .idleMode(IdleMode.kBrake)
            .inverted(false);

        invertedConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .inverted(true);

        motorA.configure(
            normalConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);

        motorB.configure(
            invertedConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);
    }

    public Command stop(){
        return this.runOnce(() -> {
            motorA.set(0);
            motorB.set(0);
        }).withName("Stop");
    }

    // Everything in here looks good to me. Good job.
    // Well beed a method to open the claws as well

    public Command clawClose(){
        return this.run(() -> {
            if (encoderA.get() <= Constants.ClimberConstants.kMaxMotorA) {
                motorA.set(Constants.ClimberConstants.kSpeed);
            } else {
                motorA.set(0);
            }

            if (encoderB.get() <= Constants.ClimberConstants.kMaxMotorB) {
                motorB.set(Constants.ClimberConstants.kSpeed);
            } else {
                motorB.set(0);
            }
        }).withName("Close");
    }

    public Command clawOpen(){
        return this.run(() -> {
            if (encoderA.get() >= Constants.ClimberConstants.kMinMoterA) {
                motorA.set(-Constants.ClimberConstants.kSpeed);
            } else {
                motorA.set(0);
            }
            if (encoderB.get() >= Constants.ClimberConstants.kMinMotorB) {
                motorB.set(-Constants.ClimberConstants.kSpeed);
            } else {
                motorB.set(0);
            }
        }).withName("Open");
    }

    public void periodic(){
        SmartDashboard.putNumber("Encoder A Pos", encoderA.get());
        SmartDashboard.putNumber("Encoder B Pos", encoderB.get());
        SmartDashboard.putData(this);

    }

    
}
