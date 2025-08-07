// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;



/** Add your docs here. */
public class SwerveModule {

    private final TalonFX drivemotor;
    private final TalonFX turnMotor;
    private final CANcoder absoluteEncoder;

    private final VelocityVoltage driverequest = new VelocityVoltage(0);
    private final MotionMagicVoltage turnrequest = new MotionMagicVoltage(0);

    private static final double wheel_circumference = 0.319;
    private static final double drive_gear_ratio = 6.75;
    private static final double turn_gear_ratio = 21.429;

    
    /**
     * 
     *
     * @param driveMotorID 
     * @param turnMotorID 
     * @param encoderID 
     * @param encoderOffset 
     */

     public SwerveModule(int driveMotorID, int turnMotorID, int encoderID, int encoderOffset){
        drivemotor = new TalonFX(driveMotorID);
        turnMotor = new TalonFX(turnMotorID);
        absoluteEncoder = new CANcoder(encoderID);

        configureDriveMotor();
        configureTurnMotor();
        configureEncoder(encoderOffset);
             
    }
   
    private void configureDriveMotor(){ 

        var config = new TalonFXConfiguration();

        config.Slot0.kP = 10.0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;

        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;



        drivemotor.getConfigurator().apply(config);
    }
    
    private void configureTurnMotor(){

        var config = new TalonFXConfiguration();

        config.Slot0.kP = 10.0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;

        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotionMagic.MotionMagicCruiseVelocity = 20;
        config.MotionMagic.MotionMagicAcceleration = 10;

        turnMotor.getConfigurator().apply(config);

    }

    private void configureEncoder(double offset){
        var config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = offset;
        absoluteEncoder.getConfigurator().apply(config);

    }

    public void setDriveVelocity(double VelocityMPS){
        double motorRPS = VelocityMPS/wheel_circumference*drive_gear_ratio;
        drivemotor.setControl(driverequest.withVelocity(motorRPS));

    }

    public void setTurnAngle(double angleRadians){
        double motorRotation = angleRadians/(2*Math.PI) * turn_gear_ratio;
        turnMotor.setControl(turnrequest.withPosition(motorRotation));
    }

    /** 
    *@param VelocityMPS
    *@param angleRadians 

     */

    public void setVelocityAndAngle(double velocityMPS, double angleRadians){
        setDriveVelocity(velocityMPS);
        setTurnAngle(angleRadians);
    }

    /** 
     * @param desiredState
    */

    public void setDesiredState(SwerveModuleState desiredState){
        double currentAngle = getCurrentAngle();

        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, new Rotation2d());

        setDriveVelocity(optimizedState.speedMetersPerSecond);
        setTurnAngle(optimizedState.angle.getRadians());
        
    }

    public void stop(){
        drivemotor.stopMotor();
        turnMotor.stopMotor();
    }

    /**
     * @return 
     */
    public double getCurrentAngle(){
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble()*2*Math.PI;
    }

    /**
     * @return
     */

    public double getCurrentVelocity(){
        double motorRPS = drivemotor.getVelocity().getValueAsDouble();
        return motorRPS*wheel_circumference/drive_gear_ratio;
     }

    /**
     * @return
     */

    public SwerveModuleState getCurrentState(){
        return new SwerveModuleState(
            getCurrentVelocity(), new Rotation2d()
        );
    }

    
}
