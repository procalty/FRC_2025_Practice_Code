// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot;


//Phenoix 6 imports for the motors
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

//WPIlib imports for controls n stuff
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  
  public TalonFX motor_1 = new TalonFX(1); //need to change CAN ID
  public TalonFX motor_2 = new TalonFX(2); //need to change CAN ID
  public DutyCycleOut dutycycleout = new DutyCycleOut(1);
  public PositionVoltage positionvoltage = new PositionVoltage(12);
  public StatusSignal<Angle> position_1;
  public StatusSignal<Angle> position_2;
  public StatusSignal<Current> current_1;
  public StatusSignal<Current> current_2;
  public StatusSignal<AngularVelocity> velocity_1;
  public StatusSignal<AngularVelocity> velocity_2;
  MotionMagicVelocityVoltage motionmagicrequest_velocity = new MotionMagicVelocityVoltage(0);


  public Flywheel() {
    position_1 = motor_1.getPosition();
    position_2 = motor_2.getPosition();
    current_1 = motor_1.getStatorCurrent();
    current_2 = motor_2.getStatorCurrent();
    velocity_1 = motor_1.getVelocity();
    velocity_2 = motor_2.getVelocity();

    //Bruh what the fuck is this bullshit they getting rid of .setInverted() smh
    //Had to make two seperate configs just for ONE FUCKING MOTOR TO SPIN THE OTHER FUCKING WAY
    TalonFXConfiguration config_1 = new TalonFXConfiguration();
    TalonFXConfiguration config_2 = new TalonFXConfiguration();

    config_1.MotionMagic.MotionMagicCruiseVelocity = 250;
    config_1.MotionMagic.MotionMagicAcceleration = 125;
    config_1.MotionMagic.MotionMagicJerk = 1000;
    

    config_1.Slot0.kP = 0.4;
    config_1.Slot0.kI = 0.6;

    config_2.MotionMagic.MotionMagicCruiseVelocity = 250;
    config_2.MotionMagic.MotionMagicAcceleration = 125;
    config_2.MotionMagic.MotionMagicJerk = 1000;
    
    config_2.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config_2.Slot0.kP = 0.4;
    config_2.Slot0.kI = 0.6;


    motor_1.getConfigurator().apply(config_1);  
    motor_2.getConfigurator().apply(config_2);

    Command run = this.runEnd(()->{
      motor_1.setControl(motionmagicrequest_velocity.withVelocity(250));
      motor_2.setControl(motionmagicrequest_velocity.withVelocity(250));
    },()->{motor_1.stopMotor();motor_2.stopMotor();});

    SmartDashboard.putData("Run Motor", run);

    HumanControls.LT.onTrue(run);



    
  }
  public double getMotorPosition_1(){
    return position_1.refresh().getValueAsDouble();
  }

  public double getMotorPosition_2(){
    return position_2.refresh().getValueAsDouble();
  }

  public double getVelocity_1(){
    return velocity_1.refresh().getValueAsDouble();
  }

  public double getVelocity_2(){
    return velocity_2.refresh().getValueAsDouble();
  }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("Motor_Positon_1", getMotorPosition_1());
    SmartDashboard.putNumber("Motor Position_2", getMotorPosition_2());
    SmartDashboard.putNumber("Velocity 1", getVelocity_1());
    SmartDashboard.putNumber("Velocity 2", getVelocity_2());
    SmartDashboard.putNumber("Motor Current_1", current_1.refresh().getValueAsDouble());
    SmartDashboard.putNumber("Motor Current_2", current_2.refresh().getValueAsDouble());
  }
}
