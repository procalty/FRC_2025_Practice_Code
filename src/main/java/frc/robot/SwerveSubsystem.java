// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.wpilibj.SPI;

public class SwerveSubsystem extends SubsystemBase {
  //swerve modules
  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  //pigeon stuff
  private final Pigeon2 gyro = new Pigeon2(0);
  private final StatusSignal<Double> yaw;
  private final StatusSignal<Double> pitch;
  private final StatusSignal<Double> roll;

  private final SwerveDriveKinematics kinematics;

  private static final double Max_speed = 4.0; //m/s
  private static final double Max_angular_speed = Math.PI;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {

    //follows this order, drive motor ID, turn motor id, endocder id, encoder offset
    frontLeftModule = new SwerveModule(0, 0, 0, 0);
    frontRightModule = new SwerveModule(0, 0, 0, 0);
    backLeftModule = new SwerveModule(0, 0, 0, 0);
    backRightModule = new SwerveModule(0, 0, 0, 0);

    kinematics = new SwerveDriveKinematics(
      new Translation2d(0.3, 0.3), //front left
      new Translation2d(0.3, -0.3), //front right
      new Translation2d(-0.3, 0.3),//back left
      new Translation2d(-0.3, -0.3)//back right

    );
    /**
     * @param velocityMPS
     */

     public void setFrontLeftVelocity(double velocityMPS) {
      frontLeftModule.setDriveVelocity(velocityMPS);
  }
  
        /**
     * @param velocityMPS
     */

     public void setFrontRightVelocity(double velocityMPS){
      frontRightModule.setDriveVelocity(velocityMPS);
   }
        /**
     * @param velocityMPS
     */

     public void setBackLeftVelocity(double velocityMPS){
      backLeftModule.setDriveVelocity(velocityMPS);
   }
       /**
     * @param velocityMPS
     */

     public void setBackRightVelocity(double velocityMPS){
      backRightModule.setDriveVelocity(velocityMPS);
   }



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
