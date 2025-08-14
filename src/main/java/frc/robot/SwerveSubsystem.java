// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveSubsystem extends SubsystemBase {
  //swerve modules
  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  //pigeon stuff
  private final Pigeon2 gyro = new Pigeon2(0);
  private final StatusSignal<Angle> yaw;
  private final StatusSignal<Angle> pitch;
  private final StatusSignal<Angle> roll;

  private final SwerveDriveKinematics kinematics;

  private static final double Max_speed = 4.0; //m/s
  private static final double Max_angular_speed = Math.PI;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {

    yaw = gyro.getYaw();
    pitch = gyro.getPitch();
    roll = gyro.getRoll();

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

    

  }

  public void resetHeading() {
    gyro.setYaw(0);
}
/**
* @return heading
*/
public double getHeading() {
   return Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360);
}
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


   /**
    * @param angleRadians
    */
    public void setFrontLeftAngle(double angleRadians){
      frontLeftModule.setTurnAngle(angleRadians);
    }

       /**
    * @param angleRadians
    */
    public void setFrontRightAngle(double angleRadians){
      frontRightModule.setTurnAngle(angleRadians);
    }

       /**
    * @param angleRadians
    */
    public void setBackLeftAngle(double angleRadians){
      backLeftModule.setTurnAngle(angleRadians);
    }


       /**
    * @param angleRadians
    */
    public void setBackRightAngle(double angleRadians){
      backRightModule.setTurnAngle(angleRadians);
    }

    /**
     * @param velocityMPS
     * @param angleRadians
     */
    public void setFrontLeftModule(double velocityMPS, double angleRadians){
      frontLeftModule.setVelocityAndAngle(velocityMPS, angleRadians);
    }

        /**
     * @param velocityMPS
     * @param angleRadians
     */
    public void setFrontRightModule(double velocityMPS, double angleRadians){
      frontRightModule.setVelocityAndAngle(velocityMPS, angleRadians);
    }

        /**
     * @param velocityMPS
     * @param angleRadians
     */
    public void setBaackLeftModule(double velocityMPS, double angleRadians){
      backLeftModule.setVelocityAndAngle(velocityMPS, angleRadians);
    }

        /**
     * @param velocityMPS
     * @param angleRadians
     */
    public void setBackRightModule(double velocityMPS, double angleRadians){
      backRightModule.setVelocityAndAngle(velocityMPS, angleRadians);
    }


    /**
     * @param moduleStates
     */

     public void applyModuleState(SwerveModuleState[] moduleStates){
      frontLeftModule.setDesiredState(moduleStates[0]);
      frontRightModule.setDesiredState(moduleStates[1]);
      backLeftModule.setDesiredState(moduleStates[2]);
      backRightModule.setDesiredState(moduleStates[3]);
     }

     /**
      * @param flVel
      * @param frVel
      * @param blVel
      * @param brVel
      */
      public void applyDriveVelocity(double flVel, double frVel, double blVel, double brVel){
        frontLeftModule.setDriveVelocity(flVel);
        frontRightModule.setDriveVelocity(frVel);
        backLeftModule.setDriveVelocity(blVel);
        backRightModule.setDriveVelocity(brVel);
      }

      /**
       * @param flAngle
       * @param frAngle
       * @param blAngle
       * @param brAngle
       */

       public void applyTurnAngles(double flAngle, double frAngle, double blAngle, double brAngle){
        frontLeftModule.setTurnAngle(flAngle);
        frontRightModule.setTurnAngle(frAngle);
        backLeftModule.setTurnAngle(blAngle);
        backRightModule.setTurnAngle(brAngle);
       }

      public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

       /**
        * @param speedx
        * @param speedy
        * @param rotation
        * @param fieldRelative
        */

        public void drive(double speedx, double speedy, double rotation, boolean frieldRelative){
          speedx *= Max_speed;
          speedy *= Max_speed;
          rotation *= Max_angular_speed;

          ChassisSpeeds chassisspeeds;

          if(frieldRelative){
            chassisspeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedx, speedy, rotation, getRotation2d());
          }
          else{
            chassisspeeds = new ChassisSpeeds(speedx, speedy, rotation);
          }

          SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisspeeds);
          SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Max_speed);

          applyModuleState(moduleStates);
        }

        public void stopall(){
          frontLeftModule.stop();
          frontRightModule.stop();
          backLeftModule.stop();
          backRightModule.stop();
        }

        public void stopSpecificModule(String module){
          switch(module.toLowerCase()){
            case "fl":
            case "frontleft":
              frontLeftModule.stop();
              break;  
            case "fr":
            case "frontright":
                frontRightModule.stop();
                break;
            case "bl":
            case "backleft":
                backLeftModule.stop();
                break;
            case "br":
            case "backright":
                backRightModule.stop();
                break;
          }
        }
        

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Gyro Heading", getHeading());
    SmartDashboard.putNumber("FL Velocity", frontLeftModule.getCurrentVelocity());
    SmartDashboard.putNumber("FR Velocity", frontRightModule.getCurrentVelocity());
    SmartDashboard.putNumber("BL Velocity", backLeftModule.getCurrentVelocity());
    SmartDashboard.putNumber("BR Velocity", backRightModule.getCurrentVelocity());
    SmartDashboard.putNumber("FL Angle", Math.toDegrees(frontLeftModule.getCurrentAngle()));
    SmartDashboard.putNumber("FR Angle", Math.toDegrees(frontRightModule.getCurrentAngle()));
    SmartDashboard.putNumber("BL Angle", Math.toDegrees(backLeftModule.getCurrentAngle()));
    SmartDashboard.putNumber("BR Angle", Math.toDegrees(backRightModule.getCurrentAngle()));

  }
}
