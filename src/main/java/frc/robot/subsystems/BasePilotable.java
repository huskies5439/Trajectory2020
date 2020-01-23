/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BasePilotable extends SubsystemBase {
  private CANSparkMax neod1 = new CANSparkMax(24, MotorType.kBrushless);
    private CANSparkMax neod2 = new CANSparkMax(25, MotorType.kBrushless);
    private CANSparkMax neog1 = new CANSparkMax(22, MotorType.kBrushless);
    private CANSparkMax neog2 = new CANSparkMax(23, MotorType.kBrushless);
   
    private SpeedControllerGroup neod = new SpeedControllerGroup(neod1, neod2);
    private SpeedControllerGroup neog = new SpeedControllerGroup(neog1, neog2);
    private DifferentialDrive drive = new DifferentialDrive(neog, neod);
   
    private PigeonIMU gyro = new PigeonIMU(3);
    private double[] ypr = new double[3];
    private double[] ypr_dps = new double[3];
    
    private CANEncoder encodeurd = neod1.getEncoder();
    private CANEncoder encodeurg = neog1.getEncoder();
    
    private DifferentialDriveOdometry odometrie;

  public BasePilotable() {
   encodeurg.setPositionConversionFactor(1/4.67*0.1016*Math.PI);
   encodeurd.setPositionConversionFactor(1/4.67*0.1016*Math.PI);
   encodeurg.setVelocityConversionFactor(0.1016*Math.PI/(4.67*60));
   encodeurd.setVelocityConversionFactor(0.1016*Math.PI/(4.67*60));
   resetEncoders();
   odometrie = new DifferentialDriveOdometry(Rotation2d.fromDegrees(angle()));
   resetGyro();
   drive.setSafetyEnabled(false);
  }

  @Override
  public void periodic() {
    odometrie.update(Rotation2d.fromDegrees(angle()),encodeurg.getPosition(), -encodeurd.getPosition());
    SmartDashboard.putNumber("VelocityG", neog1.getEncoder().getVelocity());
    SmartDashboard.putNumber("VelocityD", -neod1.getEncoder().getVelocity());
    SmartDashboard.putNumber("PositionG", neog1.getEncoder().getPosition());
    SmartDashboard.putNumber("PositionD", -neod1.getEncoder().getPosition());
    SmartDashboard.putNumber("Angle", angle());
  }

  public double angle() {
    gyro.getYawPitchRoll(ypr);
    return ypr[0]*-1;
  }
 
  public void resetEncoders(){
    encodeurd.setPosition(0);
    encodeurg.setPosition(0);
  }
  public Pose2d getPose() {
    return odometrie.getPoseMeters();
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(encodeurg.getVelocity(), -encodeurd.getVelocity());
  }
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometrie.resetPosition(pose, Rotation2d.fromDegrees(angle()));
  }
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    neog.setVoltage(leftVolts);
    neod.setVoltage(-rightVolts);
  }
  public double getAverageEncoderDistance() {
    return (encodeurg.getPosition() + -encodeurd.getPosition()) / 2.0;
  }
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }
  public void zeroHeading() {
    gyro.setYaw(0);
  }
  public double getTurnRate() {
    gyro.getRawGyro(ypr_dps);
    
    return ypr_dps[0]*-1;
  }

  public void resetGyro(){
    gyro.setYaw(0);
  }
}
