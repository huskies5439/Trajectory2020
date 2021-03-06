/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.BasePilotable;

public class RobotContainer {
  private final BasePilotable basePilotable = new BasePilotable();
  Trajectory exampleTrajectory = null;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
  }

  public Command getAutonomousCommand() {
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(0.128, 1.83, 0.204),
        Constants.kinematics, 10); // 0.109, 1.87, 0.216

    TrajectoryConfig config = new TrajectoryConfig(1, 0.5)// max speed, max acceleration
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.kinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);
    
    String trajectoryJSON = "output/test.wpilib.json";
    try
    {
    //Fait un S si il trouve son path
     var path = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
     DriverStation.reportWarning("Path : " + path,false);
     exampleTrajectory = TrajectoryUtil.fromPathweaverJson(path);
     var transform = basePilotable.getPose().minus(exampleTrajectory.getInitialPose());
     exampleTrajectory= exampleTrajectory.transformBy(transform);
     
    } 
    catch (IOException e)
    {
      // An example trajectory to follow.  All units in meters.
     exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(
          new Translation2d(1, 1),
          new Translation2d(2, -1)
      ),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(3, 0, new Rotation2d(0)),
      // Pass config
      config
  );
        DriverStation.reportError("Unable to open trajectory : " + trajectoryJSON, e.getStackTrace());
    }
    

    

    RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, basePilotable::getPose,
        new RamseteController(2, 0.7), 
        new SimpleMotorFeedforward(0.166, 1.8, 0.186), 
        Constants.kinematics,
        basePilotable::getWheelSpeeds, 
        new PIDController(1, 0, 0), 
        new PIDController(1, 0, 0), // 7.8
        // RamseteCommand passes volts to the callback
        basePilotable::tankDriveVolts, basePilotable);// 8.92

    return ramseteCommand.andThen(() -> 
      basePilotable.tankDriveVolts(0, 0));
      //.beforeStarting(()-> basePilotable.resetOdometry(new Pose2d()));
    // return new RunCommand(()-> basePilotable.tankDriveVolts(0.2,0.2));

  }
}
