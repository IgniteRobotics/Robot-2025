// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Paths {

     public PathPlannerPath createTestPath(){
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(2.254, 6.476, Rotation2d.fromDegrees(0)),
        new Pose2d(4.471, 6.524, Rotation2d.fromDegrees(0)),
        new Pose2d(5.586, 5.577, Rotation2d.fromDegrees(-45))
            );

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
              waypoints,
               constraints,
               null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0, Rotation2d.fromDegrees(60)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;

        return path;
    }

    public PathPlannerPath createHPPath(){
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(1.95, 5.21, Rotation2d.fromDegrees(135)),
        new Pose2d(1.2, 6, Rotation2d.fromDegrees(135))
            );

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
              waypoints,
               constraints,
               null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0, Rotation2d.fromDegrees(135)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;

        return path;
    }

    public PathPlannerPath createREEF1Path(){
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(1.95, 5.21, Rotation2d.fromDegrees(-50)),
        new Pose2d(3, 3.72, Rotation2d.fromDegrees(0))
            );

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
              waypoints,
               constraints,
               null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0, Rotation2d.fromDegrees(180)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;

        return path;
    }

}
