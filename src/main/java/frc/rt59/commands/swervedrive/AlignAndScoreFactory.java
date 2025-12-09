package frc.rt59.commands.swervedrive;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.rt59.RobotVisualizer;
import frc.rt59.subsystems.swervedrive.SwerveSubsystem;
import frc.rt59.subsystems.swervedrive.Vision;
import frc.rt59.subsystems.swervedrive.Vision.Cameras;

public class AlignAndScoreFactory {

    /**
     * Returns a command group that:
     * 1) reads vision and builds a PathPlannerPath (at execute time)
     * 2) publishes visuals to Field2d/visualizer
     * 3) resets odometry (to avoid stale follower state)
     * 4) schedules the PathPlanner follow command
     *
     * Use: AlignAndScoreFactory.autoAlign(drivebase, visualizer).schedule();
     */
    public static Command autoAlign(SwerveSubsystem drivebase, RobotVisualizer visualizer) {
        AtomicReference<PathPlannerPath> pathRef = new AtomicReference<>();
        AtomicReference<Pose2d> tagPoseRef = new AtomicReference<>();

        // 1) Build path and publish visuals (executed when the command runs)
        InstantCommand buildAndPublish = new InstantCommand(() -> {
            Optional<PhotonPipelineResult> best = Vision.getCamera(Cameras.RT_1).getBestResult();
            if (best.isEmpty() || !best.get().hasTargets()) {
                System.out.println("[Align] no vision targets");
                pathRef.set(null);
                return;
            }

            int nearestReefTag = best.get().getTargets().stream()
                    // Filter only valid tag IDs (6–11 or 17–22)
                    .filter(t -> {
                        int id = t.getFiducialId();
                        return (id >= 6 && id <= 11) || (id >= 17 && id <= 22);
                    })
                    // Sort or just find the minimum by distance
                    .min(Comparator.comparingDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm()))
                    .get().fiducialId;

            int bestTarget = nearestReefTag;
            if (!((bestTarget >= 6 && bestTarget <= 11) || (bestTarget >= 17 && bestTarget <= 22))) {
                System.out.println("[Align] target id out of range: " + bestTarget);
                pathRef.set(null);
                return;
            }

            Pose2d tagPose = Vision.getAprilTagPose(bestTarget,
                    new Transform2d(new Translation2d(), new Rotation2d(Units.degreesToRadians(180))));
            tagPoseRef.set(tagPose);

            // Build the path using the robot's current pose
            PathPlannerPath drivePath = new PathPlannerPath(
                    PathPlannerPath.waypointsFromPoses(
                            drivebase.getPose(),
                            tagPose.transformBy(new Transform2d(new Translation2d(-0.7747, 0.166), new Rotation2d(0)))),
                    new PathConstraints(2, 2, 240, 240),
                    new IdealStartingState(0, drivebase.getPose().getRotation()),
                    new GoalEndState(0, tagPose.getRotation()));
            drivePath.preventFlipping = true;
            pathRef.set(drivePath);

            // publish visuals
            drivebase.getField().getObject("Auto Align Tag").setPose(tagPose);
            visualizer.putPose(tagPose, "Tag Pose");
            List<Pose2d> poses = new ArrayList<>();
            poses.addAll(
                    drivePath.getAllPathPoints().stream()
                            .map(pt -> new Pose2d(pt.position.getX(), pt.position.getY(), new Rotation2d()))
                            .collect(Collectors.toList()));
            drivebase.getField().getObject("drivePath").setPoses(poses);
            visualizer.putTrajectory(poses, "Drive Path");

            // debug
            System.out.println("[Align] built path with " + drivePath.getAllPathPoints().size() + " points");
            System.out.println("[Align] robotPose=" + drivebase.getPose() + " tagPose=" + tagPose);
        });

        // 2) Attempt to start following: reset odometry and schedule follow command
        InstantCommand scheduleFollow = new InstantCommand(() -> {
            if (DriverStation.isDisabled()) {
                System.out.println("[Align] DS disabled; not scheduling");
                return;
            }
            PathPlannerPath p = pathRef.get();
            Pose2d tag = tagPoseRef.get();
            if (p == null || tag == null) {
                System.out.println("[Align] no path to follow");
                return;
            }

            // reset odometry to current pose to ensure follower controller re-inits cleanly
            drivebase.resetOdometry(drivebase.getPose());
            System.out.println("[Align] resetOdometry to " + drivebase.getPose());

            Command follow = drivebase.followPath(p);
            if (follow == null) {
                System.out.println("[Align] follow command is null");
                return;
            }
            System.out.println("[Align] scheduling follow command: " + follow);
            follow.schedule();
        });

        // Sequence: build/publish -> schedule follow
        // NOTE: this group intentionally does NOT add the drivebase as a requirement,
        // so the follow command can take the subsystem when scheduled.
        return new SequentialCommandGroup(buildAndPublish, scheduleFollow);
    }
}
