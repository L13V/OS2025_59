package frc.rt59.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import org.photonvision.targeting.PhotonPipelineResult;

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
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.rt59.RobotVisualizer;
import frc.rt59.subsystems.swervedrive.SwerveSubsystem;
import frc.rt59.subsystems.swervedrive.Vision;
import frc.rt59.subsystems.swervedrive.Vision.Cameras;

public class AlignAndScore extends Command {
    private final SwerveSubsystem drivebase;
    private final RobotVisualizer visualizer;
    private Pose2d tagPose;
    private PathPlannerPath drivePath;

    private boolean startedDriving = false;
    private boolean validTarget = false;

    /** Creates a new AlignAndScore. */
    public AlignAndScore(SwerveSubsystem drivebase, RobotVisualizer visualizer) {
        this.drivebase = drivebase;
        this.visualizer = visualizer;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        /*
         * Get Result and find target from Photon
         */
        Optional<PhotonPipelineResult> latestresult = Vision.getCamera(Cameras.RT_1).getLatestResult();
        if (latestresult.isPresent()) {
            PhotonPipelineResult fetchedresult = latestresult.get();
            if (fetchedresult.hasTargets()) {
                // Find best tag
                int bestTarget = fetchedresult.getBestTarget().getFiducialId();
                if ((bestTarget >= 6 && bestTarget <= 11) || (bestTarget >= 17 && bestTarget <= 22)) {
                    // Generate and publish target pose
                    tagPose = Vision.getAprilTagPose(bestTarget,
                            new Transform2d(new Translation2d(), new Rotation2d(Units.degreesToRadians(180))));
                    drivebase.getField().getObject("Auto Align Tag").setPose(tagPose);
                    visualizer.putPose(tagPose, "Tag Pose");

                    validTarget = true;
                }
            }
        }
        if (!validTarget) {
            end(true);
        }
        /*
         * Create the path
         */
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                drivebase.getPose(),
                tagPose.transformBy(new Transform2d(new Translation2d(-0.7747, 0.16499967), new Rotation2d(0))));
        // tagPose);
        drivePath = new PathPlannerPath(waypoints, new PathConstraints(1, 1, 240, 240),
                new IdealStartingState(0, drivebase.getPose().getRotation()),
                new GoalEndState(0, tagPose.getRotation()));
        drivePath.preventFlipping = true;

        /*
         * Publish proposed path
         */
        List<Pose2d> poses = new ArrayList<>();
        poses.addAll(
                drivePath.getAllPathPoints().stream()
                        .map(
                                point -> new Pose2d(
                                        point.position.getX(), point.position.getY(),
                                        new Rotation2d()))
                        .collect(Collectors.toList()));
        drivebase.getField().getObject("drivePath").setPoses(poses);
        visualizer.putTrajectory(poses, "Drive Path");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!startedDriving) {
            Command follow = drivebase.followPath(drivePath);
            follow.schedule();
            startedDriving = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {

        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean finished = (!drivebase.isFollowing() && startedDriving) || DriverStation.isDisabled();
        System.out.println("isFinished? " + finished);
        return false;
    }
}
