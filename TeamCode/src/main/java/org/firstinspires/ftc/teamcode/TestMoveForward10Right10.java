package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "TestMoveForward10Right10", group = "Autonomous")
public class TestMoveForward10Right10 extends OpMode {

    private Follower follower;
    private Path forwardPath;
    private Path rightPath;
    private int pathState = 0;
    private long pauseStartTime = 0;

    private static final long PAUSE_MS = 2000; // 2 second pause

    // =========================================================================
    // INIT
    // =========================================================================
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.update();

        // X = forward, Y = right in Pedro's coordinate system
        forwardPath = new Path(new BezierLine(
                new Pose(0, 0, 0),
                new Pose(10, 0, 0)   // forward 10 inches
        ));
        forwardPath.setConstantHeadingInterpolation(0);

        rightPath = new Path(new BezierLine(
                new Pose(10, 0, 0),
                new Pose(10, 10, 0)  // right 10 inches
        ));
        rightPath.setConstantHeadingInterpolation(0);

        telemetry.addData("Status", "Initialized — ready to run");
        telemetry.update();
    }

    // =========================================================================
    // START
    // =========================================================================
    @Override
    public void start() {
        follower.followPath(forwardPath, true);
        pathState = 1;
    }

    // =========================================================================
    // LOOP
    // =========================================================================
    @Override
    public void loop() {
        follower.update();

        switch (pathState) {

            case 1: // moving forward
                if (!follower.isBusy()) {
                    pauseStartTime = System.currentTimeMillis();
                    pathState = 2;
                }
                break;

            case 2: // pausing for 2 seconds
                if (System.currentTimeMillis() - pauseStartTime >= PAUSE_MS) {
                    follower.followPath(rightPath, true);
                    pathState = 3;
                }
                break;

            case 3: // moving right
                if (!follower.isBusy()) {
                    pathState = 4;
                }
                break;

            case 4: // done
                break;
        }

        String stateLabel;
        switch (pathState) {
            case 1:  stateLabel = "Moving Forward"; break;
            case 2:  stateLabel = "Pausing";        break;
            case 3:  stateLabel = "Moving Right";   break;
            default: stateLabel = "Done";           break;
        }

        telemetry.addData("Path State",    stateLabel);
        telemetry.addData("X",             "%.2f", follower.getPose().getX());
        telemetry.addData("Y",             "%.2f", follower.getPose().getY());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

} // end class