package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;

@Autonomous(name = "OdoTest", group = "Autonomous")
public class OdoOp extends LinearOpMode {
    Pose currPose;
    Pose ballPose;
    Lodo lodo = new Lodo();


    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetry.setMsTransmissionInterval(50);

        final double ARRIVAL_RADIUS = 3.0;

        //double[] ballpos = new double[] {67.5, 0.0, 0.0};
        waitForStart();
        while (opModeIsActive()) {
            follower.update();
            drawCurrentAndHistory();

            ballPose = new Pose(27.0, 0.0, 0.0);
            currPose = follower.getPose();

            //telemetry.addData("Go To (x,y,h)", "(%.2f,%.2f,%.2f)",67.5, 0.0, 0.0);
            double dx = ballPose.getX() - currPose.getX();
            double dy = ballPose.getY() - currPose.getY();

            double distance = Math.hypot(dx, dy);

            if (distance <= ARRIVAL_RADIUS) {

                // We've reached the ball
                follower.breakFollowing();

                telemetry.addData("ARRIVED", "true");

            } else if (!follower.isBusy()) {

                PathChain path = follower.pathBuilder()
                        .addPath(new BezierLine(currPose, ballPose))
                        .setLinearHeadingInterpolation(
                                currPose.getHeading(),
                                ballPose.getHeading()
                        )
                        .build();

                follower.followPath(path, true);
            }
        }
    }
}