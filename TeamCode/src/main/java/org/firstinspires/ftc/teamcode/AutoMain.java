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

@Autonomous(name = "AutoMain", group = "Autonomous")
public class AutoMain extends LinearOpMode {

    // ------------------- Pedro Pathing / Vision -------------------
    Pose currPose;
    Pose ballPose;
    Lodo lodo = new Lodo();

    private enum State {
        SEARCHING,   // no ball detected yet, everything idle
        APPROACHING, // ball detected, driving to it, intake running
        COLLECTING   // arrived at ball, running control transfer + outtake for a fixed duration
    }

    private State state = State.SEARCHING;
    private long collectStartTime = 0;

    @Override
    public void runOpMode() {
        IntakeAuto intaker = new IntakeAuto(hardwareMap);
        double[] ballpos;

        // ------------------- Mechanisms (ported from TeleOp) -------------------
        DcMotorEx intake_motor;
        DcMotorEx control_motor;
        DcMotorEx outtake_motor;

        // Intake: goBILDA 5203 19.2:1, 435 RPM target
        final double INTAKE_TICKS_PER_REV = 384.5;
        final double INTAKE_TARGET_RPM = 435;
        final double INTAKE_TICKS_PER_SEC =
                (INTAKE_TARGET_RPM * INTAKE_TICKS_PER_REV) / 60.0;

        // Control: goBILDA 5203 19.2:1
        final double CONTROL_TICKS_PER_REV = 384.5;
        final double CONTROL_RPM_STAGE_1 = 5;
        final double CONTROL_RPM_STAGE_2 = 1000;
        final double CONTROL_TICKS_PER_SEC_STAGE_1 =
                (CONTROL_RPM_STAGE_1 * CONTROL_TICKS_PER_REV) / 60.0;
        final double CONTROL_TICKS_PER_SEC_STAGE_2 =
                (CONTROL_RPM_STAGE_2 * CONTROL_TICKS_PER_REV) / 60.0;

        // Outtake: goBILDA 5203 Yellow Jacket (no gearbox, 6000 RPM), target 5500 RPM
        final double OUTTAKE_TICKS_PER_REV = 28.0;
        final double OUTTAKE_TARGET_RPM = 5500;
        final double OUTTAKE_TICKS_PER_SEC =
                (OUTTAKE_TARGET_RPM * OUTTAKE_TICKS_PER_REV) / 60.0;

        // ------------------- Automatic state machine -------------------
        // How close (odometry units, same units as your Pose x/y) counts as "arrived at ball".
        // TUNE THIS to your field/robot geometry.
        final double ARRIVAL_RADIUS = 3.0;

        // How long control_motor spends at each stage during collection, in ms.
        // TUNE THESE to how long the 5 RPM "settle" stage and 1000 RPM "transfer" stage actually need.
        final long CONTROL_STAGE_1_DURATION_MS = 500;
        final long CONTROL_STAGE_2_DURATION_MS = 1000;
        final long COLLECT_DURATION_MS = CONTROL_STAGE_1_DURATION_MS + CONTROL_STAGE_2_DURATION_MS;


        // follower/telemetryM/poseHistory are only assigned inside Tuning.onSelect(),
        // which never runs when GoToBall is launched directly. Build everything here.
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0)); // set this to your real starting pose
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetry.setMsTransmissionInterval(50);

        Tuning.initPoseHistory(); // lets drawCurrentAndHistory() work from this package

        // ---------------- Hardware Map: Mechanisms ----------------
        intake_motor  = hardwareMap.get(DcMotorEx.class, "intake_motor");
        control_motor = hardwareMap.get(DcMotorEx.class, "control_motor");
        outtake_motor = hardwareMap.get(DcMotorEx.class, "outtake_motor");

        intake_motor.setDirection(DcMotor.Direction.FORWARD);
        control_motor.setDirection(DcMotor.Direction.FORWARD);
        outtake_motor.setDirection(DcMotor.Direction.REVERSE);

        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        control_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        control_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        control_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            follower.update();
            drawCurrentAndHistory();

            currPose = follower.getPose();

            ballpos = lodo.getBall(
                    new double[]{currPose.getX(), currPose.getY(), Math.toDegrees(currPose.getHeading())},
                    hardwareMap
            );

            if (ballpos != null && ballpos.length != 0) {
                ballPose = new Pose(ballpos[0], ballpos[1], Math.toRadians(ballpos[2]));
                telemetry.addData("Current Position:", "(x,y,h): (%.2f,%.2f,%.2f)", currPose.getX(), currPose.getY(), Math.toDegrees(currPose.getHeading()));
                telemetry.addData("Ball Position:", "(x,y): (%.2f,%.2f)",ballpos[0], ballpos[1]);
                telemetry.update();

                double dx = ballPose.getX() - currPose.getX();
                double dy = ballPose.getY() - currPose.getY();

                double distance = Math.hypot(dx, dy);
                if (distance <= ARRIVAL_RADIUS) {

                    // We've reached the ball
                    follower.breakFollowing();
                    intake_motor.setVelocity(0);

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

                    intake_motor.setVelocity(INTAKE_TICKS_PER_SEC);
                }
            } else {
                control_motor.setVelocity(0);
                outtake_motor.setVelocity(0);
            }

            telemetryM.addData("State", state);
            telemetryM.update();
        }

        // Stop everything on exit
        intake_motor.setVelocity(0);
        control_motor.setVelocity(0);
        outtake_motor.setVelocity(0);
    }
}