package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.HeadingInterpolator;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.configurables.annotations.Configurable;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * MecanumTeleOp - Pedro Pathing version with:
 *   - Pedro handles all motor control and localization
 *   - Cubic input curve  (smoother low-speed control)
 *   - Strafe correction  (lateral power boost)
 *   - Toggle slow mode   (A button)
 *   - Field-centric mode (B button toggles)
 *   - Automated path     (right bumper snaps to a field position)
 *
 * GAMEPAD 1 CONTROLS:
 *   Left Stick  Y-axis   → Forward / Backward
 *   Left Stick  X-axis   → Strafe left / right
 *   Right Stick X-axis   → Rotate left / right
 *   A (press)            → Toggle slow mode (~40% power)
 *   B (press)            → Toggle field-centric / robot-centric
 *   Right Bumper         → Trigger automated path to a preset position
 *   Left Bumper          → Cancel automated path, return to manual drive
 */
@Configurable
@Autonomous(name = "Mecanum Auto", group = "Autonomous")
public class MecanumAuto extends OpMode {

    // -------------------------------------------------------------------------
    // Pedro Pathing
    // -------------------------------------------------------------------------
    private Follower follower;
    private TelemetryManager telemetryM;
    private boolean automatedDrive = false;

    // -------------------------------------------------------------------------
    // Tuning constants
    // -------------------------------------------------------------------------
    private static final double DEADZONE            = 0.05;
    private static final double STRAFE_CORRECTION   = 0.85;
    private static final double SLOW_MODE_MULTIPLIER = 0.4;

    // -------------------------------------------------------------------------
    // Runtime state
    // -------------------------------------------------------------------------
    private boolean slowModeOn     = false;
    private boolean fieldCentricOn = true;

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------
    private double applyCurve(double raw) {
        return raw * raw * raw;
    }

    private double processAxis(double raw) {
        return Math.abs(raw) > DEADZONE ? applyCurve(raw) : 0.0;
    }

    // =========================================================================
    // INIT
    // =========================================================================
    @Override
    public void init() {
        // Pedro creates and manages the motors itself using your Constants file.
        // Make sure your motor names in Constants.java match your hardware config:
        //   "left_front_motor", "left_back_motor", "right_front_motor", "right_back_motor"
        follower = Constants.createFollower(hardwareMap);

        // Set where the robot starts — change this to match your actual start position
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls",
                "LS: drive/strafe | RS-X: rotate | A: slow | B: field-centric | RB: auto path | LB: cancel");
        telemetry.update();
    }

    // =========================================================================
    // START
    // =========================================================================
    @Override
    public void start() {
        // Hand motor control to the driver
        follower.startTeleopDrive();
    }

    // =========================================================================
    // LOOP
    // =========================================================================
    @Override
    public void loop() {
        // Always call every loop
        follower.update();
        telemetryM.update();

        // -----------------------------------------------------------------
        // BUTTON TOGGLES
        // -----------------------------------------------------------------
        if (gamepad1.aWasPressed()) slowModeOn     = !slowModeOn;
        if (gamepad1.bWasPressed()) fieldCentricOn = !fieldCentricOn;

        // -----------------------------------------------------------------
        // MANUAL DRIVE
        // -----------------------------------------------------------------
        if (!automatedDrive) {
            double y  = processAxis(-gamepad1.left_stick_y);
            double x  = processAxis(-gamepad1.left_stick_x) * STRAFE_CORRECTION;
            double rx = processAxis(-gamepad1.right_stick_x);

            if (slowModeOn) {
                y  *= SLOW_MODE_MULTIPLIER;
                x  *= SLOW_MODE_MULTIPLIER;
                rx *= SLOW_MODE_MULTIPLIER;
            }

            // false = field-centric, true = robot-centric
            follower.setTeleOpDrive(y, x, rx, !fieldCentricOn);
        }

        // -----------------------------------------------------------------
        // AUTOMATED PATH (right bumper triggers, left bumper cancels)
        // -----------------------------------------------------------------
        if (gamepad1.rightBumperWasPressed()) {
            // Change this Pose to wherever you want the robot to snap to
            Path autoPath = new Path(new BezierLine(
                    follower.getPose(),
                    new Pose(45, 98, 0)
            ));
            autoPath.setConstantHeadingInterpolation(0);
            follower.followPath(autoPath);
            automatedDrive = true;
        }

        // Return to manual when path finishes or driver cancels
        if (automatedDrive && (gamepad1.leftBumperWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        // -----------------------------------------------------------------
        // TELEMETRY
        // -----------------------------------------------------------------
        telemetry.addData("Status",         "Running");
        telemetry.addData("Slow Mode",       slowModeOn     ? "ON"           : "OFF");
        telemetry.addData("Drive Mode",      fieldCentricOn ? "Field-Centric" : "Robot-Centric");
        telemetry.addData("Automated Drive", automatedDrive ? "ON"           : "OFF");
        telemetry.addLine("--- Position ---");
        telemetry.addData("X",              "%.2f", follower.getPose().getX());
        telemetry.addData("Y",              "%.2f", follower.getPose().getY());
        telemetry.addData("Heading (deg)",  "%.1f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

} // end class MecanumTeleOp