package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.configurables.annotations.Configurable;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;


/**
 * MecanumTeleOp - Improved competition TeleOp with:
 *   - Cubic input curve  (smoother low-speed control, fixes twitchiness)
 *   - Strafe correction  (lateral power boost for balanced feel)
 *   - Toggle slow mode   (A button press, not held trigger)
 *   - Field-centric mode (B button toggles; stick direction = field direction)
 *
 * GAMEPAD 1 CONTROLS:
 *   Left Stick  Y-axis   → Forward / Backward
 *   Left Stick  X-axis   → Strafe left / right
 *   Right Stick X-axis   → Rotate left / right
 *   A (press)            → Toggle slow mode (~40% power)
 *   B (press)            → Toggle field-centric / robot-centric driving
 *   Right Trigger        → Reset IMU heading (re-zero field orientation)
 *
 * HARDWARE MAP NAMES:
 *   "left_front_motor"   "left_back_motor"
 *   "right_front_motor"  "right_back_motor"
 *   "imu"                (built-in REV Hub IMU, required for field-centric)
 *
 * PHYSICAL SETUP:
 *   1. Mecanum rollers must form an X when viewed from above.
 *   2. Set LOGO_FACING_UP / USB_FACING_FORWARD to match your hub orientation.
 *   3. Run manual drive tests (see original checklist) before competition.
 */
@TeleOp(name = "Mecanum TeleOp", group = "TeleOp")
public class MecanumTeleOp extends LinearOpMode {

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------
    private DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
    private IMU imu;

    // -------------------------------------------------------------------------
    // Tuning constants — adjust these to taste
    // -------------------------------------------------------------------------

    /** Inputs below this value are treated as zero. Prevents gamepad drift. */
    private static final double DEADZONE = 0.05;

    /**
     * Strafe correction factor.
     * Mecanum wheels produce less lateral force than forward/backward force.
     * Values 0.8–1.0 typical; increase if strafing feels sluggish vs. forward.
     */
    private static final double STRAFE_CORRECTION = 0.85;

    /**
     * Power multiplier when slow mode is ON.
     * Lower = more precise; 0.25 for very fine control, 0.5 for moderate.
     */
    private static final double SLOW_MODE_MULTIPLIER = 0.4;

    /**
     * REV Hub orientation — change these two lines if your hub is mounted
     * in a different direction. Options: LOGO_FACING_*, USB_FACING_*
     */
    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;      // ← change this
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT;  // ← and this

    // -------------------------------------------------------------------------
    // Runtime state
    // -------------------------------------------------------------------------
    private boolean slowModeOn       = false;
    private boolean fieldCentricOn   = true;   // start field-centric by default
    private boolean prevA            = false;   // previous A-button state (edge detect)
    private boolean prevB            = false;   // previous B-button state (edge detect)
    private boolean prevRightTrigger = false;   // previous trigger state  (edge detect)

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    /**
     * Applies a cubic curve to joystick input.
     *
     * Why cubic?  A linear stick maps 10% deflection → 10% power (twitchy).
     * Cubing it maps 10% → 0.1% and 100% → 100%, so small movements are far
     * gentler while full deflection still delivers maximum power.
     *
     * @param raw raw joystick value in [-1, 1]
     * @return curved value, same sign, in [-1, 1]
     */
    private double applyCurve(double raw) {
        return raw * raw * raw;   // x³ preserves sign automatically
    }

    /**
     * Applies deadzone then cubic curve to a single joystick axis.
     * Inputs with |value| <= DEADZONE are zeroed before curving.
     */
    private double processAxis(double raw) {
        return Math.abs(raw) > DEADZONE ? applyCurve(raw) : 0.0;
    }

    /**
     * Rising-edge detector — returns true only on the frame a button first
     * becomes pressed, preventing a single hold from toggling multiple times.
     *
     * @param current  current button state this loop iteration
     * @param previous button state from the previous iteration
     */
    private boolean risingEdge(boolean current, boolean previous) {
        return current && !previous;
    }

    // =========================================================================
    @Override
    public void runOpMode() {

        // -------------------------------------------------------------------------
        // 1. MOTOR INIT
        // -------------------------------------------------------------------------
        leftFrontMotor  = hardwareMap.get(DcMotor.class, "left_front_motor");
        leftBackMotor   = hardwareMap.get(DcMotor.class, "left_back_motor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front_motor");
        rightBackMotor  = hardwareMap.get(DcMotor.class, "right_back_motor");

        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // -------------------------------------------------------------------------
        // 2. IMU INIT (required for field-centric mode)
        //    If you don't have an IMU or don't want field-centric, you can safely
        //    delete this block and set fieldCentricOn = false above.
        // -------------------------------------------------------------------------
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(LOGO_DIR, USB_DIR)
        ));
        imu.resetYaw();   // define "forward" as wherever the robot faces at INIT

        // -------------------------------------------------------------------------
        // 3. PRE-START TELEMETRY
        // -------------------------------------------------------------------------
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls",
                "LS: drive/strafe | RS-X: rotate | A: slow toggle | B: field-centric toggle | RT: reset heading");
        telemetry.update();

        waitForStart();

        // =========================================================================
        // 4. MAIN TELEOP LOOP
        // =========================================================================
        while (opModeIsActive()) {

            // -----------------------------------------------------------------
            // 4a. BUTTON TOGGLE — Slow Mode (A) and Field-Centric (B)
            //     Rising-edge detection ensures one press = one toggle.
            // -----------------------------------------------------------------
            boolean currA = gamepad1.a;
            boolean currB = gamepad1.b;
            boolean currRightTrigger = gamepad1.right_trigger > 0.5;

            if (risingEdge(currA, prevA)) {
                slowModeOn = !slowModeOn;
            }
            if (risingEdge(currB, prevB)) {
                fieldCentricOn = !fieldCentricOn;
            }
            // Right trigger resets the IMU heading so current facing = "forward"
            if (risingEdge(currRightTrigger, prevRightTrigger)) {
                imu.resetYaw();
            }

            prevA            = currA;
            prevB            = currB;
            prevRightTrigger = currRightTrigger;

            // -----------------------------------------------------------------
            // 4b. READ & PROCESS JOYSTICK INPUTS
            //     Deadzone + cubic curve applied to every axis.
            //     Lateral axis also gets STRAFE_CORRECTION to compensate for
            //     the Mecanum wheel's reduced lateral efficiency.
            // -----------------------------------------------------------------
            double axial   = processAxis( gamepad1.left_stick_y);          // fwd positive
            double lateral = processAxis( gamepad1.left_stick_x) * STRAFE_CORRECTION;
            double yaw     = processAxis( gamepad1.right_stick_x);

            // -----------------------------------------------------------------
            // 4c. FIELD-CENTRIC ROTATION
            //     Rotate the axial/lateral vector by the robot's current heading
            //     so the driver always pushes "away" regardless of robot facing.
            //     Yaw (rotation) is never rotated — it's always relative to the robot.
            // -----------------------------------------------------------------
            if (fieldCentricOn) {
                double heading = imu.getRobotYawPitchRollAngles()
                        .getYaw(AngleUnit.RADIANS);
                // Rotate input vector by -heading to convert field coords → robot coords
                double rotatedAxial   =  axial   * Math.cos(-heading) - lateral * Math.sin(-heading);
                double rotatedLateral =  axial   * Math.sin(-heading) + lateral * Math.cos(-heading);
                axial   = rotatedAxial;
                lateral = rotatedLateral;
            }

            // -----------------------------------------------------------------
            // 4d. MECANUM KINEMATICS
            // -----------------------------------------------------------------
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // -----------------------------------------------------------------
            // 4e. NORMALIZE so no motor exceeds ±1.0 (preserves ratios)
            // -----------------------------------------------------------------
            double denominator = Math.max(
                    Math.max(Math.abs(leftFrontPower),  Math.abs(leftBackPower)),
                    Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower))
            );
            denominator = Math.max(denominator, 1.0);

            leftFrontPower  /= denominator;
            rightFrontPower /= denominator;
            leftBackPower   /= denominator;
            rightBackPower  /= denominator;

            // -----------------------------------------------------------------
            // 4f. SLOW MODE SCALE (applied after normalization)
            // -----------------------------------------------------------------
            if (slowModeOn) {
                leftFrontPower  *= SLOW_MODE_MULTIPLIER;
                rightFrontPower *= SLOW_MODE_MULTIPLIER;
                leftBackPower   *= SLOW_MODE_MULTIPLIER;
                rightBackPower  *= SLOW_MODE_MULTIPLIER;
            }

            // -----------------------------------------------------------------
            // 4g. SEND POWER
            // -----------------------------------------------------------------
            leftFrontMotor.setPower(leftFrontPower);
            rightFrontMotor.setPower(rightFrontPower);
            leftBackMotor.setPower(leftBackPower);
            rightBackMotor.setPower(rightBackPower);

            // -----------------------------------------------------------------
            // 4h. TELEMETRY
            // -----------------------------------------------------------------
            telemetry.addData("Status",        "Running");
            telemetry.addData("Slow Mode",      slowModeOn     ? "ON (A to toggle)"  : "OFF (A to toggle)");
            telemetry.addData("Drive Mode",     fieldCentricOn ? "Field-Centric (B)"  : "Robot-Centric (B)");
            telemetry.addData("Heading (deg)",  "%.1f",
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addLine("--- Inputs (after curve) ---");
            telemetry.addData("Axial   (fwd/back)", "%.2f", axial);
            telemetry.addData("Lateral (strafe)  ", "%.2f", lateral);
            telemetry.addData("Yaw     (rotate)  ", "%.2f", yaw);
            telemetry.addLine("--- Motor Powers ---");
            telemetry.addData("Left  Front", "%.2f", leftFrontPower);
            telemetry.addData("Left  Back ", "%.2f", leftBackPower);
            telemetry.addData("Right Front", "%.2f", rightFrontPower);
            telemetry.addData("Right Back ", "%.2f", rightBackPower);
            telemetry.update();

        } // end while (opModeIsActive())

        // -------------------------------------------------------------------------
        // 5. SAFETY STOP
        // -------------------------------------------------------------------------
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

    } // end runOpMode()

} // end class MecanumTeleOp