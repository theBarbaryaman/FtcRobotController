package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * MecanumTeleOp - A competition-ready TeleOp LinearOpMode for a 4-wheel Mecanum drive robot.
 *
 * GAMEPAD 1 CONTROLS:
 *   Left Stick  Y-axis      → Forward / Backward translation
 *   Left Stick  X-axis      → Strafe left / right
 *   Right Stick X-axis      → Rotate (turn) left / right
 *   Right Trigger (held)    → Slow mode (~40% power) for precise control
 *
 * HARDWARE MAP NAMES (must match exactly in the Driver Station configuration):
 *   "left_front_motor"  → Front-left  Mecanum wheel
 *   "left_back_motor"   → Rear-left   Mecanum wheel
 *   "right_front_motor" → Front-right Mecanum wheel
 *   "right_back_motor"  → Rear-right  Mecanum wheel
 *
 * PHYSICAL SETUP CHECKLIST (verify on robot before first run):
 *   1. Mecanum rollers must form an X pattern when viewed from above (not O pattern).
 *   2. If robot moves in wrong direction, flip FORWARD/REVERSE on the relevant side.
 *   3. Verify with these manual tests after deploying:
 *        - Left stick forward  → robot drives straight forward
 *        - Left stick right    → robot strafes right with no rotation
 *        - Right stick right   → robot spins clockwise in place
 */
@TeleOp(name = "Mecanum TeleOp", group = "TeleOp")
public class MecanumTeleOp extends LinearOpMode {

    // -------------------------------------------------------------------------
    // Hardware declarations
    // -------------------------------------------------------------------------
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;

    // -------------------------------------------------------------------------
    // Constants
    // -------------------------------------------------------------------------

    // Joystick inputs below this threshold are treated as zero.
    // Prevents slow unintended motor creep caused by physical gamepad drift.
    private static final double DEADZONE = 0.05;

    // Power multiplier applied when slow mode is active (right trigger held).
    // Reduce further (e.g. 0.25) if more precision is needed.
    private static final double SLOW_MODE_MULTIPLIER = 0.4;

    // Right trigger must exceed this threshold to activate slow mode.
    // Adjust lower (e.g. 0.2) for a more hair-trigger slow mode activation.
    private static final double SLOW_MODE_TRIGGER_THRESHOLD = 0.5;

    @Override
    public void runOpMode() {

        // -------------------------------------------------------------------------
        // 1. HARDWARE MAP INITIALIZATION
        //    Retrieve each motor from the hardware map using the exact config names.
        // -------------------------------------------------------------------------
        leftFrontMotor  = hardwareMap.get(DcMotor.class, "left_front_motor");
        leftBackMotor   = hardwareMap.get(DcMotor.class, "left_back_motor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front_motor");
        rightBackMotor  = hardwareMap.get(DcMotor.class, "right_back_motor");

        // -------------------------------------------------------------------------
        // 2. MOTOR DIRECTIONS
        //    Left motors run FORWARD, right motors run REVERSE so that positive
        //    power moves all wheels in the direction that drives the robot forward.
        //
        //    If your robot drives incorrectly on first run, swap FORWARD/REVERSE
        //    on the side that is spinning the wrong way.
        // -------------------------------------------------------------------------
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        // -------------------------------------------------------------------------
        // 3. RUN MODE
        //    RUN_WITHOUT_ENCODER puts the motor controller in direct power mode.
        //    Explicitly setting this prevents erratic behavior if a previous OpMode
        //    left motors in RUN_TO_POSITION or RUN_USING_ENCODER mode.
        // -------------------------------------------------------------------------
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // -------------------------------------------------------------------------
        // 4. BRAKE BEHAVIOR
        //    BRAKE makes motors resist movement when power is zero, giving the
        //    driver tighter and more predictable stopping control.
        // -------------------------------------------------------------------------
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // -------------------------------------------------------------------------
        // 5. PRE-START TELEMETRY
        //    Displayed on the Driver Station after INIT and before PLAY is pressed.
        // -------------------------------------------------------------------------
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls",
                "LS-Y: fwd/back | LS-X: strafe | RS-X: rotate | RT: slow mode");
        telemetry.update();

        // Wait for the driver to press PLAY on the Driver Station.
        waitForStart();

        // -------------------------------------------------------------------------
        // 6. MAIN TELEOP LOOP
        //    Runs repeatedly until the driver presses STOP.
        // -------------------------------------------------------------------------
        while (opModeIsActive()) {

            // -----------------------------------------------------------------
            // 6a. READ JOYSTICK INPUTS WITH DEADZONE
            //
            //   axial   – forward/backward   (left stick Y, negated because the
            //             SDK reports "up" as a negative Y value)
            //   lateral – strafe left/right  (left stick X)
            //   yaw     – rotate left/right  (right stick X)
            //
            //   Inputs below DEADZONE are zeroed to prevent unintended creep
            //   from physical gamepad imprecision.
            // -----------------------------------------------------------------
            double axial   = Math.abs(gamepad1.left_stick_y)  > DEADZONE ? -gamepad1.left_stick_y  : 0.0;
            double lateral = Math.abs(gamepad1.left_stick_x)  > DEADZONE ?  gamepad1.left_stick_x  : 0.0;
            double yaw     = Math.abs(gamepad1.right_stick_x) > DEADZONE ?  gamepad1.right_stick_x : 0.0;

            // -----------------------------------------------------------------
            // 6b. SLOW MODE
            //
            //   Holding the right trigger reduces all motor power to
            //   SLOW_MODE_MULTIPLIER (40%) for precise positioning.
            //   Applied after normalization to keep the scaling logic clean.
            // -----------------------------------------------------------------
            double speedMultiplier = gamepad1.right_trigger > SLOW_MODE_TRIGGER_THRESHOLD
                    ? SLOW_MODE_MULTIPLIER
                    : 1.0;

            // -----------------------------------------------------------------
            // 6c. MECANUM KINEMATICS
            //
            //   Standard Mecanum holonomic drive mixing equations.
            //   Roller geometry at 45° produces these power combinations:
            //
            //     Left  Front = axial + lateral + yaw
            //     Right Front = axial - lateral - yaw
            //     Left  Back  = axial - lateral + yaw
            //     Right Back  = axial + lateral - yaw
            //
            //   Sign explanation:
            //     axial   → (+) all four wheels          (forward/backward)
            //     lateral → (+) LF & RB, (-) RF & LB    (strafe)
            //     yaw     → (+) left side, (-) right side (rotation)
            // -----------------------------------------------------------------
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // -----------------------------------------------------------------
            // 6d. POWER NORMALIZATION
            //
            //   Find the largest absolute power across all four motors.
            //   If it exceeds 1.0, scale all four values down proportionally
            //   so the largest motor gets exactly 1.0 and ratios are preserved.
            //   If no value exceeds 1.0, denominator stays 1.0 — no change.
            // -----------------------------------------------------------------
            double denominator = Math.max(
                    Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower)),
                    Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower))
            );
            denominator = Math.max(denominator, 1.0);

            leftFrontPower  /= denominator;
            rightFrontPower /= denominator;
            leftBackPower   /= denominator;
            rightBackPower  /= denominator;

            // -----------------------------------------------------------------
            // 6e. APPLY SPEED MULTIPLIER
            //
            //   Multiplied after normalization so the denominator logic above
            //   is unaffected, and the final output cleanly scales to the
            //   desired speed range.
            // -----------------------------------------------------------------
            leftFrontPower  *= speedMultiplier;
            rightFrontPower *= speedMultiplier;
            leftBackPower   *= speedMultiplier;
            rightBackPower  *= speedMultiplier;

            // -----------------------------------------------------------------
            // 6f. SEND POWER TO MOTORS
            // -----------------------------------------------------------------
            leftFrontMotor.setPower(leftFrontPower);
            rightFrontMotor.setPower(rightFrontPower);
            leftBackMotor.setPower(leftBackPower);
            rightBackMotor.setPower(rightBackPower);

            // -----------------------------------------------------------------
            // 6g. TELEMETRY
            //    Updated every loop iteration. Shows driver-facing status,
            //    raw joystick inputs, and final normalized motor powers.
            // -----------------------------------------------------------------
            telemetry.addData("Status",    "Running");
            telemetry.addData("Slow Mode",  gamepad1.right_trigger > SLOW_MODE_TRIGGER_THRESHOLD
                    ? "ON (40%)" : "OFF (100%)");
            telemetry.addLine("--- Joystick Inputs ---");
            telemetry.addData("Axial   (fwd/back)", "%.2f", axial);
            telemetry.addData("Lateral (strafe)  ", "%.2f", lateral);
            telemetry.addData("Yaw     (rotate)  ", "%.2f", yaw);
            telemetry.addLine("--- Motor Powers (final) ---");
            telemetry.addData("Left  Front", "%.2f", leftFrontPower);
            telemetry.addData("Left  Back ", "%.2f", leftBackPower);
            telemetry.addData("Right Front", "%.2f", rightFrontPower);
            telemetry.addData("Right Back ", "%.2f", rightBackPower);
            telemetry.update();

        } // end while (opModeIsActive())

        // -------------------------------------------------------------------------
        // 7. SAFETY STOP
        //    Explicitly zero all motor powers when the OpMode ends.
        //    Good practice even though the SDK handles this automatically.
        // -------------------------------------------------------------------------
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

    } // end runOpMode()

} // end class MecanumTeleOp