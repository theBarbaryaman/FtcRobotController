package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Mecanum 4-Wheel Drive Op Mode
 *
 * Control Scheme:
 *   Left  Joystick X-axis -> Rotate / Heading (turn left/right)
 *   Right Joystick Y-axis -> Forward / Backward
 *   Right Joystick X-axis -> Strafe left / right
 */
@TeleOp(name = "Mecanum Drive", group = "TeleOp")
public class FourWheelOpMode extends LinearOpMode {

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;

    private static final double MAX_SPEED = 1.0;
    private static final double DEADZONE  = 0.05;

    @Override
    public void runOpMode() {

        // Hardware Map
        leftFrontMotor  = hardwareMap.get(DcMotor.class, "left_front_motor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front_motor");
        leftBackMotor   = hardwareMap.get(DcMotor.class, "left_back_motor");
        rightBackMotor  = hardwareMap.get(DcMotor.class, "right_back_motor");

        // Motor Directions — reverse left side so positive power = forward on all wheels
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Zero Power Behavior
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run Mode
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized - waiting for Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Read raw joystick values ---
            double rawDrive  = -gamepad1.right_stick_y; // negate: stick up = +1
            double rawStrafe =  gamepad1.right_stick_x;
            double rawRotate =  gamepad1.left_stick_x;

            // --- Deadzone: zero out drift below threshold, rescale rest to [0,1] ---
            double drive, strafe, rotate;

            if (Math.abs(rawDrive) < DEADZONE) {
                drive = 0.0;
            } else {
                drive = (rawDrive - Math.signum(rawDrive) * DEADZONE) / (1.0 - DEADZONE);
            }

            if (Math.abs(rawStrafe) < DEADZONE) {
                strafe = 0.0;
            } else {
                strafe = (rawStrafe - Math.signum(rawStrafe) * DEADZONE) / (1.0 - DEADZONE);
            }

            // Rotate: deadzone + cubic curve (x^3) to suppress accidental nudges
            //   stick 0.10 -> ~0.001 (nearly zero)
            //   stick 0.50 ->  0.125 (moderate)
            //   stick 1.00 ->  1.000 (full power, unchanged)
            double rotateDeadzoned;
            if (Math.abs(rawRotate) < DEADZONE) {
                rotateDeadzoned = 0.0;
            } else {
                rotateDeadzoned = (rawRotate - Math.signum(rawRotate) * DEADZONE) / (1.0 - DEADZONE);
            }
            rotate = rotateDeadzoned * rotateDeadzoned * rotateDeadzoned;

            // --- Mecanum wheel power equations ---
            // Pure forward (drive=1, strafe=0, rotate=0): all four motors = 1.0
            double leftFrontPower  = drive + strafe + rotate;
            double rightFrontPower = drive - strafe - rotate;
            double leftBackPower   = drive - strafe + rotate;
            double rightBackPower  = drive + strafe - rotate;

            // --- Normalize so no motor exceeds MAX_SPEED ---
            double maxPower = Math.max(
                    Math.max(Math.abs(leftFrontPower),  Math.abs(rightFrontPower)),
                    Math.max(Math.abs(leftBackPower),   Math.abs(rightBackPower))
            );
            if (maxPower > MAX_SPEED) {
                leftFrontPower  /= maxPower;
                rightFrontPower /= maxPower;
                leftBackPower   /= maxPower;
                rightBackPower  /= maxPower;
            }

            // Final safety clamp
            leftFrontPower  = Range.clip(leftFrontPower,  -MAX_SPEED, MAX_SPEED);
            rightFrontPower = Range.clip(rightFrontPower, -MAX_SPEED, MAX_SPEED);
            leftBackPower   = Range.clip(leftBackPower,   -MAX_SPEED, MAX_SPEED);
            rightBackPower  = Range.clip(rightBackPower,  -MAX_SPEED, MAX_SPEED);

            // --- Send power to motors ---
            leftFrontMotor.setPower(leftFrontPower);
            rightFrontMotor.setPower(rightFrontPower);
            leftBackMotor.setPower(leftBackPower);
            rightBackMotor.setPower(rightBackPower);

            // --- Telemetry ---
            telemetry.addData("-- Raw Inputs ----------", "");
            telemetry.addData("Raw Drive",  "%.3f", rawDrive);
            telemetry.addData("Raw Strafe", "%.3f", rawStrafe);
            telemetry.addData("Raw Rotate", "%.3f", rawRotate);
            telemetry.addData("-- Processed -----------", "");
            telemetry.addData("Drive",  "%.3f", drive);
            telemetry.addData("Strafe", "%.3f", strafe);
            telemetry.addData("Rotate", "%.3f", rotate);
            telemetry.addData("-- Motor Powers --------", "");
            telemetry.addData("Left  Front", "%.2f", leftFrontPower);
            telemetry.addData("Right Front", "%.2f", rightFrontPower);
            telemetry.addData("Left  Back",  "%.2f", leftBackPower);
            telemetry.addData("Right Back",  "%.2f", rightBackPower);
            telemetry.update();
        }

        // Stop all motors on exit
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
}