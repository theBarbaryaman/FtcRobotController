package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Intake Motor Test Op Mode
 *
 * Control Scheme:
 *   D-Pad UP -> Run intake at steady speed
 */
@TeleOp(name = "Test Intake", group = "Test")
public class TestIntake extends LinearOpMode {

    private DcMotor intakeMotor;

    private static final double INTAKE_SPEED = 0.8;

    @Override
    public void runOpMode() {

        // Hardware Map
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");

        // Motor Direction — change to REVERSE if intake spins the wrong way
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Zero Power Behavior
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run Mode
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized - waiting for Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Hold D-Pad UP to run the intake
            if (gamepad1.dpad_up) {
                intakeMotor.setPower(INTAKE_SPEED);
            } else {
                intakeMotor.setPower(0);
            }

            // --- Telemetry ---
            telemetry.addData("-- Intake --------------", "");
            telemetry.addData("D-Pad UP",     gamepad1.dpad_up);
            telemetry.addData("Motor Power",  "%.2f", intakeMotor.getPower());
            telemetry.addData("Status",       gamepad1.dpad_up ? "RUNNING" : "STOPPED");
            telemetry.update();
        }

        // Stop motor on exit
        intakeMotor.setPower(0);
    }
}