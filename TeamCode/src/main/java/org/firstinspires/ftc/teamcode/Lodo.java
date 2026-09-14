package org.firstinspires.ftc.teamcode;
// Limelight camera function for

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.lang.Math;

// to run in the main function Lodo lodo = new Lodo(hardwareMap);  hardwareMap exists here, because this IS the OpMode
//double[] ballCoords = lodo.getBall(roboCoords);

public class Lodo {
    private Limelight3A limelight;
    private boolean initialized = false;
    private double HEIGHT = 16.94; // cm — camera-mount height used in the distance calc below
    private String lastStatus = "not yet called";

    // Camera's physical position relative to the robot's odometry center (the point
    // x0/y0/theta refer to), measured directly off the chassis, in cm.
    // CAMERA_FORWARD_OFFSET: positive = camera is forward of the odometry center.
    // CAMERA_LATERAL_OFFSET: positive = camera is left of the odometry center.
    // MEASURE THESE on your robot and update — currently 0.0 placeholders.
    private static final double CAMERA_FORWARD_OFFSET = 22.5;
    private static final double CAMERA_LATERAL_OFFSET = 0.0;

    public String getLastStatus() {
        return lastStatus;
    }

    public double[] getBall(double[] roboCoords, HardwareMap hardwareMap){
        /* roboCoords is the length three array showing the coordinates on the odometry plane
        (in cm) and the angle theta from the x-axis counter-clockwise

        */

        double x0 = roboCoords[0];
        double y0 = roboCoords[1];
        double theta = roboCoords[2];

        // Only set up the Limelight once. Calling pipelineSwitch()/start() on every
        // loop iteration (as before) kept restarting the pipeline and never let it
        // settle into producing a valid result.
        if (!initialized) {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0); // whichever pipeline index detects the ball
            limelight.start();
            initialized = true;
        }

        // get camera input
        LLResult result = limelight.getLatestResult();

        if (result == null) {
            lastStatus = "result is null (no data from camera yet)";
            return new double[]{};
        }

        if (!result.isValid()) {
            lastStatus = "result invalid (no target detected by pipeline)";
            return new double[]{};
        }

        {
            // calculate relative position
            double tx = result.getTx();
            double ty = result.getTy();

            lastStatus = "valid: tx=" + tx + " ty=" + ty;

            // calculate forward ground distance to target (cm).
            // Per diagram: angle at camera = (69 - ty) degrees, between vertical
            // side h (adjacent) and the hypotenuse; delta_x (opposite) is the
            // ground distance. tan(angle) = delta_x / h  =>  delta_x = h * tan(angle)
            double angleRad = ((69.0 + ty) / 180.0) * Math.PI;
            double delta_x = HEIGHT * Math.tan(angleRad);

            double delta_y; // cm
            delta_y = delta_x * Math.tan(-(tx / 180.0) * Math.PI);

            // account for the camera not being physically at the robot's odometry center
            double totalForward = delta_x - CAMERA_FORWARD_OFFSET;
            double totalLateral = delta_y + CAMERA_LATERAL_OFFSET;

            // adjust for angle

            double xB = x0 + totalForward * Math.cos(theta) - totalLateral * Math.sin(theta);
            double yB = y0 + totalForward * Math.sin(theta) + totalLateral * Math.cos(theta);


            double heading = Math.atan2(delta_y, delta_x) + theta;
            return new double[]{xB, yB, Math.toDegrees(heading)};
        }
    }
}