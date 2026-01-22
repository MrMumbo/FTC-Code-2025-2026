package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Camera Calibration Capture", group = "Vision")
public class CameraCalibrationCapture extends LinearOpMode {

    private VisionPortal visionPortal;
    private FrameCaptureProcessor capture;

    @Override
    public void runOpMode() {

        capture = new FrameCaptureProcessor();
        visionPortal = WebcamConfig.buildVisionPortal(hardwareMap, capture);

        telemetry.addLine("Press A to save image");
        telemetry.addLine("Images: /sdcard/FIRST/calib/");
        telemetry.update();

        waitForStart();

        boolean lastA = false;

        while (opModeIsActive()) {
            boolean a = gamepad1.a;

            // Rising edge: only triggers once per press
            if (a && !lastA) {
                capture.requestSaveNextFrame();
            }
            lastA = a;

            telemetry.addData("Images saved", capture.getImageCount());
            telemetry.addData("Last saved", capture.getLastSavedPath());
            telemetry.addData("Last error", capture.getLastError());
            telemetry.update();
        }

        // Clean shutdown
        if (visionPortal != null) visionPortal.close();
        if (capture != null) capture.shutdown();
    }
}
