package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "AprilTag Auto Debug", group = "Vision")
public class AprilTagAuto extends OpMode {
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
        telemetry.addLine("AprilTag webcam init complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update detections from the camera
        aprilTagWebcam.update();

        // Get *all* current detections
        List<AprilTagDetection> detections = aprilTagWebcam.getDetectedTags();

        telemetry.addData("Detected tags", detections.size());

        if (detections.isEmpty()) {
            telemetry.addLine("No tags detected this frame");
        } else {
            // Print info for each detected tag
            for (AprilTagDetection detection : detections) {
                aprilTagWebcam.displayDetectionTelemetry(detection);
            }
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}
