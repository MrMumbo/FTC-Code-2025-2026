package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.HwChasis;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name="AprilTag Follow", group="Robot")
public class AprilTagFollow extends OpMode {

    private AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private HwChasis hwchasis = new HwChasis();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
        hwchasis.init(hardwareMap);

        hwchasis.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hwchasis.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hwchasis.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hwchasis.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hwchasis.backRight.setDirection(DcMotor.Direction.REVERSE);
        hwchasis.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        //apriltag
        aprilTagWebcam.update();

        java.util.List<AprilTagDetection> detections = aprilTagWebcam.getDetectedTags();
        telemetry.addData("Tag count", detections.size());

        if (!detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);
            aprilTagWebcam.displayDetectionTelemetry(tag);

            telemetry.addData("Tag ID", tag.id);
            telemetry.addData("Tag range (in)", tag.ftcPose.range);
        } else {
            telemetry.addLine("No tags detected");
        }

        float rx;

        if(detections.isEmpty()){
            rx = 0.0f;
        } else {
            if(aprilTagWebcam.rotationToAprilTag > 0){
                rx = 0.5f;
            } else {
                rx = -0.5f;
            }
        }

        if(!gamepad1.left_bumper){
            float horizantalMovement = gamepad1.left_stick_y;
            float verticalMovement   =  -gamepad1.left_stick_x;
            float rotationalMovement =  gamepad1.right_stick_x;

            // drive system (strafe, turn, forward/backward)
            hwchasis.backLeft.setPower( (-verticalMovement + horizantalMovement - rotationalMovement));
            hwchasis.frontLeft.setPower((-verticalMovement - horizantalMovement + rotationalMovement));
            hwchasis.backRight.setPower((-verticalMovement - horizantalMovement - rotationalMovement));
            hwchasis.frontRight.setPower( (verticalMovement - horizantalMovement - rotationalMovement));
        } else {
            if(aprilTagWebcam.distanceToAprilTag >= 200){
                hwchasis.backLeft.setPower(1.0f + rx);
                hwchasis.frontLeft.setPower(-1.0f - rx);
                hwchasis.backRight.setPower(-1.0f + rx);
                hwchasis.frontRight.setPower(-1.0f + rx);
            } else{
                hwchasis.backLeft.setPower(0.0f + rx);
                hwchasis.frontLeft.setPower(0.0f - rx);
                hwchasis.backRight.setPower(0.0f + rx);
                hwchasis.frontRight.setPower(0.0f + rx);
            }
        }
        telemetry.addData("rotation",  "%.2f", aprilTagWebcam.rotationToAprilTag);
        telemetry.addData("forward",    "%.2f", aprilTagWebcam.distanceToAprilTag);

        telemetry.update();
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}
