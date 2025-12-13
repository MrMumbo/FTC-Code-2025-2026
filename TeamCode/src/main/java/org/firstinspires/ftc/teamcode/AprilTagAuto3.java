package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.HwChasis;

@Autonomous(name = "Far Auto", group = "Auto")
public class AprilTagAuto3 extends OpMode {
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private HwChasis hwchasis = new HwChasis();
    private boolean running;
    private int inte = 0;

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
        telemetry.addLine("AprilTag webcam init complete");
        telemetry.update();
        hwchasis.init(hardwareMap);
        hwchasis.holdLeft.setPosition(0);
        hwchasis.holdRight.setPosition(0);
    }

    @Override
    public void start(){
        running = true;
        hwchasis.frontRight.setPower(-0.5);
        hwchasis.frontLeft.setPower(0.5);
        hwchasis.backRight.setPower(0.5);
        hwchasis.backLeft.setPower(0.5);
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        hwchasis.frontRight.setPower(0);
        hwchasis.frontLeft.setPower(0);
        hwchasis.backRight.setPower(0);
        hwchasis.backLeft.setPower(0);
    }

    @Override
    public void loop() {
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}
