package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.HwChasis;

@Autonomous(name = "Close Auto BLUE", group = "Auto")
public class AprilTagAuto2 extends OpMode {
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
            sleep(400);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        hwchasis.frontRight.setPower(0);
        hwchasis.frontLeft.setPower(0);
        hwchasis.backRight.setPower(0);
        hwchasis.backLeft.setPower(0);

        while (running){
            if (-hwchasis.shootPower.getVelocity() < 1450){
                hwchasis.shootPower.setPower(1);
            } else {
                hwchasis.shootPower.setPower(0);
                shootServo();
                try {
                    sleep(2500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                inte ++;
            }
            if (inte == 4){
                hwchasis.frontRight.setPower(0.5);
                hwchasis.frontLeft.setPower(0.5);
                hwchasis.backRight.setPower(0.5);
                hwchasis.backLeft.setPower(-0.5);
                try {
                    sleep(1200);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                hwchasis.frontRight.setPower(0);
                hwchasis.frontLeft.setPower(0);
                hwchasis.backRight.setPower(0);
                hwchasis.backLeft.setPower(0);
                running = false;
                stop();
            }
        }
    }

    public void shootServo(){
        telemetry.addLine("reset");
        hwchasis.holdLeft.setPosition(0);
        hwchasis.holdRight.setPosition(0);
        telemetry.update();
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        telemetry.addLine("shooting");
        hwchasis.holdLeft.setPosition(1);
        hwchasis.holdRight.setPosition(1);
        telemetry.update();
    }

    @Override
    public void loop() {
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}
