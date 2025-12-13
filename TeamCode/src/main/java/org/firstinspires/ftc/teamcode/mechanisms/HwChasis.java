package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HwChasis {
    // define motors
    public DcMotor backRight;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor frontLeft;
    public DcMotorEx shootPower;
    public Servo holdLeft;
    public Servo holdRight;

    public double shootVel;

    public void init(HardwareMap hwMap) {
        backRight   = hwMap.get(DcMotor.class, "backRight");
        frontRight  = hwMap.get(DcMotor.class, "frontRight");
        backLeft    = hwMap.get(DcMotor.class, "backLeft");
        frontLeft   = hwMap.get(DcMotor.class, "frontLeft");
        shootPower  = hwMap.get(DcMotorEx.class, "shootPower");
        holdLeft    = hwMap.get(Servo.class,   "holdLeft");
        holdRight   = hwMap.get(Servo.class,   "holdRight");

        // set motor directions
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        holdLeft.setDirection(Servo.Direction.REVERSE);
        holdRight.setDirection(Servo.Direction.FORWARD);
        shootPower.setDirection(DcMotorEx.Direction.REVERSE);

        shootPower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // reset servos
        holdLeft.setPosition(0.00);
        holdRight.setPosition(0.00);
    }
}