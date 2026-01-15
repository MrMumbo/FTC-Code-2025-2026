package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

public class HwChasis {
    // define motors
    public DcMotor backRight;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor frontLeft;
    public LED LED1red;
    public LED LED2red;
    public LED LED3red;
    public LED LED4red;
    public LED LED1green;
    public LED LED2green;
    public LED LED3green;
    public LED LED4green;
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
        LED1red = hwMap.get(LED.class,"LED1red");
        LED2red = hwMap.get(LED.class,"LED2red");
        LED3red = hwMap.get(LED.class,"LED3red");
        LED4red = hwMap.get(LED.class,"LED4red");
        LED1green = hwMap.get(LED.class,"LED1green");
        LED2green = hwMap.get(LED.class,"LED2green");
        LED3green = hwMap.get(LED.class,"LED3green");
        LED4green = hwMap.get(LED.class,"LED4green");

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