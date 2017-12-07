package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


/**
 * Created by evandehlinger on 10/24/17.
 */
@TeleOp(name = "TeleOp Mode", group = "Bit Boi")
//@Disabled
public class bitBoiTeleOp extends OpMode{
    DcMotor motorL;
    DcMotor motorTL;
    DcMotor motorR;
    DcMotor motorTR;

    public void init() {
        motorTR=hardwareMap.dcMotor.get("motor1");
        motorTL=hardwareMap.dcMotor.get("motor2");
        motorR=hardwareMap.dcMotor.get("motor3");
        motorL=hardwareMap.dcMotor.get("motor4");


        motorR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorTR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop() {

        motorL.setPower(gamepad1.right_stick_y);
        motorR.setPower(gamepad1.right_stick_y);
        motorTL.setPower(gamepad1.right_stick_y/5);
        motorTR.setPower(gamepad1.right_stick_y/5);

        telemetry.addData("right stick y", gamepad1.right_stick_y);
        telemetry.addData("right stick x", gamepad1.right_stick_x);



    }
}
