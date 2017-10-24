package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by evandehlinger on 10/24/17.
 */
@TeleOp(name = "TeleOp Mode", group = "Bit Boi")
//@Disabled
public class bitBoiTeleOp extends OpMode{
    DcMotor motor1;
    DcMotor motor2;

    public void init() {
        motor1=hardwareMap.dcMotor.get("motor_1");
        motor2=hardwareMap.dcMotor.get("motor_2");

        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop() {
        int x =1;
        int y = 1;
        if(gamepad1.left_stick_y<0){
            x=-1;
        }
        if(gamepad1.right_stick_y<0){
            y=-1;
        }
        motor1.setPower(Math.pow(y*gamepad1.left_stick_y,2));
        motor2.setPower(Math.pow(x*gamepad1.right_stick_y,2));

    }
}
