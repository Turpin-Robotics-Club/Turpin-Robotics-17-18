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
    DcMotor motorL;
    DcMotor motorR;

    public void init() {
        motorL=hardwareMap.dcMotor.get("motor_1");
        motorR=hardwareMap.dcMotor.get("motor_2");

        motorR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop() {

        //if(gamepad1.right_stick_y>0) {
            if (gamepad1.right_stick_x < 0) {
                motorL.setPower(gamepad1.right_stick_y );
                motorR.setPower((gamepad1.right_stick_y ) + (  gamepad1.right_stick_x ));
            } else {
                motorL.setPower((gamepad1.right_stick_y ) + (-1 * gamepad1.right_stick_x ));
                motorR.setPower(gamepad1.right_stick_y );
            }
        /*}else{
            if (gamepad1.right_stick_x > 0) {
                motorL.setPower(gamepad1.right_stick_y );
                motorR.setPower((gamepad1.right_stick_y ) + (  gamepad1.right_stick_x ));
            } else {
                motorL.setPower((gamepad1.right_stick_y ) + (-1 * gamepad1.right_stick_x ));
                motorR.setPower(gamepad1.right_stick_y );
            }*/
        //}



    }
}
