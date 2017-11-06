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
    DcMotor motorR;
    DcMotor motorC;
    OpticalDistanceSensor odSensor;

    public void init() {
        motorL=hardwareMap.dcMotor.get("motor_1");
        motorR=hardwareMap.dcMotor.get("motor_2");
        motorC=hardwareMap.dcMotor.get("motor_3");
        odSensor = hardwareMap.opticalDistanceSensor.get("od_sensor");
        motorR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop() {

        if(gamepad1.right_stick_y<0) {
            if (gamepad1.right_stick_x > 0) {
                motorL.setPower(gamepad1.right_stick_y - gamepad1.right_stick_x );
                motorR.setPower(1*Math.pow(gamepad1.right_stick_y,3) );
            } else {
                motorL.setPower(1*Math.pow(gamepad1.right_stick_y,3) );
                motorR.setPower(gamepad1.right_stick_y + gamepad1.right_stick_x);
            }

        }else{
            if (gamepad1.right_stick_x > 0) {
                motorL.setPower(gamepad1.right_stick_y + gamepad1.right_stick_x );
                motorR.setPower(Math.pow(gamepad1.right_stick_y,3)  );
            } else {
                motorR.setPower(gamepad1.right_stick_y  + (-1 * gamepad1.right_stick_x ));
                motorL.setPower(Math.pow(gamepad1.right_stick_y,3) );
            }
        }
        motorC.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
        telemetry.addData("Raw",    odSensor.getRawLightDetected());
        telemetry.addData("Normal", odSensor.getLightDetected());



    }
}
