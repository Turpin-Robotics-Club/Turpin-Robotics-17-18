package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
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
    ModernRoboticsI2cGyro gyro;

    public void init() {
        motorTR=hardwareMap.dcMotor.get("motor1");
        motorTL=hardwareMap.dcMotor.get("motor2");
        motorR=hardwareMap.dcMotor.get("motor3");
        motorL=hardwareMap.dcMotor.get("motor4");
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

        motorR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorTR.setDirection(DcMotorSimple.Direction.REVERSE);

        gyro.calibrate();

        gyro.resetZAxisIntegrator();

    }

    public void loop() {

        double LTR;
        double RTL;
        if(gyro.rawZ()>0)
        {
            LTR = -Math.asin(gyro.getIntegratedZValue());
            RTL = Math.acos(gyro.getIntegratedZValue());
        }else{
            RTL = -Math.asin(gyro.getIntegratedZValue());
            LTR = Math.acos(gyro.getIntegratedZValue());
        }



        motorL.setPower((gamepad1.right_stick_y-gamepad1.right_stick_x-gamepad1.left_stick_x)*LTR);
        motorR.setPower((gamepad1.right_stick_y+gamepad1.right_stick_x+gamepad1.left_stick_x)*RTL);
        motorTL.setPower((gamepad1.right_stick_y/5+gamepad1.right_stick_x/5-gamepad1.left_stick_x/5)*RTL);
        motorTR.setPower((gamepad1.right_stick_y/5-gamepad1.right_stick_x/5+gamepad1.left_stick_x/5)*LTR);

        telemetry.addData("right stick y", gamepad1.right_stick_y);
        telemetry.addData("right stick x", gamepad1.right_stick_x);
        telemetry.addData("gyro z:",gyro.getIntegratedZValue());



    }
}
