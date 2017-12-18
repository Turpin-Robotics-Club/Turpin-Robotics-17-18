package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by evandehlinger on 12/13/17.
 */
@TeleOp(name = "GigaDrive", group = "Giga Boi")
public class gigaBoiTeleOp extends OpMode {

    DcMotor motorLB;
    DcMotor motorRB;
    DcMotor motorLF;
    DcMotor motorRF;
    ModernRoboticsI2cGyro gyro;

    public void init()
    {
        motorLF=hardwareMap.dcMotor.get("motor1");
        motorRF=hardwareMap.dcMotor.get("motor2");
        motorLB=hardwareMap.dcMotor.get("motor3");
        motorRB=hardwareMap.dcMotor.get("motor4");
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

        motorRF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRB.setDirection(DcMotorSimple.Direction.REVERSE);

        gyro.calibrate();

        gyro.resetZAxisIntegrator();
    }

    public void loop()
    {
        motorLB.setPower(gamepad1.right_stick_y);
        motorRB.setPower(gamepad1.right_stick_y);
        motorLF.setPower(gamepad1.right_stick_y);
        motorRF.setPower(gamepad1.right_stick_y);

        telemetry.addData("MotorLB:",gamepad1.left_stick_x);
        telemetry.addData("MotorRB:",(gamepad1.left_stick_y));
        telemetry.addData("MotorLF:",gamepad1.right_stick_x);
        telemetry.addData("MotorRF:",gamepad1.right_stick_y);


    }



}
