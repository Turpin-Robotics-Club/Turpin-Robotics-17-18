package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.Sensors;
import org.firstinspires.ftc.teamcode.utils.oldSensors;


@TeleOp(name = "Mecanum Drive")
public class mecanumDrive extends OpMode {



    private double joy;
    private double joyLeft = 0;
    private double G1_Lstk_x;
    private double G1_Lstk_y;
    private double currentPos;
    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;
    private double flvalue;
    private double frvalue;
    private double blvalue;
    private double brvalue;
    private double relativeHeading;
    private double xmove;
    private double ymove;
    private float turnRate = 0.75f;
    private float driveRate = 1.5f;
    private double bumperPower = 0.3;
    private double spinRate = 0.1;

    private ElapsedTime runtime_y = new ElapsedTime();
    private ElapsedTime runtime_b = new ElapsedTime();

    public void init() {

        backright = hardwareMap.dcMotor.get("front_left");
        backleft = hardwareMap.dcMotor.get("front_right");
        frontright = hardwareMap.dcMotor.get("back_left");
        frontleft = hardwareMap.dcMotor.get("back_right");


        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);

        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        runtime_y.reset();
    }

    public void loop() {

        G1_Lstk_x = gamepad1.left_stick_x;
        G1_Lstk_y = gamepad1.left_stick_y;
        currentPos = Sensors.readGyro();


        telemetry.addData("Joystick value left", joyLeft);
        telemetry.addData("robot heading", currentPos);
        telemetry.addData("intention", relativeHeading);
        telemetry.addData("Gyro Heading", Sensors.angles.thirdAngle);
        telemetry.addData("Driver Offset", Sensors.gyroInitial);
        telemetry.update();

        //turn direction
        //starts at up, turns right
        if(gamepad1.right_stick_x >= 0 && gamepad1.right_stick_y < 0)
        {
            joy = Math.toDegrees(Math.atan2(Math.abs(gamepad1.right_stick_x), Math.abs(gamepad1.right_stick_y)));
            joy = joy + 0;
        }
        if(gamepad1.right_stick_x > 0 && gamepad1.right_stick_y >= 0)
        {
            joy = Math.toDegrees(Math.atan2(Math.abs(gamepad1.right_stick_y), Math.abs(gamepad1.right_stick_x)));
            joy = joy + 90;
        }
        if(gamepad1.right_stick_x <= 0 && gamepad1.right_stick_y > 0)
        {
            joy = Math.toDegrees(Math.atan2(Math.abs(gamepad1.right_stick_x), Math.abs(gamepad1.right_stick_y)));
            joy = joy + 180;
        }
        if(gamepad1.right_stick_x < 0 && gamepad1.right_stick_y <= 0)
        {
            joy = Math.toDegrees(Math.atan2(Math.abs(gamepad1.right_stick_y), Math.abs(gamepad1.right_stick_x)));
            joy = joy + 270;
        }


        //beginning of drive section
        if(G1_Lstk_x >= 0 && G1_Lstk_y < 0)
        {
            joyLeft = Math.toDegrees(Math.atan2(Math.abs(G1_Lstk_x), Math.abs(G1_Lstk_y)));
            joyLeft = joyLeft + 0;
        }
        if(G1_Lstk_x > 0 && G1_Lstk_y >= 0)
        {
            joyLeft = Math.toDegrees(Math.atan2(Math.abs(G1_Lstk_y), Math.abs(G1_Lstk_x)));
            joyLeft = joyLeft + 90;
        }
        if(G1_Lstk_x <= 0 && G1_Lstk_y > 0)
        {
            joyLeft = Math.toDegrees(Math.atan2(Math.abs(G1_Lstk_x), Math.abs(G1_Lstk_y)));
            joyLeft = joyLeft + 180;
        }
        if(G1_Lstk_x < 0 && G1_Lstk_y <= 0)
        {
            joyLeft = Math.toDegrees(Math.atan2(Math.abs(G1_Lstk_y), Math.abs(G1_Lstk_x)));
            joyLeft = joyLeft + 270;
        }






        //make it robot centric
        relativeHeading = joyLeft - currentPos;
        if(relativeHeading < 0)
        {
            relativeHeading = 360 + relativeHeading;
        }


        if(relativeHeading >= 0 && relativeHeading < 90)
        {
            //separate x and y                                                   set magnitude of vector                      set speed so it doesn't max
            //telemetry.addData("Quadrant", "1");
            xmove = Math.sin(Math.toRadians(relativeHeading)) * Math.pow(Math.sqrt(Math.pow(G1_Lstk_x, 2) + Math.pow(G1_Lstk_y, 2)), 2) * driveRate;
            ymove = Math.cos(Math.toRadians(relativeHeading)) * Math.pow(Math.sqrt(Math.pow(G1_Lstk_x, 2) + Math.pow(G1_Lstk_y, 2)), 2) * driveRate;
        }
        if(relativeHeading >= 90 && relativeHeading < 180)
        {
            //telemetry.addData("Quadrant", "4");
            xmove = Math.cos(Math.toRadians(relativeHeading-90)) * Math.pow(Math.sqrt(Math.pow(G1_Lstk_x, 2) + Math.pow(G1_Lstk_y, 2)), 2) * driveRate;
            ymove = -Math.sin(Math.toRadians(relativeHeading-90)) * Math.pow(Math.sqrt(Math.pow(G1_Lstk_x, 2) + Math.pow(G1_Lstk_y, 2)), 2) * driveRate;
        }
        if(relativeHeading >= 180 && relativeHeading < 270)
        {
            //telemetry.addData("Quadrant", "3");
            xmove = -Math.sin(Math.toRadians(relativeHeading-180)) * Math.pow(Math.sqrt(Math.pow(G1_Lstk_x, 2) + Math.pow(G1_Lstk_y, 2)), 2) * driveRate;
            ymove = -Math.cos(Math.toRadians(relativeHeading-180)) * Math.pow(Math.sqrt(Math.pow(G1_Lstk_x, 2) + Math.pow(G1_Lstk_y, 2)), 2) * driveRate;
        }
        if(relativeHeading >= 270 && relativeHeading < 360)
        {
            //telemetry.addData("Quadrant", "2");
            xmove = -Math.cos(Math.toRadians(relativeHeading-270)) * Math.pow(Math.sqrt(Math.pow(G1_Lstk_x, 2) + Math.pow(G1_Lstk_y, 2)), 2) * driveRate;
            ymove = Math.sin(Math.toRadians(relativeHeading-270)) * Math.pow(Math.sqrt(Math.pow(G1_Lstk_x, 2) + Math.pow(G1_Lstk_y, 2)), 2) * driveRate;
        }

        flvalue = (ymove + xmove);
        frvalue = (ymove - xmove);
        blvalue = (ymove - xmove);
        brvalue = (ymove + xmove);

        //End of drive
        //Beginning of turn

        /*
        if((joy < currentPos + 180 && joy > currentPos) || joy < (currentPos + 180) -360)
        {
            //left
            flvalue = flvalue + -Math.min(0.75, Math.pow(Math.sqrt(Math.pow(gamepad1.right_stick_x, 2) + Math.pow(gamepad1.right_stick_y, 2)) * turnRate, 2) * Math.pow((joy-currentPos), 2) * spinRate);
            frvalue = frvalue + Math.min(0.75, Math.pow(Math.sqrt(Math.pow(gamepad1.right_stick_x, 2) + Math.pow(gamepad1.right_stick_y, 2)) * turnRate, 2) * Math.pow((joy-currentPos), 2) * spinRate);
            blvalue = blvalue + -Math.min(0.75, Math.pow(Math.sqrt(Math.pow(gamepad1.right_stick_x, 2) + Math.pow(gamepad1.right_stick_y, 2)) * turnRate, 2) * Math.pow((joy-currentPos), 2) * spinRate);
            brvalue = brvalue + Math.min(0.75, Math.pow(Math.sqrt(Math.pow(gamepad1.right_stick_x, 2) + Math.pow(gamepad1.right_stick_y, 2)) * turnRate, 2) * Math.pow((joy-currentPos), 2) * spinRate);
        }
        if((joy > currentPos - 180 && joy < currentPos) || joy > 360 - (180 - currentPos))
        {
            //right
            flvalue = flvalue + Math.min(0.75, Math.pow(Math.sqrt(Math.pow(gamepad1.right_stick_x, 2) + Math.pow(gamepad1.right_stick_y, 2)) * turnRate, 2) * Math.pow((joy-currentPos), 2) * spinRate);
            frvalue = frvalue + -Math.min(0.75, Math.pow(Math.sqrt(Math.pow(gamepad1.right_stick_x, 2) + Math.pow(gamepad1.right_stick_y, 2)) * turnRate, 2) * Math.pow((joy-currentPos), 2) * spinRate);
            blvalue = blvalue + Math.min(0.75, Math.pow(Math.sqrt(Math.pow(gamepad1.right_stick_x, 2) + Math.pow(gamepad1.right_stick_y, 2)) * turnRate, 2) * Math.pow((joy-currentPos), 2) * spinRate);
            brvalue = brvalue + -Math.min(0.75, Math.pow(Math.sqrt(Math.pow(gamepad1.right_stick_x, 2) + Math.pow(gamepad1.right_stick_y, 2)) * turnRate, 2) * Math.pow((joy-currentPos), 2) * spinRate);
        }
        */
        flvalue += gamepad1.right_stick_x * turnRate;
        frvalue -= gamepad1.right_stick_x * turnRate;
        blvalue += gamepad1.right_stick_x * turnRate;
        brvalue -= gamepad1.right_stick_x * turnRate;


        //overall divide
        flvalue = (flvalue / 2);
        frvalue = (frvalue / 2);
        blvalue = (blvalue / 2);
        brvalue = (brvalue / 2);


        if(gamepad1.a)
        {
            Sensors.resetGyro();
        }
/*
        telemetry.addData("flvalue", flvalue);
        telemetry.addData("frvalue", frvalue);
        telemetry.addData("blvalue", blvalue);
        telemetry.addData("brvalue", brvalue);
*/
        if(flvalue > 1)
        {
            flvalue = 1;
        }
        if(flvalue < -1)
        {
            flvalue = -1;
        }
        if(frvalue > 1)
        {
            frvalue = 1;
        }
        if(frvalue < -1)
        {
            frvalue = -1;
        }
        if(blvalue > 1)
        {
            blvalue = 1;
        }
        if(blvalue < -1)
        {
            blvalue = -1;
        }
        if(brvalue > 1)
        {
            brvalue = 1;
        }
        if(brvalue < -1)
        {
            brvalue = -1;
        }





        if(gamepad1.right_bumper) {
            frontleft.setPower((flvalue)* bumperPower);
            frontright.setPower((frvalue) * bumperPower);
            backleft.setPower((blvalue) * bumperPower);
            backright.setPower((brvalue) * bumperPower);
        } else {
            frontleft.setPower(flvalue);
            frontright.setPower((frvalue));
            backleft.setPower((blvalue));
            backright.setPower((brvalue));
        }

    }

    @Override
    public void stop() {
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }

}
