package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.Sensors;


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
    private float turnRate = 1.1f;
    private float driveRate = 1.5f;
    private double bumperPower = 0.3;
    private double spinRate = 0.1;
    private Servo clamp;
    private Servo clamp2;
    private DcMotor liftMotor;
    private DcMotor liftMotor2;
    private DcMotor liftMotor3;
    private DcMotor relic;
    private boolean closed = false;
    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;
    private Servo relicServo;
    private Servo ljewel;
    private Servo rjewel;
    private double speedMod = 2;



    private ElapsedTime runtime_y = new ElapsedTime();
    private ElapsedTime runtime_b = new ElapsedTime();

    public void init() {

        liftMotor = hardwareMap.dcMotor.get("lift");
        liftMotor2 = hardwareMap.dcMotor.get("lift2");
        liftMotor3 = hardwareMap.dcMotor.get("lift3");
        clamp = hardwareMap.servo.get("clamp");
        clamp2 = hardwareMap.servo.get("clamp2");
        relic = hardwareMap.dcMotor.get("relic");
        relicServo = hardwareMap.servo.get("relic2");
        ljewel = hardwareMap.servo.get("raisin");
        rjewel = hardwareMap.servo.get("raisin2");
        backright = hardwareMap.dcMotor.get("front_left");
        backleft = hardwareMap.dcMotor.get("front_right");
        frontright = hardwareMap.dcMotor.get("back_left");
        frontleft = hardwareMap.dcMotor.get("back_right");


        relic.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //relic.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        relic.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        //liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        Sensors.initVuforia();
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
        if(gamepad1.left_bumper)
        {
            Sensors.vuMark();
            if(RobotConstants.vuMark == RelicRecoveryVuMark.UNKNOWN)
            {
                flvalue+=1;
                frvalue-=1;
                blvalue+=1;
                brvalue-=1;
            }
            else
            {
                telemetry.addData("Angles", Sensors.angle().toString());


            }
        }

        else
        {
            flvalue += gamepad1.right_stick_x * turnRate;
            frvalue -= gamepad1.right_stick_x * turnRate;
            blvalue += gamepad1.right_stick_x * turnRate;
            brvalue -= gamepad1.right_stick_x * turnRate;
        }

        //overall divide
        if(gamepad1.right_bumper) speedMod = 5;
        else speedMod = 2;

        flvalue = (flvalue / speedMod);
        frvalue = (frvalue / speedMod);
        blvalue = (blvalue / speedMod);
        brvalue = (brvalue / speedMod);


        if(gamepad1.dpad_up)
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

        if(gamepad1.a)
            Sensors.resetGyro();









        //OPERATOR

        // Probably (should?) work- try to get lift motors mapped to right and left bumpers
        if (gamepad2.right_bumper)
        {
            liftMotor.setPower(0.6);
            liftMotor2.setPower(0.6);
            liftMotor3.setPower(0.6*0.3);
        }
        else if (gamepad2.right_trigger>0.6)
        {
            liftMotor.setPower(-(gamepad2.right_trigger/2)+0.35);
            liftMotor2.setPower(-(gamepad2.right_trigger/2)+0.35);
            liftMotor3.setPower((-(gamepad2.right_trigger/2)+0.25)*1.2);
        }
        else if(gamepad2.left_bumper)
        {
            liftMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setTargetPosition(0);
            liftMotor2.setTargetPosition(0);
            liftMotor3.setTargetPosition(0);
            liftMotor.setPower(0.8);
            liftMotor2.setPower(0.8);
            liftMotor3.setPower(0.8);




        }
        else
        {
            liftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor.setPower(0);
            liftMotor2.setPower(0);
            liftMotor3.setPower(0);
        }
        //waiting for encoders


        if(gamepad2.a){
            clamp.setPosition(0.95);
            clamp2.setPosition(1-0.95);} //close
        else if(gamepad2.b){
            clamp.setPosition(0.962);
            clamp2.setPosition(1-0.962);}//open
        else if(gamepad2.y) {
            clamp.setPosition(0.99);
            clamp2.setPosition(1-0.99);}//extra open



        if(gamepad2.dpad_right) relic.setPower(1);
        else if(gamepad2.dpad_left) relic.setPower(-1);
        else relic.setPower(0);


        if (gamepad2.dpad_up) relicServo.setPosition(0.08); //up
        else if(gamepad2.dpad_down) relicServo.setPosition(0.025); //down

        if(gamepad1.x) {ljewel.setPosition(1);rjewel.setPosition(0);}
        if(gamepad1.y) {ljewel.setPosition(0);rjewel.setPosition(1);}

        if(gamepad2.right_stick_y>0.05||gamepad2.right_stick_y<-0.05) relicServo.setPosition(relicServo.getPosition()-(gamepad2.right_stick_y*0.0005));











    }

    @Override
    public void stop() {
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }

}
