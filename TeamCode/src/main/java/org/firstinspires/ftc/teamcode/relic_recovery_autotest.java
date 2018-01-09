package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.move;
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
/**
 * Created by Patrick on 11/20/2017.
 */
@Autonomous (name="relic_autotest", group="Autonomous Finals")
public class relic_recovery_autotest extends LinearOpMode{

    DcMotor flmotor;
    DcMotor frmotor;
    DcMotor blmotor;
    DcMotor brmotor;
    private Servo clamp;
    private DcMotor liftMotor;
    private DcMotor liftMotor2;
    private DcMotor liftMotor3; // motor that pushes down the lift
    private DcMotor relic; // relic motor for lifting
    // tell us how far and what orientation robot is from target
    double tX; // X value from target
    double tZ; // Same as above but for Z
    double tY; // Same as above but for Y
    // -----------------------------------
    double rX; // X value rotationally
    double rY; // Same as above but for Y
    double rZ; // Same as above but for Z

    VuforiaLocalizer vuforia;
    OpenGLMatrix lastLocation = null;

    public void runOpMode() throws InterruptedException{

        liftMotor = hardwareMap.dcMotor.get("lift");
        liftMotor2 = hardwareMap.dcMotor.get("lift2");
        liftMotor3 = hardwareMap.dcMotor.get("lift3");
        relic = hardwareMap.dcMotor.get("relic");
        clamp = hardwareMap.servo.get("clamp");
        flmotor = hardwareMap.get(DcMotor.class, "front_left");
        frmotor = hardwareMap.get(DcMotor.class, "front_right");
        blmotor = hardwareMap.get(DcMotor.class, "back_left");
        brmotor = hardwareMap.get(DcMotor.class, "back_right");
        flmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        flmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initializing vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AYt3nnz/////AAAAGeWqnGxuREQ8gPunRf7bkzYaJ6lsas+H/ryI/7UQ6Kg/QpCi1ObUnVT96byceD0lMQsIV4bqROkXYKwfjL+79oOM19r9qKF3OnRKItM47YmGatBI9Z0u2rkFRRz1rd/ESSZxvBKLsnVn5uaNvTIgkMJ/Lh0HCl0aQfAf1khSVuZR/6mlcAwf++ejAl+lXPdk716k7fXZvnvEDAkWu7GqG2esiLDoXPcsrWIKAbv9UAwSLIvxVIzHTJBgncJ5a3etLPI0bxwlk/1AZb4ZZ6iDFXLoyv7suXac2ek30Tar6UdJ1EXSxdOMlCZRfes8HdpbmBcyElEmC8+mBsJhaaMN+erUF6Es5eCgilirNZ/Rbf0S";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        // could be front or back
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate(); // Activate Vuforia
        while (opModeIsActive())
        {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) { // Test to see if image is visable
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose(); // Get Positional value to use later
                telemetry.addData("Pose", format(pose));
                if (pose != null)
                {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    tX = trans.get(0);
                    tY = trans.get(1);
                    tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot. NOTE: VERY IMPORTANT IF BASING MOVEMENT OFF OF THE IMAGE!!!!
                    rX = rot.firstAngle;
                    rY = rot.secondAngle;
                    rZ = rot.thirdAngle;
                }
                if (vuMark == RelicRecoveryVuMark.LEFT)
                { // Test to see if Image is the "LEFT" image and display value.
                    telemetry.addData("VuMark is", "Left");
                    telemetry.addData("X =", tX);
                    telemetry.addData("Y =", tY);
                    telemetry.addData("Z =", tZ);
                    // ADD CODE HERE FOR WHERE TO MOVE
                } else if (vuMark == RelicRecoveryVuMark.RIGHT)
                { // Test to see if Image is the "RIGHT" image and display values.
                    telemetry.addData("VuMark is", "Right");
                    telemetry.addData("X =", tX);
                    telemetry.addData("Y =", tY);
                    telemetry.addData("Z =", tZ);
                    // ADD CODE HERE FOR WHERE TO MOVE
                } else if (vuMark == RelicRecoveryVuMark.CENTER)
                { // Test to see if Image is the "CENTER" image and display values.
                    telemetry.addData("VuMark is", "Center");
                    telemetry.addData("X =", tX);
                    telemetry.addData("Y =", tY);
                    telemetry.addData("Z =", tZ);
                    // ADD CODE HERE FOR WHERE TO MOVE
                }
            } else
            {
                telemetry.addData("VuMark", "not visible");
            }
            telemetry.update();
        }
    }
    String format(OpenGLMatrix transformationMatrix)
    {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
    }

