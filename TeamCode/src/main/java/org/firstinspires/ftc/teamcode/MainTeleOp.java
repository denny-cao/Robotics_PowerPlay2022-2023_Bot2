package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.RobotConstants.BOTTOM_LIFT_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.GROUND_JUNCTION_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEFT_SERVO_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEFT_SERVO_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.LIFT_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.RobotConstants.LIFT_POSITION_COEFFICIENT;
import static org.firstinspires.ftc.teamcode.RobotConstants.LIFT_POSITION_TOLERANCE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LOW_JUNCTION_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.MID_JUNCTION_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.HIGH_JUNCTION_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.RIGHT_SERVO_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.RIGHT_SERVO_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.TOP_LIFT_POSITION;

@TeleOp(name="MainOpMode")
public class MainTeleOp extends LinearOpMode {

    private Motor fL, fR, bL, bR;
    private MecanumDrive m_drive;
    private GamepadEx driverController1;
    private GamepadEx driverController2;

    private Motor lLift, rLift;
    private MotorGroup lift;

    private int liftPosition;

    private final int[] LIFTPOSITIONS = new int[]{GROUND_JUNCTION_POSITION, LOW_JUNCTION_POSITION, MID_JUNCTION_POSITION, HIGH_JUNCTION_POSITION}; //MUST BE LEAST TO GREATEST
    private boolean[] liftPositionsReached = {true, false, false, false};

    private Servo leftServo, rightServo;

    @Override
    public void runOpMode() throws InterruptedException{
        driverController1 = new GamepadEx(gamepad1);
        driverController2 = new GamepadEx(gamepad2);

        fL = new Motor(hardwareMap, "fL"); //port 0
        fR = new Motor(hardwareMap, "fR"); //port 1
        bL = new Motor(hardwareMap, "bL"); //port 2
        bR = new Motor(hardwareMap, "bR"); //port 3

        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        m_drive = new MecanumDrive(fL, fR, bL, bR);

        lLift = new Motor(hardwareMap, "lLift", Motor.GoBILDA.RPM_312);
        rLift = new Motor(hardwareMap, "rLift", Motor.GoBILDA.RPM_312);

        lLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        rLift.setInverted(true);

        rLift.resetEncoder();
        lLift.resetEncoder();

        lift = new MotorGroup(lLift, rLift);

        lift.setRunMode(Motor.RunMode.PositionControl);
        lift.setPositionCoefficient(LIFT_POSITION_COEFFICIENT);
        lift.setPositionTolerance(LIFT_POSITION_TOLERANCE);

        liftPosition = lLift.getCurrentPosition();

        Arrays.sort(LIFTPOSITIONS);

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        waitForStart();
        while(opModeIsActive()){

             if(driverController2.getButton(GamepadKeys.Button.Y) || driverController2.getButton(GamepadKeys.Button.A)){
                 // Go up
                 if (driverController2.getButton(GamepadKeys.Button.Y)){
                    // Currently at ground junction
                    if (liftPositionsReached[0]){
                        liftPosition = LIFTPOSITIONS[1];
                        liftPositionsReached[0] = false;
                        liftPositionsReached[1] = true;
                    }

                    // Currently at low junction
                    else if (liftPositionsReached[1]) {
                        liftPosition = LIFTPOSITIONS[2];
                        liftPositionsReached[1] = false;
                        liftPositionsReached[2] = true;
                    }

                    // Currently at mid junction
                    else if (liftPositionsReached[2]) {
                        liftPosition = LIFTPOSITIONS[3];
                        liftPositionsReached[2] = false;
                        liftPositionsReached[3] = true;
                    }

                    // Currently at high junction = do nothing
                 }
                 // Go down
                 else {
                    // Currently at ground junction = do nothing

                    // Currently at low junction
                    if (liftPositionsReached[1]){
                        liftPosition = LIFTPOSITIONS[0];
                        liftPositionsReached[1] = false;
                        liftPositionsReached[0] = true;
                    }

                    // Currently at mid junction
                    else if (liftPositionsReached[2]) {
                        liftPosition = LIFTPOSITIONS[1];
                        liftPositionsReached[2] = false;
                        liftPositionsReached[1] = true;
                    }

                    // Currently at high junction
                    else if (liftPositionsReached[3]) {
                        liftPosition = LIFTPOSITIONS[2];
                        liftPositionsReached[3] = false;
                        liftPositionsReached[2] = true;
                    }
                 }
            }else{
                 liftPosition += gamepad2.right_trigger * LIFT_MULTIPLIER;
                 liftPosition -= gamepad2.left_trigger * LIFT_MULTIPLIER;
             }

            if (liftPosition > TOP_LIFT_POSITION){
                liftPosition = TOP_LIFT_POSITION;
            }else if (liftPosition <= BOTTOM_LIFT_POSITION){
                liftPosition = BOTTOM_LIFT_POSITION;
            }

            lift.setTargetPosition(liftPosition);


            if(lift.atTargetPosition()){
                lift.set(0);
            }else{
                lift.set(1);
            }

            if(gamepad2.x){
                leftServo.setPosition(LEFT_SERVO_CLOSE_POSITION);
                rightServo.setPosition(RIGHT_SERVO_CLOSE_POSITION);
            }else if(gamepad2.b){
                leftServo.setPosition(LEFT_SERVO_OPEN_POSITION);
                rightServo.setPosition(RIGHT_SERVO_OPEN_POSITION);
            }

            telemetry.addData("Target Position:", liftPosition);
            telemetry.addData("Position Left:", lLift.getCurrentPosition());
            telemetry.addData("Position Right:", rLift.getCurrentPosition());

            telemetry.addData("leftServo Position:", leftServo.getPosition());
            telemetry.addData("rightServo Position", rightServo.getPosition());

            telemetry.update();

            m_drive.driveRobotCentric(driverController1.getLeftX(), driverController1.getLeftY(), driverController1.getRightX());


        }
    }
}
