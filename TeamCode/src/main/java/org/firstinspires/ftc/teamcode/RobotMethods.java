//Allows this file to be used by other files, and for this file to use other files:
package org.firstinspires.ftc.teamcode;
//Importing necessary files:
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//This class contains all the methods and hardware variables you will need to use the robot, and more of each can be added as necessary:
public class RobotMethods extends LinearOpMode {
    //Global variable declaration:
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public DcMotor armMotor;
    public Servo intakeJoint;
    public Servo intakeSystem;
    public DistanceSensor distanceSensor;
    public static boolean test2 = false;
    //Set the gear ratio (if you're using more than 1 gear for the motor):
    private double degRatio = 3 * 1.5; //3 for the motor to arm gear ratio on the test robot, and 1.5 for the encoder to motor ratio.

    //Required to have this method when extending LinearOpMode:
    @Override
    public void runOpMode() {
    }

    //Declares all of the motors and servos, and can add more if needed:
    public RobotMethods(HardwareMap hardwareMap) {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        // NOTE: Not all of these pieces of hardware are technically needed, comment any of these
        // that aren't used (Keep in mind that other parts of the code may not work without these variables)
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");
        armMotor = hardwareMap.get(DcMotor.class, "am1");
        intakeJoint = hardwareMap.get(Servo.class, "servoangle");
        intakeSystem = hardwareMap.get(Servo.class, "servowheel");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "DS");
        setupArmMotor(); //Method for all of the arm motor setup. Doesn't need to be in a method but it's more organized that way.
    }

    //Sets the direction of the motor and enables the encoder for the motor:
    public void setupArmMotor() {
        armMotor.setDirection(DcMotor.Direction.FORWARD); //Sets direction
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Sets current position to degree 0
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Starts encoder
    }

    //Moves the robot according to given parameters: (Note: the method runs once and sets the power, it does not stop the robot automatically, the user must do that manually by using move(0,0,0) )
    public void move(double axial, double lateral, double yaw) {
        double max;
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;
        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        //Stops wheels from going past 100% speed while maintaining the same speed compared to other wheels:
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        //Sets the new modified power to each wheel:
        leftFrontDrive.setPower(leftFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightFrontDrive.setPower(rightFrontPower);
        rightBackDrive.setPower(rightBackPower);
    }

    //Sets the direction of the 4 wheels to be 'forward':
    public void SetDirectionForward() {
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    //Sets the direction of the 4 wheels to be 'backwards':
    public void SetDirectionBackwards() {
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    //Sets the motors to brake when their power is set to zero:
    public void setZeroBehaviorAll() {
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //This method moves the robot for a set amount of time depending on the call's arguments (Only use for autonomous):
    //haltDistance is the distance in CM of which the robot will stop if the Distance Sensor detects something closer than the specified distance (Used to stop collisions)
    //Set haltDistance to 0 or less if you do not want to use the distance feature.
    public void timedMotorMove(int maxtime, double axial, double lateral, double yaw, int haltDistance) {
        test2 = opModeIsActive();
        for (double startTime = runtime.milliseconds(); runtime.milliseconds() - startTime < maxtime && distanceSensor.getDistance(DistanceUnit.CM) > haltDistance;) {
            move(axial, lateral, yaw);
        }
        move(0,0,0); //Stops robot after moving
    }
    //Sets arm to requested degree:
    public void setArmDegree(int degree){
        armMotor.setTargetPosition((int)(degree*degRatio));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.1);
    }
    //Changes arm by requested degree:
    public void changeArmDegree(int degree){
        armMotor.setTargetPosition((int)(armMotor.getCurrentPosition()+(degree*degRatio)));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.1);
    }
    public int getArmDegree(){
        return (int)(armMotor.getCurrentPosition()/degRatio);
    }
    // This method is used to control the arm and intake system's position during autonomous,
    // Need to give time to move otherwise it will not be able to complete the action (can just set a static time in the method),
    // This method does not need a timed loop if the method call is already in a long enough loop:
    public void intakeAuto(int position, int timeToMove) {
        //Position 0 is starting position, Position 1 is to intake pixels, Position 2 is to go to the backboard, and position 3 is to put a pixel on the stripe:
        double[][] armPositions = {{0,0.98},{0,0.4},{170,0.95},{20,0.32}};
        for (double time = runtime.milliseconds(); runtime.milliseconds()-time<timeToMove;){
            intakeJoint.setPosition(armPositions[position][1]);
            setArmDegree((int)armPositions[position][0]);
        }
        armMotor.setPower(0);
    }
    public void setArmDegreeAuto(int degree){
        int maxTime = 5000; //Limits movement attempt to 5 seconds
        for(double time = runtime.milliseconds();runtime.milliseconds()-time<maxTime && Math.abs(degree-getArmDegree()) > 5;){ // Will stop the loop when the arm is within 5 degrees of the target
            setArmDegree(degree);
        }
    }
    /**/public void IOSystem(boolean intake, int time){
        for(double starttime = runtime.milliseconds(); runtime.milliseconds()-starttime<time;){
            if(intake){intakeSystem.setPosition(0.9);}
            else{intakeSystem.setPosition(0.1);}
        }
        intakeSystem.setPosition(0.5);
    }
}

/*
leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
rightBackDrive = hardwareMap.get(DcMotor.class, "BR");
armMotor = hardwareMap.get(DcMotor.class, "am1");
intakeJoint = hardwareMap.get(Servo.class, "servoangle");
intakeSystem = hardwareMap.get(Servo.class, "servowheel");
distanceSensor = hardwareMap.get(DistanceSensor.class, "DS");
leftFrontDrive = hardwareMap.get(DcMotor.class, "FRONT LEFT WHEEL NAME");
leftBackDrive = hardwareMap.get(DcMotor.class, "BACK LEFT WHEEL NAME");
rightFrontDrive = hardwareMap.get(DcMotor.class, "FRONT RIGHT WHEEL NAME");
rightBackDrive = hardwareMap.get(DcMotor.class, "BACK RIGHT WHEEL NAME");
armMotor = hardwareMap.get(DcMotor.class, "ARM MOTOR NAME");
intakeJoint = hardwareMap.get(Servo.class, "INTAKE ANGLE SERVO NAME");
intakeSystem = hardwareMap.get(Servo.class, "INTAKE SYSTEM INPUT/OUTPUT NAME");
distanceSensor = hardwareMap.get(DistanceSensor.class, "DISTANCE SENSOR NAME");
 */