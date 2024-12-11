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
    public DcMotor linearMotor;
    public DcMotor climbMotor;
    public Servo intakeJoint;
    public Servo intakeSystem;
    public Servo claw;
    public DistanceSensor distanceSensor;
    private double encoderRatio = 1.5;
    //Set the gear and encoder ratio (if your robot has no encoder then set this to 1):
    private double degRatio = 3 * encoderRatio; //3 for the motor to arm gear ratio on the test robot, and 1.5 for the encoder to motor ratio (encoder data is precisely inaccurate)
    public static double armTarget = 0.0;
    public static double slideTarget = 0.0;
    //Required to have this method when extending LinearOpMode:
    @Override
    public void runOpMode(){}
// Dash told me to put a comment - Brady
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
        linearMotor = hardwareMap.get(DcMotor.class, "linslide");
        claw = hardwareMap.get(Servo.class, "c1");
        //intakeJoint = hardwareMap.get(Servo.class, "servoangle");
        //intakeSystem = hardwareMap.get(Servo.class, "servowheel");
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "DS");
        setupMotors(); //Method for all of the arm motor setup. Doesn't need to be in a method but it's more organized that way.
    }

    //Sets the direction of the motor and enables the encoder for the motor:
    public void setupMotors() {
        armMotor.setDirection(DcMotor.Direction.FORWARD); //Sets direction
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Sets current position to degree 0
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Starts encoder
        linearMotor.setDirection(DcMotor.Direction.FORWARD); //Sets direction
        linearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Sets current position to degree 0
        linearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Starts encoder
        /*leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); //Sets direction
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Sets current position to degree 0
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Starts encoder
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD); //Sets direction
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Sets current position to degree 0
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Starts encoder
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD); //Sets direction
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Sets current position to degree 0
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Starts encoder
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD); //Sets direction
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Sets current position to degree 0
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Starts encoder*/
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
    public void timedMotorMove(int time, double axial, double lateral, double yaw) {
        for (double startTime = runtime.milliseconds(); runtime.milliseconds() - startTime < time;) {
            move(axial, lateral, yaw);
        }
        move(0,0,0); //Stops robot after moving
    }
    //Sets arm to requested degree:
    public void setArmDegree(int degree){
        armMotor.setTargetPosition((int)(degree*degRatio));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.9);
    }
    //Changes arm by requested degree:
    public void changeArmDegree(int degree){
        armMotor.setTargetPosition((int)(getArmDegree()+(degree*encoderRatio)));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
    }
    public int getArmDegree(){
        return (int)(armMotor.getCurrentPosition()/encoderRatio);
    }

    //This method was designed for FTC Centerstage 2023-2024:
    // This method is used to control the arm and intake system's position during autonomous,
    // Need to give time to move otherwise it will not be able to complete the action (can just set a static time in the method)
    /*public void intakeAuto(int position, int timeToMove) {
        //Position 0 is starting position, Position 1 is to intake pixels, Position 2 is to go to the backboard, and position 3 is to put a pixel on the stripe:
        double[][] armPositions = {{0,0.98},{0,0.4},{170,0.95},{20,0.32}};
        for (double time = runtime.milliseconds(); runtime.milliseconds()-time<timeToMove;){
            intakeJoint.setPosition(armPositions[position][1]);
            setArmDegree((int)armPositions[position][0]);
        }
        armMotor.setPower(0);
    }*/
    public void armAuto(int position){
        int allocatedTime = 5000; //Milliseconds it is allowed to move
        double[][] armPositions = {{0,0},{0,-2},{170,0.95},{20,0.32}};
    }
    public void setArmDegreeAuto(int degree){
        int maxTime = 5000; //Limits movement attempt to 5 seconds
        for(double time = runtime.milliseconds();runtime.milliseconds()-time<maxTime && Math.abs(degree-getArmDegree()) > 5;){ // Will stop the loop when the arm is within 5 degrees of the target
            setArmDegree(degree);
        }
    }
    /**/public void IOSystem(boolean intake, int time){
        if(intake) {
            for (double starttime = runtime.milliseconds(); runtime.milliseconds() - starttime < time; ) {
                intakeSystem.setPosition(0.9);
            }
        }
        else {
            for (double starttime = runtime.milliseconds(); runtime.milliseconds() - starttime < time; ) {
                intakeSystem.setPosition(0.1);
            }
        }
        intakeSystem.setPosition(0.5);
    }
    public void setArmDistance(double distance){
        linearMotor.setTargetPosition((int)(distance));
        linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearMotor.setPower(1);
    }
    public void changeArmDistance(double distance){
        linearMotor.setTargetPosition((int)(linearMotor.getCurrentPosition()+(distance*encoderRatio)));
        linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearMotor.setPower(0.8);
    }
    public int getSlideDegree(){
        return (int)(linearMotor.getCurrentPosition()/encoderRatio);
    }
    public void setDriveDegree(int LF, int LB, int RF, int RB){
        setWheelDegree(leftFrontDrive, LF);
        setWheelDegree(leftBackDrive, LB);
        setWheelDegree(rightFrontDrive, RF);
        setWheelDegree(rightBackDrive, RB);
    }
    public void testDriveDegree_set(int LF, int LB, int RF, int RB){
        int maxDeg = LF;
        maxDeg = Math.max(LF,LB);
        maxDeg = Math.max(LB,RF);
        maxDeg = Math.max(RF,RB);
        testWheelDegree_set(leftFrontDrive, LF, ((double)LF/maxDeg));
        testWheelDegree_set(leftBackDrive, LB, ((double)LB/maxDeg));
        testWheelDegree_set(rightFrontDrive, RF, ((double)RF/maxDeg));
        testWheelDegree_set(rightBackDrive, RB, ((double)RB/maxDeg));
    }
    public void testDriveDegree_change(int LF, int LB, int RF, int RB){
        int maxDeg = LF;
        maxDeg = Math.max(LF,LB);
        maxDeg = Math.max(LB,RF);
        maxDeg = Math.max(RF,RB);
        testWheelDegree_change(leftFrontDrive, LF, ((double)LF/maxDeg));
        testWheelDegree_change(leftBackDrive, LB, ((double)LB/maxDeg));
        testWheelDegree_change(rightFrontDrive, RF, ((double)RF/maxDeg));
        testWheelDegree_change(rightBackDrive, RB, ((double)RB/maxDeg));
    }
    public void changeDriveDegree(int LF, int LB, int RF, int RB){
        changeWheelDegree(leftFrontDrive, LF);
        changeWheelDegree(leftBackDrive, LB);
        changeWheelDegree(rightFrontDrive, RF);
        changeWheelDegree(rightBackDrive, RB);
    }
    public void setDegreeForward(int degree){
        setWheelDegree(leftFrontDrive, degree);
        setWheelDegree(leftBackDrive, degree);
        setWheelDegree(rightFrontDrive, degree);
        setWheelDegree(rightBackDrive, degree);
    }
    public int getWheelDegree(DcMotor wheel){
        return (int)(wheel.getCurrentPosition()/encoderRatio);
    }
    public void setWheelDegree(DcMotor wheel, int degree){
        wheel.setTargetPosition(degree);
        wheel.setPower(1);
    }
    public void testWheelDegree_set(DcMotor wheel, int degree, double power){
        wheel.setTargetPosition(degree);
        wheel.setPower(power);
    }
    public void testWheelDegree_change(DcMotor wheel, int degree, double power){
        wheel.setTargetPosition(degree+wheel.getCurrentPosition());
        wheel.setPower(power);
    }
    public void changeWheelDegree(DcMotor wheel, int degree){
        wheel.setTargetPosition(degree+wheel.getCurrentPosition());
        wheel.setPower(1);
    }
    public void stopUsingDriveEncoders(){
        //Note: Will reset encoder position to zero if turned back on again:
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void startUsingDriveEncoders(){
        //Note: Will reset encoder position to zero:
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void openClaw(){
        //claw1.setPosition(claw1.getPosition()+degree);
        claw.setPosition(0.1);
    }
    public void closeClaw(){
        //claw1.setPosition(claw1.getPosition()-degree);
        claw.setPosition(0.9);
    }
    public void stopClaw(){
        claw.setPosition(0.5);
    }
}