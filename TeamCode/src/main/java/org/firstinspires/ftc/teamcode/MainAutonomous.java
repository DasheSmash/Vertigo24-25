/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
//Allows for this file to use other files:
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Main Autonomous", group="Robot")
//This code should be run when the robot starts on the right side of the truss:
public class MainAutonomous extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    DistanceSensor distanceSensor;

    @Override
    public void runOpMode() {
        RobotMethods RMO = new RobotMethods(hardwareMap);
        RMO.SetDirectionForward();
        RMO.setZeroBehaviorAll();
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        boolean test = false;
        //Moves forward at 0.3 speed for half a second, and then turns right at 0.2 speed for 3/10 of a second, and does not use the haltDistance feature:
        test = opModeIsActive();
        RMO.timedMotorMove(1000, 0.3, 0, 0, 0);
        RMO.timedMotorMove(1000, 0, 0, 0.2, 0);
        RMO.setArmDegreeAuto(170);
        RMO.IOSystem(false,3000);
        RMO.setArmDegreeAuto(0);
        for(double starttime = runtime.milliseconds(); runtime.milliseconds() - starttime < 3000;){RMO.intakeJoint.setPosition(0.4);}
        sleep(3000);
        RMO.IOSystem(true,3000);
        //Outputs information until the end of the Autonomous Period:
        while (opModeIsActive()) {
            telemetry.addData("Distance: ", RMO.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Op mode is active?: ", test);
            telemetry.addData("Op mode is active2?: ", RobotMethods.test2);
            telemetry.update();
        }
    }
}