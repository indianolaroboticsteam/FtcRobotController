package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.Alliance;

/** Red alliance – Target start. */
public class Auto_Red_Target extends BaseAuto {
    @Override protected Alliance alliance() { return Alliance.RED; }

    @Override
    protected void runSequence() throws InterruptedException {
        double startHeading = drive.heading();
        turnToGoalTag(1500);
        aimSpinUntilReady(2500);
        fireN(3);
        turnBackTo(startHeading);
        driveForwardInches(24.0);
    }
}
