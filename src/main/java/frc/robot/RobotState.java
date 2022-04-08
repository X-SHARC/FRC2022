// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class RobotState {
    //create an enum for alignment consisting off values TIMEOUT, SUCCESS, FAIL, ALIGNING
    public enum AlignmentState {
        TIMEOUT, SUCCESS, FAIL, ALIGNING, IDLE
    }

    //create an enum for distance consisting off values TIMEOUT, SUCCESS, FAIL, ALIGNING
    public enum DistanceState {
        TIMEOUT, SUCCESS, FAIL, ALIGNING, IDLE
    }

    private AlignmentState alignmentState = AlignmentState.IDLE;
    private DistanceState distanceState = DistanceState.IDLE;

    public void setState(AlignmentState alignmentState, DistanceState distanceState) {
        setAlignmentState(alignmentState);
        setDistanceState(distanceState);
    }

    public void setAlignmentState(AlignmentState alignmentState) {
        this.alignmentState = alignmentState;
    }

    public void setDistanceState(DistanceState distanceState) {
        this.distanceState = distanceState;
    }

    public AlignmentState getAlignmentState() {
        return alignmentState;
    }

    public DistanceState getDistanceState() {
        return distanceState;
    }

}
