package org.firstinspires.ftc.teamcode.test.autonomous;

import org.firstinspires.ftc.teamcode.utils.Enums;

public class BaseAutonomous {
    public Enums.AutonomousStates Latched () {
        return Enums.AutonomousStates.LATCHED;
    }
    public Enums.AutonomousStates Drop () {
        // do something to drop
        return Enums.AutonomousStates.DROPPED;
    }
    public Enums.AutonomousStates LocateMineral () {
        return Enums.AutonomousStates.MINERAL_LOCATED;
    }
    public Enums.AutonomousStates DriveToMineral () {
        return Enums.AutonomousStates.AT_MINERAL;
    }
    public Enums.AutonomousStates PushMineral () {
        return Enums.AutonomousStates.MINERAL_PUSHED;
    }
    public Enums.AutonomousStates DriveToDepot () {
        return Enums.AutonomousStates.AT_DEPOT;
    }
    public Enums.AutonomousStates DropMarker () {
        return Enums.AutonomousStates.DROPPED_MARKER;
    }
    public Enums.AutonomousStates DriveToWall () {
        return Enums.AutonomousStates.AT_WALL;
    }
    public Enums.AutonomousStates StraightenOnWall () {
        return Enums.AutonomousStates.STRAIGHTENED_ON_WALL;
    }
    public Enums.AutonomousStates DriveToCrater () {
        return Enums.AutonomousStates.AT_CRATER;
    }
    public Enums.AutonomousStates ParkInCrater () {
        return Enums.AutonomousStates.PARKED_IN_CRATER;
    }
}
