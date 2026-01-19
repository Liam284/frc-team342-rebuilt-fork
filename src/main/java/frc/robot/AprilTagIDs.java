// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class AprilTagIDs {
    public enum hubTagsIDs {
        FRONT_CENTER(0, 0),
        FRONT_OFFSET(0, 0),
        RIGHT_CENTER(0, 0),
        RIGHT_OFFSET(0, 0),
        LEFT_CENTER(0, 0),
        LEFT_OFFSET(0, 0),
        BACK_CENTER(0, 0),
        BACK_OFFSET(0, 0);

        private int blueHubTagID;
        private int redHubTagID;

        hubTagsIDs(int blueHubTagID, int redHubTagID) {
            this.blueHubTagID = blueHubTagID;
            this.redHubTagID = redHubTagID;
        }

        public int getBlueHubTagID() {
            return blueHubTagID;
        }

        public int getRedHubTagID() {
            return redHubTagID;
        }
    }

    public enum outpostTagsIDs {
        CENTER(0, 0),
        OFFSET(0, 0);

        private int blueOutpostTagID;
        private int redOutpostTagID;

        outpostTagsIDs(int blueOutpostTagID, int redOutpostTagID) {
            this.blueOutpostTagID = blueOutpostTagID;
            this.redOutpostTagID = redOutpostTagID;
        }

        public int getBlueOutpostTagID() {
            return blueOutpostTagID;
        }

        public int getRedOutpostTagID() {
            return redOutpostTagID;
        }
    }

    public enum towerTagsIDs {
        CENTER(0, 0),
        OFFSET(0, 0);

        private int blueTowerTagID;
        private int redTowerTagID;

        towerTagsIDs(int blueTowerTagID, int redTowerTagID) {
            this.blueTowerTagID = blueTowerTagID;
            this.redTowerTagID = redTowerTagID;
        }

        public int getBlueTowerTagID() {
            return blueTowerTagID;
        }

        public int getRedTowerTagID() {
            return redTowerTagID;
        }
    }

}
