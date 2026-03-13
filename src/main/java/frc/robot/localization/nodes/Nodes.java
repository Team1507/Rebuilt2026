//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.localization.nodes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Full competitive navigation node map for the 2026 REBUILT field.
 *
 * All coordinates are:
 *  - In meters
 *  - In WPILib field coordinates (origin at Blue DS corner)
 *  - Derived from official AprilTag locations and REBUILT field geometry
 *
 * Approach nodes:
 *  - 1.0 m away from the structure
 *  - Facing directly toward the structure center
 */
public final class Nodes {

    private static final double APPROACH_DISTANCE_METERS = 1.0;

    private Nodes() {}

    private static Pose2d approach(Pose2d from, Pose2d to) {
        double dx = to.getX() - from.getX();
        double dy = to.getY() - from.getY();
        double angle = Math.atan2(dy, dx);
        double ax = from.getX() - APPROACH_DISTANCE_METERS * Math.cos(angle);
        double ay = from.getY() - APPROACH_DISTANCE_METERS * Math.sin(angle);
        return new Pose2d(ax, ay, new Rotation2d(angle));
    }

    // ============================
    // HUB NODES (Blue-side reference)
    // ============================
    public static final class Hub {
        // Center of the Hub (computed from all 8 tags)
        public static final Pose2d CENTER =
                new Pose2d(4.6, 4.03, Rotation2d.fromDegrees(0.0));

        // Faces (approximate, based on tag clusters)
        public static final Pose2d FRONT_LEFT =
                new Pose2d(4.612, 4.626, Rotation2d.fromDegrees(90.0));
        public static final Pose2d FRONT_RIGHT =
                new Pose2d(4.256, 4.626, Rotation2d.fromDegrees(90.0));
        public static final Pose2d BACK_LEFT =
                new Pose2d(2.0, 6.0, Rotation2d.fromDegrees(270.0));
        public static final Pose2d BACK_RIGHT =
                new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(270.0));

        // Approach nodes (1.0 m away, facing center)
        public static final Pose2d APPROACH_FRONT =
                approach(FRONT_LEFT, CENTER);
        public static final Pose2d APPROACH_BACK =
                approach(BACK_LEFT, CENTER);
        public static final Pose2d APPROACH_LEFT =
                approach(FRONT_RIGHT, CENTER);
        public static final Pose2d APPROACH_RIGHT =
                new Pose2d(0.557, 2.468, Rotation2d.fromDegrees(0.0));
               
    }

    // ============================
    // TOWER NODES (Blue-side reference)
    // ============================
    public static final class Tower {
        // Tower center from tags at (0.55 in, 146.86/163.86 in)
        public static final Pose2d CENTER =
                new Pose2d(0.514, 3.947, Rotation2d.fromDegrees(270.0));

        // Virtual face points around the tower (for approach geometry)
        private static final Pose2d FRONT_POINT =
                new Pose2d(CENTER.getX() + 0.50, CENTER.getY(), Rotation2d.fromDegrees(270.0));
        private static final Pose2d BACK_POINT =
                new Pose2d(CENTER.getX() - 0.50, CENTER.getY(), Rotation2d.fromDegrees(270.0));
        private static final Pose2d LEFT_POINT =
                new Pose2d(CENTER.getX(), CENTER.getY() + 0.50, Rotation2d.fromDegrees(270.0));
        private static final Pose2d RIGHT_POINT =
                new Pose2d(CENTER.getX(), CENTER.getY() - 0.50, Rotation2d.fromDegrees(270.0));

        public static final Pose2d APPROACH_FRONT =
                approach(FRONT_POINT, CENTER);
        public static final Pose2d APPROACH_BACK =
                approach(BACK_POINT, CENTER);
        public static final Pose2d APPROACH_LEFT =
                approach(LEFT_POINT, CENTER);
        public static final Pose2d APPROACH_RIGHT =
                new Pose2d(1.15, 2.45, Rotation2d.fromDegrees(-8.5));
        public static final Pose2d CLIMB =
                new Pose2d(1.15, 2.85, Rotation2d.fromDegrees(-8.5));
    }

    // ============================
    // DEPOT NODES (Blue-side reference)
    // ============================
    public static final class Depot {
        // Depot center from tags at (0.54 in, 25.62/42.62 in)
        public static final Pose2d CENTER =
                new Pose2d(0.014, 0.867, Rotation2d.fromDegrees(0.0));

        private static final Pose2d LEFT_POINT =
                new Pose2d(CENTER.getX(), CENTER.getY() + 0.50, Rotation2d.fromDegrees(90.0));
        private static final Pose2d RIGHT_POINT =
                new Pose2d(CENTER.getX(), CENTER.getY() - 0.50, Rotation2d.fromDegrees(270.0));

        public static final Pose2d ENTRY_LEFT =
                approach(LEFT_POINT, CENTER);
        public static final Pose2d ENTRY_RIGHT =
                approach(RIGHT_POINT, CENTER);

        public static final Pose2d PICKUP_LEFT =
                approach(LEFT_POINT, CENTER);
        public static final Pose2d PICKUP_RIGHT =
                approach(RIGHT_POINT, CENTER);

        public static final Pose2d INTAKE_DEPOT =
                new Pose2d(2.0, 5.8, Rotation2d.fromDegrees(180.0)) ;
        
    }

    // ============================
    // OUTPOST NODES (Blue-side reference)
    // ============================
    public static final class Outpost {
        // From tags at (0.54 in, 25.62 in) and (0.54 in, 42.62 in)
        public static final Pose2d LEFT =
                new Pose2d(0.014, 0.651, Rotation2d.fromDegrees(0.0));
        public static final Pose2d RIGHT =
                new Pose2d(0.014, 1.082, Rotation2d.fromDegrees(0.0));

        public static final Pose2d LEFT_APPROACH_POINT =
                new Pose2d(1.0, 0.651, Rotation2d.fromDegrees(0.0));
        public static final Pose2d RIGHT_APPROACH_POINT =
                new Pose2d(0.7, 1.1, Rotation2d.fromDegrees(180.0));
        public static final Pose2d RIGHT_APPROACH_POINT_QUEST =
                new Pose2d(1.1, 1.1, Rotation2d.fromDegrees(180.0));

        public static final Pose2d APPROACH_LEFT =
                approach(LEFT_APPROACH_POINT, LEFT);
        public static final Pose2d APPROACH_RIGHT =
                approach(RIGHT_APPROACH_POINT, RIGHT);
    }

    // ============================
    // TRENCH NODES (Blue-side reference)
    // ============================
    public static final class Trench {
        // From tags at (183.03 in, 24.85 in) and (183.03 in, 291.79 in)
        public static final Pose2d NEAR =
                new Pose2d(4.649, 0.631, Rotation2d.fromDegrees(0.0));
        public static final Pose2d FAR =
                new Pose2d(4.649, 7.409, Rotation2d.fromDegrees(0.0));

        public static final Pose2d ENTRY =
                new Pose2d(3.810, 0.631, Rotation2d.fromDegrees(0.0));
        public static final Pose2d EXIT =
                new Pose2d(3.810, 7.409, Rotation2d.fromDegrees(0.0));
    }

    // ============================
    // MIDFIELD NODES
    // ============================
    public static final class Midfield {
        // Using REBUILT carpet edges (B2) – approximate center and safe lanes
        // These values assume a field roughly 16.46 m long by ~8.23 m wide.
        public static final Pose2d CENTER =
                new Pose2d(8.230, 4.115, Rotation2d.fromDegrees(0.0));

        public static final Pose2d SAFE_LEFT =
                new Pose2d(8.230, 5.000, Rotation2d.fromDegrees(0.0));
        public static final Pose2d SAFE_RIGHT =
                new Pose2d(8.230, 3.200, Rotation2d.fromDegrees(0.0));

        public static final Pose2d TOWER_BYPASS_LEFT =
                new Pose2d(1.000, 5.000, Rotation2d.fromDegrees(0.0));
        public static final Pose2d TOWER_BYPASS_RIGHT =
                new Pose2d(1.000, 3.200, Rotation2d.fromDegrees(0.0));

        //subway nodes
        public static final Pose2d RIGHT_RIGHT_SUBWAY =
                new Pose2d(7.75, 0.85, Rotation2d.fromDegrees(90.0));
        public static final Pose2d LOWER_RIGHT_RIGHT_SUBWAY =
                new Pose2d(6.3 , 0.85, Rotation2d.fromDegrees(90.0));
        public static final Pose2d LEFT_RIGHT_SUBWAY =
                new Pose2d(7.75,2.95,Rotation2d.fromDegrees(90.0));
        public static final Pose2d MIDDLE_RIGHT_SUBWAY =
                new Pose2d(6.3,4.0,Rotation2d.fromDegrees(90.0));
        public static final Pose2d MIDDLE_18INCH_SUBWAY =
                new Pose2d(7.75,4.0,Rotation2d.fromDegrees(90.0));
        public static final Pose2d RIGHT_LEFT_SUBWAY =
                new Pose2d(7.75, 5.0, Rotation2d.fromDegrees(270.0));
        public static final Pose2d LEFT_LEFT_SUBWAY =
                new Pose2d(7.75, 7.4, Rotation2d.fromDegrees(270.0));
        public static final Pose2d DOUBLE_LEFT_SUBWAY =
                new Pose2d(7.75, 7.4, Rotation2d.fromDegrees(90.0));
        public static final Pose2d LEFT_FOOTLONG_SUBWAY =
                new Pose2d(7.75, 7.2, Rotation2d.fromDegrees(270.0));
        public static final Pose2d SUBWAY_AROUND_THE_HUB =
                new Pose2d(2.0, 5.0, Rotation2d.fromDegrees(270.0));

        //subway red nodes
        public static final Pose2d RIGHT_RIGHT_SUBWAY_RED =
                new Pose2d(8.7, 7.35, Rotation2d.fromDegrees(270.0));
        public static final Pose2d LOWER_RIGHT_RIGHT_SUBWAY_RED =
                new Pose2d(6.3 , 0.85, Rotation2d.fromDegrees(90.0));
        public static final Pose2d LEFT_RIGHT_SUBWAY_RED =
                new Pose2d(8.75,5.0,Rotation2d.fromDegrees(270.0));
        public static final Pose2d MIDDLE_RIGHT_SUBWAY_RED =
                new Pose2d(6.3,4.0,Rotation2d.fromDegrees(90.0));
        public static final Pose2d MIDDLE_18INCH_SUBWAY_RED =
                new Pose2d(7.75,4.0,Rotation2d.fromDegrees(90.0));
        public static final Pose2d RIGHT_LEFT_SUBWAY_RED =
                new Pose2d(8.76, 3.04, Rotation2d.fromDegrees(270.0));
        public static final Pose2d LEFT_LEFT_SUBWAY_RED =
                new Pose2d(8.81, 0.64, Rotation2d.fromDegrees(270.0));
        public static final Pose2d DOUBLE_LEFT_SUBWAY_RED =
                new Pose2d(7.75, 7.4, Rotation2d.fromDegrees(90.0));
        public static final Pose2d LEFT_FOOTLONG_SUBWAY_RED =
                new Pose2d(7.75, 7.2, Rotation2d.fromDegrees(270.0));
        public static final Pose2d SUBWAY_AROUND_THE_HUB_RED =
                new Pose2d(2.0, 5.0, Rotation2d.fromDegrees(270.0));


        //bump nodes
        public static final Pose2d RIGHT_OVER_BUMP =
                new Pose2d(6.3,2.3, Rotation2d.fromDegrees(55.0));
        public static final Pose2d RIGHT_OVER_BUMP2 =
                new Pose2d(6.3,2.3, Rotation2d.fromDegrees(170.0));
        public static final Pose2d LEFT_OVER_BUMP =
                new Pose2d(6.3,5.6, Rotation2d.fromDegrees(315.0));
        public static final Pose2d LEFT_OVER_BUMP2 =
                new Pose2d(6.3,5.6, Rotation2d.fromDegrees(315.0));
        public static final Pose2d RIGHT_BEFORE_BUMP =
                new Pose2d(7.5,2.0, Rotation2d.fromDegrees(170.0));
        public static final Pose2d LEFT_BEFORE_BUMP =
                new Pose2d(7.5,5.6, Rotation2d.fromDegrees(315.0));
        public static final Pose2d RIGHT_TURN =
               new Pose2d(1.95,2.6, Rotation2d.fromDegrees(170.0));

               public static final class Red{
               public static final Pose2d RIGHT_OVER_BUMP =
                new Pose2d(10.7,5.7, Rotation2d.fromDegrees(225.0));
        public static final Pose2d RIGHT_BEFORE_BUMP =
                new Pose2d(9.8,5.7, Rotation2d.fromDegrees(170.0));
               }
    }

    // ============================
    // STARTING POSITIONS (Blue-side)
    // ============================
    public static final class Start {
        public static final class Red {
                // 2.0 m from Blue wall, spread across width
                public static final Pose2d LEFT =
                        new Pose2d(3.5,5.6,Rotation2d.fromDegrees(315.0));
                public static final Pose2d CENTER =
                        new Pose2d(2.000, 4.115, Rotation2d.fromDegrees(0.0));
                public static final Pose2d RIGHT =
                        new Pose2d(12.8,5.3, Rotation2d.fromDegrees(225.0));
                public static final Pose2d LEFT_OVER_BUMP =
                        new Pose2d(10.21,2.44, Rotation2d.fromDegrees(315.0));
                public static final Pose2d LEFT_BEFORE_BUMP =
                        new Pose2d(9.01,2.44, Rotation2d.fromDegrees(315.0));

                 
        }

        public static final class Blue {
                // 2.0 m from Blue wall, spread across width
                public static final Pose2d LEFT =
                        new Pose2d(3.5,5.6,Rotation2d.fromDegrees(315.0));
                public static final Pose2d CENTER =
                        new Pose2d(2.000, 4.115, Rotation2d.fromDegrees(0.0));
                public static final Pose2d RIGHT =
                        new Pose2d(3.5,2.5, Rotation2d.fromDegrees(55.0));

                public static final Pose2d RIGHT_RED =
                        new Pose2d(11.5,5.7, Rotation2d.fromDegrees(55.0));
                public static final Pose2d LEFT_RED =
                        new Pose2d(13.01,8.04,Rotation2d.fromDegrees(315.0));
        }
    }
    

    // ============================
    // PARKING / ENDGAME (Red-side)
    // ============================
    public static final class Park {
        // 2.0 m from Red wall, mirrored in X from Blue side
        // Assuming field length ≈ 16.46 m
        private static final double FIELD_LENGTH_METERS = 16.460;
        private static final double PARK_X = FIELD_LENGTH_METERS - 2.000;

        public static final Pose2d LEFT =
                new Pose2d(PARK_X, 2.000, Rotation2d.fromDegrees(180.0));
        public static final Pose2d CENTER =
                new Pose2d(PARK_X, 4.115, Rotation2d.fromDegrees(180.0));
        public static final Pose2d RIGHT =
                new Pose2d(PARK_X, 6.200, Rotation2d.fromDegrees(180.0));
    }

    // ============================
    // Alliance Zone (Red)
    // ============================
    public static final class AllianceZoneRed {

        public static final Pose2d LEFT =
                new Pose2d(15.0, 2.000, Rotation2d.fromDegrees(180.0));
        public static final Pose2d CENTER =
                new Pose2d(15.0, 4.115, Rotation2d.fromDegrees(180.0));
        public static final Pose2d RIGHT =
                new Pose2d(15.0, 6.200, Rotation2d.fromDegrees(180.0));
    }    

    // ============================
    // Alliance Zone (Blue)
    // ============================
    public static final class AllianceZoneBlue {

        public static final Pose2d LEFT =
                new Pose2d(1.0, 2.000, Rotation2d.fromDegrees(0.0));
        public static final Pose2d CENTER =
                new Pose2d(2.5, 4.03, Rotation2d.fromDegrees(0.0));
        public static final Pose2d RIGHT =
                new Pose2d(1.0, 6.200, Rotation2d.fromDegrees(0.0));
        public static final Pose2d BACK_LEFT =
                new Pose2d(2.0, 5.0, Rotation2d.fromDegrees(270.0));
        public static final Pose2d BACK_RIGHT =
                new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(30.0));
        
    }

}
