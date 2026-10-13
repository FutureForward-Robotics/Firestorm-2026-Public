package org.firstinspires.ftc.teamcode.field;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;

import org.junit.jupiter.api.Test;

/** Waypoint resolution under {@link Field#SYMMETRY}, which is ROTATIONAL. */
class WaypointTest {

    private static final double EPS = 1e-9;

    @Test
    void redReturnsAuthoredValues() {
        Waypoint w = Waypoint.red("score", 119.380, 128.800, 225);
        assertEquals(119.380, w.x(Alliance.RED), EPS);
        assertEquals(128.800, w.y(Alliance.RED), EPS);
        assertEquals(Math.toRadians(-135), w.heading(Alliance.RED), EPS);
    }

    @Test
    void blueIsRedRotatedByDefault() {
        Waypoint w = Waypoint.red("score", 119.380, 128.800, 30);
        assertEquals(144.0 - 119.380, w.x(Alliance.BLUE), EPS);
        assertEquals(144.0 - 128.800, w.y(Alliance.BLUE), EPS);
        assertEquals(Math.toRadians(-150), w.heading(Alliance.BLUE), EPS);
        assertFalse(w.isBluePinned());
        assertEquals(0, w.blueDriftInches(), EPS);
    }

    @Test
    void pinnedBlueWinsOverTheMirror() {
        Waypoint w = Waypoint.red("score", 119.380, 128.800, 225).blue(27.0, 130.0, -40);

        assertEquals(27.0, w.x(Alliance.BLUE), EPS);
        assertEquals(130.0, w.y(Alliance.BLUE), EPS);
        assertEquals(Math.toRadians(-40), w.heading(Alliance.BLUE), EPS);
        assertTrue(w.isBluePinned());
    }

    @Test
    void pinningBlueLeavesRedAlone() {
        Waypoint w = Waypoint.red("score", 119.380, 128.800, 225).blue(27.0, 130.0, -40);

        assertEquals(119.380, w.x(Alliance.RED), EPS);
        assertEquals(128.800, w.y(Alliance.RED), EPS);
        assertEquals(Math.toRadians(-135), w.heading(Alliance.RED), EPS);
    }

    @Test
    void waypointsAreImmutable() {
        Waypoint red = Waypoint.red("score", 119.380, 128.800, 225);
        Waypoint pinned = red.blue(27.0, 130.0, -40);

        assertNotSame(red, pinned);
        assertFalse(red.isBluePinned());
        assertEquals(144.0 - 119.380, red.x(Alliance.BLUE), EPS);
    }

    @Test
    void driftMeasuresDistanceFromThePureRotation() {
        // Rotation gives (24.620, 15.200). Pin blue 3 inches further out and 4 down.
        Waypoint w = Waypoint.red("score", 119.380, 128.800, 225).blue(21.620, 11.200, 45);
        assertEquals(5.0, w.blueDriftInches(), 1e-9);
    }

    @Test
    void measuresDistanceBetweenWaypoints() {
        Waypoint a = Waypoint.red("a", 100, 100, 0);
        Waypoint b = Waypoint.red("b", 103, 104, 90);

        assertEquals(5.0, a.distanceTo(b, Alliance.RED), 1e-9);
        assertEquals(5.0, b.distanceTo(a, Alliance.RED), 1e-9, "symmetric");
        assertEquals(0.0, a.distanceTo(a, Alliance.RED), 1e-9);
    }

    /** Every FieldSymmetry is an isometry, so unpinned waypoints keep their spacing on blue. */
    @Test
    void distanceSurvivesTheAllianceTransform() {
        Waypoint a = Waypoint.red("a", 100, 100, 0);
        Waypoint b = Waypoint.red("b", 103, 104, 90);

        assertEquals(a.distanceTo(b, Alliance.RED), a.distanceTo(b, Alliance.BLUE), 1e-9);
    }

    @Test
    void pinningBlueChangesTheBlueDistance() {
        Waypoint a = Waypoint.red("a", 100, 100, 0);
        // b rotates to (41, 40), so the pin moves it in both axes.
        Waypoint b = Waypoint.red("b", 103, 104, 90).blue(38, 36, 270);

        assertEquals(5.0, a.distanceTo(b, Alliance.RED), 1e-9);
        // a rotates to (44, 44); the pinned b sits 6 across and 8 down from it.
        assertEquals(10.0, a.distanceTo(b, Alliance.BLUE), 1e-9);
    }

    @Test
    void measuresDistanceFromARobotPose() {
        Waypoint goal = Waypoint.red("goal", 100, 100, 0);

        assertEquals(5.0, goal.distanceTo(new Pose(96, 103, 0), Alliance.RED), 1e-9);
        assertEquals(0.0, goal.distanceTo(goal.pose(Alliance.BLUE), Alliance.BLUE), 1e-9);
    }

    @Test
    void headingsAreAlwaysWrapped() {
        Waypoint w = Waypoint.red("spin", 10, 10, 720 + 45);
        assertEquals(Math.toRadians(45), w.heading(Alliance.RED), EPS);
        assertTrue(w.heading(Alliance.BLUE) >= -Math.PI && w.heading(Alliance.BLUE) < Math.PI);
    }

    @Test
    void poseCarriesTheResolvedValues() {
        Waypoint w = Waypoint.red("score", 119.380, 128.800, 225);
        assertEquals(144.0 - 119.380, w.pose(Alliance.BLUE).x(), EPS);
        assertEquals(119.380, w.pose(Alliance.RED).x(), EPS);
    }

    /** Values from the 2025 TopRed12 / TopBlue12 paths, which were tuned separately. */
    @Test
    void expressesLastSeasonsPerSideTuning() {
        Waypoint scorePreload =
                Waypoint.red("scorePreload", 99.533, 98.933, 240).blue(53.000, 94.000, 240);

        assertEquals(99.533, scorePreload.x(Alliance.RED), EPS);
        assertEquals(53.000, scorePreload.x(Alliance.BLUE), EPS);
        // The symmetry gives x=44.467; the robot needed 53.0.
        assertEquals(44.467, Field.SYMMETRY.x(99.533, 98.933), 1e-9);
        assertTrue(scorePreload.blueDriftInches() > 8.0);
    }

    @Test
    void headingAndPoseReportTheSameDirectionInDifferentRanges() {
        Waypoint w = Waypoint.red("corner", 10, 10, -45);

        assertEquals(Math.toRadians(-45), w.heading(Alliance.RED), EPS, "signed, for differences");
        assertEquals(
                Math.toRadians(315),
                w.pose(Alliance.RED).heading(),
                EPS,
                "Pedro stores a Pose heading in [0, 2pi)");
        assertEquals(
                0,
                Field.normalize(w.pose(Alliance.RED).heading() - w.heading(Alliance.RED)),
                EPS,
                "same direction either way");
    }
}
