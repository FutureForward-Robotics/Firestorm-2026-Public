package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Builds path legs from named waypoints. Each leg starts where the previous one ended. Heading
 * interpolation comes from the waypoints: constant when the two headings match, linear otherwise.
 *
 * <p>Legs are static geometry. Build them at init, not per loop.
 *
 * <p>A leg is a plain {@link Path} and carries no follower, so a route can be built and inspected
 * off-robot. Per-leg limits are {@code Modifier}s rather than a constraints object: {@code
 * path.with(Constants.algorithmConfig.maxPathSpeed.at(0.5))} holds one leg to half the robot's top
 * speed and reverts afterwards.
 *
 * <pre>{@code
 * Route route = drive.route(alliance, Waypoints.START);
 * Command auto = new SequentialCommandGroup(
 *         drive.follow(route.lineTo(Waypoints.SCORE)),
 *         drive.follow(route.curveTo(Waypoints.GATE, Waypoints.GATE_CONTROL)),
 *         drive.follow(route.lineTo(Waypoints.PICKUP)));
 * }</pre>
 */
public final class Route {

    private static final double HEADING_EPSILON_RAD = 1e-6;

    private final Alliance alliance;
    private final List<String> visited = new ArrayList<>();
    private Waypoint cursor;

    private Route(Alliance alliance, Waypoint start) {
        this.alliance = alliance;
        this.cursor = start;
        visited.add(start.name());
    }

    public static Route from(Alliance alliance, Waypoint start) {
        return new Route(alliance, start);
    }

    public Alliance alliance() {
        return alliance;
    }

    public Waypoint cursor() {
        return cursor;
    }

    /** Waypoint names in visit order. */
    public List<String> visited() {
        return new ArrayList<>(visited);
    }

    public Path lineTo(Waypoint next) {
        Pose from = cursor.pose(alliance);
        Pose to = next.pose(alliance);
        return finish(Paths.line(from, to), from, to, next);
    }

    /**
     * One continuous path across several waypoints. Each leg keeps its own heading interpolation,
     * but the robot brakes only at the last waypoint instead of stopping at every one.
     *
     * <p>Prefer this over a sequence of {@link #lineTo} legs wherever nothing has to happen at the
     * intermediate waypoints. {@code Paths.through} is a different thing: it smooths one curve
     * through the poses and takes a single heading interpolation for the whole path.
     */
    public Path continuousTo(Waypoint... next) {
        Path[] legs = new Path[next.length];
        for (int i = 0; i < next.length; i++) {
            legs[i] = lineTo(next[i]);
        }
        return Paths.path(legs);
    }

    /**
     * Curve to {@code next} through the given control points. Control points shape the path and are
     * not poses the robot holds, so their headings are unused; they are {@link Waypoint}s so that
     * they mirror with the rest of the route.
     */
    public Path curveTo(Waypoint next, Waypoint... via) {
        Pose from = cursor.pose(alliance);
        Pose to = next.pose(alliance);
        return finish(Paths.curve(controlPoints(from, to, via)), from, to, next);
    }

    /**
     * Curve to {@code next} driving backwards, with the heading following the path tangent. Use
     * where the robot should retreat along a curve it just drove, rather than turn around; the
     * waypoint's own heading is then only the starting heading for the following leg.
     */
    public Path reversedCurveTo(Waypoint next, Waypoint... via) {
        Pose from = cursor.pose(alliance);
        Pose to = next.pose(alliance);
        Path path = Paths.curve(controlPoints(from, to, via)).reverseTangent();
        cursor = next;
        visited.add(next.name());
        return path;
    }

    /** Moves the cursor without emitting a leg, after the robot is repositioned by other means. */
    public Route jumpTo(Waypoint next) {
        cursor = next;
        visited.add(next.name());
        return this;
    }

    private Pose[] controlPoints(Pose from, Pose to, Waypoint[] via) {
        Pose[] points = new Pose[via.length + 2];
        points[0] = from;
        for (int i = 0; i < via.length; i++) {
            points[i + 1] = via[i].pose(alliance);
        }
        points[points.length - 1] = to;
        return points;
    }

    private Path finish(Path path, Pose from, Pose to, Waypoint next) {
        boolean headingIsHeld =
                Math.abs(Field.normalize(to.heading() - from.heading())) < HEADING_EPSILON_RAD;
        Path headed =
                headingIsHeld
                        ? path.constant(from.heading())
                        : turningFrom(path, from.heading(), to.heading());
        cursor = next;
        visited.add(next.name());
        return headed;
    }

    /**
     * Turns steadily from {@code startHeading} to {@code endHeading} along the path.
     *
     * <p>Pedro 3.0.0's {@code Path.linear} reads its two headings in opposite orders for a line
     * and for a curve: on a line the first argument is where the robot ends up, on a curve it is
     * where it starts. A leg given them the wrong way round starts with a sudden turn to the end
     * heading and then turns back. So this asks the path which way it took them and swaps if
     * needed. Pinned by {@code RouteTest} for both kinds of leg.
     */
    private static Path turningFrom(Path path, double startHeading, double endHeading) {
        Path headed = path.linear(startHeading, endHeading);
        boolean startsOnStartHeading =
                Math.abs(Field.normalize(headed.heading(0) - startHeading)) < HEADING_EPSILON_RAD;
        return startsOnStartHeading ? headed : path.linear(endHeading, startHeading);
    }

    @Override
    public String toString() {
        return alliance + " route " + Arrays.toString(visited.toArray());
    }
}
