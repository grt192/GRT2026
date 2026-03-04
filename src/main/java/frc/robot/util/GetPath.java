package frc.robot.util;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

public final class GetPath {
    private GetPath () {}

    public static PathPlannerPath getAlignPath(String pathName) {
        try {
            return PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    // public PathPlannerPath getAlignPath (List<Waypoint> pathWaypoints, GoalEndState goalEndState){

    //     PathPlannerPath getAlignPath = new PathPlannerPath (
    //         pathWaypoints,
    //         constraints,
    //         null, 
    //         goalEndState
    //     );

    // }
}
