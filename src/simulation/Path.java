package simulation;

import geometry.Edge;
import geometry.Vertex;

import java.io.IOException;
import java.util.concurrent.CopyOnWriteArrayList;

import com.fasterxml.jackson.databind.ObjectMapper;

import localization.Orientation;
import localization.PathNode;
import localization.Pose;
import localization.Position;

/**
 * The Path class represents a pseudo directed (it has a start and an end, and edges are in some manner ordered) path,
 * made up from a collection of Edges. For every Edge in the path, the end point of one edge is the start point of
 * the next one, except for the last one (which has no next edge).
 *
 * The class has methods for computing the <it>carrot point</it>, from a given position (and within a specified edge
 * interval). The algorithm used to do this is described in Ola Ringdahl's master thesis which can be found at
 * <a href=http://www8.cs.umu.se/~ringdahl/publications.html#master_thesis">his publications site</a>.
 *
 * @see Edge
 * @see <a href=http://www8.cs.umu.se/~ringdahl/publications.html#master_thesis">Ola Ringdahl master thesis</a>
 */
public class Path {

    private CopyOnWriteArrayList<Edge> edges;
    private Vertex pathStart = null;
    private Vertex lastAddedVertex = null;
    private int indexOfLastCarrotEdge;

    /**
     * Constructs a new empty Path.
     */
    public Path() {
        edges = new CopyOnWriteArrayList<>();
    }

    /**
     * Returns the last Vertex added to the Path
     * @return the last added Vertex
     * @see Vertex
     */
    public Vertex getLastAddedVertex() {
        return lastAddedVertex;
    }

    /**
     * Extends the Path with a given Vertex, creating an Edge between the last added Vertex and the given Vertex,
     * and updates the last added Vertex to the given Vertex.
     * @param vertex a Vertex to extend the Path with
     * @see Vertex
     * @see Edge
     */
    public void concatPath(Vertex vertex) {
        if(lastAddedVertex == null) {
            lastAddedVertex = vertex;
            pathStart = vertex;
        } else {
            edges.add(new Edge(lastAddedVertex, vertex));
            lastAddedVertex =  vertex;
        }
    }

    /**
     * Returns the index of the Edge closest to the given Vertex within the specified Edge interval.
     *
     * It should be noted that the given Vertex could better be described as a point in a cartesian coordinate system
     *
     * @param vertex the Vertex from which the closest Edge on the path should be found
     * @param edgeIntervalStart the start of the Edge interval
     * @param edgeIntervalEnd the end of the Edge interval
     * @return the index of the Edge closest to the given Vertex within the specified Edge interval or -1 if there
     * are no Edges in the Path
     * @see Vertex
     * @see Edge
     */
    private int getIndexOfClosestEdgeToVertex(Vertex vertex, int edgeIntervalStart, int edgeIntervalEnd) {
        edgeIntervalEnd = Math.min(edgeIntervalEnd,edges.size() - 1);
        if(!edges.isEmpty()) {

            Vertex closestVertexOnEdge = edges.get(0).getClosestVertexOnEdge(vertex);
            double minD = vertex.distanceTo(closestVertexOnEdge);
            int index = 0;

            for(int i = edgeIntervalStart; i < edgeIntervalEnd; i++) {

                closestVertexOnEdge = edges.get(i).getClosestVertexOnEdge(vertex);
                double d    = vertex.distanceTo(closestVertexOnEdge);
                minD        = (d <= minD ? d : minD);
                index       = (d <= minD ? i : index);
            }
            return index;
        }
        return -1;
    }

    /**
     * Converts every Vertex from start to end of the Path to PathNodes and then into a json String.
     * @return a json String representation of every Vertex in the Path.
     * @throws IOException if there was an error writing the json String
     * @see Vertex
     */
    public String toJson() throws IOException {

    	String json;

    	PathNode[] pathNodes = new PathNode[edges.size()];

    	for(int i = 0 ; i < edges.size(); i++) {

    		Vertex v = edges.get(i).start;
    		pathNodes[i] = new PathNode();
    		pathNodes[i].pose = new Pose();
    		pathNodes[i].status = "4";
    		pathNodes[i].timestamp = "0";
    		pathNodes[i].pose.orientation = new Orientation();
    		pathNodes[i].pose.position = new Position(v.x, v.y);

    	}

    	ObjectMapper mapper = new ObjectMapper();
        json = mapper.writeValueAsString(pathNodes);

    	return json;

    }

    /**
     * Returns the carrot point from a given Vertex, on a given look-ahead distance, within a specified Edge interval.
     *
     * It should be noted that the given Vertex could better be described as a point in a cartesian coordinate system.
     * The same goes for the returned carrot point.
     *
     * @param vertex the given Vertex from which to find the carrot point
     * @param lookAheadDistance the look-ahead distance
     * @param edgeIntervalStart the start of the Edge interval
     * @param edgeIntervalEnd the end of the Edge interval
     * @return the carrot point from the given Vertex, at the given look-ahead distance,
     * within the specified Edge interval.
     * @see Vertex
     * @see Edge
     */
    public Vertex getCarrotPointFrom(Vertex vertex,double lookAheadDistance,int edgeIntervalStart,int edgeIntervalEnd) {
        return getCarrotPathFrom(vertex, lookAheadDistance, edgeIntervalStart, edgeIntervalEnd).getEnd();
    }

    /**
     * Returns the carrot Path from a given Vertex, at a given look-ahead distance, within a specified Edge interval.
     * The carrot path is a sub Path of the Path.
     * The algorithm used in this method is based upon the follow-the-carrot algorithm described in Ola Ringdahl's
     * master thesis mentioned above.
     *
     * It should be noted that the given Vertex could better be described as a point in a cartesian coordinate system.
     *
     * @param vertex the Vertex from which the carrot path should be calculated
     * @param lookAheadDistance the look-ahead distance
     * @param edgeIntervalStart the start of the Edge interval
     * @param edgeIntervalEnd the end of the Edge interval
     * @return the carrot Path from the given Vertex, at the given look-ahead distance,
     * within the specified Edge interval.
     * @see Vertex
     * @see Edge
     */
    public Path getCarrotPathFrom(Vertex vertex, double lookAheadDistance, int edgeIntervalStart, int edgeIntervalEnd) {
        Path carrotPath = new Path();

        /* Get the Edge closest to the given Vertex within the specified Edge interval */
        int indexOfStartEdge = getIndexOfClosestEdgeToVertex(vertex, edgeIntervalStart, edgeIntervalEnd);
        if(indexOfStartEdge < 0) {
            return carrotPath;
        }

        Edge startEdge = edges.get(indexOfStartEdge);

        /* Get the closest Vertex on the closest edge and add it to the carrot path */
        Vertex start = startEdge.getClosestVertexOnEdge(vertex);
        carrotPath.concatPath(start);

        /* Set the remaining distance to be the full look-ahead distance */
        double remainingDistance = lookAheadDistance;

        /* Loop until remaining distance is 0 (carrot point has been reached) or
         until the end of the path has been reached
          */
        for (int i = indexOfStartEdge; i < edges.size() && remainingDistance > 0; i++) {

            Edge currentEdge = i == indexOfStartEdge ? new Edge(start, startEdge.end) : edges.get(i);
            Vertex end = currentEdge.getVertexAlongEdgeAtDistance(remainingDistance);

            /* If the distance to the supposed end-point from the current start point exceeds
             the current edges end point
              */
            if(start.distanceTo(end) > start.distanceTo(currentEdge.end)) {
                end = currentEdge.end;

                /* if the current edge is the last one, we have arrived */
                if(i == edges.size() - 1) { // if last edge of path
                    remainingDistance = 0;
                    /* else subtract the distance from the current start point to the end of the current edge */
                } else {
                    remainingDistance-= start.distanceTo(end);

                }
                /* else we have arrived */
            } else {
                if(!start.equals(end)) {
                    remainingDistance = 0;
                }
            }

            /* Add the edge to the carrotPath */
            carrotPath.concatPath(end);

            /* Set the start of the next Edge to be the end of the current */
            start = currentEdge.end;

            /* Store the index of the carrot Edge */
            indexOfLastCarrotEdge = i;
        }

        return carrotPath;

    }

    /**
     * Returns the Edges of the Path as an ArrayList
     * @return the Edges of the Path as an ArrayList
     * @see Edge
     * @see java.util.ArrayList
     */
    public CopyOnWriteArrayList<Edge> getEdges() {
        return edges;
    }

    /**
     * Returns the start Vertex of the Path. If the Path is empty, the method will return null
     * @return the start Vertex if the Path or null.
     * @see Vertex
     */
    public Vertex getStart() {
        return pathStart;
    }

    /**
     * Constructs a Path from a PathNode array. Each PathNode will be a Vertex in the constructed Path. An Edge will
     * be formed between each PathNode and its following PathNode,which will then be added to the Path.
     * @param pathNodes the array of PathNodes to construct the Path
     * @return the Path from the PathNode array
     * @see PathNode
     * @see Vertex
     * @see Edge
     */
	public static Path fromPathNodes(PathNode[] pathNodes) {

		Path path = new Path();

        for (PathNode pathNode : pathNodes) {

            Position p = pathNode.pose.position;

            Vertex start = new Vertex(p.x, p.y);
            path.concatPath(start);

        }

		return path;
	}

    /**
     * Returns the length of the Path. The length of Path is defined as the cumulative length of every Edge in the Path.
     * @return the length of the Path
     * @see Edge
     */
	public double lengthOfPath() {

		double length = 0;

		for(Edge e : edges) {

			length += e.getLength();
		}

		return length;
	}

    /**
     * Returns the average Edge length of the Path.
     * @return the average Edge length of the Path.
     */
	public double avgEdgeLength() {
		return lengthOfPath() / edges.size();
	}

    /**
     * Return the end, the last Vertex, of the Path
     * @return the end of the Path
     * @see Vertex
     */
	public Vertex getEnd() {

		return lastAddedVertex;
	}

    /**
     * Return the index of the last "carrot Edge". A carrot Edge is defined as the Edge at which a carrot point lies.
     * If the carrot point has never been calculated, the method will return null.
     * @return the index of the last carrot Edge
     * @see Edge
     */
    public int getIndexOfLastCarrotEdge() {
        return indexOfLastCarrotEdge;
    }
}
