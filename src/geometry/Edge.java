package geometry;

/**
 * The class Edge represents a geometrical edge. An Edge consists of two two-dimensional Vertices.
 *
 * The class has a method for computing the closest point on the Edge from another given point. The algorithm used
 * to calculate this is described in Ola Ringdahl's master thesis which can be found at
 * <a href=http://www8.cs.umu.se/~ringdahl/publications.html#master_thesis">his publications site</a>.
 *
 * @see Vertex
 * @see <a href=http://www8.cs.umu.se/~ringdahl/publications.html#master_thesis">Ola Ringdahl master thesis</a>
 *
 * @author Marcus Lööw (oi12mlw@cs.umu.se)
 * @author Andreas Günzel (dv13agl@cs.umu.se)
 */
public class Edge {

    public Vertex start;
    public Vertex end;

    public Edge(Vertex start, Vertex end) {
        this.start = start;
        this.end = end;
    }

    /**
     * Returns the length of the Edge. The length is defined as the distance between the start Vertex and the
     * end Vertex.
     * @return the length of the Edge
     */
    public double getLength() {
        return start.distanceTo(end);
    }

    /**
     * Determines whether or not the Edge contains a given Vertex. If a the given Vertex is either the end or the
     * start of the Edge, the Edge contains the Vertex
     * @param vertex the Vertex to be determined contained or not by the Edge
     * @return true if the Edge contains the Vertex, false otherwise
     * @see Vertex
     */
    public boolean contains(Vertex vertex) {
        return vertex.equals(start) || vertex.equals(end);
    }


    /**
     * Determines whether or not the Edge is equals to a given Object. If the given Object is an Edge and the start of
     * the given Edge is equal to start of the Edge and the end of the given Edge is equal to the end of the Edge, or
     * vice versa, the Edges are equal
     * @param obj the Object to be determined equal or not to the Edge
     * @return true if the Object and the Edge is equal, fale otherwise
     */
    @Override
    public boolean equals(Object obj) {
        if(obj instanceof Edge) {
            return (((Edge)obj).start.equals(start) && ((Edge)obj).end.equals(end))
                    || (((Edge)obj).start.equals(end) && ((Edge)obj).end.equals(start));
        } else {
            return false;
        }
    }

    /**
     * Returns a string representation of the Edge
     * @return a string representation of the Edge
     */
    @Override
    public String toString() {
        return "[" + start.toString() + ", " + end.toString() + "]";
    }


    /**
     * Returns the closest Vertex on the Edge to a given Vertex.
     *
     * The algorithm to retrieve the closest Vertex is
     * described in Ola Ringdahl's master thesis mentioned above.
     *
     * It should be noted that both the given Vertex and the returned Vertex could better be described as a point in
     * a cartesian coordinate system.
     * @param vertex the Vertex to which the closes Vertex on the Edge should be returned
     * @return the closest Vertex on the Edge to the given Vertex
     * @see Vertex
     */
    public Vertex getClosestVertexOnEdge(Vertex vertex) {

        /* If the start and the end of the Edge are equals, there can only be one Vertex on it that is the closest,
        which is the start or the end.
         */
        if(start.equals(end)) {
            return start;
        }

        Vertex closest;

        /* Construct Vertices that represents the vectors between the end and start of the Edge, and between the given
        Vertex and the start of the Edge.
         */
        Vertex v = end.differenceTo(start);
        Vertex w = vertex.differenceTo(start);

        /* Compute the constants c1 and c2. These constants determines whether the given Vertex is to the left
        or to the right of the Edge. The ratio between c1 and c2, b, is later used to compute the closest Vertex.
         */
        double c1 = w.dotProduct(v);
        double c2 = v.dotProduct(v);
        double b = c1/c2;

        double angleInStart;
        double angleInEnd;

        /* Construct a triangle from the end and start of the Edge, and the given Vertex and computes the angles in the
        end points of the Edge
         */
        Triangle triangle = new Triangle(start, end, vertex);
        try {
            angleInStart = triangle.getAngleInVertex(start, Triangle.SUGGESTED_ERROR);
            angleInEnd = triangle.getAngleInVertex(end, Triangle.SUGGESTED_ERROR);
        } catch (Exception e) {
            angleInStart = 0;
            angleInEnd = 0;
        }

        /* If the angle in the start of the Edge is greater than 90 degrees, the given Vertex is to the left of the
        Edge, and thus the closest Vertex shall be the start of the Edge. If the angle in the end of the Edge is
        greater than 90 degrees, the given Vertex is to the right of the Edge. The closest Vertex is therefore
        the end of the Edge. If both the angles are zero, the closest Vertex is in line with the Edge. In this case
        we have three different scenarios: the Vertex is to the left of the Edge, in which case c1 is less than zero
        (the closest Vertex is the start of the Edge); the Vertex is to right of the Edge, in which case c2 is less
        than or equals to c1 (the closest Vertex is the end of the Edge); or the Vertex is between the Edge's
        end points. In the last case the closest Vertex is computed by extending the start of the Edge, with the vector
        from the end of the Edge to the start of the Edge, multiplied with the scalar b.
         */
        if (angleInStart >= 90) {
            closest = start;
        } else if (angleInEnd >= 90) {
            closest = end;
        } else if(angleInStart == 0 && angleInEnd == 0 && (c1 < 0 || c2 <= c1)) {
        	closest = c1 < 0 ? start : end;
        } else {
            closest = start.concat(v.scale(b));
        }

        return closest;
    }

    /**
     * Returns the Vertex along the Edge at a given distance
     * @param distance the given distance along the Edge at which the Vertex to be returned is located
     * @return the Vertex along the Edge at the given distance
     */
    public Vertex getVertexAlongEdgeAtDistance(double distance) {

        if(start.equals(end)) {
            return end;
        }

        double ratio = distance / getLength();

        double x = ratio * (end.x - start.x);
        double y = ratio * (end.y - start.y);

        return start.concat(new Vertex(x, y));

    }
}
