package geometry;

/**
 * Created by marcus on 2015-09-18.
 */
public class Edge {

    public Vertex start;
    public Vertex end;

    public Edge(Vertex start, Vertex end) {
        this.start = start;
        this.end = end;
    }

    public double getLength() {
        return start.distanceTo(end);
    }

    public boolean contains(Vertex v) {
        return v.equals(start) || v.equals(end);
    }

    @Override
    public boolean equals(Object obj) {
        if(obj instanceof Edge) {
            return (((Edge)obj).start.equals(start) && ((Edge)obj).end.equals(end))
                    || (((Edge)obj).start.equals(end) && ((Edge)obj).end.equals(start));
        } else {
            return false;
        }
    }

    @Override
    public String toString() {
        return "[" + start.toString() + ", " + end.toString() + "]";
    }

    public Vertex getClosestVertexOnEdge(Vertex vertex) {

        if(start.equals(end)) {
            return start;
        }
        Vertex closest;
        Triangle triangle = new Triangle(start, end, vertex);
        Vertex v = end.differenceTo(start);
        Vertex w = vertex.differenceTo(start);

        double c1 = w.dotProduct(v);
        double c2 = v.dotProduct(v);
        double b = c1/c2;

        double angleInStart;
        double angleInEnd;

        try {
            angleInStart = triangle.getAngleInVertex(start, Triangle.SUGGESTED_ERROR);
            angleInEnd = triangle.getAngleInVertex(end, Triangle.SUGGESTED_ERROR);
        } catch (Exception e) {
            angleInStart = 0;
            angleInEnd = 0;
        }

        if (angleInStart >= 90) {
            closest = start;
        } else if (angleInEnd >= 90) {
            closest = end;
        } else if(angleInStart == 0 && angleInEnd == 0) {
        	closest = c1 < 0 ? start : end;
        } else {
            closest = start.concat(v.scale(b));
        }

        return closest;
    }

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
