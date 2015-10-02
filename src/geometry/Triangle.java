package geometry;

/**
 * The Triangle class represents a geometrical triangle. The Triangle is constructed with three Vertices. The internal
 * representation of the Triangle is three connected Edges between the three Vertices.
 *
 * The class has methods for calculating angles in each of the Triangle's Vertices, extracting the opposing Edge to a
 * given Vertex of the Triangle, retrieving the adjacent Edges of a given Vertex in the Triangle.
 *
 * @see Edge
 * @see Vertex
 *
 * @author Marcus Lööw (oi12mlw@cs.umu.se)
 * @author Andreas Günzel (dv13agl@cs.umu.se)
 */
public class Triangle {

    public static final double SUGGESTED_ERROR = 0.00000000001;

    private final Edge a;
    private final Edge b;
    private final Edge c;

    /**
     * Constructs a new Triangle from three Vertices
     * @param v1 the first Vertex
     * @param v2 the second Vertex
     * @param v3 the third Vertex
     * @see Vertex
     */
    public Triangle(Vertex v1, Vertex v2, Vertex v3) {

        this.a = new Edge(v1, v2);
        this.b = new Edge(v2, v3);
        this.c = new Edge(v3, v1);

    }

    /**
     * Returns the adjacent Edges in the Triangle of a given Vertex
     * @param vertex the Vertex whose adjacent Edges shall be returned
     * @return the adjacent Edges of the given Vertex
     * @see Edge
     * @see Vertex
     */
    public Edge[] getAdjacentEdges(Vertex vertex)  {
        return new Edge[]{ a.contains(vertex) ? a : b, c.contains(vertex) ? c : b};
    }

    /**
     * Returns the angle in the given Vertex of the Triangle. Angle is calculated by using the law of cosine.
     * @param vertex the given Vertex in the Triangle
     * @return the angle in the given Vertex
     * @see Vertex
     */
    public double getAngleInVertex(Vertex vertex) {

        double opposingLength = getOpposingEdge(vertex).getLength();
        double adjacentLength1 = getAdjacentEdges(vertex)[0].getLength();
        double adjacentLength2 = getAdjacentEdges(vertex)[1].getLength();

        double adjacentSquareSum = Math.pow(adjacentLength1, 2) + Math.pow(adjacentLength2, 2);
        double adjacentProduct = 2 * adjacentLength1 * adjacentLength2;

        double cosine = (adjacentSquareSum - Math.pow(opposingLength, 2) )/ adjacentProduct;

        return Math.toDegrees(Math.acos(cosine));
    }

    /**
     * Returns the angle in the given Vertex of the Triangle, rounded to the nearest given 1/nth.
     * Angle is calculated by using the law of cosine.
     * @param vertex the given Vertex in the Triangle
     * @param roundedToNearest the 1/nth the angle should be rounded to
     * @return the rounded to the nearest 1/nth angle
     */
    public double getAngleInVertex(Vertex vertex, double roundedToNearest) {
        return Math.round(getAngleInVertex(vertex) * (1/roundedToNearest)) / (1/roundedToNearest);
    }

    /**
     * Returns the opposing Edge of a given Vertex in the Triangle
     * @param vertex the vertex to which the adjacent Edge should be returned
     * @return the opposing Edge of a given Vertex in the Triangle
     */
    public Edge getOpposingEdge(Vertex vertex) {
        return a.contains(vertex) ? (b.contains(vertex) ? c : b) : a;
    }
}
