package geometry;

/**
 * Created by marcus on 2015-09-18.
 */
public class Triangle {

    public static final double SUGGESTED_ERROR = 0.00000000001;

    private final Edge a;
    private final Edge b;
    private final Edge c;

    public Triangle(Vertex v1, Vertex v2, Vertex v3) {

        this.a = new Edge(v1, v2);
        this.b = new Edge(v2, v3);
        this.c = new Edge(v3, v1);

    }

    public Edge[] getAdjacentEdges(Vertex v) throws Exception {
        return new Edge[]{ a.contains(v) ? a : b, c.contains(v) ? c : b};
    }

    public double getAngleInVertex(Vertex vertex) throws Exception {

        double opposingLength = getOpposingEdge(vertex).getLength();
        double adjacentLength1 = getAdjacentEdges(vertex)[0].getLength();
        double adjacentLength2 = getAdjacentEdges(vertex)[1].getLength();

        double adjacentSquareSum = Math.pow(adjacentLength1, 2) + Math.pow(adjacentLength2, 2);
        double adjacentProduct = 2 * adjacentLength1 * adjacentLength2;

        double cosine = (adjacentSquareSum - Math.pow(opposingLength, 2) )/ adjacentProduct;

        return Math.toDegrees(Math.acos(cosine));
    }

    public double getAngleInVertex(Vertex vertex, double roundedToNearest) throws Exception {
        return Math.round(getAngleInVertex(vertex) * (1/roundedToNearest)) / (1/roundedToNearest);
    }

    public Edge getOpposingEdge(Vertex vertex) throws Exception {
        return a.contains(vertex) ? (b.contains(vertex) ? c : b) : a;

    }
}
