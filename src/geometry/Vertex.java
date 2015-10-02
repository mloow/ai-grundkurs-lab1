package geometry;

import localization.Position;

/**
 * A Vertex represents a two-dimensional geometrical vertex.
 *
 * The Vertex class also doubles as a two-dimensional with methods for dot product, scaling, and concatenation.
 *
 * @author Marcus Lööw (oi12mlw@cs.umu.se)
 * @author Andreas Günzel (dv13agl@cs.umu.se)
 */
public class Vertex {

    public double x;
    public double y;

    /**
     * Constructs a new Vertex on (x, y).
     * @param x the x coordinate
     * @param y the y coordinate
     */
    public Vertex(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Creates a new Vertex on the given Position
     * @param position the given position
     */
    public Vertex(Position position) {

    	this.x = position.x;
    	this.y = position.y;
	}

    /**
     * Determines if this Vertex is equal to a given Object. If the Object is a Vertex and the x's and  y's are equal,
     * the Object and the Vertex is equal
     * @param obj the Object to be determined equal or not to this Vertex
     * @return true if the Object and this Vertex are equal
     */
	@Override
    public boolean equals(Object obj) {
        if(obj instanceof Vertex) {
            return (((Vertex) obj).x == x && ((Vertex) obj).y == y);
        } else {
            return false;
        }
    }

    /**
     * Returns a string representation of this Vertex on the format (x, y)
     * @return a string representation of this Vertex
     */
    @Override
    public String toString() {
        return "(" + x + ", " + y + ")";
    }

    /**
     * Returns the distance between this Vertex and a given Vertex
     * @param vertex the Vertex to which the distance should be computed
     * @return the distance between this Vertex and a given Vertex
     */
    public double distanceTo(Vertex vertex) {
        return Math.hypot(x - vertex.x, y- vertex.y);
    }

    /**
     * Returns a new Vertex where the x and y is the difference between this Vertex's a given Vertex's x's and y's.
     * @param vertex the given Vertex
     * @return a Vertex where the x and y is the difference between this Vertex's a given Vertex's x's and y's.
     */
    public Vertex differenceTo(Vertex vertex) {
        return new Vertex(x - vertex.x, y - vertex.y);
    }

    /**
     * Takes a Vertex, treats it as a vector, and computes and returns the dot product of it and this Vertex
     * treated as a vector
     * @param vertex the given Vertex treated as a vector
     * @return the dot product of this Vertex (as a vector) and the given Vertix (as a vector)
     */
    public double dotProduct(Vertex vertex) {
        return x*vertex.x + y*vertex.y;
    }

    /**
     * Returns this Vertex with its x and y scaled with the given scalar.
     * @param scalar the scalar to scale this Vertex's x and y
     * @return the scaled Vertex
     */
    public Vertex scale(double scalar) {
        return new Vertex(x*scalar, y*scalar);
    }

    /**
     * Adds the x and y of this Vector with the x and y of a given Vertex and returns a new Vector with the resulting
     * x and y.
     * @param vertex the Vertex concatenate this Vector with
     * @return the concatenated Vertex
     */
    public Vertex concat(Vertex vertex) {
        return new Vertex(x + vertex.x, y + vertex.y);
    }

    /**
     * Converts this Vector to a Position
     * @return this Vector converted to a Position
     * @see Position
     */
	public Position toPosition() {

		return new Position(x, y);
	}
}
