/**
 * Copyright (c) 2015 wait2wait@hotmail.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package wait2wait.geoindex;

import java.util.ArrayList;
import java.util.List;


/**
 * A GeoIndex is a way of expressing a location in the world using a 64 bits
 * integer.
 *
 * 该方法把地球表面划分为(4^24)*20个形状和面积近似的三角形, 每个三角形用唯一的整数编码.
 * 以下说明如何得到这些三角形:
 *
 * 假设地球是一个正球体, 其中有一个内接正二十面体, 它的12个顶点与球体表面相交, 它的每个表面
 * 都是一个正三角形.
 *
 * 以一个正三角形为例, 找到三条边的中点, 从球心到这三个中点的向量交于球体表面的三个点.
 * 连接这三个交点和正三角形的顶点, 得到四个较小的三角形, 并编码为
 *
 *     00, 01, 10, 11.
 *
 * 对这四个小三角形做同样的操作，得到十六个更小的三角形, 并编码为
 *
 *     0000, 0001, 0010, 0011, 0100, 0101, 0110, 0111,
 *     1000, 1001, 1010, 1011, 1100, 1101, 1110, 1111.
 *
 * 对正二十面体的每个正三角形都做24次这样的操作，最终得到(4^24)*20个形状和面积近似的三角形.
 *
 * 在细分过程中，较小三角形的编码前缀正是前次细分较大三角形的编码, 这一特性可以用于按不同精度
 * 检索地理数据.
 */
public class GeoIndex {

    // DEBUG.
    private static final boolean DEBUG = true;

    // The length of the Equator.
    private static final double EQUATOR_LENGTH = 40075036.0;

    // Max search distance.
    public static final double MAX_SEARCH_DISTANCE = (EQUATOR_LENGTH / 5.0) * (Math.sqrt(3.0) / 2.0);

    /**
     * Gets search distance.
     * @param depth
     * @return
     */
    public static double getSearchDistance(int depth) {
        if (depth < 0 || depth > MAX_DEPTH)
            throw new IllegalArgumentException();
        return MAX_SEARCH_DISTANCE / (1 << depth);
    }

    // 赤道周长约40075.036千米，均分5段，每段长度约8015千米。
    // 再把这段长度平分2^24段, 每段长度约0.48米。
    public static final int MAX_DEPTH = 24;

    // The bit 47 to bit 0 represent the offset of the tile in its zone.
    private static final int OFFSET_SHIFT = 0;
    private static final int OFFSET_BITS = MAX_DEPTH << 1;
    private static final long OFFSET_MAX = (1L << OFFSET_BITS) - 1L;
    private static final long OFFSET_MASK = OFFSET_MAX << OFFSET_SHIFT;

    // The bit 52 to bit 48 represent the index of zone.
    private static final int ZONE_SHIFT = 48;
    private static final int ZONE_BITS = 5;
    private static final long ZONE_MAX = (1L << ZONE_BITS) - 1L;
    private static final long ZONE_MASK = ZONE_MAX << ZONE_SHIFT;

    // The bit 60 to bit 56 represent the depth of this GeoIndex.
    private static final int DEPTH_SHIFT = 56;
    private static final int DEPTH_BITS = 5;
    private static final long DEPTH_MAX = (1L << DEPTH_BITS) - 1L;
    private static final long DEPTH_MASK = DEPTH_MAX << DEPTH_SHIFT;

    // The bit 62 to bit 63 represent the edge of a triangle,
    // it is used for calculating only, but not included in a valid GeoIndex.
    private static final int EDGE_SHIFT = 62;
    private static final int EDGE_BITS = 2;
    private static final long EDGE_MAX = (1L << EDGE_BITS) - 1L;
    private static final long EDGE_MASK = EDGE_MAX << EDGE_SHIFT;

    public static final int EDGE_A = 1;
    public static final int EDGE_B = 2;
    public static final int EDGE_C = 3;


    public static long offset(long v) {
        return (v >> OFFSET_SHIFT) & OFFSET_MAX;
    }
    private static long offset(long v, long offset) {
        return (v & (~OFFSET_MASK)) | ((offset & OFFSET_MAX) << OFFSET_SHIFT);
    }
    public static int offsetAt(long v, int depth) {
        if (depth < 1 || depth > depth(v))
            throw new IllegalArgumentException();
        return (int) ((v >> ((MAX_DEPTH - depth) << 1)) & 0x3L);
    }
    public static long offsetAt(long v, int depth, long offset) {
        if (depth < 1 || depth > MAX_DEPTH)
            throw new IllegalArgumentException();
        int shift = (MAX_DEPTH - depth) << 1;
        return (v & (~(0x3L << shift))) | ((offset & 0x3L) << shift);
    }

    public static int zone(long v) {
        return (int) ((v >> ZONE_SHIFT) & ZONE_MAX);
    }
    private static long zone(long v, long zone) {
        return (v & (~ZONE_MASK)) | ((zone & ZONE_MAX) << ZONE_SHIFT);
    }

    public static int depth(long v) {
        return (int) ((v >> DEPTH_SHIFT) & DEPTH_MAX);
    }
    private static long depth(long v, long depth) {
        return (v & (~DEPTH_MASK)) | ((depth & DEPTH_MAX) << DEPTH_SHIFT);
    }

    private static int edge(long v) {
        return (int) ((v >> EDGE_SHIFT) & EDGE_MAX);
    };
    private static long edge(long v, long edge) {
        return (v & (~EDGE_MASK)) | ((edge & EDGE_MAX) << EDGE_SHIFT);
    }


    private static long sibling(long v, long offset) {
        return offsetAt(v, depth(v), offset);
    }

    private static long parent(long v) {
        long depth = depth(v);
        if (depth < 1)
            throw new IllegalArgumentException();

        if (depth == 1) {
            return offset(depth(v, 0), 0);
        } else {
            return depth(v, depth - 1);
        }
    }

    private static long child(long v, long offset) {
        int depth = depth(v) + 1;
        if (depth > MAX_DEPTH)
            throw new IllegalArgumentException();
        return offsetAt(depth(v, depth), depth, offset);
    }

    private static long min(long v) {
        return depth(v & (~((1L << ((MAX_DEPTH - depth(v)) << 1)) - 1L)), MAX_DEPTH);
    }
    private static long max(long v) {
        return depth(v | ((1L << ((MAX_DEPTH - depth(v)) << 1)) - 1L), MAX_DEPTH);
    }


    // 正二十面体的顶点参数
    private static final double M = Math.sqrt(50.0 + 10.0 * Math.sqrt(5.0)) / 10.0;
    private static final double N = Math.sqrt(50.0 - 10.0 * Math.sqrt(5.0)) / 10.0;

    // 正二十面体的十二个顶点
    private static final Vector X0 = Vector.cartesian(0, M, N);
    private static final Vector X1 = Vector.cartesian(0, -M, N);
    private static final Vector X2 = Vector.cartesian(0, -M, -N);
    private static final Vector X3 = Vector.cartesian(0, M, -N);

    private static final Vector Y0 = Vector.cartesian(N, 0, M);
    private static final Vector Y1 = Vector.cartesian(N, 0, -M);
    private static final Vector Y2 = Vector.cartesian(-N, 0, -M);
    private static final Vector Y3 = Vector.cartesian(-N, 0, M);

    private static final Vector Z0 = Vector.cartesian(M, N, 0);
    private static final Vector Z1 = Vector.cartesian(-M, N, 0);
    private static final Vector Z2 = Vector.cartesian(-M, -N, 0);
    private static final Vector Z3 = Vector.cartesian(M, -N, 0);

    // 正二十面体的二十个面
    private static final Triangle[] ZONES = new Triangle[20];
    static {
        ZONES[0]  = new Triangle(X0, Y0, Z0);
        ZONES[2]  = new Triangle(X0, Z0, X3);
        ZONES[4]  = new Triangle(X0, X3, Z1);
        ZONES[6]  = new Triangle(X0, Z1, Y3);
        ZONES[8]  = new Triangle(X0, Y3, Y0);

        ZONES[1]  = new Triangle(Z3, Z0, Y0);
        ZONES[3]  = new Triangle(Y1, X3, Z0);
        ZONES[5]  = new Triangle(Y2, Z1, X3);
        ZONES[7]  = new Triangle(Z2, Y3, Z1);
        ZONES[9]  = new Triangle(X1, Y0, Y3);

        ZONES[10] = new Triangle(X2, Y1, Z3);
        ZONES[12] = new Triangle(X2, Y2, Y1);
        ZONES[14] = new Triangle(X2, Z2, Y2);
        ZONES[16] = new Triangle(X2, X1, Z2);
        ZONES[18] = new Triangle(X2, Z3, X1);

        ZONES[11] = new Triangle(Z0, Z3, Y1);
        ZONES[13] = new Triangle(X3, Y1, Y2);
        ZONES[15] = new Triangle(Z1, Y2, Z2);
        ZONES[17] = new Triangle(Y3, Z2, X1);
        ZONES[19] = new Triangle(Y0, X1, Z3);
    }

    // 正二十面体每条边的重叠边
    private static final long[][] OVERLAPPED_EDGES = new long[20][];
    static {
        for (int zone = 0; zone < 20; zone++) {
            Triangle t = ZONES[zone];

            OVERLAPPED_EDGES[zone] = new long[4];
            OVERLAPPED_EDGES[zone][EDGE_A] = findOverlappedEdge(zone, t.b, t.c);
            OVERLAPPED_EDGES[zone][EDGE_B] = findOverlappedEdge(zone, t.c, t.a);
            OVERLAPPED_EDGES[zone][EDGE_C] = findOverlappedEdge(zone, t.a, t.b);
        }
    }

    /**
     * 查找指定边的重叠边
     * @param zone
     * @param v0
     * @param v1
     * @return
     */
    private static long findOverlappedEdge(int zone, Vector v0, Vector v1) {
        long edgeIndex = 0;

        for (int i = 0; i < 20 && edgeIndex == 0; i++) {
            if (i == zone)
                continue;

            Triangle t = ZONES[i];
            if ((t.b == v0 && t.c == v1) || (t.b == v1 && t.c == v0)) {
                edgeIndex = zone(edgeIndex, i);
                edgeIndex = edge(edgeIndex, EDGE_A);
            }
            else if ((t.c == v0 && t.a == v1) || (t.c == v1 && t.a == v0)) {
                edgeIndex = zone(edgeIndex, i);
                edgeIndex = edge(edgeIndex, EDGE_B);
            }
            else if ((t.a == v0 && t.b == v1) || (t.a == v1 && t.b == v0)) {
                edgeIndex = zone(edgeIndex, i);
                edgeIndex = edge(edgeIndex, EDGE_C);
            }
        }
        if (edgeIndex == 0)
            throw new UnknownError();

        return edgeIndex;
    }

    /**
     * valueOf.
     * @param latitude
     * @param longitude
     * @return
     */
    public static long valueOf(double latitude, double longitude) {
        return valueOf(latitude, longitude, MAX_DEPTH);
    }

    /**
     * valueOf.
     * @param latitude
     * @param longitude
     * @param depth
     */
    private static long valueOf(double latitude, double longitude, int depth) {
        if (depth < 0 || depth > MAX_DEPTH)
            throw new IllegalArgumentException();

        double theta = (90.0 - latitude) * Math.PI / 180.0;
        double phi = longitude * Math.PI / 180.0;
        Vector v = Vector.spherical(1, theta, phi);

        long offset = 0;
        int zone = 0;

        for (; zone < ZONES.length; zone++) {
            try {
                offset = ZONES[zone].offset(v, depth);
                break;
            } catch (UnsupportedOperationException ex) {
                continue;
            }
        }
        if (zone >= ZONES.length)
            throw new UnknownError();

        long r = 0L;
        r = zone(r, zone);
        r = depth(r, depth);
        r = offset(r, offset);
        return r;
    }

    public static class Range {
        public final long min;
        public final long max;

        public Range(long min, long max) {
            this.min = min; this.max = max;
        }
    }

    /**
     * findNearby.
     * @param geoIndex
     * @return
     */
    public static List<Range> findNearby(long geoIndex) {
        return mergeTiles(findNearbyTiles(geoIndex, false));
    }

    /**
     * findNearby.
     * @param latitude
     * @param longitude
     * @param distance
     * @return
     */
    public static List<Range> findNearby(double latitude, double longitude, double distance) {
        return mergeTiles(findNearbyTiles(latitude, longitude, distance, false));
    }

    /**
     * findRing.
     * @param latitude
     * @param longitude
     * @param distance
     * @return
     */
    public static List<Range> findRing(double latitude, double longitude, double distance) {
        List<Long> innerTiles = findNearbyTiles(latitude, longitude, distance / 2.0, false);
        List<Long> outerTiles = findNearbyTiles(latitude, longitude, distance, true);

        for (Long e : innerTiles)
            outerTiles.remove(e);

        return mergeTiles(outerTiles);
    }

    /**
     * findNearbyTiles.
     * @param geoIndex
     * @param listChildren
     * @return
     */
    private static List<Long> findNearbyTiles(long geoIndex, boolean listChildren) {
        List<Long> tiles = new ArrayList<Long>();
        addTiles(tiles, findNearbyTilesByEdge(edge(geoIndex, EDGE_A), listChildren));
        addTiles(tiles, findNearbyTilesByEdge(edge(geoIndex, EDGE_B), listChildren));
        addTiles(tiles, findNearbyTilesByEdge(edge(geoIndex, EDGE_C), listChildren));
        return tiles;
    }

    /**
     * findNearbyTiles.
     * @param latitude
     * @param longitude
     * @param distance
     * @return
     */
    private static List<Long> findNearbyTiles(
            double latitude, double longitude, double distance, boolean listChildren) {

        if (distance < 1.0 || distance > MAX_SEARCH_DISTANCE)
            throw new IllegalArgumentException();

        // 计算需要的深度.
        double r = MAX_SEARCH_DISTANCE;
        int depth = 1;
        for (; depth <= MAX_DEPTH; depth++) {
            r /= 2.0;
            if (distance > r)
                break;
        }
        depth--;

        return findNearbyTiles(valueOf(latitude, longitude, depth), listChildren);
    }

    /**
     * findNearbyTilesByEdge.
     * @param edgeIndex
     * @return
     */
    private static List<Long> findNearbyTilesByEdge(final long edgeIndex, final boolean listChildren) {
        List<Long> tiles = new ArrayList<Long>();
        long v, tmpIndex = edgeIndex;

        // 从指定边出发, 以逆时针方向寻找相邻的Tile.
        do {
            tmpIndex = anticlockwise(overlap(tmpIndex));
            v = edge(tmpIndex, 0);

            if (listChildren) {
                tiles.add(child(v, 0));
                tiles.add(child(v, 1));
                tiles.add(child(v, 2));
                tiles.add(child(v, 3));
            } else {
                tiles.add(v);
            }
        } while (tmpIndex != edgeIndex);

        return tiles;
    }

    /**
     * mergeTiles.
     * @param tiles
     * @return
     */
    private static List<Range> mergeTiles(List<Long> tiles) {
        List<Range> ranges = new ArrayList<Range>();
        Range r = null;

        for (Long e : tiles) {
            if (r == null || (r.max + 1) != min(e)) {
                r = new Range(min(e), max(e));
                ranges.add(r);
            } else {
                ranges.remove(r);
                r = new Range(r.min, max(e));
                ranges.add(r);
            }
        }

        return ranges;
    }

    /**
     * addTiles.
     * @param dest
     * @param src
     */
    private static void addTiles(List<Long> dest, List<Long> src) {
        for (Long e : src) {
            int i, cnt = dest.size();

            for (i = 0; i < cnt; i++) {
                long v = dest.get(i);

                if (e < v) {
                    dest.add(i, e);
                    break;
                }
                if (e == v) {
                    break;
                }
            }
            if (i == cnt)
                dest.add(e);
        }
    }

    /**
     * 获取与指定边重叠的边
     * @return
     */
    private static long overlap(final long edgeIndex) {
        long retIndex = 0, tmpIndex = edgeIndex;
        boolean reverse = true;

        if (edge(edgeIndex) == 0)
            throw new IllegalArgumentException();

        while (retIndex == 0) {

            // 查找当前深度的重叠边
            retIndex = overlapInTile(tmpIndex);

            // 未找到当前深度的重叠边
            if (retIndex == 0) {

                // 若当前深度大于0，则进入父级深度寻找
                if (depth(tmpIndex) > 0) {
                    tmpIndex = parent(tmpIndex);
                    continue;
                }

                // 若当前深度等于0，则在相邻的Zone上寻找重叠边
                int zone = zone(tmpIndex);
                int edge = edge(tmpIndex);

                retIndex = OVERLAPPED_EDGES[zone][edge];
                reverse = (edge == edge(retIndex));
            }

            // 找到了重叠边，再逐步加大深度寻找重叠边
            int depth = depth(retIndex);
            int maxDepth = depth(edgeIndex);

            for (; depth < maxDepth; depth = depth(retIndex)) {
                if (reverse) {
                    switch (offsetAt(edgeIndex, depth + 1)) {
                        case 1:
                            switch (edge(retIndex)) {
                                case EDGE_B: retIndex = child(retIndex, 3); break;
                                case EDGE_C: retIndex = child(retIndex, 2); break;
                                default: throw new IllegalStateException();
                            }
                            break;
                        case 2:
                            switch (edge(retIndex)) {
                                case EDGE_C: retIndex = child(retIndex, 1); break;
                                case EDGE_A: retIndex = child(retIndex, 3); break;
                                default: throw new IllegalStateException();
                            }
                            break;
                        case 3:
                            switch (edge(retIndex)) {
                                case EDGE_A: retIndex = child(retIndex, 2); break;
                                case EDGE_B: retIndex = child(retIndex, 1); break;
                                default: throw new IllegalStateException();
                            }
                            break;
                        default: throw new IllegalArgumentException();
                    }
                } else {
                    switch (offsetAt(edgeIndex, depth + 1)) {
                        case 1:
                            switch (edge(retIndex)) {
                                case EDGE_B: retIndex = child(retIndex, 1); break;
                                case EDGE_C: retIndex = child(retIndex, 1); break;
                                default: throw new IllegalStateException();
                            }
                            break;
                        case 2:
                            switch (edge(retIndex)) {
                                case EDGE_B: retIndex = child(retIndex, 3); break;
                                default: throw new IllegalStateException();
                            }
                            break;
                        case 3:
                            switch (edge(retIndex)) {
                                case EDGE_C: retIndex = child(retIndex, 2); break;
                                default: throw new IllegalStateException();
                            }
                            break;
                        default: throw new IllegalArgumentException();
                    }
                }
            }
        }
        return retIndex;
    }

    /**
     * 获取指定边在相同Tile中的重叠边
     * @param edgeIndex
     * @return
     */
    private static long overlapInTile(final long edgeIndex) {
        int depth = depth(edgeIndex);
        if (depth == 0)
            return 0;

        switch (offsetAt(edgeIndex, depth)) {
            case 0:
                switch (edge(edgeIndex)) {
                    case EDGE_A: return edge(sibling(edgeIndex, 1), EDGE_A);
                    case EDGE_B: return edge(sibling(edgeIndex, 2), EDGE_B);
                    case EDGE_C: return edge(sibling(edgeIndex, 3), EDGE_C);
                    default: throw new IllegalArgumentException();
                }
            case 1:
                switch (edge(edgeIndex)) {
                    case EDGE_A: return edge(sibling(edgeIndex, 0), EDGE_A);
                    case EDGE_B: return 0;
                    case EDGE_C: return 0;
                    default: throw new IllegalArgumentException();
                }
            case 2:
                switch (edge(edgeIndex)) {
                    case EDGE_A: return 0;
                    case EDGE_B: return edge(sibling(edgeIndex, 0), EDGE_B);
                    case EDGE_C: return 0;
                    default: throw new IllegalArgumentException();
                }
            case 3:
                switch (edge(edgeIndex)) {
                    case EDGE_A: return 0;
                    case EDGE_B: return 0;
                    case EDGE_C: return edge(sibling(edgeIndex, 0), EDGE_C);
                    default: throw new IllegalArgumentException();
                }
            default: throw new IllegalArgumentException();
        }
    }

    /**
     * 获取指定边在顺时针方向上的相邻边
     * @param edgeIndex
     * @return
     */
    private static long clockwise(final long edgeIndex) {
        switch (edge(edgeIndex)) {
            case EDGE_A: return edge(edgeIndex, EDGE_B);
            case EDGE_B: return edge(edgeIndex, EDGE_C);
            case EDGE_C: return edge(edgeIndex, EDGE_A);
            default: throw new IllegalArgumentException();
        }
    }

    /**
     * 获取指定边在逆时针方向上的相邻边
     * @param edgeIndex
     * @return
     */
    private static long anticlockwise(final long edgeIndex) {
        switch (edge(edgeIndex)) {
            case EDGE_A: return edge(edgeIndex, EDGE_C);
            case EDGE_B: return edge(edgeIndex, EDGE_A);
            case EDGE_C: return edge(edgeIndex, EDGE_B);
            default: throw new IllegalArgumentException();
        }
    }

    /**
     * Constructor.
     */
    private GeoIndex() {}


    /**
     * Triangle.
     */
    private static class Triangle {

        public final Vector a;
        public final Vector b;
        public final Vector c;

        /**
         * Constructor.
         * @param a
         * @param b
         * @param c
         */
        public Triangle(Vector a, Vector b, Vector c) {
            this.a = a; this.b = b; this.c = c;
        }

        /**
         * offset.
         * @param v
         * @param depth
         * @return
         */
        public long offset(Vector v, int depth) {
            if (depth < 0 || depth > MAX_DEPTH)
                throw new IllegalArgumentException();
            if (position(v) != 0)
                throw new UnsupportedOperationException();

            long offset = 0;
            int shift = (MAX_DEPTH - 1) << 1;

            for (Triangle outer = this; depth > 0; depth--) {

                // Get the inner triangle.
                Triangle inner = new Triangle(
                        Vector.midPoint(outer.b, outer.c).normalize(),
                        Vector.midPoint(outer.c, outer.a).normalize(),
                        Vector.midPoint(outer.a, outer.b).normalize()
                );

                int pos = inner.position(v);
                offset |= ((long) pos << shift);
                shift -= 2;

                switch (pos) {
                    case 0: outer = inner; break;
                    case 1: outer = new Triangle(outer.a, inner.c, inner.b); break;
                    case 2: outer = new Triangle(inner.c, outer.b, inner.a); break;
                    case 3: outer = new Triangle(inner.b, inner.a, outer.c); break;
                }
            }

            return offset;
        }

        /**
         * position.
         * @param v
         * @return
         */
        private int position(Vector v) {
            if (Vector.angle(Vector.cross(c, b), v) > 0)
                return 1;
            if (Vector.angle(Vector.cross(a, c), v) > 0)
                return 2;
            if (Vector.angle(Vector.cross(b, a), v) > 0)
                return 3;
            return 0;
        }
    }

    /**
     * Vector.
     */
    private static class Vector {

        private double x;
        private double y;
        private double z;

        /**
         * Returns a vector in Cartesian coordinate system.
         * @param x
         * @param y
         * @param z
         * @return
         */
        public static Vector cartesian(double x, double y, double z) {
            return new Vector(x, y, z);
        }

        /**
         * Returns a vector in spherical coordinate system.
         * @param r
         * @param theta
         * @param phi
         * @return
         */
        public static Vector spherical(double r, double theta, double phi) {
            double sinTheta = Math.sin(theta);
            double cosTheta = Math.cos(theta);
            double sinPhi = Math.sin(phi);
            double cosPhi = Math.cos(phi);

            return new Vector(r * sinTheta * cosPhi, r * sinTheta * sinPhi, r * cosTheta);
        }

        /**
         * Returns the middle point between a and b.
         * @param a
         * @param b
         * @return
         */
        public static Vector midPoint(Vector a, Vector b) {
            return new Vector((a.x + b.x) / 2.0, (a.y + b.y) / 2.0, (a.z + b.z) / 2.0);
        }

        /**
         * Returns the dot product of a and b.
         * @param a
         * @param b
         * @return
         */
        public static double dot(Vector a, Vector b) {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        /**
         * Returns the cross product of a and b.
         * @param a
         * @param b
         * @return
         */
        public static Vector cross(Vector a, Vector b) {
            return new Vector(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
        }

        /**
         * Returns cosine value of the angle between a and b.
         * @param a
         * @param b
         * @return
         */
        public static double angle(Vector a, Vector b) {
            return dot(a, b) / (a.length() * b.length());
        }

        /**
         * Returns the length of this vector.
         * @return
         */
        public double length() {
            return Math.sqrt(x * x + y * y + z * z);
        }

        /**
         * Constructor.
         * @param x
         * @param y
         * @param z
         */
        private Vector(double x, double y, double z) {
            this.x = x; this.y = y; this.z = z;
        }

        /**
         * normalize.
         */
        public Vector normalize() {
            double length = length();
            if (length == 0)
                throw new UnsupportedOperationException();
            x /= length;
            y /= length;
            z /= length;
            return this;
        }
    }
}
