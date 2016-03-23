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

import wait2wait.geoindex.GeoIndex;
import org.testng.annotations.Test;
import java.util.Date;
import java.util.List;


public class GeoIndexTest {

    @Test
    public void testSearchDistance() {
        for (int i = GeoIndex.MAX_DEPTH; i >= 0; i--)
            System.out.printf("SearchDistance[%d]: %fm\n", i, GeoIndex.getSearchDistance(i));
    }

    //@Test
    public void test() {
        long aIndex = GeoIndex.valueOf(0.1, 0.1);
        long bIndex = GeoIndex.valueOf(0.1, 0.100001);

        System.out.printf("A: %d\n", aIndex);
        System.out.printf("B: %d\n", bIndex);
    }

    //@Test
    public void sample() {
        long bjIndex = GeoIndex.valueOf(39.55, 116.24);
        long tjIndex = GeoIndex.valueOf(39.02, 117.12);

        System.out.printf("BEIJING: %d\n", bjIndex);
        System.out.printf("TIANJIN: %d\n", tjIndex);

        List<GeoIndex.Range> ranges = GeoIndex.findNearby(39.55, 116.24, 500000);
        for (GeoIndex.Range r : ranges) {
            if (r.min <= tjIndex && r.max >= tjIndex) {
                System.out.printf("found!\n");
                break;
            }
        }
    }

    //@Test
    public void run() {
        long geoIndex = 0;
        long start, end;

        // Create.
        start = new Date().getTime();
        for (int i = 0; i < 100000; i++) {
            geoIndex = GeoIndex.valueOf(39, 116);
        }
        //geoIndex = GeoIndex.valueOf(0, 0, 24);
        end = new Date().getTime();

        // ZoneIndex.
        System.out.printf("[%d,%d]", GeoIndex.zone(geoIndex), GeoIndex.depth(geoIndex));

        // Offset.
        long offset = GeoIndex.offset(geoIndex);
        for (int i = 46; i >= 0; i-=2) {
            System.out.printf("%d", ((offset >> i) & 0x3));
        }
        System.out.printf(", ");

        // OffsetAt(depth).
        long depth = GeoIndex.depth(geoIndex);
        for (int i = 1; i <= depth; i++) {
            System.out.printf("%d", GeoIndex.offsetAt(geoIndex, i));
        }
        System.out.printf(", ");

        // Value.
        System.out.printf("%d, %dms\n", geoIndex, end - start);
    }

    //@Test
    public void findNearbyWithGeoIndex() {
        List<GeoIndex.Range> ranges = null;
        long start, end;

        long geoIndex = GeoIndex.valueOf(0, 45);
        start = new Date().getTime();
        for (int i = 0; i < 100000; i++) {
            ranges = GeoIndex.findNearby(geoIndex);
        }
        end = new Date().getTime();
        System.out.printf("findNearbyWithGeoIndex(%d ranges) in %dms\n", ranges.size(), end - start);
    }

    //@Test
    public void findNearbyWithDistance() {
        List<GeoIndex.Range> ranges = null;
        long start, end;

        start = new Date().getTime();
        for (int i = 0; i < 100000; i++) {
            ranges = GeoIndex.findNearby(-60, -30, 6700000);
        }
        end = new Date().getTime();
        System.out.printf("findNearbyWithDistance(%d ranges) in %dms\n", ranges.size(), end - start);
    }

    //@Test
    public void findAround() {
        List<GeoIndex.Range> ranges = null;
        long start, end;

        start = new Date().getTime();
        for (int i = 0; i < 100000; i++) {
            ranges = GeoIndex.findRing(39, 116, 32000);
        }
        end = new Date().getTime();
        System.out.printf("findRing(%d ranges) in %dms\n", ranges.size(), end - start);
    }

}