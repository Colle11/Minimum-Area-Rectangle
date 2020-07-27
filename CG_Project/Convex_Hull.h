#ifndef CONVEX_HULL_H
#define CONVEX_HULL_H

/*
 * Convex Hull Algorithms
 *
 * March 2015
 */

#include <vector>
#include <cstdlib>
#include <algorithm>
#include <iterator>
#include "Environment.h"

// debug
// #include <iostream>
//

namespace CGP {

// ----------  Convex Hull Algorithms  ----------


// Basic tools


// Location of a point w.r.t. a directed line

bool liesToTheLeftOf( Point q, Point p1, Point p2 ) {
  
    // return CGAL::left_turn( p1, p2, q );
  
    return CGAL::left_turn( p1, p2, q ) || CGAL::are_strictly_ordered_along_line( p1, q, p2 );
}


// Testing lexicographical order of points

bool before( Point p, Point q ) {
  
    return CGAL::lexicographically_xy_smaller( p, q );
}


// 0. Naive approach


// Naive convex hull
    
void naiveConvexHull( PointVector &points, SegmentVector &edges ) {

    int n = points.size();
  
    edges.clear();
    
    for ( int i=0; i<n; i++ ) {
        for ( int j=0; j<n; j++ ) {
            if ( i != j ) {
      
                bool candidate = true;  // candidate directed segment pi-pj
        
                for ( int k=0; candidate&&(k<n); k++ ) {
                    if ( (k != i) && (k != j) ) {
          
                        candidate = liesToTheLeftOf( points[k], points[i], points[j] );
                }}
                if ( candidate ) {
        
                    edges.push_back( Segment(points[i],points[j]) );
                }
    }}}
}
  
  
// Sorting edges and vertices counterclockwise
    
void sortEdges( SegmentVector &edges, PointVector &hull ) {

    int n = edges.size();
  
    hull.clear();
  
    int k = 0;
    Point p = edges[0].source();
    hull.push_back( p );
  
    for ( int i=1; i<n; i++ ) {
  
        p = edges[k].target();
        edges.erase( edges.begin()+k );
    
        for ( int j=0; j<edges.size(); j++ ) {
    
            Point q = edges[j].source();
      
            if ( p == q ) {
                k = j;
                break;
        }}
        hull.push_back( p );
    }
    edges.erase( edges.begin() );
}


// Naive driver

void naiveDriver( PointVector &points, PointVector &hull ) {
    SegmentVector edges;

    naiveConvexHull(points, edges);
    sortEdges(edges, hull);
}


// 1. Jarvis' March

int jarvisMarch( PointVector &points, PointVector &hull ) {

    int n = points.size();
  
    hull.clear();
  
    // leftmost point
  
    int leftmost = 0;
  
    for ( int i=1; i<n; i++ ) {
        if ( before(points[i],points[leftmost]) ) {
        leftmost = i;
    }}
    int k = leftmost;
  
    do {
  
        hull.push_back( points[k] );
    
        int j = (k + 1) % n;
    
        for ( int i=(j+1)%n; i!=k; i=(i+1)%n ) {
    
            if ( liesToTheLeftOf( points[j], points[k], points[i] ) ) {
                j = i;
        }}
        k = j;
  
    } while ( k != leftmost );
  
    return hull.size();
}


// 2. Graham's scan


// Lower hull: to be invoked before UpperHull!

void lowerHull( PointVector &points, PointVector &hull ) {

    int n = points.size();
  
    hull.push_back( points[0] );
    hull.push_back( points[1] );
  
    for ( int i=2; i<n; i++ ) {
  
        int j = hull.size() - 1;
    
        while ( (j > 0) && !CGAL::left_turn(hull[j-1],hull[j],points[i])) {
    
            hull.erase( hull.begin()+j );
            j = j - 1;
        }
        hull.push_back( points[i] );
    }
}


// Upper hull: to be invoked after LowerHull!

void upperHull( PointVector &points, PointVector &hull ) {

    int n = points.size();
    int k = hull.size() - 1;
  
    hull.push_back( points[n-2] );
  
    for ( int i=n-3; i>=0; i-- ) {
  
        int j = hull.size() - 1;
    
        while ( (j > k) && !CGAL::left_turn(hull[j-1],hull[j],points[i]) ) {
    
            hull.erase( hull.begin()+j );
            j = j - 1;
        }
        hull.push_back( points[i] );
    }
    hull.erase( hull.end()-1 );
}


// Graham's style algorithm

int grahamConvexHull( PointVector &points, PointVector &hull ) {

    int n = points.size();

    hull.clear();
  
    if ( n < 3 ) {

        for ( int i=0; i<n; i++ ) {
            hull.push_back( points[i] );
        }

        return n;
    }
    // otherwise
  
    std::sort( points.begin(), points.end(), before );
  
    lowerHull( points, hull );
    upperHull( points, hull );
  
    return hull.size();
}


// 3. Divide-et-impera approach (Preparata & Hong)


// Counterclockwise triangle

void ccwTriangle( PointVector &triangle ) {

    if ( CGAL::right_turn(triangle[0],triangle[1],triangle[2]) ) {
  
        Point q = triangle[1];
        triangle[1] = triangle[2];
        triangle[2] = q;
    }
}


// Base-case construction

void baseCaseConstruction( int k1, int k2, PointVector &points, PointVector &hull, int &leftmost, int &rightmost ) {

    leftmost = 0;   // either 2 or 3 points
    rightmost = 0;
  
    for ( int i=k1; i<k2; i++ ) {
        hull.push_back( points[i] );
    }
    if ( k2 == k1 + 3 ) {
        ccwTriangle( hull );
    }
    if ( before(hull[0],hull[1]) ) {
        leftmost = 0;
        rightmost = 1;
    } else {
        leftmost = 1;
        rightmost = 0;
    }
    if ( hull.size() > 2 ) {
        if ( before(hull[2],hull[leftmost]) ) {
            leftmost = 2;
        } else if ( before(hull[rightmost],hull[2]) ) {
            rightmost = 2;
    }}
}


// Recursive construction

void recursiveConstruction( int k1, int k2, PointVector &points, PointVector &hull, int &leftmost, int &rightmost ) {

    if ( k2 <= k1 + 3 ) {  // either 2 or 3 points: brute force or whatever...
  
        baseCaseConstruction( k1, k2, points, hull, leftmost, rightmost );
    
        return;
    }
    // otherwise
  
    int k = (k1 + k2) / 2;
    int l1, r1, l2, r2;
  
    PointVector leftHull;
    PointVector rightHull;
  
    recursiveConstruction( k1, k, points, leftHull, l1, r1 );
    recursiveConstruction( k, k2, points, rightHull, l2, r2 );
  
    int n1 = leftHull.size();
    int n2 = rightHull.size();
  
    int u1 = r1;  // search for endpoints of the lower connecting edge,
    int u2 = l2;  // starting from lexicographically closest vertices of the hulls
  
    while ( ( (u1 != l1) && !CGAL::left_turn(leftHull[(u1+n1-1)%n1],leftHull[u1],rightHull[u2]) ) ||
            ( (u2 != r2) && !CGAL::left_turn(leftHull[u1],rightHull[u2],rightHull[(u2+1)%n2]) )
            ) {
    
        while ( (u1 != l1) && !CGAL::left_turn(leftHull[(u1+n1-1)%n1],leftHull[u1],rightHull[u2]) ) {
            u1 = (u1 + n1 - 1) % n1;
        }
        while ( (u2 != r2) && !CGAL::left_turn(leftHull[u1],rightHull[u2],rightHull[(u2+1)%n2]) ) {
            u2 = (u2 + 1) % n2;
        }
    }
    int v1 = r1;  // search for endpoints of the upper connecting edge,
    int v2 = l2;  // starting again from lexicographically closest vertices of the hulls
  
    while ( ( (v2 != r2) && !CGAL::left_turn(rightHull[(v2+n2-1)%n2],rightHull[v2],leftHull[v1]) ) ||
            ( (v1 != l1) && !CGAL::left_turn(rightHull[v2],leftHull[v1],leftHull[(v1+1)%n1]) )
            ) {
      
        while ( (v2 != r2) && !CGAL::left_turn(rightHull[(v2+n2-1)%n2],rightHull[v2],leftHull[v1]) ) {
            v2 = (v2 + n2 - 1) % n2;
        }
        while ( (v1 != l1) && !CGAL::left_turn(rightHull[v2],leftHull[v1],leftHull[(v1+1)%n1]) ) {
            v1 = (v1 + 1) % n1;
        }
    }
    hull.push_back( leftHull[v1] );  // building the left chain of the convex hull
    leftmost = 0;
    if ( u1 != v1 ) {
        int i = v1;
        do {
            i = (i + 1) % n1;
            hull.push_back( leftHull[i] );
            if ( before(leftHull[i],hull[leftmost]) ) {  // leftmost hull vertex?
                leftmost = hull.size() - 1;
            }
        } while ( i != u1 );
    }
    hull.push_back( rightHull[u2] );  // building the right chain of the convex hull
    rightmost = hull.size() - 1;
    if ( v2 != u2 ) {
        int i = u2;
        do {
            i = (i + 1) % n2;
            hull.push_back( rightHull[i] );
            if ( before(hull[rightmost],rightHull[i]) ) {  // rightmost hull vertex?
                rightmost = hull.size() - 1;
            }
        } while ( i != v2 );
    }
}


// Divide-et-impera algorithm

int recursiveConvexHull( PointVector &points, PointVector &hull ) {

    hull.clear();
    int n = points.size();
    int lm, rm;
  
    if ( n < 3 ) {

        for ( int i=0; i<n; i++ ) {
            hull.push_back( points[i] );
        }

        return n;
    }
    // otherwise
  
    std::sort( points.begin(), points.end(), before );
  
    recursiveConstruction( 0, n, points, hull, lm, rm );
  
    return hull.size();
}


// 4. Randomized incremental convex hull


// Data structure to represent conflict information

class EdgeInfo {

public:

    int s, t;
    std::vector<int> conflicts;
  
    EdgeInfo *prec, *next;
  
    EdgeInfo( int i, int j );

};

EdgeInfo::EdgeInfo( int i, int j ) {

    s = i;
    t = j;
}


// Intersection between ray and segment

bool areInConflict( Point q0, Point q1, Point v1, Point v2 ) {
  
    Ray r( q0, q1 );
    Segment e( v1, v2 );
  
    return CGAL::do_intersect( r, e ) && CGAL::right_turn( v1, v2, q1 );
  
    // return liesToTheLeftOf( v2, v1, q1 );  // another opportunity!
}


// Randomization

void shuffle( PointVector &points ) {
  
    // if appropriate, points can be shuffled here
  
    int n = points.size() - 3;
    int k = 3;
  
    Triangle triangle( points[0], points[1], points[2] );
  
    while ( triangle.is_degenerate() && (k < n) ) {  // degeneracy?
  
        int i = std::rand() % 3;
        int j = std::rand() % n + 3;
    
        Point q = points[i];    // swap points pi and pj
        points[i] = points[j];
        points[j] = q;
    
        triangle = Triangle( points[0], points[1], points[2] );
        k = k + 1;
    }
}


// Linking edges

void chain( EdgeInfo *e0, EdgeInfo *e1, EdgeInfo *e2, EdgeInfo *e3 ) {
  
    e0 -> next = e1;  // counterclockwise chain: e0 - e1 - e2 - e3
    e1 -> prec = e0;
    e1 -> next = e2;
    e2 -> prec = e1;
    e2 -> next = e3;
    e3 -> prec = e2;
}


// Randomized incremental convex hull

int randomizedIncrConvexHull( PointVector &points, PointVector &hull ) {

    hull.clear();
    int n = points.size();
  
    if ( n < 3 ) {

        for ( int i=0; i<n; i++ ) {
            hull.push_back( points[i] );
        }

        return n;
    }
    // otherwise
  
    shuffle( points );
  
    ccwTriangle( points );  // core triangle: initial convex hull
  
    EdgeInfo *t0 = new EdgeInfo( 0, 1 );
    EdgeInfo *t1 = new EdgeInfo( 1, 2 );
    EdgeInfo *t2 = new EdgeInfo( 2, 0 );
  
    chain( t0, t1, t2, t0 );
  
    Point q = CGAL::centroid( points[0], points[1], points[2] );  // Assumption: p0, p1, p2 not collinear (!)
  
    std::vector<EdgeInfo*> edge( n, (EdgeInfo*) NULL );  // conflict edge for each point
  
    for ( int i=3; i<n; i++ ) {  // conflict links for the edges of the core triangle
  
        if ( areInConflict( q, points[i], points[t0->s], points[t0->t] ) ) {
            edge[i] = t0;
            t0 -> conflicts.push_back( i );
        } else if ( areInConflict( q, points[i], points[t1->s], points[t1->t] ) ) {
            edge[i] = t1;
            t1 -> conflicts.push_back( i );
        } else if ( areInConflict( q, points[i], points[t2->s], points[t2->t] ) ) {
            edge[i] = t2;
            t2 -> conflicts.push_back( i );
    }}
  
    EdgeInfo *e1 = t0;  // just in case the final convex hull has only three vertices...
    EdgeInfo *e2 = t1;
  
    for ( int i=3; i<n; i++ ) {  // for each point p to be added
  
        if ( edge[i] != NULL ) {   // outside current hull H
    
            std::vector<int> conflicts( edge[i]->conflicts );  // conflicts and neighboring edges
            EdgeInfo *e0 = edge[i] -> prec;
            EdgeInfo *e3 = edge[i] -> next;
      
            delete ( edge[i] );      // edge e in conflict with p is removed
            edge[i] = NULL;
      
            // recovering convexity and registering conflicts to re-consider after adding new point p...
      
            while ( !CGAL::left_turn( points[e0->s], points[e0->t], points[i] ) ) {
      
                conflicts.insert( conflicts.end(), e0->conflicts.begin(), e0->conflicts.end() );
                e0 = e0 -> prec;
                delete ( e0 -> next );  // edge removed while walking backwards from e on H's boundary
            }
            while ( !CGAL::left_turn( points[i], points[e3->s], points[e3->t] ) ) {
      
                conflicts.insert( conflicts.end(), e3->conflicts.begin(), e3->conflicts.end() );
                e3 = e3 -> next;
                delete ( e3 -> prec );  // edge removed while walking forward from e on H's boundary
            }
            e1 = new EdgeInfo( e0->t, i );  // new pair of edges
            e2 = new EdgeInfo( i, e3->s );
      
            chain( e0, e1, e2, e3 );  // updated chain: e0 - e1 - e2 - e3 (counterclockwise)
      
            for ( int k=0; k<conflicts.size(); k++ ) {  // updating conflict graph links
      
                int j = conflicts[k];
                edge[j] = NULL;
        
                if ( areInConflict( q, points[j], points[e1->s], points[e1->t] ) ) {
        
                    edge[j] = e1;
                    e1 -> conflicts.push_back( j );
        
                } else if ( areInConflict( q, points[j], points[e2->s], points[e2->t] ) ) {
        
                    edge[j] = e2;
                    e2 -> conflicts.push_back( j );
            }}
    }}
    EdgeInfo *he = e1;  // building polygonal hull
    do {
  
        hull.push_back( points[he->s] );
        he = he -> next;
  
    } while ( he != e1 );
  
    return hull.size();
}


// Akl–Toussaint heuristic (pre-processing/filtering of points)

void heuristicAklToussaint( PointVector &points ) {

    Point top_p          = points[0],
          bottom_p       = points[0],
          left_p         = points[0],
          right_p        = points[0],
          top_right_p    = points[0],
          top_left_p     = points[0],
          bottom_left_p  = points[0],
          bottom_right_p = points[0];

    for (PointVector::iterator it = points.begin(); it != points.end(); ++it) {

        Point p = *it;

        if (p.y() < bottom_p.y()) bottom_p = p;
        if (p.x() > right_p.x())  right_p = p;
        if (p.y() > top_p.y())    top_p = p;
        if (p.x() < left_p.x())   left_p = p;

        if (p.x() + p.y() > top_right_p.x() + top_right_p.y())
            top_right_p = p;
        if (p.x() - p.y() < top_left_p.x() - top_left_p.y())
            top_left_p = p;
        if (p.x() + p.y() < bottom_left_p.x() + bottom_left_p.y())
            bottom_left_p = p;
        if (p.x() - p.y() > bottom_right_p.x() - bottom_right_p.y())
            bottom_right_p = p;

    }

    PointVector filtered_points;
    Polygon octagon;
    octagon.push_back(bottom_p);
    filtered_points.push_back(bottom_p);

    if (right_p != bottom_p) {

        if (bottom_right_p != bottom_p && bottom_right_p != right_p) {

            octagon.push_back(bottom_right_p);
            filtered_points.push_back(bottom_right_p);

        }

        octagon.push_back(right_p);
        filtered_points.push_back(right_p);

    }
    
    if (top_p != bottom_p && top_p != right_p) {

        if (top_right_p != right_p && top_right_p != top_p) {

            octagon.push_back(top_right_p);
            filtered_points.push_back(top_right_p);

        }

        octagon.push_back(top_p);
        filtered_points.push_back(top_p);

    }

    if (left_p != bottom_p && left_p != right_p && left_p != top_p) {

        if (top_left_p != top_p && top_left_p != left_p) {

            octagon.push_back(top_left_p);
            filtered_points.push_back(top_left_p);

        }

        octagon.push_back(left_p);
        filtered_points.push_back(left_p);

        if (bottom_left_p != left_p && bottom_left_p != bottom_p) {

            octagon.push_back(bottom_left_p);
            filtered_points.push_back(bottom_left_p);

        }

    }

    for (PointVector::iterator it = points.begin(); it != points.end(); ++it) {

        Point p = *it;

        bool is_inside = (octagon.bounded_side(p) == CGAL::ON_BOUNDED_SIDE ||
                          octagon.bounded_side(p) == CGAL::ON_BOUNDARY);

        if (!is_inside)
            filtered_points.push_back(p);

    }

    points = filtered_points;

}


// ----------  Convex Hull Driver  ----------

template <class InputIterator, class OutputIterator>
inline
OutputIterator 
convex_hull_2(InputIterator first, InputIterator last, 
              OutputIterator  result )
{
    PointVector points(first, last);
    PointVector hull;

    if (points.size() == 1) {
        *result = *first;
        ++result;
    } else if (points.size() > 1) {
        heuristicAklToussaint(points);
        randomizedIncrConvexHull(points, hull);

        for (PointVector::iterator it = hull.begin(); it != hull.end(); ++it) {
            *result = *it;
            ++result;
        }
    }

    return result;
}

} //namespace CGP

#endif // CONVEX_HULL_H
