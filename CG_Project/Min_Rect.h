#ifndef MIN_RECT_H
#define MIN_RECT_H

/*
 * Minimum-Area Rectangle Containing a Set of Points Algorithms
 *
 * February 2020
 */

#include <limits>
#include <iterator>
#include <array>
#include <vector>
#include <utility>
#include <algorithm>
#include "Environment.h"

// debug
// #include <iostream>
//

namespace CGP {

// ----------  Min Rect Algorithms  ----------


// Basic tools


// Vector normalization

void Normalize( Vector &v ) {

    v /= CGAL::sqrt( v.squared_length() );

}


// Data structure to represent rectangle

class Rectangle {

public:

    VectorVector axis;      // rectangle X/Y-axis (object space)
    Float width, height, area;
    PointVector rect;       // 4 rectangle vertices
    std::array<int,4> index;        // order: bottom, right, top, left

};


// One-point rect

Rectangle onePointRect(Point p) {

    Rectangle rect;

    Point arbitrary = p + Vector(1.0, 0.0);        // arbitrary point
    Vector x_axis(p, arbitrary);                   // arbitrary x_axis
    Normalize(x_axis);      // length of x_axis is 1
    Vector y_axis( x_axis.perpendicular(CGAL::COUNTERCLOCKWISE) );      // arbitrary y_axis derived from the x_axis; it's already normalized

    rect.axis.clear();
    rect.axis.push_back( x_axis );
    rect.axis.push_back( y_axis );
    rect.width = 0.0;
    rect.height = 0.0;
    rect.area = 0.0;

    rect.rect.clear();
    for (int i = 0; i < 4; ++i)
        rect.rect.push_back( p );       // first = second = third = fourth rectangle vertex
    
    rect.index.fill(0);
        
    return rect;

}


// 0. Exhaustive Search Algorithm

Rectangle exhaustiveMinRect( PointVector &hull ) {

    // Assumption: the convex hull have no triple of collinear vertices

    int n = hull.size();

    Rectangle minRect;
    minRect.area = std::numeric_limits<double>::max();      // largest finite double number

    // degenerate case: one-point convex hull
    if (n == 1) {

        minRect = onePointRect(hull[0]);
        
        return minRect;

    }
    // otherwise

    for (int prev = n-1, next = 0; next < n; prev = next++) {       // iterate over the edges of the convex hull
        
        Point origin = hull[prev];
        Vector x_axis(origin, hull[next]);      // line of the edge
        Normalize(x_axis);      // length of x_axis is 1
        Vector y_axis( x_axis.perpendicular(CGAL::COUNTERCLOCKWISE) );      // line perpendicular to the convex hull edge; it's already normalized
        Float x_min(0.0), x_max(0.0);       // projection onto x_axis is [x_min, x_max]
        Float y_max(0.0);       // projection onto y_axis is [0, y_max], y_min = 0 is guaranteed

        for (int i = 0; i < n; ++i) {

            Vector v(origin, hull[i]);

            // projection of the convex hull vertices onto the line of the edge
            Float dot_prod( CGAL::scalar_product(x_axis, v) );

            if (dot_prod < x_min)
                x_min = dot_prod;
            else if (dot_prod > x_max)
                x_max = dot_prod;
            
            // projection of the convex hull vertices onto the line perpendicular to the polygon edge
            dot_prod = CGAL::scalar_product(y_axis, v);

            if (dot_prod > y_max)
                y_max = dot_prod;

        }

        Float width( x_max - x_min );       // the maximum distance between the projected vertices onto the line of the edge is the width of the rectangle
        Float height( y_max );              // the maximum distance between the projected vertices onto the line perpendicular to the convex hull edge is the height of the rectangle
        Float area( width * height );

        if (area < minRect.area) {          // of all n rectangles, choose the one with the minimum area

            minRect.axis.clear();
            minRect.axis.push_back( x_axis );
            minRect.axis.push_back( y_axis );
            minRect.width = width;
            minRect.height = height;
            minRect.area = area;

            minRect.rect.clear();

            Vector x_max_disp( x_axis * CGAL::to_double(x_max) );           // x_max displacement from the origin
            minRect.rect.push_back( origin + x_max_disp );                  // first rectangle vertex

            Vector y_max_disp( y_axis * CGAL::to_double(minRect.height) );  // y_max displacement from the first rectangle vertex
            minRect.rect.push_back( minRect.rect[0] + y_max_disp );         // second rectangle vertex: calculated from the first one

            Vector x_min_disp( -x_axis * CGAL::to_double(minRect.width) );  // x_min displacement from the second rectangle vertex
            minRect.rect.push_back( minRect.rect[1] + x_min_disp );         // third rectangle vertex: calculated from the second one

            minRect.rect.push_back( minRect.rect[0] + x_min_disp );         // fourth rectangle vertex: calculated from the first one

        }

    }

    return minRect;

}


// 1. Rotating Calipers Algorithm


// Calculate the rectangle vertices

void calculateRectVertices(Rectangle &rect, PointVector &hull) {

    // Assumption: normalized x/y-axis

    rect.rect.clear();

    Vector b_r_dist(hull[rect.index[0]], hull[rect.index[1]]);          // distance from b to r supporting vertices
    Float x_max( CGAL::scalar_product(rect.axis[0], b_r_dist) );        // x-axis projection
    Vector x_max_disp( rect.axis[0] * CGAL::to_double(x_max) );         // x_max displacement from b
    rect.rect.push_back( hull[rect.index[0]] + x_max_disp );            // first rectangle vertex

    Vector y_max_disp( rect.axis[1] * CGAL::to_double(rect.height) );   // y_max displacement from the first rectangle vertex
    rect.rect.push_back( rect.rect[0] + y_max_disp );                   // second rectangle vertex: calculated from the first one

    Vector x_min_disp( -rect.axis[0] * CGAL::to_double(rect.width) );   // x_min displacement from the second rectangle vertex
    rect.rect.push_back( rect.rect[1] + x_min_disp );                   // third rectangle vertex: calculated from the second one

    rect.rect.push_back( rect.rect[0] + x_min_disp );                   // fourth rectangle vertex: calculated from the first one

}


// Convert the point p to the given coordinate system

Point convertToCoordinateSystem(Point &p, Point &origin, VectorVector &axis) {

    Vector disp(origin, p);     // p-displacement from the origin
    return Point(CGAL::scalar_product(axis[0], disp), CGAL::scalar_product(axis[1], disp));

}


// Compute the initial smallest rectangle for the convex hull edge <prev,next>

Rectangle initialSmallestRect(int prev, int next, PointVector &hull) {

    int n = hull.size();
    
    Rectangle rect;

    // degenerate case: one-point convex hull
    if (n == 1) {

        rect = onePointRect(hull[0]);
        
        return rect;

    }
    // otherwise

    rect.axis.clear();
    rect.axis.push_back( Vector(hull[prev], hull[next]) );     // x_axis: line of the edge; NOT normalized
    rect.axis.push_back( Vector( rect.axis[0].perpendicular(CGAL::COUNTERCLOCKWISE) ) );     // y_axis: line perpendicular to the convex hull edge; NOT normalized

    for (int i = 0; i < 4; ++i)
        rect.index[i] = next;

    Point origin = hull[next];
    Point zero(0.0, 0.0);
    std::array<Point,4> support = { zero, zero, zero, zero };      // order: bottom, right, top, left

    for (int i = 0; i < n; ++i) {

        Point conv_p( convertToCoordinateSystem(hull[i], origin, rect.axis) );      // convert the hull[i] point to the rectangle coordinate system

        // the assumption of no triple of collinear vertices guarantees that rect.index[0] is next
        // the right-most vertex of the bottom edge is hull[next]

        if ( conv_p.x() > support[1].x() || ( conv_p.x() == support[1].x() && conv_p.y() > support[1].y() ) ) {
            // New right maximum OR same right maximum but closer to top
            rect.index[1] = i;
            support[1] = conv_p;
        }

        if ( conv_p.y() > support[2].y() || ( conv_p.y() == support[2].y() && conv_p.x() < support[2].x() ) ) {
            // New top maximum OR same top maximum but closer to left
            rect.index[2] = i;
            support[2] = conv_p;
        }

        if ( conv_p.x() < support[3].x() || ( conv_p.x() == support[3].x() && conv_p.y() < support[3].y() ) ) {
            // New left minimum OR same left minimum but closer to bottom
            rect.index[3] = i;
            support[3] = conv_p;
        }

    }

    rect.width = support[1].x() - support[3].x();       // width = V_r.x - V_l.x; NOT normalized width
    rect.height = support[2].y();                       // because support[0] = zero; NOT normalized height
    rect.area = CGAL::approximate_division( (rect.width * rect.height), Float( rect.axis[0].squared_length() ) );       // normalized area
    rect.width = CGAL::approximate_division( rect.width, Float( CGAL::sqrt( rect.axis[0].squared_length() ) ) );        // normalized width
    rect.height = CGAL::approximate_division( rect.height, Float( CGAL::sqrt( rect.axis[0].squared_length() ) ) );      // normalized height

    return rect;

}


// Compute the angles for the polygon edges emanating from the support vertices of the current rect

std::vector<std::pair<Float, int>> computeAngles( PointVector &hull, Rectangle &rect ) {

    int n = hull.size();

    std::vector<std::pair<Float, int>> A;

    for (int prev = 3, next = 0; next < 4; prev = next++) {

        if (rect.index[prev] != rect.index[next]) {

            Vector rect_edge_dir;

            if (prev == 3)
                rect_edge_dir = -rect.axis[1];      // -y_axis
            else if (prev == 0)
                rect_edge_dir = rect.axis[0];       // x_axis
            else if (prev == 1)
                rect_edge_dir = rect.axis[1];       // y_axis
            else if (prev == 2)
                rect_edge_dir = -rect.axis[0];      // -x_axis
            
            int s_v = rect.index[prev];     // support vertex index
            int n_v = (s_v + 1) % n;        // next vertex w.r.t. s_v
            Vector poly_edge( hull[s_v], hull[n_v] );       // polygon edge
            Float dot_prod( CGAL::scalar_product( rect_edge_dir, poly_edge.perpendicular(CGAL::CLOCKWISE) ) );
            Float sin_theta_sqr( CGAL::approximate_division( (dot_prod * dot_prod), Float( poly_edge.squared_length() ) ) );      // angle quantity
            A.push_back( std::make_pair(sin_theta_sqr, prev) );     // store the pair ("angle", index)

        }

    }

    std::sort(A.begin(), A.end(), [](auto &left, auto &right) {     // sort A
        return left.first < right.first;
    });

    return A;

}


// Update the support information

bool updateSupport( PointVector &hull, Rectangle &rect, std::vector<std::pair<Float, int>> &A, std::vector<bool> &visited ) {

    int n = hull.size();

    std::vector<int> M;     // minimum angle indeces; when M has multiple elements, the minimum angle is attained multiple times

    std::vector<std::pair<Float, int>>::iterator first = A.begin();
    std::vector<std::pair<Float, int>>::iterator last  = A.end();
    std::vector<std::pair<Float, int>>::iterator smallest = first;
    ++first;

    M.push_back( (*smallest).second );    // A is NOT empty

    while (first != last && (*smallest).first == (*first).first) {
    
        M.push_back( (*first).second );
        ++first;

    }

    // index update
    for (std::vector<int>::iterator it = M.begin(); it != M.end(); ++it) {

        rect.index[*it] = (rect.index[*it] + 1) % n;    // for each m in M, index[m]++ modulo n

    }

    int m_0 = *std::min_element(M.begin(), M.end());    // the smallest element of M
    std::array<int,4>::iterator i_m_0 = std::next(rect.index.begin(), m_0);
    std::rotate(rect.index.begin(), i_m_0, rect.index.end());       // cycle the elements of the updated I so that index[m_0] occurs first, which produces I'

    if (visited[rect.index[0]])
        return false;   // this polygon edge was already processed
    else
        visited[rect.index[0]] = true;

    // axis update
    int prev = rect.index[0] - 1;

    if (prev == -1)
        prev = n - 1;
    
    Point origin = hull[prev];
    rect.axis[0] = Vector( origin, hull[rect.index[0]] );                           // x_axis: line of the edge; NOT normalized
    rect.axis[1] = Vector( rect.axis[0].perpendicular(CGAL::COUNTERCLOCKWISE) );    // y_axis: line perpendicular to the convex hull edge; NOT normalized

    // Point conv_b( convertToCoordinateSystem( hull[rect.index[0]], origin, rect.axis) );     // convert the V_b point to the rectangle coordinate system
    Point conv_r( convertToCoordinateSystem( hull[rect.index[1]], origin, rect.axis) );     // convert the V_r point to the rectangle coordinate system
    Point conv_t( convertToCoordinateSystem( hull[rect.index[2]], origin, rect.axis) );     // convert the V_t point to the rectangle coordinate system
    Point conv_l( convertToCoordinateSystem( hull[rect.index[3]], origin, rect.axis) );     // convert the V_l point to the rectangle coordinate system

    rect.width = conv_r.x() - conv_l.x();       // width = V_r.x - V_l.x; NOT normalized width
    rect.height = conv_t.y();                   // height = V_t.y; NOT normalized height
    rect.area = CGAL::approximate_division( (rect.width * rect.height), Float( rect.axis[0].squared_length() ) );       // normalized area
    rect.width = CGAL::approximate_division( rect.width, Float( CGAL::sqrt( rect.axis[0].squared_length() ) ) );        // normalized width
    rect.height = CGAL::approximate_division( rect.height, Float( CGAL::sqrt( rect.axis[0].squared_length() ) ) );      // normalized height
    
    return true;

}


// Rotating Calipers Driver

Rectangle rotatingCalipersMinRect( PointVector &hull ) {

    // Assumption: the convex hull have no triple of collinear vertices

    int n = hull.size();

    std::vector<bool> visited(n);     // mark visited edges
    std::fill(visited.begin(), visited.end(), false);

    Rectangle minRect = initialSmallestRect(n-1, 0, hull);
    visited[minRect.index[0]] = true;
    Rectangle rect = minRect;

    for (int i = 0; i < n; ++i) {

        std::vector<std::pair<Float, int>> A( computeAngles(hull, rect) );     // set of pairs ("angle", index) sorted on the "angle" component

        if (A.empty()) break;       // never the case
        else {

            if ( !updateSupport( hull, rect, A, visited ) ) break;      // the rect polygon edge was already processed, so the search is over

            if (rect.area < minRect.area)
                minRect = rect;

        }

    }

    Normalize(minRect.axis[0]);    // length of x_axis is 1
    Normalize(minRect.axis[1]);    // length of y_axis is 1

    calculateRectVertices(minRect, hull);

    return minRect;

}


// ----------  Min Rect Driver  ----------

template <class InputIterator, class OutputIterator>
inline
OutputIterator
min_rectangle_2(InputIterator first, InputIterator last,
                OutputIterator result )
{
    PointVector hull(first, last);
    Rectangle minRect = rotatingCalipersMinRect(hull);
    
    for (PointVector::iterator it = minRect.rect.begin(); it != minRect.rect.end(); ++it) {
        *result = *it;
        ++result;
    }

    return result;
}

} //namespace CGP

#endif // MIN_RECT_H
