#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <vector>

// CGAL headers

#include <CGAL/Cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
// #include <CGAL/Gmpq.h>
#include <CGAL/MP_Float.h>
#include <CGAL/number_utils.h>

namespace CGP {

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
// typedef CGAL::Cartesian<CGAL::Gmpq> K;
// typedef CGAL::Cartesian<double> K;
// typedef CGAL::Cartesian<float> K;
typedef CGAL::MP_Float Float;     // an object of the class MP_Float is able to represent a floating point value with arbitrary precision; for more details see: https://doc.cgal.org/latest/Number_types/classCGAL_1_1MP__Float.html

typedef K::Point_2 Point;               // CGAL::Point_2<K>
typedef K::Segment_2 Segment;           // CGAL::Segment_2<K>
typedef K::Ray_2 Ray;                   // CGAL::Ray<K>
typedef K::Triangle_2 Triangle;         // CGAL::Triangle<K>
typedef K::Vector_2 Vector;             // CGAL::Vector_2<K>
typedef CGAL::Polygon_2<K> Polygon;     // CGAL::Polygon_2<K>

typedef std::vector<Point> PointVector;
typedef std::vector<Segment> SegmentVector;
typedef std::vector<Vector> VectorVector;

} //namespace CGP

#endif // ENVIRONMENT_H
