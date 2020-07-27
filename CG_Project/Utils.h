#ifndef UTILS_H
#define UTILS_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <random>
#include <algorithm>
#include <vector>
#include <iterator>
#include "Environment.h"

namespace CGP {


// PRNG

double Random(double a, double b) { 

    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(a,b);
    return dis(gen);
}


// Data structure to represent randomly generated points on an ellipse

class Random_points_on_ellipse_2 {

public:

    PointVector ellipse_points;

    Random_points_on_ellipse_2( int number_of_points, double a, double b );

};

Random_points_on_ellipse_2::Random_points_on_ellipse_2( int number_of_points, double a, double b ) {

    std::vector<double> angles;

    for (int i = 0; i < number_of_points; ++i) {

        angles.push_back( Random(0.0, 2*M_PI) );

    }

    std::sort( angles.begin(), angles.end() );

    for (std::vector<double>::iterator it = angles.begin(); it != angles.end(); ++it) {

        ellipse_points.push_back( Point(a * std::cos(*it), b * std::sin(*it)) );

    }

}


} //namespace CGP

#endif // UTILS_H
