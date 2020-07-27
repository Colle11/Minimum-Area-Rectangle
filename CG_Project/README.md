# README

### Prerequisites

*CG Project* requires *c++14*, *CGAL* and *Qt5*.  

### Compiling

1. <pre> cd /path/to/Collevati_CG/CG_Project        # go to the project directory </pre>  
2. <pre> cmake -DCMAKE_BUILD_TYPE=Release  .  .  -Bbuild        # configure the project (out-of-source build) </pre>  
3. <pre> cd build/ </pre>  
4. <pre> make        # build the project </pre>  
5. <pre> ./CG_Project        # launch the project </pre>  

### Cleaning

1. <pre> cd /path/to/Collevati_CG/CG_Project/build        # go to the build directory </pre>  
2. <pre> make clean        # clean the project </pre>  
3. <pre> rm -rf ./*        # remove configuration files from the build directory </pre>  

### Source files

- `CG_Project.cpp`: *CG Project* driver, based on `Bounding_volumes.cpp` (Copyright &copy; 2010 ETH Zurich). In function `on_actionInsertRandomPoints_triggered()` you can change the random generation of points in different ways;  
- `Environment.h`: general context based on the *CGAL* library;  
- `Utils.h`: supporting algorithms and data structures;  
- `Convex_Hull.h`: Planar Convex Hull algorithms (Copyright &copy; [dott. Claudio Mirolo](https://users.dimi.uniud.it/~claudio.mirolo/teaching/geom_comput/pages/examples/convex_hull.cpp "dott. Claudio Mirolo")) (added Akl–Toussaint heuristic);  
- `Min_Rect.h`: Minimum-Area Rectangle Containing a Set of Points algorithms.  

