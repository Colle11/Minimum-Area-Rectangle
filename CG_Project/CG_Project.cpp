#include <fstream>
#include <cmath>

// CGAL headers
#include <CGAL/Cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/convex_hull_2.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/random_convex_set_2.h>
#include <CGAL/random_polygon_2.h>
#include <CGAL/Polygon_2.h>
//#include <CGAL/min_quadrilateral_2.h>

// Qt headers
#include <QtGui>
#include <QString>
#include <QActionGroup>
#include <QFileDialog>
#include <QInputDialog>
#include <QGraphicsRectItem>

// GraphicsView items and event filters (input classes)

#include <CGAL/Qt/PointsGraphicsItem.h>
#include <CGAL/Qt/PolygonGraphicsItem.h>

// for viewportsBbox
#include <CGAL/Qt/utility.h>
  
#include <CGAL/Qt/GraphicsViewPolylineInput.h>

// the two base classes
#include "ui_CG_Project.h"
#include <CGAL/Qt/DemosMainWindow.h>

#include "Convex_Hull.h"
#include "Min_Rect.h"
#include "Utils.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef K::Iso_rectangle_2 Iso_rectangle_2;
typedef CGAL::Polygon_2<K> Polygon_2;

typedef CGAL::Creator_uniform_2<double,Point_2> Creator;

class MainWindow :
    public CGAL::Qt::DemosMainWindow,
    public Ui::CG_Project
{
    Q_OBJECT
  
private:  
    Polygon_2 convex_hull, min_rectangle;
    QGraphicsScene scene;  

    std::vector<Point_2> points; 
    CGAL::Qt::PointsGraphicsItem<std::vector<Point_2> > * pgi;
    CGAL::Qt::PolygonGraphicsItem<Polygon_2> * convex_hull_gi;
    CGAL::Qt::PolygonGraphicsItem<Polygon_2> * min_rectangle_gi;

    CGAL::Qt::GraphicsViewPolylineInput<K> * pi;

public:
    MainWindow();

public Q_SLOTS:

    void update();

    void update_from_points();

    void processInput(CGAL::Object o);

    void on_actionShowMinRectangle_toggled(bool checked);

    void on_actionShowConvexHull_toggled(bool checked);

    void on_actionInsertPoint_toggled(bool checked);
  
    void on_actionInsertRandomPoints_triggered();

    void on_actionLoadPoints_triggered();

    void on_actionSavePoints_triggered();

    void on_actionClear_triggered();

    void on_actionRecenter_triggered();

    virtual void open(QString fileName);

Q_SIGNALS:
    void changed();
};


MainWindow::MainWindow()
  : DemosMainWindow()
{
    setupUi(this);

    QObject::connect(this, SIGNAL(changed()), this, SLOT(update()));

    // Graphics Item for the input point set
    pgi = new CGAL::Qt::PointsGraphicsItem<std::vector<Point_2> >(&points);

    QObject::connect(this, SIGNAL(changed()),
		    pgi, SLOT(modelChanged()));
    pgi->setVerticesPen(QPen(Qt::black, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    scene.addItem(pgi);


    // Graphics Item for the convex hull
    convex_hull_gi = new CGAL::Qt::PolygonGraphicsItem<Polygon_2>(&convex_hull);

    QObject::connect(this, SIGNAL(changed()),
		    convex_hull_gi, SLOT(modelChanged()));
    convex_hull_gi->setEdgesPen(QPen(Qt::black, 0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    scene.addItem(convex_hull_gi);


    // Graphics Item for the min rectangle
    min_rectangle_gi = new CGAL::Qt::PolygonGraphicsItem<Polygon_2>(&min_rectangle);

    QObject::connect(this, SIGNAL(changed()),
		    min_rectangle_gi, SLOT(modelChanged()));
    min_rectangle_gi->setEdgesPen(QPen(Qt::green, 0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    scene.addItem(min_rectangle_gi);


    // Setup input handlers. They get events before the scene gets them
    // and the input they generate is passed to the triangulation with 
    // the signal/slot mechanism    
    pi = new CGAL::Qt::GraphicsViewPolylineInput<K>(this, &scene, 1);

    scene.installEventFilter(pi);

    QObject::connect(pi, SIGNAL(generate(CGAL::Object)),
		    this, SLOT(processInput(CGAL::Object)));


    // 
    // Manual handling of actions
    //

    QObject::connect(this->actionQuit, SIGNAL(triggered()), 
		    this, SLOT(close()));

    //
    // Setup the scene and the view
    //
    scene.setItemIndexMethod(QGraphicsScene::NoIndex);
    scene.setSceneRect(-100, -100, 100, 100);
    this->graphicsView->setScene(&scene);
    this->graphicsView->setMouseTracking(true);

    // Turn the vertical axis upside down
    this->graphicsView->matrix().scale(1, -1);
                                                      
    // The navigation adds zooming and translation functionality to the
    // QGraphicsView
    this->addNavigation(this->graphicsView);

    this->setupStatusBar();
    this->setupOptionsMenu();
    this->addAboutDemo(":/cgal/help/about_CG_Project.html");
    this->addAboutCGAL();

    this->addRecentFiles(this->menuFile, this->actionQuit);
    connect(this, SIGNAL(openRecentFile(QString)),
	          this, SLOT(open(QString)));
}

void
MainWindow::update()
{
    if(this->actionShowConvexHull->isChecked()){
        convex_hull_gi->show();
    }else {
        convex_hull_gi->hide();
    }

    if(this->actionShowMinRectangle->isChecked()){
        min_rectangle_gi->show();
    }else {
        min_rectangle_gi->hide();
    }
}


void
MainWindow::update_from_points()
{
    convex_hull.clear();
    //CGAL::convex_hull_2(points.begin(), points.end(), std::back_inserter(convex_hull));
    CGP::convex_hull_2(points.begin(), points.end(), std::back_inserter(convex_hull));
   
    min_rectangle.clear();
    //CGAL::min_rectangle_2(convex_hull.vertices_begin(), convex_hull.vertices_end(), std::back_inserter(min_rectangle));
    CGP::min_rectangle_2(convex_hull.vertices_begin(), convex_hull.vertices_end(), std::back_inserter(min_rectangle));
}


void
MainWindow::processInput(CGAL::Object o)
{
    std::list<Point_2> input;
    if(CGAL::assign(input, o)){
        Point_2 p = input.front();
    
        points.push_back(p);

        convex_hull.push_back(p);
        Polygon_2 tmp;
        //CGAL::convex_hull_2(convex_hull.vertices_begin(), convex_hull.vertices_end(), std::back_inserter(tmp));
        CGP::convex_hull_2(convex_hull.vertices_begin(), convex_hull.vertices_end(), std::back_inserter(tmp));
        convex_hull = tmp;

        min_rectangle.clear();
        //CGAL::min_rectangle_2(convex_hull.vertices_begin(), convex_hull.vertices_end(), std::back_inserter(min_rectangle));
        CGP::min_rectangle_2(convex_hull.vertices_begin(), convex_hull.vertices_end(), std::back_inserter(min_rectangle));
    }
    Q_EMIT( changed());
}


/* 
 *  Qt Automatic Connections
 *  https://doc.qt.io/qt-5/designer-using-a-ui-file.html#automatic-connections
 * 
 *  setupUi(this) generates connections to the slots named
 *  "on_<action_name>_<signal_name>"
 */
void
MainWindow::on_actionInsertPoint_toggled(bool checked)
{
    if(checked){
        scene.installEventFilter(pi);
    } else {
        scene.removeEventFilter(pi);
    }
}

void
MainWindow::on_actionShowMinRectangle_toggled(bool checked)
{
    min_rectangle_gi->setVisible(checked);
    Q_EMIT( changed());
}

void
MainWindow::on_actionShowConvexHull_toggled(bool checked)
{
    convex_hull_gi->setVisible(checked);
    Q_EMIT( changed());
}

void
MainWindow::on_actionClear_triggered()
{
    points.clear();
    convex_hull.clear();
    min_rectangle.clear();
    
    Q_EMIT( changed());
}


void
MainWindow::on_actionInsertRandomPoints_triggered()
{
    QRectF rect = CGAL::Qt::viewportsBbox(&scene);
    CGAL::Qt::Converter<K> convert;  
    Iso_rectangle_2 isor = convert(rect);
    CGAL::Random_points_in_iso_rectangle_2<Point_2> pg((isor.min)(), (isor.max)());       // A) creates points uniformly distributed in an open rectangle
    // CGAL::Random_points_in_disc_2<Point_2,Creator> pg(150.0);         // B) creates points uniformly distributed in an open disc
    // CGAL::Random_points_in_square_2<Point_2,Creator> pg(150.0);       // C) creates points uniformly distributed in a open square
    // CGAL::Random_points_on_circle_2<Point_2,Creator> pg(150.0);       // D) creates points uniformly distributed on a circle. The generated points are computed using floating point arithmetic, whatever the Kernel is, thus they are on the circle/sphere only up to rounding errors
    // CGAL::Random_points_on_square_2<Point_2,Creator> pg(150.0);       // E) creates points uniformly distributed on the boundary of a square
    bool ok = false;

    const int number_of_points = 
        QInputDialog::getInt(this, 
                             tr("Number of random points"),
                             tr("Enter number of random points"),
			                       100,
			                       0,
			                       (std::numeric_limits<int>::max)(),
			                       1,
			                       &ok);

    if(!ok) {
        return;
    }

    typedef CGAL::Random_points_in_square_2<Point_2,Creator> Point_generator;

    // F) computes a random convex planar point set of given size where the points are drawn from a specific domain
    // std::vector<Point_2> random_convex_set;
    // CGAL::random_convex_set_2(
    //             number_of_points,
    //             std::back_inserter(random_convex_set),
    //             Point_generator(150.0));
    // std::vector<Point_2>::iterator pg = random_convex_set.begin();

    // G) constructs a random simple polygon from points that are drawn from a specific domain
    // std::vector<Point_2> point_set;
    // CGAL::copy_n_unique(
    //             Point_generator(150.0),
    //             number_of_points,
    //             std::back_inserter(point_set));
    // Polygon_2 polygon;
    // CGAL::random_polygon_2(
    //             point_set.size(),
    //             std::back_inserter(polygon),
    //             point_set.begin());
    // auto pg = polygon.vertices_begin();

    // H) creates points uniformly distributed on an ellipse. The generated points are computed using floating point arithmetic, whatever the Kernel is, thus they are on the ellipse only up to rounding errors
    // CGP::Random_points_on_ellipse_2 ellipse(number_of_points, 250.0, 100.0);
    // std::vector<Point_2>::iterator pg = ellipse.ellipse_points.begin();

    // wait cursor
    QApplication::setOverrideCursor(Qt::WaitCursor);
    for(int i = 0; i < number_of_points; ++i){
        Point_2 p = *pg++;
        points.push_back(p);
    }

    update_from_points();

    // default cursor
    QApplication::restoreOverrideCursor();
    Q_EMIT( changed());
}


void
MainWindow::on_actionLoadPoints_triggered()
{
    QString fileName = QFileDialog::getOpenFileName(this,
						                                        tr("Open Points file"),
						                                        ".");
    if(! fileName.isEmpty()){
        open(fileName);
    }
}


void
MainWindow::open(QString fileName)
{
    // wait cursor
    QApplication::setOverrideCursor(Qt::WaitCursor);
    std::ifstream ifs(qPrintable(fileName));
  
    K::Point_2 p;
    while(ifs >> p) {
        points.push_back(p);
    }
    update_from_points();

    // default cursor
    QApplication::restoreOverrideCursor();
    this->addToRecentFiles(fileName);
    actionRecenter->trigger();
    Q_EMIT( changed());
    
}

void
MainWindow::on_actionSavePoints_triggered()
{
    QString fileName = QFileDialog::getSaveFileName(this,
						                                        tr("Save points"),
						                                        ".");
    if(! fileName.isEmpty()){
        std::ofstream ofs(qPrintable(fileName));
        for(std::vector<Point_2>::iterator  
            vit = points.begin(),
            end = points.end();
            vit!= end; ++vit)
        {
            ofs << *vit << std::endl;
        }
    }
}


void
MainWindow::on_actionRecenter_triggered()
{
    this->graphicsView->setSceneRect(min_rectangle_gi->boundingRect());
    this->graphicsView->fitInView(min_rectangle_gi->boundingRect(), Qt::KeepAspectRatio);  
}


#include "CG_Project.moc"
#include <CGAL/Qt/resources.h>

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    app.setApplicationName("CG_Project");

    // Import resources from libCGAL (Qt5).
    // See https://doc.qt.io/qt-5/qdir.html#Q_INIT_RESOURCE
    CGAL_QT_INIT_RESOURCES;

    MainWindow mainWindow;
    mainWindow.show();
    return app.exec();
}
