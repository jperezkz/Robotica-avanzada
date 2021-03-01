//
// Created by pbustos on 1/3/21.
//

#include "elastic_band.h"
#include <cppitertools/zip.hpp>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/enumerate.hpp>
#include <QVector2D>
#include <QtCore>

void ElasticBand::initialize()
{

}

void ElasticBand::compute_forces(std::vector<QPointF> &path,
                                    const std::vector<QPointF> &laser_cart,
                                    const QPolygonF &laser_poly,
                                    const QPolygonF &current_robot_polygon,
                                    const QPointF &current_robot_nose)
{
    if (path.size() < 3)
        return;
    int nonVisiblePointsComputed = 0;

    // Go through points using a sliding windows of 3
    for (auto &&[i, group] : iter::enumerate(iter::sliding_window(path, 3)))
    {
        if (group.size() < 3)
            continue; // break if too short

        auto p1 = QVector2D(group[0]);
        auto p2 = QVector2D(group[1]);
        auto p3 = QVector2D(group[2]);
        if(p1==p2 or p2==p3)
            continue;
        QPointF p = group[1];
        int index_of_p_in_path = i+1;  //index of p in path

        ////////////////////////////////
        /// INTERNAL curvature forces on p2. Stretches the path locally
        /// Approximates the angle between adjacent segments: p2->p1, p2->p3
        ////////////////////////////////
        QVector2D iforce = ((p1 - p2) / (p1 - p2).length() + (p3 - p2) / (p3 - p2).length());

        ////////////////////////////////////////////
        /// External forces caused by obstacles repulsion field
        ///////////////////////////////////////////7
        float min_dist;
        QVector2D eforce;

        qDebug() << __FUNCTION__  << nonVisiblePointsComputed;

//        // compute forces from map on not visible points
        if ( not is_visible(p, laser_poly ))
            continue;
//        {
//            auto [obstacleFound, vectorForce] = grid.vectorToClosestObstacle(p);
//            if (( not obstacleFound) or (nonVisiblePointsComputed > 10))
//            {
//                qDebug ()  << __FUNCTION__ << "No obstacles found in map for not visible point or it is more than 10 not visible points away";
//                nonVisiblePointsComputed++;
//                continue;
//            }
//            else
//            {
//                qDebug()  << __FUNCTION__  << "--- Obstacle found in grid ---";
//                min_dist = vectorForce.length() - (ROBOT_LENGTH / 2);   // subtract robot semi-width
//                if (min_dist <= 0)    // hard limit to close obstables
//                    min_dist = 0.01;
//                eforce = vectorForce;
//            }
//            nonVisiblePointsComputed++;
//        }

        // compute forces from laser on visible point
//       else
        // {
        // vector holding a) distance from laser tip to p, vector from laser tip to p, laser tip plane coordinates
        std::vector<std::tuple<float, QVector2D, QPointF>> distances;
        // Apply to all laser points a functor to compute the distances to point p2. laser_cart must be up to date
        std::transform(std::begin(laser_cart), std::end(laser_cart), std::back_inserter(distances), [p, RL=ROBOT_LENGTH](const QPointF &laser)
        {   // compute distance from laser measure to point minus RLENGTH/2 or 0 and keep it positive
            float dist = (QVector2D(p) - QVector2D(laser)).length() - (RL / 2);
            if (dist <= 0)
                dist = 0.01;
            return std::make_tuple(dist,  QVector2D(p)-QVector2D(laser), laser);
        });
        // compute min of all laser to p distances
        auto min = std::min_element(std::begin(distances), std::end(distances), [](auto &a, auto &b)
        {
            return std::get<float>(a) < std::get<float>(b);
        });
        min_dist = std::get<float>(*min);
        eforce = std::get<QVector2D>(*min);
        //}
        /// Note: instead of min, we could compute the resultant of all forces acting on the point, i.e. inside a given radius.
        /// a logarithmic law can be used to compute de force from the distance.
        /// To avoid constants, we need to compute de Jacobian of the sum of forces wrt the (x,y) coordinates of the point

        // rescale min_dist so 1 is ROBOT_LENGTH
        float magnitude = (1.f / ROBOT_LENGTH) * min_dist;
        // compute inverse square law
        magnitude = 10.f / (magnitude * magnitude);
        //if(magnitude > 25) magnitude = 25.;
        QVector2D f_force = magnitude * eforce.normalized();

        // Remove tangential component of repulsion force by projecting on line tangent to path (base_line)
//        QVector2D base_line = (p1 - p3).normalized();
//        const QVector2D itangential = QVector2D::dotProduct(f_force, base_line) * base_line;
//        f_force = f_force - itangential;

        // update node pos. KI and KE are approximating inverse Jacobians modules. This should be CHANGED
        // Directions are taken as the vector going from p to closest obstacle.
        auto total = (KI * iforce) + (KE * f_force);
        //
        // limiters CHECK!!!!!!!!!!!!!!!!!!!!!!
        if (total.length() > 30)
            total = 8 * total.normalized();
        if (total.length() < -30)
            total = -8 * total.normalized();

        /// Compute additional restrictions to be forced in the minimization process
        // A) Check boundaries for final displacements
        // A.1) Move nodes only if it does not move inside objects
        // A.2) Does not move underneath the robot.
        // A.3) Does not exit the laser polygon
        QPointF temp_p = p + total.toPointF();
        //qInfo()  << __FUNCTION__  << "Total force "<< total.toPointF()<< " New Point "<< temp_p;
        if (is_point_visitable(temp_p) and (not current_robot_polygon.containsPoint(temp_p, Qt::OddEvenFill))
            //and (std::none_of(std::begin(intimateSpaces), std::end(intimateSpaces),[temp_p](const auto &poly) { return poly.containsPoint(temp_p, Qt::OddEvenFill);}))
            //and (std::none_of(std::begin(personalSpaces), std::end(personalSpaces),[temp_p](const auto &poly) { return poly.containsPoint(temp_p, Qt::OddEvenFill);}))
                )
        {
            path[index_of_p_in_path] = temp_p;
        }
//            if( auto it = find_if(pathPoints.begin(), pathPoints.end(), [p] (auto & s){ return (s.x() == p.x() and s.y() == p.y() );}); it != pathPoints.end())
//            {
//                int index = std::distance(pathPoints.begin(), it);
//                pathPoints[index] = temp_p;
//            }
    }
    // Check if robot nose is inside the laser polygon
    if(is_visible(current_robot_nose, laser_poly))
        path[0] = current_robot_nose;
    else
        qWarning() << __FUNCTION__  << "Robot Nose not visible -- NEEDS REPLANNING ";
}

void ElasticBand::clean_points(std::vector<QPointF> &path, const QPolygonF &laser_poly,  const QPolygonF &current_robot_polygon)
{
    qDebug() << __FUNCTION__;
    std::vector<QPointF> points_to_remove;
    for (const auto &group : iter::sliding_window(path, 2))
    {
        const auto &p1 = group[0];
        const auto &p2 = group[1];

        if (not is_visible(p1, laser_poly) or not is_visible(p2, laser_poly)) //not visible
            continue;

        if (p2 == path.back())
            break;
        // check if p1 was marked to erase in the previous iteration
        if (std::find(std::begin(points_to_remove), std::end(points_to_remove), p1) != std::end(points_to_remove))
            continue;

        float dist = QVector2D(p1 - p2).length();
        qDebug() << __FUNCTION__ << " dist <" << dist << 0.5 * ROAD_STEP_SEPARATION;
        if (dist < 0.5 * ROAD_STEP_SEPARATION)
            points_to_remove.push_back(p2);

        else if(current_robot_polygon.containsPoint(p2, Qt::OddEvenFill))
        {
            qDebug()<<"-------------" << __FUNCTION__ << "------------- Removing point inside robot ";
            points_to_remove.push_back(p2);
        }
    }
    qDebug() << __FUNCTION__ << "Removed: " << points_to_remove.size();
    for (auto &&p : points_to_remove)
        path.erase(std::remove_if(path.begin(), path.end(), [p](auto &r) { return p == r; }), path.end());
}

void ElasticBand::add_points(std::vector<QPointF> &path, const QPolygonF &laser_poly, const QPolygonF &current_robot_polygon)
{
    // qDebug()<<"Navigation - "<< __FUNCTION__;
    std::vector<std::tuple<int, QPointF>> points_to_insert;
    for (auto &&[k, group] : iter::enumerate(iter::sliding_window(path, 2)))
    {
        auto &p1 = group[0];
        auto &p2 = group[1];

        if ( not is_visible(p1, laser_poly) or not is_visible(p2, laser_poly)) //not visible
            continue;

        float dist = QVector2D(p1 - p2).length();
        qDebug() << __FUNCTION__ << " dist >" << dist << ROAD_STEP_SEPARATION;
        if (dist > ROAD_STEP_SEPARATION)
        {
            //Crucial que el punto se ponga mas cerca que la condición de entrada
            float l = 0.9 * ROAD_STEP_SEPARATION / dist;
            QLineF line(p1, p2);
            points_to_insert.emplace_back(k + 1, QPointF{line.pointAt(l)});
        }
    }
    qDebug() << __FUNCTION__ << "Added: " << points_to_insert.size();
    for (const auto &[l, p] : iter::enumerate(points_to_insert))
    {
        if ( not current_robot_polygon.containsPoint(std::get<QPointF>(p), Qt::OddEvenFill))
            path.insert(path.begin() + std::get<int>(p) + l, std::get<QPointF>(p));
    }
}

bool ElasticBand::is_visible(QPointF p, const QPolygonF &laser_poly)
{
    //std::optional<Mat::Vector3d> pointInLaser = inner_eigen->transform(laser_name, Mat::Vector3d (p.x(),p.y(), 0), world_name);
    //return laser_poly.containsPoint(QPointF(pointInLaser.value().x(), pointInLaser.value().y()), Qt::OddEvenFill);
}

bool ElasticBand::is_point_visitable(QPointF point)
{
    return true;  //// NEEDS the GRID
}
