//
// Created by pbustos on 1/3/21.
//

#ifndef PIONEER_CONTROLLER_ELASTIC_BAND_H
#define PIONEER_CONTROLLER_ELASTIC_BAND_H

#include <QGraphicsPolygonItem>

class ElasticBand
{
    public:
        void initialize();

    private:

        const float ROBOT_LENGTH = 500;
        const float ROAD_STEP_SEPARATION = ROBOT_LENGTH * 0.9;
        float KE = 40;
        float KI = 300;

        void compute_forces(std::vector<QPointF> &path, const std::vector<QPointF> &laser_cart,
        const QPolygonF &laser_poly,   const QPolygonF &current_robot_polygon,
        const QPointF &current_robot_nose);
        void add_points(std::vector<QPointF> &path, const QPolygonF &laser_poly,  const QPolygonF &current_robot_polygon);
        void clean_points(std::vector<QPointF> &path, const QPolygonF &laser_poly,  const QPolygonF &current_robot_polygon);

        bool is_visible(QPointF p, const QPolygonF &laser_poly);
        bool is_point_visitable(QPointF point);

};

#endif //PIONEER_CONTROLLER_ELASTIC_BAND_H
