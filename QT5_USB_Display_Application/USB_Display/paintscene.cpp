/*
 * File:   paintscene.cpp
 * Author:  Gaurav
 * website: www.circuitvalley.com
 * Created on Jan 19, 2020, 1:33 AM
 *	This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *	Email: gauravsingh@circuitvalley.com
************************************************************************/


#include "paintscene.h"

paintScene::paintScene(QObject *parent) : QGraphicsScene(parent)
{
    this->pen_size = 1;
}

paintScene::~paintScene()
{

}

void paintScene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    // When you click the mouse, we draw an ellipse
    addEllipse(event->scenePos().x() - qRound(static_cast<double>(this->pen_size)/2),
               event->scenePos().y() - qRound(static_cast<double>(this->pen_size)/2),
               this->pen_size,
               this->pen_size,
               QPen(Qt::NoPen),
               QBrush(this->pen_color));
    // Save the coordinates of the click point
    previousPoint = event->scenePos();
}

void paintScene::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    // Draw lines using the previous coordinate
    addLine(previousPoint.x(),
            previousPoint.y(),
            event->scenePos().x(),
            event->scenePos().y(),
            QPen(this->pen_color,this->pen_size,Qt::SolidLine,Qt::RoundCap));
    // Updating data on the previous coordinate
    previousPoint = event->scenePos();
}

void paintScene::set_color(QColor color)
{
    this->pen_color = color;
}

void paintScene::set_pen_size(size_t pen_size)
{
    this->pen_size = pen_size;
}
