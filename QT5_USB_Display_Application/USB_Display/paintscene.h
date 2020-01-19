#ifndef PAINTSCENE_H
#define PAINTSCENE_H

#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QTimer>
#include <QDebug>

class paintScene : public QGraphicsScene
{

    Q_OBJECT

public:
    explicit paintScene(QObject *parent = 0);
    ~paintScene();

private:
    QPointF     previousPoint;      //Previous point coordinates

private:
    // For drawing we use mouse events
    void mousePressEvent(QGraphicsSceneMouseEvent * event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    QColor pen_color;
    size_t pen_size;

public:
    void set_color(QColor color);
    void set_pen_size(size_t pen_size);

};

#endif // PAINTSCENE_H
