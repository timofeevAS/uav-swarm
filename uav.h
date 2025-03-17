#ifndef UAV_H
#define UAV_H

#include "QVector3D"

class Uav
{
public:
    Uav(QVector3D startPos,
        double interractionRadius,
        double selfRadius);

    double CurrentPos() const;
    double InterractionRadius() const;
    double SelfRadius() const;
    QVector3D Speed() const;

    void setCurrentPos(const QVector3D & pos);
    void setSpeed(const QVector3D & newSpeed);

private:
    // Position of drone in space.
    QVector3D m_CurrentPosition;
    // Current velocity of drone;
    QVector3D m_Speed;

    // Radius of interraction with other drones;
    double m_InterractionRadius;

    // Radius of self "care": radius which represents "real" sizes of drone;
    double m_SelfRadius;
};

#endif // UAV_H
