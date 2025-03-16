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
    double Speed() const;

    void setCurrentPos(const QVector3D & pos);
    void setSpeed(double newSpeed);

private:
    // Position of drone in space.
    QVector3D m_CurrentPosition;

    // Radius of interraction with other drones;
    double m_InterractionRadius;

    // Radius of self "care": radius which represents "real" sizes of drone;
    double m_SelfRadius;

    // Current velocity of drone;
    double m_Speed;
};

#endif // UAV_H
