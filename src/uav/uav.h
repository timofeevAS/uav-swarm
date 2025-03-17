#ifndef UAV_H
#define UAV_H

#include "QVector3D"

class Uav
{
public:
    Uav(QVector3D startPos,
        double interractionRadius,
        double selfRadius);

    QVector3D CurrentPos() const;
    double InterractionRadius() const;
    double SelfRadius() const;
    QVector3D Speed() const;

    void SetCurrentPos(const QVector3D & pos);
    void SetSpeed(const QVector3D & newSpeed);

    QVector3D CurrentGoal() const;
    void SetCurrentGoal(const QVector3D &newCurrentGoal);

private:
    // Position of drone in space.
    QVector3D m_CurrentPosition;
    // Current velocity of drone;
    QVector3D m_Speed;
    // Current goal of drone;
    QVector3D m_CurrentGoal;

    // Radius of interraction with other drones;
    double m_InterractionRadius;

    // Radius of self "care": radius which represents "real" sizes of drone;
    double m_SelfRadius;
};

#endif // UAV_H
