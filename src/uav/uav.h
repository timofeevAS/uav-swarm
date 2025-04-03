#ifndef UAV_H
#define UAV_H

#include "QVector3D"

class Uav
{
public:
    Uav(QVector3D startPos,
        double interractionRadius,
        double selfRadius,
        uint16_t id,
        uint8_t maxVelocity);

    enum State
    {
        MOVE,
        FINISH,
    };

    // Position of drone in space.
    QVector3D m_CurrentPosition;
    // Current velocity of drone: direction + velocity;
    QVector3D m_Direction;
    uint8_t m_MaxVelocity;
    uint8_t m_CurrentVelocity;

    // Current goal of drone;
    QVector3D m_CurrentGoal;
    // Radius of interraction with other drones;
    double m_InterractionRadius;
    // Radius of self "care": radius which represents "real" sizes of drone;
    double m_SelfRadius;
    uint16_t m_Id;
};

#endif // UAV_H
