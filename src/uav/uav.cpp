#include "uav.h"

Uav::Uav(QVector3D startPos, double interractionRadius, double selfRadius, uint16_t id, uint8_t maxVelocity)
    : m_CurrentPosition(startPos)
    , m_InterractionRadius(interractionRadius)
    , m_SelfRadius(selfRadius)
    , m_Direction(QVector3D()) // default constructor of QVector3D returns (0;0;0).
    , m_CurrentGoal(QVector3D())
    , m_Id(id)
    , m_MaxVelocity(maxVelocity)
    , m_CurrentVelocity(0)
{
}
