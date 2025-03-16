#include "uav.h"

Uav::Uav(QVector3D startPos, double interractionRadius, double selfRadius)
    : m_CurrentPosition(startPos)
    , m_InterractionRadius(interractionRadius)
    , m_SelfRadius(selfRadius)
    , m_Speed(0.0)
{
}

double Uav::InterractionRadius() const
{
    return m_InterractionRadius;
}

double Uav::SelfRadius() const
{
    return m_SelfRadius;
}

double Uav::Speed() const
{
    return m_Speed;
}

void Uav::setCurrentPos(const QVector3D & pos)
{
    m_CurrentPosition = pos;
}

void Uav::setSpeed(double newSpeed)
{
    m_Speed = newSpeed;
}
