#include "uav.h"

Uav::Uav(QVector3D startPos, double interractionRadius, double selfRadius)
    : m_CurrentPosition(startPos)
    , m_InterractionRadius(interractionRadius)
    , m_SelfRadius(selfRadius)
    , m_Speed(QVector3D()) // default constructor of QVector3D returns (0;0;0).
    , m_CurrentGoal(QVector3D())
{
}

QVector3D Uav::CurrentPos() const
{
    return m_CurrentPosition;
}

double Uav::InterractionRadius() const
{
    return m_InterractionRadius;
}

double Uav::SelfRadius() const
{
    return m_SelfRadius;
}

QVector3D Uav::Speed() const
{
    return m_Speed;
}

void Uav::SetCurrentPos(const QVector3D & pos)
{
    m_CurrentPosition = pos;
}

void Uav::SetSpeed(const QVector3D & newSpeed)
{
    m_Speed = newSpeed;
}

QVector3D Uav::CurrentGoal() const
{
    return m_CurrentGoal;
}

void Uav::SetCurrentGoal(const QVector3D &newCurrentGoal)
{
    m_CurrentGoal = newCurrentGoal;
}
