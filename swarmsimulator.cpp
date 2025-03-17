#include "swarmsimulator.h"

#include <QRandomGenerator>
#include <QDateTime>
#include <QtSql/QSqlQuery>
#include <QtSql/QSqlError>

SwarmSimulator::SwarmSphereFormation::SwarmSphereFormation(int agentsCount,
                                                           QVector3D swarmCenterPoint,
                                                           double swarmDensity)
    : AGENTS_COUNT(agentsCount)
    , DENSITY(swarmDensity)
    , WORLD_MAX_SIZE(100.0) // Now using hard-code value for WORLD_MAX_SIZE.
    , SWARM_CENTER(swarmCenterPoint)
    , m_CurrentIteration(0)
{

    // Configure connection for sqlite3 database.
    m_Database = QSqlDatabase::addDatabase("QSQLITE");
    m_Database.setDatabaseName("C:\\Projects\\uav-swarm\\uav_swarm.db");
    if (!m_Database.open())
    {
        qWarning() << "Failed to open database:" << m_Database.lastError().text();
        return;
    }

    // Get new experiment id for current simulation.
    QSqlQuery query("SELECT MAX(experiment_id) FROM swarm_state", m_Database);
    query.exec();
    if (query.next() && !query.value(0).isNull())
    {
        EXPERIMENT_ID = query.value(0).toInt() + 1;
    }
    else
    {
        EXPERIMENT_ID = 1;
    }

    InitSwarm();
}

SwarmSimulator::SwarmSphereFormation::~SwarmSphereFormation()
{
    m_Database.close();
}

void SwarmSimulator::SwarmSphereFormation::InitSwarm()
{
    // Using seed = 42 for repeatables.
    // TODO: Make parameter for seed.
    QRandomGenerator qrng(42);
    for (int i = 0; i < AGENTS_COUNT; i++)
    {
        // TODO: Decide what would be in z-coord.
        QVector3D randomUavStartPos(qrng.bounded(WORLD_MAX_SIZE),
                                    qrng.bounded(WORLD_MAX_SIZE),
                                    0);

        // TODO: Now define uav's parameters hard-code, future make it prettier.
        Uav agent(randomUavStartPos, 50.0, 3.0);

        m_Agents.push_back(agent);

    }

    SaveSwarmStateIntoDB();
}


void SwarmSimulator::SwarmSphereFormation::SaveSwarmStateIntoDB()
{
    QSqlQuery query(m_Database);
    query.prepare("INSERT INTO swarm_state "
                  "(timestamp, iteration, "
                  "position_x, position_y, position_z, "
                  "speed_x, speed_y, speed_z, "
                  "state, experiment_id) "
                  "VALUES (:timestamp, :iteration, "
                  ":position_x, :position_y, :position_z, "
                  ":speed_x, :speed_y, :speed_z, "
                  ":state, :experiment_id)");

    for(QVector<Uav>::ConstIterator it = m_Agents.constBegin(); it != m_Agents.constEnd(); it++)
    {
        QVector3D agentPosition = it->CurrentPos();
        QVector3D agentSpeed = it->Speed();

        query.bindValue(":timestamp", QDateTime::currentDateTimeUtc());
        query.bindValue(":iteration", m_CurrentIteration);
        query.bindValue(":position_x", agentPosition.x());
        query.bindValue(":position_y", agentPosition.y());
        query.bindValue(":position_z", agentPosition.z());
        query.bindValue(":speed_x", agentSpeed.x());
        query.bindValue(":speed_y", agentSpeed.y());
        query.bindValue(":speed_z", agentSpeed.z());
        query.bindValue(":state", "");
        query.bindValue(":experiment_id", EXPERIMENT_ID);

        if (!query.exec())
        {
            qWarning() << "Failed to insert data:" << query.lastError().text();
        }
    }
}
