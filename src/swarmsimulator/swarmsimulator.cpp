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
        agent.SetCurrentGoal(SWARM_CENTER);

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

void SwarmSimulator::SwarmSphereFormation::BoidsStep()
{
    static const double SEPARATION_DISTANCE = 10; // TODO: prove it;

    QVector<QVector<double>> distances(AGENTS_COUNT, QVector<double>(AGENTS_COUNT, -1));
    for(int i = 0; i < AGENTS_COUNT; i++)
    {
        for(int j = 0; j < AGENTS_COUNT; j++)
        {
            distances[i][j] = m_Agents[i].CurrentPos().distanceToPoint(m_Agents[j].CurrentPos());
            distances[j][i] = distances[i][j];
        }
    }

    // Move each agents;
    for(int i = 0; i < AGENTS_COUNT; i++)
    {
        Uav agent = m_Agents[i];
        // Rule `Cohesion`: steer to move towards the average position (center of mass) of local flockmates;
        QVector3D cohesion;
        for(int j = 0; j < AGENTS_COUNT; j++)
        {
            if (i != j && distances[i][j] <= agent.InterractionRadius())
            {
                cohesion += m_Agents[j].CurrentPos();
            }
        }
        cohesion /= AGENTS_COUNT - 1;
        // Moving vector of Agent `i` towards center of mass for 1%.
        // TODO: Prove this coefficient;
        cohesion = (cohesion - m_Agents[i].CurrentPos()) / 100;

        // Rule `Separation`: steer avoid crowding local flockmates;
        QVector3D separation;
        for(int j = 0; j < AGENTS_COUNT; j++)
        {

            if (i != j && distances[i][j] <= agent.InterractionRadius())
            {
                if (distances[i][j] <= SEPARATION_DISTANCE)
                {
                    separation -= (agent.CurrentPos() - m_Agents[j].CurrentPos());
                }
            }
        }
        // Rule `Alignment`: steer towards the average heading of local flockmates;
        QVector3D alignment;
        // TODO: Pass this.

        // Rule `Tendency`: steer towards self point;
        QVector3D tendency;
        tendency = (agent.CurrentGoal() - agent.CurrentPos()) / 100; // TODO: prove it.

        agent.SetSpeed(agent.Speed() + cohesion + separation + tendency);
    }
}

void SwarmSimulator::SwarmSphereFormation::RunSimulation(int steps)
{
    m_CurrentIteration = 0;
    while (m_CurrentIteration < steps)
    {
        BoidsStep();
        SaveSwarmStateIntoDB();
        m_CurrentIteration++;
    }
}
