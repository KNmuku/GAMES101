#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

        // Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes) {
            double s = double(i) / (num_nodes-1);
            Vector2D pos = (1-s) * start + s * end;
            masses.push_back(new Mass(pos, node_mass, false));
        }
        for (int i = 0; i < num_nodes - 1; ++i) {
            springs.push_back(new Spring(masses.at(i), masses.at(i+1), k));
        }
        for (auto &i : pinned_nodes) {
            masses.at(i)->pinned = true;
        } 
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D m1_m2 = s->m1->position - s->m2->position;
            double changed_length = m1_m2.norm();
            Vector2D m1_m2_normalized = m1_m2 / changed_length;
            double difference = changed_length - s->rest_length;
            s->m1->forces += - s->k * (m1_m2_normalized) * difference;
            s->m2->forces += - s->k * (-m1_m2_normalized) * difference;
            // TODO (Part 2): Add global damping
            double kd = 0.00005;
            Vector2D m1_m2_velocity = s->m1->velocity - s->m2->velocity;
            s->m1->forces += - kd * dot(m1_m2_normalized, m1_m2_velocity) * m1_m2_normalized;  
            s->m2->forces += - kd * dot(m1_m2_normalized, m1_m2_velocity) * (-m1_m2_normalized);
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position

                Vector2D accelerate = m->forces / m->mass + gravity;
                // explicit Euler's method
                /*
                m->position += m->velocity * delta_t;
                m->velocity += accelerate * delta_t; */

                // semi-implicit Euler's method
                m->velocity +=accelerate * delta_t;
                m->position += m->velocity * delta_t;

            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D m1_m2 = s->m1->position - s->m2->position;
            double changed_length = m1_m2.norm();
            Vector2D m1_m2_normalized = m1_m2 / changed_length;
            double difference = changed_length - s->rest_length;
            s->m1->forces += - s->k * (m1_m2_normalized) * difference;
            s->m2->forces += - s->k * (-m1_m2_normalized) * difference;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 3.1): Set the new position of the rope mass
                Vector2D temp_position = m->position;
                Vector2D accelerate = m->forces / m->mass + gravity; 
                // TODO (Part 4): Add global Verlet damping
                double damping_factor = 5e-5;
                m->position = temp_position + (1 - damping_factor) 
                            * (temp_position - m->last_position) + accelerate * delta_t * delta_t;
                m->last_position = temp_position;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }
}
