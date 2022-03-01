#include "mj_sim.h"

std::vector<std::string> MjSim::q_names;

std::map<std::string, mjtNum> MjSim::q_inits;

std::map<std::string, mjtNum> MjSim::q_refs;
std::map<std::string, mjtNum> MjSim::dq_refs;
std::map<std::string, mjtNum> MjSim::ddq_refs;

std::vector<double> MjSim::Kp;
std::vector<double> MjSim::Kv;
std::vector<double> MjSim::Ki;

mjtNum *MjSim::u = NULL;

double MjSim::sim_start = 0.;

MjSim::MjSim()
{

}

void MjSim::init()
{
    sim_start = d->time;

    u = (mjtNum *)malloc(m->nv * sizeof(mjtNum *)); mju_zero(u, m->nv);
    tau = (mjtNum *)malloc(m->nv * sizeof(mjtNum *)); mju_zero(tau, m->nv);

    mjtNum q_init_array[m->nv] = {0.};
    for (std::string q_name : q_names)
    {
        int idx = mj_name2id(m, mjtObj::mjOBJ_JOINT, q_name.c_str());
        e_sum[q_name] = 0.;
        q_refs[q_name] = q_inits[q_name];
        dq_refs[q_name] = 0.;
        ddq_refs[q_name] = 0.;
        q_init_array[idx] = q_inits[q_name];
    }
    mju_copy(d->qpos, q_init_array, m->nq);
    mj_forward(m, d);
}

void MjSim::computed_torque_controller()
{
    mj_mulM(m, d, tau, u);
    for (const std::string q_name : q_names)
    {
        int idx = mj_name2id(m, mjtObj::mjOBJ_JOINT, q_name.c_str());
        tau[idx] += d->qfrc_bias[idx];
    }
    
    mju_copy(d->qfrc_applied, tau, m->nv);
}