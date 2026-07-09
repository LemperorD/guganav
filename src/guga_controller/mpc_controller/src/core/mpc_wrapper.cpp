#include "mpc_controller/core/mpc_wrapper.hpp"

namespace mpc_controller
{

MpcWrapper::MpcWrapper()
{
}

MpcWrapper::~MpcWrapper()
{
}

static void setup_solver(
    ocp_nlp_config *cfg, ocp_nlp_dims *dims,
    ocp_nlp_in *in, ocp_nlp_out *out,
    const double x_cur[3], TrajectoryType type, double t_start, double dt) {
    double lbx[NBX0]={x_cur[0],x_cur[1],x_cur[2]};
    double ubx[NBX0]={x_cur[0],x_cur[1],x_cur[2]};
    ocp_nlp_constraints_model_set(cfg,dims,in,out,0,"lbx",void_ptr(lbx));
    ocp_nlp_constraints_model_set(cfg,dims,in,out,0,"ubx",void_ptr(ubx));

    for (int i=0;i<N;++i) {
        double ref[3];
        generate_reference(type,t_start+i*dt,ref);
        double yref[NY]={ref[0],ref[1],ref[2],0.0,0.0};
        ocp_nlp_cost_model_set(cfg,dims,in,i,"y_ref",void_ptr(yref));
    }
    double ref[3];
    generate_reference(type,t_start+N*dt,ref);
    double yref_e[NYN]={ref[0],ref[1],ref[2]};
    ocp_nlp_cost_model_set(cfg,dims,in,N,"y_ref",void_ptr(yref_e));
}

}  // namespace mpc_controller
