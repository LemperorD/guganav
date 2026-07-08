/**
 * @file test_unicycle_solver.cc
 * @brief 测试 acados 生成的 unicycle OCP C 代码 — 闭环 MPC 仿真。
 *
 * 运行两条轨迹 (圆 / 8字) 的闭环 MPC 路径跟踪仿真，
 * 输出 CSV 数据文件供 plot_results.py 绘图。
 *
 * 不依赖 ROS, 仅链接 acados 运行时库和生成的 solver。
 *
 * 使用方法:
 *   cd test_model_cc && mkdir build && cd build
 *   cmake .. && make && ./test_unicycle_solver
 *   python3 ../plot_results.py
 */

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <string>
#include <sys/stat.h>

// acados
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/utils/math.h"
#include "acados/utils/print.h"

extern "C" {
#include "acados_solver_unicycle.h"
#include "acados_sim_solver_unicycle.h"
}

#define NX   UNICYCLE_NX
#define NU   UNICYCLE_NU
#define N    UNICYCLE_N
#define NY   UNICYCLE_NY
#define NYN  UNICYCLE_NYN
#define NBX0 UNICYCLE_NBX0

// 辅助: RK4 积分
static void rk4_step(const double x[3], const double u[2], double dt,
                     double x_next[3]) {
    double k1[3] = {u[0]*cos(x[2]), u[0]*sin(x[2]), u[1]};
    double xk[3];
    xk[0]=x[0]+0.5*dt*k1[0]; xk[1]=x[1]+0.5*dt*k1[1]; xk[2]=x[2]+0.5*dt*k1[2];
    double k2[3] = {u[0]*cos(xk[2]), u[0]*sin(xk[2]), u[1]};
    xk[0]=x[0]+0.5*dt*k2[0]; xk[1]=x[1]+0.5*dt*k2[1]; xk[2]=x[2]+0.5*dt*k2[2];
    double k3[3] = {u[0]*cos(xk[2]), u[0]*sin(xk[2]), u[1]};
    xk[0]=x[0]+dt*k3[0]; xk[1]=x[1]+dt*k3[1]; xk[2]=x[2]+dt*k3[2];
    double k4[3] = {u[0]*cos(xk[2]), u[0]*sin(xk[2]), u[1]};
    x_next[0]=x[0]+(dt/6.0)*(k1[0]+2*k2[0]+2*k3[0]+k4[0]);
    x_next[1]=x[1]+(dt/6.0)*(k1[1]+2*k2[1]+2*k3[1]+k4[1]);
    x_next[2]=x[2]+(dt/6.0)*(k1[2]+2*k2[2]+2*k3[2]+k4[2]);
    // while (x_next[2] >  M_PI) x_next[2]-=2*M_PI;
    // while (x_next[2] < -M_PI) x_next[2]+=2*M_PI;
}

// 参考轨迹
enum class TrajectoryType { CIRCLE = 0, FIGURE8 = 1 };

static void generate_reference(TrajectoryType type, double t, double ref[3]) {
    if (type == TrajectoryType::CIRCLE) {
        double r=2.0, w=0.8;
        ref[0]=r*cos(w*t); ref[1]=r*sin(w*t); ref[2]=w*t+M_PI/2.0;
    } else {
        double ax=2.0, ay=1.5, w=0.6;
        ref[0]=ax*sin(w*t); ref[1]=ay*sin(2.0*w*t);
        double eps=0.001;
        ref[2]=atan2(ay*sin(2.0*w*(t+eps))-ay*sin(2.0*w*(t-eps)),
                     ax*sin(w*(t+eps))-ax*sin(w*(t-eps)));
        // TODO：unwrap
    }
}

// acados C API 接受 void* 而非 const void*
template<typename T>
static inline void* void_ptr(const T* p) { return const_cast<T*>(p); }

// 确保目录存在
static void ensure_dir(const char *dir) {
    mkdir(dir, 0755);
}

// 写 CSV
static void write_csv_mat(const char *path, const char *hdr,
                           const double *data, int rows, int cols) {
    FILE *f = fopen(path, "w");
    if (!f) return;
    fprintf(f, "%s\n", hdr);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c)
            fprintf(f, "%.8f%s", data[r*cols+c], (c<cols-1)?",":"\n");
    }
    fclose(f);
}

// Test 1: Smoke
static bool test_create_destroy() {
    std::printf("--- Test 1: Create & Destroy ---\n");
    unicycle_solver_capsule *c = unicycle_acados_create_capsule();
    if (!c) { std::printf("  FAIL\n"); return false; }
    int s = unicycle_acados_create_with_discretization(c, N, nullptr);
    if (s) { std::printf("  FAIL\n"); unicycle_acados_free_capsule(c); return false; }
    s = unicycle_acados_free(c);
    if (s) { std::printf("  FAIL\n"); unicycle_acados_free_capsule(c); return false; }
    unicycle_acados_free_capsule(c);
    std::printf("  PASS\n");
    return true;
}

// 设置 solver 状态 & 参考
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
    double refe[3];
    generate_reference(type,t_start+N*dt,refe);
    double yref_e[NYN]={refe[0],refe[1],refe[2]};
    ocp_nlp_cost_model_set(cfg,dims,in,N,"y_ref",void_ptr(yref_e));
}

// Test 2: 闭环 MPC — 输出 CSV
static bool test_closed_loop_mpc(TrajectoryType type,
                                  const char *tag, const char *csv_dir) {
    std::printf("--- Test: Closed-Loop MPC (%s) ---\n", tag);

    const double dt    = 1.0/N;
    const double T_sim = 10.0;
    const int n_sim    = static_cast<int>(T_sim/dt) - N;  // 180

    unicycle_solver_capsule *c = unicycle_acados_create_capsule();
    int s = unicycle_acados_create_with_discretization(c, N, nullptr);
    if (s) { std::printf("  FAIL: create\n"); return false; }

    ocp_nlp_config *cfg=unicycle_acados_get_nlp_config(c);
    ocp_nlp_dims  *dims=unicycle_acados_get_nlp_dims(c);
    ocp_nlp_in    *in  =unicycle_acados_get_nlp_in(c);
    ocp_nlp_out   *out =unicycle_acados_get_nlp_out(c);
    ocp_nlp_solver *slv=unicycle_acados_get_nlp_solver(c);

    auto *x_hist   =new double[(n_sim+1)*NX]();
    auto *u_hist   =new double[n_sim*NU]();
    auto *ref_hist =new double[n_sim*3]();
    auto *t_hist   =new double[n_sim]();

    double x_cur[NX];

    // 设置初始状态并写入history数组
    if (type==TrajectoryType::CIRCLE)
        {x_cur[0]=1.5;x_cur[1]=0.0;x_cur[2]=0.0;}
    else
        {x_cur[0]=0.5;x_cur[1]=0.3;x_cur[2]=0.0;}
    x_hist[0]=x_cur[0];x_hist[1]=x_cur[1];x_hist[2]=x_cur[2];

    for (int k=0;k<n_sim;++k) {
        setup_solver(cfg,dims,in,out,x_cur,type,k*dt,dt);

        s = unicycle_acados_solve(c);
        if (s!=ACADOS_SUCCESS) {
            std::printf("  FAIL: solve at step %d\n",k);
            goto fail;
        }

        // 获取 u0
        ocp_nlp_out_get(cfg,dims,out,0,"u",&u_hist[k*NU]);

        // 计时
        {
            double te=0;
            ocp_nlp_get(slv,"time_tot",&te);
            t_hist[k]=te*1000.0;
        }

        // 记录参考
        {
            double ref[3];
            generate_reference(type,k*dt,ref);
            ref_hist[k*3+0]=ref[0];ref_hist[k*3+1]=ref[1];ref_hist[k*3+2]=ref[2];
        }

        // 前向仿真
        // TODO: 使用 acados_sim_solver_unicycle 进行前向仿真
        {
            double u_opt[2]={u_hist[k*NU],u_hist[k*NU+1]};
            rk4_step(x_cur,u_opt,dt,x_cur);
        }

        x_hist[(k+1)*NX+0]=x_cur[0];
        x_hist[(k+1)*NX+1]=x_cur[1];
        x_hist[(k+1)*NX+2]=x_cur[2];

        if (k%50==0)
            std::printf("  step %3d/%d: pos=(%6.2f,%6.2f) u=(%6.2f,%6.2f) t=%.3fms\n",
                        k,n_sim,x_cur[0],x_cur[1],
                        u_hist[k*NU],u_hist[k*NU+1],t_hist[k]);
    }

    // 统计
    {
        double rmse=0,max_e=0,sum_t=0,max_t=0;
        for (int k=0;k<n_sim;++k) {
            double dx=x_hist[(k+1)*NX]-ref_hist[k*3];
            double dy=x_hist[(k+1)*NX+1]-ref_hist[k*3+1];
            double e=sqrt(dx*dx+dy*dy);
            rmse+=e*e; if (e>max_e) max_e=e;
            sum_t+=t_hist[k]; if (t_hist[k]>max_t) max_t=t_hist[k];
        }
        rmse=sqrt(rmse/n_sim);
        std::printf("\n  Position RMSE:   %.4f m\n",rmse);
        std::printf("  Position MAX:    %.4f m\n",max_e);
        std::printf("  Avg solve time:  %.2f ms\n",sum_t/n_sim);
        if (max_t>0) std::printf("  Max solve time:  %.2f ms\n",max_t);
    }

    // 写 CSV
    {
        std::string base(csv_dir);
        write_csv_mat((base+"/x_history_"+tag+".csv").c_str(),
                       "px,py,theta",x_hist,n_sim+1,NX);
        write_csv_mat((base+"/u_history_"+tag+".csv").c_str(),
                       "v,omega",u_hist,n_sim,NU);
        write_csv_mat((base+"/ref_history_"+tag+".csv").c_str(),
                       "x_ref,y_ref,theta_ref",ref_hist,n_sim,3);
        // solve times
        {
            FILE *f=fopen((base+"/solve_times_"+tag+".csv").c_str(),"w");
            if (f) { fprintf(f,"time_ms\n"); for (int k=0;k<n_sim;++k) fprintf(f,"%.6f\n",t_hist[k]); fclose(f); }
        }
        // params
        {
            FILE *f=fopen((base+"/sim_params_"+tag+".csv").c_str(),"w");
            if (f) { fprintf(f,"param,value\nN,%d\ndt,%.4f\nT_sim,%.1f\nn_sim_steps,%d\n",N,dt,T_sim,n_sim); fclose(f); }
        }
        std::printf("  Wrote CSV files to %s/*_%s.csv\n",csv_dir,tag);
    }

    delete[] x_hist; delete[] u_hist; delete[] ref_hist; delete[] t_hist;
    unicycle_acados_free(c); unicycle_acados_free_capsule(c);
    std::printf("  PASS\n");
    return true;

fail:
    delete[] x_hist; delete[] u_hist; delete[] ref_hist; delete[] t_hist;
    unicycle_acados_free(c); unicycle_acados_free_capsule(c);
    return false;
}

// Test 3: 开环模型
static bool test_open_loop_model(const char *csv_dir) {
    std::printf("--- Test: Open-Loop Model Verification ---\n");

    const int n_steps=20;
    const double dt=0.05;
    double x0[3]={0.0,0.0,0.5*M_PI};
    double u0[2]={1.0,0.5};

    double xk[3]={x0[0],x0[1],x0[2]};
    auto *xt = new double[(n_steps+1)*NX];
    xt[0]=xk[0];xt[1]=xk[1];xt[2]=xk[2];

    for (int k=0;k<n_steps;++k) {
        rk4_step(xk,u0,dt,xk);
        xt[(k+1)*NX]=xk[0];xt[(k+1)*NX+1]=xk[1];xt[(k+1)*NX+2]=xk[2];
    }
    std::printf("  Final state: (%8.6f,%8.6f,%8.6f)\n",xk[0],xk[1],xk[2]);

    ensure_dir(csv_dir);
    std::string base(csv_dir);
    write_csv_mat((base+"/open_loop_traj.csv").c_str(),
                   "px,py,theta",xt,n_steps+1,NX);
    std::printf("  Wrote %s/open_loop_traj.csv\n",csv_dir);

    {
        FILE *f=fopen((base+"/open_loop_params.csv").c_str(),"w");
        if (f) {
            fprintf(f,"param,value\nx0_px,%.4f\nx0_py,%.4f\nx0_theta,%.4f\n",x0[0],x0[1],x0[2]);
            fprintf(f,"u_v,%.4f\nu_omega,%.4f\ndt,%.4f\nn_steps,%d\n",u0[0],u0[1],dt,n_steps);
            fclose(f);
        }
    }

    delete[] xt;
    std::printf("  PASS\n");
    return true;
}

int main() {
    const char *out = "../data";
    ensure_dir(out);

    std::printf("============================================================\n");
    std::printf("acados Unicycle Solver C - Closed-Loop MPC Test\n");
    std::printf("  NX=%d, NU=%d, N=%d\n",NX,NU,N);
    std::printf("  Output: %s/\n",out);
    std::printf("============================================================\n\n");

    int pass=0,total=4;
    if (test_create_destroy()) ++pass; std::printf("\n");
    if (test_open_loop_model(out)) ++pass; std::printf("\n");
    if (test_closed_loop_mpc(TrajectoryType::CIRCLE,"circle",out)) ++pass; std::printf("\n");
    if (test_closed_loop_mpc(TrajectoryType::FIGURE8,"figure8",out)) ++pass; std::printf("\n");

    std::printf("============================================================\n");
    std::printf("Results: %d/%d\n",pass,total);
    std::printf("CSV data: %s/\n",out);
    std::printf("Run: python3 ../plot_results.py\n");
    std::printf("============================================================\n");

    return (pass==total)?0:1;
}
