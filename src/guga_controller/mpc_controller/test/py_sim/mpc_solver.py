from c_codegen import MPCSolver

if __name__ == "__main__":
    mpc_solver = MPCSolver()
    mpc_solver.ocp.code_export_directory = os.path.join(os.path.dirname(__file__), "c_code")
    mpc_solver.ocp.code_export_directory = os.path.abspath(mpc_solver.ocp.code_export_directory)
    mpc_solver.ocp.export()