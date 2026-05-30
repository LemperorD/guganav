# root_finder.hpp
│
├── ① 多项式基础工具
│   ├── polyEval      多项式求值（高稳定）
│   ├── polyDeri      多项式求导
│   ├── polyConv      多项式卷积
│   └── polySqr       多项式平方
│
├── ② 低阶多项式解析解
│   ├── solveCub      三次方程解析解
│   ├── solveQuart    四次方程解析解
│
├── ③ 高阶多项式数值解
│   ├── Sturm 序列构造
│   ├── 根区间隔离（isolate）
│   ├── Safe Newton 收敛
│
├── ④ 备用方法
│   └── companion matrix + Eigen 特征值
│
└── ⑤ 统一对外接口
    └── solvePolynomial()
