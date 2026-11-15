# bg、ba 残差修改清单

改完后模型：量测残差含零偏项，动力学 v̇ = R·a + g 不含零偏，与论文式 2、式 7 一致。

## 1. ManifoldEskf.cpp 构造函数，H_I 补两列

```cpp
H_I.block<3, 3>(0, 9) = Matrix3d::Identity();   // 角速度残差对 bg
H_I.block<3, 3>(0, 18) = Matrix3d::Identity();  // 角速度残差对 omega
H_I.block<3, 3>(3, 12) = Matrix3d::Identity();  // 加速度残差对 ba
H_I.block<3, 3>(3, 21) = Matrix3d::Identity();  // 加速度残差对 acc
```

## 2. ManifoldEskf.cpp 的 imuUpdate，残差补零偏项

```cpp
r_I.segment<3>(0) = sample.measurement.angular_velocity
                    - (state.omega + state.bg);
r_I.segment<3>(3) = sample.measurement.linear_acceleration * acc_scale
                    - (state.acc + state.ba);
```

## 3. ManifoldEskf.cpp 的 f_，动力学去掉 ba

```cpp
d.segment<3>(6) = x_.rot * x_.acc + x_.gravity;
```

## 4. ManifoldEskf.cpp 的 initialize，解冻 ba 协方差

删除：

```cpp
// 加速度零偏固定, 避免点面匹配的错误残差通过 vel 把它拉偏.
P.middleRows<3>(12).setZero();
P.middleCols<3>(12).setZero();
```

重力块（15 行）的置零保留。

## 5. ManifoldEskf::initState，ba 初值用标定值

当前位置在 `ManifoldEskf.cpp`（原 Imu.cpp 的 initState 已迁入）。

```cpp
x_.ba = calibrated_acc_bias;  // 六面法标定值, 若单位是 g 先乘 9.81
```

`x_.bg = estimate.mean_gyr;` 保留，标定完成后换成标定陀螺零偏。

当前没有标定值来源，先用零值并加 TODO 注释。

## 不动

- 饱和检查与坏点过滤。
- 过程噪声 Q 的 n_bg、n_ba 项。
- 重力块冻结、F_w、P 的其余部分。

## 遗留

F_x 的 (6,0) 块仍用 `Hat(x_.acc - x_.ba)`，与新动力学 `v̇ = R·a + g` 不一致。按清单未改。

## 验证

给 IMU 注入恒定 b_g 与 b_a，看估计值是否收敛到注入值，静止时 vel 的 Z 是否停在零附近。
