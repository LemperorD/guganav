#pragma once

#include <point_lio/core/so3_math.h>
#include <IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp>

using vect3 = MTK::vect<3, double>;
using SO3 = MTK::SO3<double>;

MTK_BUILD_MANIFOLD(state_input, ((vect3, pos))((SO3, rot))((SO3, offset_R_L_I))(
                                    (vect3, offset_T_L_I))((vect3, vel))((
                                    vect3, bg))((vect3, ba))((vect3, gravity)));

MTK_BUILD_MANIFOLD(state_output,
                   ((vect3, pos))((SO3, rot))((SO3, offset_R_L_I))(
                       (vect3, offset_T_L_I))((vect3, vel))((vect3, omg))(
                       (vect3, acc))((vect3, gravity))((vect3, bg))((vect3,
                                                                     ba)));

MTK_BUILD_MANIFOLD(input_ikfom, ((vect3, acc))((vect3, gyro)));
