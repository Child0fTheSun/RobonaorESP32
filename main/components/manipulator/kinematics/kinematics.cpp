#include "kinematics.h"
#include <cmath>
#include <cstring>

// =============================================================================
// Matrix4x4
// =============================================================================

Matrix4x4::Matrix4x4() { memset(m, 0, sizeof(m)); }

Matrix4x4 Matrix4x4::identity() {
  Matrix4x4 I;
  I.m[0][0] = I.m[1][1] = I.m[2][2] = I.m[3][3] = 1.0f;
  return I;
}

Matrix4x4 Matrix4x4::operator*(const Matrix4x4 &rhs) const {
  Matrix4x4 result;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      float sum = 0.0f;
      for (int k = 0; k < 4; ++k) {
        sum += m[i][k] * rhs.m[k][j];
      }
      result.m[i][j] = sum;
    }
  }
  return result;
}

// =============================================================================
// ManipulatorKinematics
// =============================================================================

// Матрица однородного преобразования по классическому DH-соглашению:
//
//          | cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)   a·cos(θ) |
// T_i-1_i =| sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)   a·sin(θ) |
//          |   0        sin(α)          cos(α)           d    |
//          |   0          0               0              1    |
//
// где θ = q[i] + theta_offset (переменный угол серво + сдвиг нуля)
//
Matrix4x4 ManipulatorKinematics::linkTransform(size_t i, float theta) const {
  const DHParams &p = DH_TABLE[i];

  float th = theta + p.theta_offset;
  float ct = cosf(th);
  float st = sinf(th);
  float ca = cosf(p.alpha);
  float sa = sinf(p.alpha);

  Matrix4x4 T;
  T.m[0][0] = ct;
  T.m[0][1] = -st * ca;
  T.m[0][2] = st * sa;
  T.m[0][3] = p.a * ct;
  T.m[1][0] = st;
  T.m[1][1] = ct * ca;
  T.m[1][2] = -ct * sa;
  T.m[1][3] = p.a * st;
  T.m[2][0] = 0.0f;
  T.m[2][1] = sa;
  T.m[2][2] = ca;
  T.m[2][3] = p.d;
  T.m[3][0] = 0.0f;
  T.m[3][1] = 0.0f;
  T.m[3][2] = 0.0f;
  T.m[3][3] = 1.0f;

  return T;
}

Matrix4x4 ManipulatorKinematics::forwardKinematics(const JointAngles &q) const {
  Matrix4x4 T = Matrix4x4::identity();
  for (size_t i = 0; i < DOF; ++i) {
    T = T * linkTransform(i, q[i]);
  }
  return T;
}

Pose ManipulatorKinematics::computePose(const JointAngles &q) const {
  Matrix4x4 T = forwardKinematics(q);

  Pose pose;

  // Положение TCP (последний столбец матрицы)
  pose.x = T.m[0][3];
  pose.y = T.m[1][3];
  pose.z = T.m[2][3];

  // Извлечение RPY (roll-pitch-yaw, конвенция Z-Y-X)
  // pitch (Y)
  pose.pitch =
      atan2f(-T.m[2][0], sqrtf(T.m[0][0] * T.m[0][0] + T.m[1][0] * T.m[1][0]));

  float cos_pitch = cosf(pose.pitch);
  if (fabsf(cos_pitch) > 1e-6f) {
    // roll (X)
    pose.roll = atan2f(T.m[2][1] / cos_pitch, T.m[2][2] / cos_pitch);
    // yaw (Z)
    pose.yaw = atan2f(T.m[1][0] / cos_pitch, T.m[0][0] / cos_pitch);
  } else {
    // Gimbal lock: pitch = ±90°
    pose.roll = 0.0f;
    pose.yaw = atan2f(T.m[0][1], T.m[1][1]);
  }

  return pose;
}

// =============================================================================
// Обратная задача кинематики (ОЗК)
//
// Геометрическое решение для 5-DOF манипулятора:
//
//   Шаг 1: угол поворота основания
//     θ₀ = atan2(y, x)
//
//   Шаг 2: планарная 2R-задача (a₁=DH[1].a, a₂=DH[2].a)
//     r = sqrt(x²+y²) - d₄  (d₄ = DH[4].d, горизонтальное смещение запястья)
//     s = z - d₀             (d₀ = DH[0].d, высота первого звена)
//     D = sqrt(r²+s²)       (расстояние до центру запястья)
//
//   Теорема косинусов (cosθ₂ должен быть в [-1,1]):
//     cosθ₂ = (a₁² + a₂² - D²) / (2·a₁·a₂)  ← знак MINUS (закон Коша, в запросе был опечатка)
//     θ₂ = arccos(cosθ₂)
//     θ₁ = atan2(s,r) - atan2(a₂·sin(θ₂), a₁+a₂·cos(θ₂))
//
//   Шаг 3: ориентация запястья
//     θ₃ = φ - (θ₁ + θ₂),  φ = target.pitch
//
//   Шаг 4: ориентация кисти (вращение вокруг оси TCP)
//     θ₄ = target.roll
// =============================================================================
bool ManipulatorKinematics::inverseKinematics(const Pose &target,
                                              JointAngles &q_out) const {
  // Извлекаем параметры из DH-таблицы
  const float d0 = DH_TABLE[0].d; // высота первого звена (мм)
  const float a1 = DH_TABLE[1].a; // длина плеча (мм)
  const float a2 = DH_TABLE[2].a; // длина предплечья (мм)
  const float d4 = DH_TABLE[4].d; // смещение запястья (мм)

  // Шаг 1: угол поворота основания
  const float theta0 = atan2f(target.y, target.x);

  // Шаг 2: планарная 2R-задача
  const float r = sqrtf(target.x * target.x + target.y * target.y) - d4;
  const float s = target.z - d0;
  const float D = sqrtf(r * r + s * s);

  // Проверка достижимости точки
  if (D > a1 + a2) {
    return false; // Точка находится за пределами рабочей зоны
  }

  // Теорема косинусов: cosθ₂ = (a₁²+a₂²-D²)/(2·a₁·a₂)
  // (Знак перед D² — минус, как в классической теореме косинусов)
  const float cos_theta2 = (a1 * a1 + a2 * a2 - D * D) / (2.0f * a1 * a2);

  // Дополнительная защита от ошибок округления
  if (cos_theta2 < -1.0f || cos_theta2 > 1.0f) {
    return false;
  }

  const float theta2 = acosf(cos_theta2);
  const float theta1 = atan2f(s, r) -
                       atan2f(a2 * sinf(theta2), a1 + a2 * cosf(theta2));

  // Шаг 3: угол запястья
  const float theta3 = target.pitch - (theta1 + theta2);

  // Шаг 4: вращение кисти
  const float theta4 = target.roll;

  q_out[0] = theta0;
  q_out[1] = theta1;
  q_out[2] = theta2;
  q_out[3] = theta3;
  q_out[4] = theta4;

  return true;
}


// =============================================================================
// Интерполяция траектории — реализована в отдельном компоненте:
//   manipulator/trajectory/trajectory.h  →  class TrajectoryInterpolator
//
// Поддерживает:
//   - Кубический натуральный сплайн в пространстве обобщённых координат
//   - 2–5 опорных точек; каждая точка задаётся как (время, JointAngles)
//   - Evaluate(t) возвращает углы суставов в произвольный момент t
//
// Интерполяция в декартовом пространстве (Cartesian-space) — планируется.
// =============================================================================
