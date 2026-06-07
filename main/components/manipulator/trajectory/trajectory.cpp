#include "trajectory.h"
#include <cstring>
#include <cmath>

// =============================================================================
// TrajectoryInterpolator
// =============================================================================

TrajectoryInterpolator::TrajectoryInterpolator() {
    clear();
}

void TrajectoryInterpolator::clear() {
    m_count = 0;
    m_built = false;
    memset(m_times,  0, sizeof(m_times));
    memset(m_A, 0, sizeof(m_A));
    memset(m_B, 0, sizeof(m_B));
    memset(m_C, 0, sizeof(m_C));
    memset(m_D, 0, sizeof(m_D));
}

bool TrajectoryInterpolator::addWaypoint(float t, const JointAngles& q) {
    if (m_count >= MAX_WAYPOINTS)               return false;
    if (m_count > 0 && t <= m_times[m_count-1]) return false;

    m_times[m_count]  = t;
    m_points[m_count] = q;
    ++m_count;
    m_built = false;
    return true;
}

float TrajectoryInterpolator::getDuration() const {
    if (m_count < 2) return 0.0f;
    return m_times[m_count - 1] - m_times[0];
}

bool TrajectoryInterpolator::build() {
    if (m_count < MIN_WAYPOINTS) return false;

    for (size_t j = 0; j < DOF; ++j) {
        buildJoint(j);
    }
    m_built = true;
    return true;
}

// =============================================================================
// buildJoint — натуральный кубический сплайн для одного сустава j
//
// Пусть n = m_count - 1 (число сегментов), точки q[0..n] в моменты t[0..n].
// Ищем вторые производные M[0..n], граничное условие: M[0]=M[n]=0.
//
// Для каждой внутренней точки i=1..n-1 (система n-1 уравнений):
//   h[i-1]*M[i-1] + 2*(h[i-1]+h[i])*M[i] + h[i]*M[i+1]
//       = 6 * ( (q[i+1]-q[i])/h[i] - (q[i]-q[i-1])/h[i-1] )
//
// Решаем трёхдиагональную систему методом прогонки (Thomas algorithm).
//
// Коэффициенты полинома на сегменте i:
//   dt  = t - t[i]
//   A = q[i]
//   B = (q[i+1]-q[i])/h[i] - h[i]*(2*M[i]+M[i+1])/6
//   C = M[i]/2
//   D = (M[i+1]-M[i]) / (6*h[i])
// =============================================================================
void TrajectoryInterpolator::buildJoint(size_t j) {
    const size_t n = m_count - 1; // число сегментов

    // Длины сегментов
    float h[MAX_WAYPOINTS] = {};
    for (size_t i = 0; i < n; ++i) {
        h[i] = m_times[i + 1] - m_times[i];
    }

    // Вторые производные M[0..n], граничные M[0]=M[n]=0
    float M[MAX_WAYPOINTS] = {};

    if (n >= 2) {
        const size_t sz = n - 1; // число внутренних точек

        // Правая часть
        float rhs[MAX_WAYPOINTS] = {};
        for (size_t k = 0; k < sz; ++k) {
            const size_t i = k + 1; // индекс внутренней точки
            float dy_r = (m_points[i+1][j] - m_points[i  ][j]) / h[i  ];
            float dy_l = (m_points[i  ][j] - m_points[i-1][j]) / h[i-1];
            rhs[k] = 6.0f * (dy_r - dy_l);
        }

        // Метод прогонки (Thomas algorithm) для трёхдиагональной системы:
        //   нижняя диагональ: lower[k] = h[k]         (k = 1..sz-1)
        //   главная:          diag[k]  = 2*(h[k]+h[k+1])
        //   верхняя диагональ: upper[k] = h[k+1]       (k = 0..sz-2)
        float c_prime[MAX_WAYPOINTS] = {}; // модифицированная верхняя диагональ
        float d_prime[MAX_WAYPOINTS] = {}; // модифицированная правая часть

        // Первая строка (k=0)
        float diag0 = 2.0f * (h[0] + h[1]);
        c_prime[0] = (sz > 1) ? (h[1] / diag0) : 0.0f;
        d_prime[0] = rhs[0] / diag0;

        // Прямой ход (k=1..sz-1)
        for (size_t k = 1; k < sz; ++k) {
            float lower_k = h[k];
            float diag_k  = 2.0f * (h[k] + h[k+1]) - lower_k * c_prime[k-1];
            c_prime[k] = (k < sz - 1) ? (h[k+1] / diag_k) : 0.0f;
            d_prime[k] = (rhs[k] - lower_k * d_prime[k-1]) / diag_k;
        }

        // Обратный ход: x[k] → M[k+1]
        M[sz] = d_prime[sz - 1]; // M[n-1]
        for (int k = (int)sz - 2; k >= 0; --k) {
            M[k + 1] = d_prime[k] - c_prime[k] * M[k + 2];
        }
        // M[0] = M[n] = 0 — уже установлено инициализацией
    }
    // При n==1 (два узла) M[0]=M[1]=0, полином вырождается в линейный.

    // Вычисляем коэффициенты A, B, C, D для каждого сегмента
    for (size_t i = 0; i < n; ++i) {
        float qi   = m_points[i  ][j];
        float qi1  = m_points[i+1][j];
        float hi   = h[i];
        float Mi   = M[i];
        float Mi1  = M[i+1];
        float dy   = (qi1 - qi) / hi;

        m_A[i][j] = qi;
        m_B[i][j] = dy - hi * (2.0f * Mi + Mi1) / 6.0f;
        m_C[i][j] = Mi / 2.0f;
        m_D[i][j] = (Mi1 - Mi) / (6.0f * hi);
    }
}

bool TrajectoryInterpolator::evaluate(float t, JointAngles& q_out) const {
    if (!m_built) return false;

    const size_t n = m_count - 1;

    // Зажимаем на границах
    if (t <= m_times[0]) {
        q_out = m_points[0];
        return true;
    }
    if (t >= m_times[m_count - 1]) {
        q_out = m_points[m_count - 1];
        return true;
    }

    // Поиск сегмента: первый i такой, что t < t[i+1]
    size_t seg = n - 1;
    for (size_t i = 0; i < n; ++i) {
        if (t < m_times[i + 1]) {
            seg = i;
            break;
        }
    }

    float dt  = t - m_times[seg];
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;

    for (size_t j = 0; j < DOF; ++j) {
        q_out[j] = m_A[seg][j]
                 + m_B[seg][j] * dt
                 + m_C[seg][j] * dt2
                 + m_D[seg][j] * dt3;
    }
    return true;
}
