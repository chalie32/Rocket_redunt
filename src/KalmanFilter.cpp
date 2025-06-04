#include "KalmanFilter.h"
#include <math.h>

KalmanFilter::KalmanFilter(float q_angle, float q_velocity, float r_measure) :
    x_est(0.0f),
    x_vel(0.0f),
    P00(1.0f), P01(0.0f), P10(0.0f), P11(1.0f),
    Q_angle(q_angle),
    Q_velocity(q_velocity),
    R_measure(r_measure),
    last_time(0),
    initialized(false) {
}

void KalmanFilter::initialize(float initial_value) {
    x_est = initial_value;
    x_vel = 0.0f;
    
    // Initialize covariance matrix with some uncertainty
    P00 = 1.0f;
    P01 = 0.0f;
    P10 = 0.0f;
    P11 = 1.0f;
    
    last_time = millis();
    initialized = true;
}

float KalmanFilter::update(float measurement, float dt) {
    if (!initialized) {
        initialize(measurement);
        return measurement;
    }
    
    // Calculate time step if not provided
    if (dt <= 0.0f) {
        unsigned long current_time = millis();
        dt = (current_time - last_time) / 1000.0f; // Convert to seconds
        last_time = current_time;
        
        // Prevent very small or negative time steps
        if (dt <= 0.001f) {
            dt = 0.001f;
        }
        // Prevent very large time steps (system reset or long delay)
        if (dt > 1.0f) {
            dt = 1.0f;
        }
    }
    
    // ===============================
    // PREDICT STEP
    // ===============================
    
    // State prediction: x = F * x_prev
    // F = [1, dt]  (state transition matrix)
    //     [0, 1 ]
    float x_est_pred = x_est + dt * x_vel;
    float x_vel_pred = x_vel; // Assume constant velocity
    
    // Covariance prediction: P = F * P * F^T + Q
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;
    
    // Process noise matrix Q
    float Q00 = Q_angle * dt4 / 4.0f;
    float Q01 = Q_angle * dt3 / 2.0f;
    float Q10 = Q_angle * dt3 / 2.0f;
    float Q11 = Q_angle * dt2 + Q_velocity * dt;
    
    // Predicted covariance: P_pred = F * P * F^T + Q
    float P00_pred = P00 + dt * (P01 + P10) + dt2 * P11 + Q00;
    float P01_pred = P01 + dt * P11 + Q01;
    float P10_pred = P10 + dt * P11 + Q10;
    float P11_pred = P11 + Q11;
    
    // ===============================
    // UPDATE STEP
    // ===============================
    
    // Measurement residual
    float y = measurement - x_est_pred; // H = [1, 0] for position measurement
    
    // Residual covariance
    float S = P00_pred + R_measure;
    
    // Kalman gain: K = P * H^T * S^(-1)
    float K0 = P00_pred / S;
    float K1 = P10_pred / S;
    
    // State update: x = x_pred + K * y
    x_est = x_est_pred + K0 * y;
    x_vel = x_vel_pred + K1 * y;
    
    // Covariance update: P = (I - K * H) * P_pred
    float IKH00 = 1.0f - K0;
    float IKH01 = 0.0f;
    float IKH10 = -K1;
    float IKH11 = 1.0f;
    
    P00 = IKH00 * P00_pred + IKH01 * P10_pred;
    P01 = IKH00 * P01_pred + IKH01 * P11_pred;
    P10 = IKH10 * P00_pred + IKH11 * P10_pred;
    P11 = IKH10 * P01_pred + IKH11 * P11_pred;
    
    // Ensure covariance matrix remains positive definite
    if (P00 < 0.0f) P00 = 0.001f;
    if (P11 < 0.0f) P11 = 0.001f;
    
    return x_est;
}

void KalmanFilter::reset() {
    x_est = 0.0f;
    x_vel = 0.0f;
    P00 = 1.0f;
    P01 = 0.0f;
    P10 = 0.0f;
    P11 = 1.0f;
    last_time = 0;
    initialized = false;
}

void KalmanFilter::setProcessNoise(float q_angle, float q_velocity) {
    Q_angle = q_angle;
    Q_velocity = q_velocity;
}

void KalmanFilter::setMeasurementNoise(float r_measure) {
    R_measure = r_measure;
}

// =====================================================================
// IMU Extended Kalman Filter Implementation
// =====================================================================

IMUExtendedKalmanFilter::IMUExtendedKalmanFilter(float q_angle, float q_bias, float r_accel) :
    Q_angle(q_angle),
    Q_bias(q_bias),
    R_accel(r_accel),
    last_time(0),
    initialized(false) {
    
    // Initialize state vector to identity quaternion and zero bias
    state[0] = 1.0f; // q0 (w)
    state[1] = 0.0f; // q1 (x)
    state[2] = 0.0f; // q2 (y)
    state[3] = 0.0f; // q3 (z)
    state[4] = 0.0f; // bias_x
    state[5] = 0.0f; // bias_y
    state[6] = 0.0f; // bias_z
    
    // Initialize covariance matrix
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            P[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

void IMUExtendedKalmanFilter::initialize(float accel_x, float accel_y, float accel_z) {
    // Initialize orientation from accelerometer (assuming stationary)
    float accel[3] = {accel_x, accel_y, accel_z};
    normalizeVector(accel);
    
    // Calculate initial orientation assuming Z-up frame
    float pitch = asin(-accel[0]);
    float roll = atan2(accel[1], accel[2]);
    float yaw = 0.0f; // Cannot determine yaw from accelerometer alone
    
    // Convert to quaternion
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);
    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);
    
    state[0] = cp * cr * cy + sp * sr * sy; // w
    state[1] = sp * cr * cy - cp * sr * sy; // x
    state[2] = cp * sr * cy + sp * cr * sy; // y
    state[3] = cp * cr * sy - sp * sr * cy; // z
    
    // Initialize bias to zero
    state[4] = state[5] = state[6] = 0.0f;
    
    // Initialize covariance with appropriate uncertainty
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            P[i][j] = 0.0f;
        }
    }
    
    // Quaternion uncertainty
    P[0][0] = P[1][1] = P[2][2] = P[3][3] = 0.1f;
    // Bias uncertainty
    P[4][4] = P[5][5] = P[6][6] = 0.01f;
    
    last_time = millis();
    initialized = true;
}

void IMUExtendedKalmanFilter::update(float gyro_x, float gyro_y, float gyro_z,
                                   float accel_x, float accel_y, float accel_z,
                                   float dt) {
    if (!initialized) {
        initialize(accel_x, accel_y, accel_z);
        return;
    }
    
    // Calculate time step if not provided
    if (dt <= 0.0f) {
        unsigned long current_time = millis();
        dt = (current_time - last_time) / 1000.0f;
        last_time = current_time;
        
        if (dt <= 0.001f) dt = 0.001f;
        if (dt > 0.1f) dt = 0.1f; // Limit to 100ms max
    }
    
    // Apply gyroscope deadband to filter sensor noise when stationary
    const float GYRO_DEADBAND = 0.02f; // ~1 degree/second deadband
    float filtered_gyro_x = (fabsf(gyro_x) > GYRO_DEADBAND) ? gyro_x : 0.0f;
    float filtered_gyro_y = (fabsf(gyro_y) > GYRO_DEADBAND) ? gyro_y : 0.0f;
    float filtered_gyro_z = (fabsf(gyro_z) > GYRO_DEADBAND) ? gyro_z : 0.0f;
    
    // Prepare data
    float gyro[3] = {filtered_gyro_x, filtered_gyro_y, filtered_gyro_z};
    float accel[3] = {accel_x, accel_y, accel_z};
    
    // Predict step
    predict(gyro, dt);
    
    // Update step (only if accelerometer magnitude is reasonable and stable)
    float accel_mag = vectorMagnitude(accel);
    
    // Debug output every 100 updates
    static int updateCounter = 0;
    if (++updateCounter >= 100) {
        updateCounter = 0;
        Serial.print(F("[EKF Debug] Accel mag: "));
        Serial.print(accel_mag, 3);
        Serial.print(F(" m/s² - "));
        if (accel_mag > 8.5f && accel_mag < 11.0f) { // Tighter range for stability
            Serial.println(F("UPDATING"));
        } else {
            Serial.println(F("SKIPPING (out of range)"));
        }
    }
    
    // More conservative accelerometer update range for better stability
    if (accel_mag > 8.5f && accel_mag < 11.0f) { // Between 0.85g and 1.15g (tighter)
        update(accel);
    }
}

void IMUExtendedKalmanFilter::predict(float gyro[3], float dt) {
    // Remove estimated bias from gyroscope readings
    float corrected_gyro[3] = {
        gyro[0] - state[4],
        gyro[1] - state[5],
        gyro[2] - state[6]
    };
    
    // Create omega matrix for quaternion integration
    float omega_mag = vectorMagnitude(corrected_gyro);
    
    // Only skip quaternion update if gyro is EXTREMELY small, but always update covariance
    bool update_quaternion = (omega_mag > 1e-6f); // Much more reasonable threshold
    
    if (update_quaternion) {
        float half_dt = dt * 0.5f;
        float cos_half_omega_dt = cos(omega_mag * half_dt);
        
        float half_dt_gyro[3] = {
            corrected_gyro[0] * half_dt,
            corrected_gyro[1] * half_dt,
            corrected_gyro[2] * half_dt
        };
        
        // Rotation quaternion
        float dq[4] = {
            cos_half_omega_dt,
            (corrected_gyro[0] / omega_mag) * sin(omega_mag * half_dt),
            (corrected_gyro[1] / omega_mag) * sin(omega_mag * half_dt),
            (corrected_gyro[2] / omega_mag) * sin(omega_mag * half_dt)
        };
        
        // Quaternion multiplication: q = q * dq
        float old_q[4] = {state[0], state[1], state[2], state[3]};
        quaternionMultiply(old_q, dq, &state[0]);
        normalizeQuaternion();
    }
    
    // Bias remains constant (state[4], state[5], state[6] unchanged)
    
    // ALWAYS update covariance matrix - this is critical for filter stability
    float F[7][7];
    
    // Initialize F as identity
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            F[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    if (update_quaternion) {
        // Set up jacobian for quaternion part only if we updated quaternion
        float half_dt = dt * 0.5f;
        float cos_half_omega_dt = cos(omega_mag * half_dt);
        
        float half_dt_gyro[3] = {
            corrected_gyro[0] * half_dt,
            corrected_gyro[1] * half_dt,
            corrected_gyro[2] * half_dt
        };
        
        // Partial derivatives of quaternion with respect to quaternion states
        F[0][0] = cos_half_omega_dt;
        F[0][1] = -half_dt_gyro[0];
        F[0][2] = -half_dt_gyro[1];
        F[0][3] = -half_dt_gyro[2];
        
        F[1][0] = half_dt_gyro[0];
        F[1][1] = cos_half_omega_dt;
        F[1][2] = half_dt_gyro[2];
        F[1][3] = -half_dt_gyro[1];
        
        F[2][0] = half_dt_gyro[1];
        F[2][1] = -half_dt_gyro[2];
        F[2][2] = cos_half_omega_dt;
        F[2][3] = half_dt_gyro[0];
        
        F[3][0] = half_dt_gyro[2];
        F[3][1] = half_dt_gyro[1];
        F[3][2] = -half_dt_gyro[0];
        F[3][3] = cos_half_omega_dt;
        
        // Partial derivatives with respect to bias
        F[0][4] = state[1] * half_dt;
        F[0][5] = state[2] * half_dt;
        F[0][6] = state[3] * half_dt;
        
        F[1][4] = -state[0] * half_dt;
        F[1][5] = state[3] * half_dt;
        F[1][6] = -state[2] * half_dt;
        
        F[2][4] = -state[3] * half_dt;
        F[2][5] = -state[0] * half_dt;
        F[2][6] = state[1] * half_dt;
        
        F[3][4] = state[2] * half_dt;
        F[3][5] = -state[1] * half_dt;
        F[3][6] = -state[0] * half_dt;
    }
    // If quaternion wasn't updated, F remains identity for quaternion states
    
    updateCovariance(F, dt);
}

void IMUExtendedKalmanFilter::update(float accel[3]) {
    normalizeVector(accel);
    
    // Expected accelerometer reading based on current orientation
    float gravity_world[3] = {0.0f, 0.0f, -1.0f}; // Gravity in world frame (Z-down)
    float expected_accel[3];
    
    // Rotate gravity vector to body frame
    float q_conj[4] = {state[0], -state[1], -state[2], -state[3]};
    rotateVector(q_conj, gravity_world, expected_accel);
    
    // Innovation (measurement residual)
    float innovation[3] = {
        accel[0] - expected_accel[0],
        accel[1] - expected_accel[1],
        accel[2] - expected_accel[2]
    };
    
    // Measurement jacobian H (3x7)
    float H[3][7];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 7; j++) {
            H[i][j] = 0.0f;
        }
    }
    
    // Partial derivatives of expected accelerometer reading w.r.t quaternion
    H[0][0] = 2.0f * state[2];  H[0][1] = 2.0f * state[3];  H[0][2] = 2.0f * state[0];  H[0][3] = 2.0f * state[1];
    H[1][0] = -2.0f * state[1]; H[1][1] = -2.0f * state[0]; H[1][2] = 2.0f * state[3];  H[1][3] = 2.0f * state[2];
    H[2][0] = -2.0f * state[0]; H[2][1] = 2.0f * state[1];  H[2][2] = 2.0f * state[2];  H[2][3] = -2.0f * state[3];
    
    // Innovation covariance S = H * P * H^T + R
    float S[3][3];
    float HP[3][7];
    
    // Calculate HP
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 7; j++) {
            HP[i][j] = 0.0f;
            for (int k = 0; k < 7; k++) {
                HP[i][j] += H[i][k] * P[k][j];
            }
        }
    }
    
    // Calculate S = HP * H^T + R
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            S[i][j] = 0.0f;
            for (int k = 0; k < 7; k++) {
                S[i][j] += HP[i][k] * H[j][k];
            }
            if (i == j) S[i][j] += R_accel; // Add measurement noise
        }
    }
    
    // Kalman gain K = P * H^T * S^(-1)
    // For 3x3 matrix inversion, we'll use simplified approach
    float det = S[0][0] * (S[1][1] * S[2][2] - S[1][2] * S[2][1]) -
                S[0][1] * (S[1][0] * S[2][2] - S[1][2] * S[2][0]) +
                S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);
    
    if (fabsf(det) < 1e-8f) return; // Singular matrix, skip update
    
    float inv_S[3][3];
    inv_S[0][0] = (S[1][1] * S[2][2] - S[1][2] * S[2][1]) / det;
    inv_S[0][1] = (S[0][2] * S[2][1] - S[0][1] * S[2][2]) / det;
    inv_S[0][2] = (S[0][1] * S[1][2] - S[0][2] * S[1][1]) / det;
    inv_S[1][0] = (S[1][2] * S[2][0] - S[1][0] * S[2][2]) / det;
    inv_S[1][1] = (S[0][0] * S[2][2] - S[0][2] * S[2][0]) / det;
    inv_S[1][2] = (S[0][2] * S[1][0] - S[0][0] * S[1][2]) / det;
    inv_S[2][0] = (S[1][0] * S[2][1] - S[1][1] * S[2][0]) / det;
    inv_S[2][1] = (S[0][1] * S[2][0] - S[0][0] * S[2][1]) / det;
    inv_S[2][2] = (S[0][0] * S[1][1] - S[0][1] * S[1][0]) / det;
    
    // Calculate Kalman gain K = P * H^T * inv_S
    float K[7][3];
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                for (int l = 0; l < 7; l++) {
                    K[i][j] += P[i][l] * H[k][l] * inv_S[k][j];
                }
            }
        }
    }
    
    // State update: x = x + K * innovation
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 3; j++) {
            state[i] += K[i][j] * innovation[j];
        }
    }
    
    // Normalize quaternion after update
    normalizeQuaternion();
    
    // Covariance update: P = (I - K * H) * P
    float IKH[7][7];
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            IKH[i][j] = (i == j) ? 1.0f : 0.0f;
            for (int k = 0; k < 3; k++) {
                IKH[i][j] -= K[i][k] * H[k][j];
            }
        }
    }
    
    float new_P[7][7];
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            new_P[i][j] = 0.0f;
            for (int k = 0; k < 7; k++) {
                new_P[i][j] += IKH[i][k] * P[k][j];
            }
        }
    }
    
    // Copy back to P
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            P[i][j] = new_P[i][j];
        }
    }
}

void IMUExtendedKalmanFilter::updateCovariance(float F[7][7], float dt) {
    // Process noise matrix Q
    float Q[7][7];
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            Q[i][j] = 0.0f;
        }
    }
    
    // Quaternion process noise
    float q_dt = Q_angle * dt;
    Q[0][0] = Q[1][1] = Q[2][2] = Q[3][3] = q_dt;
    
    // Bias process noise
    float q_bias_dt = Q_bias * dt;
    Q[4][4] = Q[5][5] = Q[6][6] = q_bias_dt;
    
    // P = F * P * F^T + Q
    float FP[7][7];
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            FP[i][j] = 0.0f;
            for (int k = 0; k < 7; k++) {
                FP[i][j] += F[i][k] * P[k][j];
            }
        }
    }
    
    float FPFT[7][7];
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            FPFT[i][j] = 0.0f;
            for (int k = 0; k < 7; k++) {
                FPFT[i][j] += FP[i][k] * F[j][k];
            }
            P[i][j] = FPFT[i][j] + Q[i][j];
        }
    }
}

void IMUExtendedKalmanFilter::normalizeQuaternion() {
    float norm = sqrt(state[0]*state[0] + state[1]*state[1] + state[2]*state[2] + state[3]*state[3]);
    if (norm > 1e-8f) {
        state[0] /= norm;
        state[1] /= norm;
        state[2] /= norm;
        state[3] /= norm;
    } else {
        // Fallback to identity quaternion
        state[0] = 1.0f;
        state[1] = state[2] = state[3] = 0.0f;
    }
}

void IMUExtendedKalmanFilter::quaternionToEuler(float& yaw, float& pitch, float& roll) {
    float q0 = state[0], q1 = state[1], q2 = state[2], q3 = state[3];
    
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    roll = atan2(sinr_cosp, cosr_cosp) * 180.0f / PI;
    
    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f) {
        pitch = copysign(PI / 2.0f, sinp) * 180.0f / PI;
    } else {
        pitch = asin(sinp) * 180.0f / PI;
    }
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    yaw = atan2(siny_cosp, cosy_cosp) * 180.0f / PI;
}

void IMUExtendedKalmanFilter::quaternionMultiply(float q1[4], float q2[4], float result[4]) {
    result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    result[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    result[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    result[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

void IMUExtendedKalmanFilter::rotateVector(float q[4], float v[3], float result[3]) {
    // Rotate vector v by quaternion q using: v' = q * [0,v] * q*
    float temp[4];
    float v_quat[4] = {0.0f, v[0], v[1], v[2]};
    float q_conj[4] = {q[0], -q[1], -q[2], -q[3]};
    
    quaternionMultiply(q, v_quat, temp);
    quaternionMultiply(temp, q_conj, v_quat);
    
    result[0] = v_quat[1];
    result[1] = v_quat[2];
    result[2] = v_quat[3];
}

float IMUExtendedKalmanFilter::vectorMagnitude(float v[3]) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

void IMUExtendedKalmanFilter::normalizeVector(float v[3]) {
    float mag = vectorMagnitude(v);
    if (mag > 1e-8f) {
        v[0] /= mag;
        v[1] /= mag;
        v[2] /= mag;
    }
}

void IMUExtendedKalmanFilter::getEulerAngles(float& yaw, float& pitch, float& roll) {
    quaternionToEuler(yaw, pitch, roll);
}

void IMUExtendedKalmanFilter::getQuaternion(float q[4]) {
    q[0] = state[0];
    q[1] = state[1];
    q[2] = state[2];
    q[3] = state[3];
}

void IMUExtendedKalmanFilter::getGyroBias(float& bias_x, float& bias_y, float& bias_z) {
    bias_x = state[4];
    bias_y = state[5];
    bias_z = state[6];
}

void IMUExtendedKalmanFilter::reset() {
    state[0] = 1.0f;
    state[1] = state[2] = state[3] = 0.0f;
    state[4] = state[5] = state[6] = 0.0f;
    
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            P[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    last_time = 0;
    initialized = false;
}

void IMUExtendedKalmanFilter::setProcessNoise(float q_angle, float q_bias) {
    Q_angle = q_angle;
    Q_bias = q_bias;
}

void IMUExtendedKalmanFilter::setMeasurementNoise(float r_accel) {
    R_accel = r_accel;
}

float IMUExtendedKalmanFilter::getConfidence() {
    float trace = 0.0f;
    for (int i = 0; i < 7; i++) {
        trace += P[i][i];
    }
    return trace;
} 