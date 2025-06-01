import rclpy
import numpy as np

# Importación de los modelos 8D
from .motion_models.acceleration_motion_models import acceleration_motion_model_linearized_2
from .observation_models.odometry_imu_observation_models import odometry_imu_observation_model_with_acceleration_motion_model_linearized_2

from .filters.ekf import ExtendedKalmanFilter
from .kf_node import KalmanFilterFusionNode as ExtendedKalmanFilterFusionNode

def main(args=None):
    # Initialize the Kalman Filter
    # Estado inicial: [x, y, θ, v_x, v_y, ω, a_x, a_y]
    mu0 = np.zeros(8)
    Sigma0 = np.eye(8)

    # Selección de configuración de ruido (cambia este valor para probar cada caso):
    noise_config = "base"

    if noise_config == "base":
        print("Usando configuración: Caso base (valores por defecto)")
        # proc_noise_std = [x, y, theta, v_x, v_y, w, a_x, a_y]
        proc_noise_std = [0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1, 0.1]  # Ruido moderado en el modelo
        # obs_noise_std  = [x, y, theta, theta_imu, w, a_x, a_y]
        obs_noise_std  = [100.0, 100.0, 1000.0,
                          6.85e-06,
                          1.10e-06,
                          0.00153,
                          0.00153]  # Ruido moderado en las observaciones

    elif noise_config == "alto_obs":
        print("Usando configuración: Alta incertidumbre en la observación")
        proc_noise_std = [0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1, 0.1]  # Ruido moderado en el modelo
        obs_noise_std  = [1000.0, 1000.0, 10000.0,
                          6.85e-05,
                          1.10e-05,
                          0.15,
                          0.15]  # Ruido alto en las observaciones

    elif noise_config == "alto_mod":
        print("Usando configuración: Alta incertidumbre en el modelo de movimiento")
        proc_noise_std = [1.0, 1.0, 0.5, 1.0, 1.0, 1.0, 1.0, 1.0]  # Ruido alto en el modelo
        obs_noise_std  = [100.0, 100.0, 1000.0,
                          6.85e-06,
                          1.10e-06,
                          0.00153,
                          0.00153]  # Ruido moderado en las observaciones

    else:
        raise ValueError(f"Configuración de ruido no reconocida: {noise_config}")

    # Inicializar el filtro de Kalman extendido con los modelos adecuados
    ekf = ExtendedKalmanFilter(
        mu0,
        Sigma0,
        acceleration_motion_model_linearized_2,
        odometry_imu_observation_model_with_acceleration_motion_model_linearized_2,
        proc_noise_std=proc_noise_std,
        obs_noise_std=obs_noise_std
    )

    rclpy.init(args=args)
    kalman_filter_node = ExtendedKalmanFilterFusionNode(ekf)
    rclpy.spin(kalman_filter_node)
    kalman_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()