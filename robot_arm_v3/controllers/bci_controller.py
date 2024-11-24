# controllers/bci_controller.py
import numpy as np
import pybullet as p
from scipy.signal import butter, lfilter

class BCIController:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.sample_rate = 250  # Hz
        self.initialize_filters()
        self.control_states = {
            'idle': 0,
            'forward': 1,
            'backward': 2,
            'left': 3,
            'right': 4,
            'grab': 5
        }
        self.current_state = 'idle'

    def initialize_filters(self):
        """Inicializa los filtros para procesar señales EEG"""
        # Filtros para diferentes bandas de frecuencia
        self.filters = {
            'alpha': self._create_bandpass(8, 13),  # 8-13 Hz
            'beta': self._create_bandpass(13, 30),  # 13-30 Hz
            'gamma': self._create_bandpass(30, 50)  # 30-50 Hz
        }

    def _create_bandpass(self, lowcut, highcut, order=5):
        """Crea un filtro paso banda"""
        nyq = 0.5 * self.sample_rate
        low = lowcut / nyq
        high = highcut / nyq
        b, a = butter(order, [low, high], btype='band')
        return b, a

    def process_eeg_signal(self, raw_signal):
        """Procesa la señal EEG y determina la intención del usuario"""
        # Aplicar filtros a la señal
        filtered_signals = {}
        for band, (b, a) in self.filters.items():
            filtered_signals[band] = lfilter(b, a, raw_signal)

        # Detectar patrones específicos (esto sería más complejo en una implementación real)
        return self._classify_signal(filtered_signals)

    def _classify_signal(self, filtered_signals):
        """Clasifica la señal filtrada en una intención de movimiento"""
        # Aquí iría un clasificador más complejo (red neuronal, SVM, etc.)
        # Por ahora usamos una lógica simple basada en umbrales
        alpha_power = np.mean(np.abs(filtered_signals['alpha']))
        beta_power = np.mean(np.abs(filtered_signals['beta']))
        gamma_power = np.mean(np.abs(filtered_signals['gamma']))

        # Lógica simple de clasificación
        if alpha_power > beta_power and alpha_power > gamma_power:
            return 'forward'
        elif beta_power > alpha_power and beta_power > gamma_power:
            return 'backward'
        else:
            return 'idle'

    def update_robot_control(self, eeg_signal):
        """Actualiza el control del robot basado en la señal EEG"""
        intention = self.process_eeg_signal(eeg_signal)
        self.current_state = intention
        self._execute_movement(intention)

    def _execute_movement(self, intention):
        """Ejecuta el movimiento basado en la intención detectada"""
        if intention == 'idle':
            return

        # Mapeo de intenciones a movimientos del robot
        movement_mapping = {
            'forward': {'joint': 1, 'direction': 1},    # Hombro adelante
            'backward': {'joint': 1, 'direction': -1},  # Hombro atrás
            'left': {'joint': 0, 'direction': -1},      # Base izquierda
            'right': {'joint': 0, 'direction': 1},      # Base derecha
            'grab': {'joint': 5, 'direction': 1}        # Cerrar pinza
        }

        if intention in movement_mapping:
            movement = movement_mapping[intention]
            p.setJointMotorControl2(
                self.robot_id,
                movement['joint'],
                p.POSITION_CONTROL,
                targetPosition=p.getJointState(self.robot_id, movement['joint'])[0] + 
                             movement['direction'] * 0.1
            )
