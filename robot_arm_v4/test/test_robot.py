import sys
import os
from pathlib import Path

# Añadir el directorio raíz del proyecto al PATH de Python
current_dir = Path(__file__).parent
project_root = current_dir.parent
sys.path.append(str(project_root))

import pybullet as p
import numpy as np
import time

# Definir las clases de los controladores aquí ya que no existen los módulos
class BCIController:
    def __init__(self, robot_id):
        self.robot_id = robot_id

class JointController:
    def __init__(self, robot_id):
        self.robot_id = robot_id

class KeyboardController:
    def __init__(self, robot_id):
        self.robot_id = robot_id

# Funciones de utilidad
def init_simulation():
    """Inicializa la simulación de PyBullet"""
    physics_client = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    return physics_client

def load_ground():
    """Carga el plano del suelo"""
    return p.loadURDF("plane.urdf")

class TestModernArm:
    def __init__(self):
        # Inicializar la simulación
        self.physics_client = init_simulation()
        self.ground = load_ground()
        
        # Configuración de parámetros globales
        self.SIMULATION_FREQ = 240.0  # Hz
        self.TIME_STEP = 1.0 / self.SIMULATION_FREQ
        self.MAX_FORCE = 500.0  # N
        self.POSITION_TOLERANCE = 0.05  # rad
        self.VELOCITY_TOLERANCE = 0.01  # rad/s
        
        # Cargar el robot URDF
        urdf_path = os.path.join(Path(__file__).parent.parent, "models", "modern_arm.urdf")
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"No se encontró el archivo URDF en: {urdf_path}")
            
        self.robot_id = p.loadURDF(
            urdf_path,
            [0, 0, 0],
            [0, 0, 0, 1],
            useFixedBase=True,
            flags=p.URDF_USE_SELF_COLLISION
        )
        
        if self.robot_id is None:
            raise RuntimeError("No se pudo cargar el robot URDF")
        
        # Inicializar información de articulaciones
        self.initialize_joint_info()
        
        # Configuración inicial del robot
        self.setup_robot()
        
        # Inicializar controladores
        self.bci_controller = BCIController(self.robot_id)
        self.joint_controller = JointController(self.robot_id)
        self.keyboard_controller = KeyboardController(self.robot_id)

    def initialize_joint_info(self):
        """Inicialización mejorada de la información de articulaciones"""
        self.joint_info = {}
        for i in range(p.getNumJoints(self.robot_id)):
            info = p.getJointInfo(self.robot_id, i)
            joint_name = info[1].decode('utf-8')
            self.joint_info[joint_name] = {
                'id': i,
                'type': info[2],
                'lowerLimit': info[8],
                'upperLimit': info[9],
                'maxForce': info[10],
                'maxVelocity': info[11],
                'damping': 0.1,  # valor inicial de amortiguación
                'friction': 0.1,  # valor inicial de fricción
            }
            print(f"Articulación {i}: {joint_name}")

    def setup_robot(self):
        """Configuración mejorada del robot"""
        # Configurar parámetros de simulación
        p.setTimeStep(self.TIME_STEP)
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)  # Modo paso a paso para mejor control
        
        # Configurar parámetros de articulaciones
        for joint_name, info in self.joint_info.items():
            joint_id = info['id']
            # Configurar dinámica de articulaciones
            p.changeDynamics(
                self.robot_id,
                joint_id,
                linearDamping=info['damping'],
                angularDamping=info['damping'],
                jointDamping=info['damping'],
                lateralFriction=info['friction'],
                spinningFriction=info['friction'],
                rollingFriction=info['friction'],
                restitution=0.1,
                maxJointVelocity=info['maxVelocity']
            )
            # Habilitar sensores
            p.enableJointForceTorqueSensor(self.robot_id, joint_id, enableSensor=1)

    def verify_joint_position(self, joint_id, target_position, tolerance=None):
        """Verificación mejorada de posición de articulación"""
        if tolerance is None:
            tolerance = self.POSITION_TOLERANCE
            
        current_pos = p.getJointState(self.robot_id, joint_id)[0]
        current_vel = p.getJointState(self.robot_id, joint_id)[1]
        
        position_ok = abs(current_pos - target_position) < tolerance
        velocity_ok = abs(current_vel) < self.VELOCITY_TOLERANCE
        
        return position_ok and velocity_ok

    def move_to_position_with_control(self, joint_id, target_position, max_time=2.0):
        """Movimiento controlado a posición con retroalimentación"""
        start_time = time.time()
        last_error = float('inf')
        
        while time.time() - start_time < max_time:
            current_pos = p.getJointState(self.robot_id, joint_id)[0]
            error = target_position - current_pos
            
            # Si el error no está mejorando, ajustar parámetros
            if abs(error) >= abs(last_error):
                self.joint_info[list(self.joint_info.keys())[joint_id]]['damping'] *= 0.95
                
            # Control de posición con ganancias adaptativas
            p.setJointMotorControl2(
                self.robot_id,
                joint_id,
                p.POSITION_CONTROL,
                targetPosition=target_position,
                force=self.MAX_FORCE,
                positionGain=0.5,
                velocityGain=1.0
            )
            
            p.stepSimulation()
            time.sleep(self.TIME_STEP)
            
            if self.verify_joint_position(joint_id, target_position):
                return True
                
            last_error = error
            
        return False

    def test_joint_limits(self):
        """Prueba mejorada de límites de articulaciones"""
        print("\nProbando límites de articulaciones...")
        
        for joint_name, info in self.joint_info.items():
            print(f"\nProbando articulación: {joint_name}")
            joint_id = info['id']
            
            # Prueba límite inferior
            print(f"Moviendo a límite inferior: {info['lowerLimit']}")
            success = self.move_to_position_with_control(joint_id, info['lowerLimit'])
            if not success:
                print(f"¡Advertencia! No se alcanzó el límite inferior para {joint_name}")
                self.diagnose_joint_issues(joint_name)
            
            # Prueba límite superior
            print(f"Moviendo a límite superior: {info['upperLimit']}")
            success = self.move_to_position_with_control(joint_id, info['upperLimit'])
            if not success:
                print(f"¡Advertencia! No se alcanzó el límite superior para {joint_name}")
                self.diagnose_joint_issues(joint_name)
            
            # Volver a posición neutral
            print("Volviendo a posición neutral")
            self.move_to_position_with_control(joint_id, 0)

    def test_predefined_poses(self):
        """Prueba mejorada de poses predefinidas"""
        print("\nProbando poses predefinidas...")
        
        poses = {
            'home': [0, 0, 0, 0, 0],
            'ready': [0, -0.5, 1.0, -0.5, 0],
            'extended': [0, 0, 0, 1.57, 0],
            'folded': [0, -1.57, 1.57, -1.57, 0]
        }
        
        for pose_name, positions in poses.items():
            print(f"\nMoviendo a pose: {pose_name}")
            success = True
            
            # Mover cada articulación con control
            for i, target_pos in enumerate(positions):
                if not self.move_to_position_with_control(i, target_pos):
                    success = False
                    print(f"¡Advertencia! Articulación {i} no alcanzó la posición objetivo en pose {pose_name}")
                    self.diagnose_joint_issues(list(self.joint_info.keys())[i])
            
            if success:
                print(f"Pose {pose_name} alcanzada exitosamente")

    def test_smooth_movement(self):
        """Prueba mejorada de movimientos suaves"""
        print("\nProbando movimientos suaves...")
        
        # Trayectoria circular mejorada
        steps = 360
        radius = 0.3
        height = 0.5
        
        start_time = time.time()
        positions_log = []
        velocities_log = []
        
        for i in range(steps):
            angle = np.radians(i)
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            z = height
            
            # Cinemática inversa mejorada
            target_pos = p.calculateInverseKinematics(
                self.robot_id,
                4,  # end effector link
                [x, y, z],
                maxNumIterations=100,
                residualThreshold=0.01
            )
            
            # Aplicar movimiento con control suave
            for joint_id, pos in enumerate(target_pos):
                p.setJointMotorControl2(
                    self.robot_id,
                    joint_id,
                    p.POSITION_CONTROL,
                    targetPosition=pos,
                    force=self.MAX_FORCE,
                    positionGain=0.5,
                    velocityGain=1.0
                )
            
            p.stepSimulation()
            
            # Registrar datos para análisis
            current_pos = [p.getJointState(self.robot_id, i)[0] for i in range(len(target_pos))]
            current_vel = [p.getJointState(self.robot_id, i)[1] for i in range(len(target_pos))]
            positions_log.append(current_pos)
            velocities_log.append(current_vel)
            
            time.sleep(self.TIME_STEP)
        
        # Analizar suavidad del movimiento
        self.analyze_movement_smoothness(positions_log, velocities_log)

    def test_complex_trajectory(self):
        """Prueba mejorada de trayectoria compleja"""
        print("\nProbando trayectoria compleja...")
        
        trajectory = [
            {'joints': [0.5, 0.5, 0.5, 0.5, 0.5], 'time': 2.0},
            {'joints': [-0.5, 0.2, -0.3, 0.1, 0.4], 'time': 2.0},
            {'joints': [0.0, 0.0, 0.0, 0.0, 0.0], 'time': 2.0}
        ]
        
        for point in trajectory:
            success = True
            for i, target_pos in enumerate(point['joints']):
                if not self.move_to_position_with_control(i, target_pos, max_time=point['time']):
                    success = False
                    print(f"¡Advertencia! Articulación {i} no alcanzó la posición objetivo")
                    self.diagnose_joint_issues(list(self.joint_info.keys())[i])
            
            if success:
                print("Punto de trayectoria alcanzado exitosamente")

    def test_emergency_stop(self):
        """Prueba mejorada de parada de emergencia"""
        print("\nProbando parada de emergencia...")
        
        # Iniciar movimiento
        print("Iniciando movimiento...")
        target_positions = [1.0, 1.0, 1.0, 1.0, 1.0]
        for i, pos in enumerate(target_positions):
            p.setJointMotorControl2(
                self.robot_id,
                i,
                p.POSITION_CONTROL,
                targetPosition=pos,
                force=self.MAX_FORCE
            )
        
        # Registrar posiciones iniciales
        initial_positions = [p.getJointState(self.robot_id, i)[0] for i in range(len(target_positions))]
        
        # Simular parada de emergencia
        time.sleep(0.5)
        print("¡Parada de emergencia activada!")
        
        # Detener todos los motores
        for i in range(len(target_positions)):
            p.setJointMotorControl2(
                self.robot_id,
                i,
                p.VELOCITY_CONTROL,
                targetVelocity=0,
                force=self.MAX_FORCE
            )
        
        # Verificar la parada
        time.sleep(0.5)
        final_positions = [p.getJointState(self.robot_id, i)[0] for i in range(len(target_positions))]
        final_velocities = [p.getJointState(self.robot_id, i)[1] for i in range(len(target_positions))]
        
        # Analizar resultados
        for i in range(len(target_positions)):
            if abs(final_velocities[i]) > self.VELOCITY_TOLERANCE:
                print(f"¡Advertencia! La articulación {i} no se detuvo completamente")
                print(f"Velocidad final: {final_velocities[i]}")
            movement = abs(final_positions[i] - initial_positions[i])
            print(f"Articulación {i} - Movimiento durante parada: {movement:.4f} rad")

    def test_response_times(self):
        """Prueba mejorada de tiempos de respuesta"""
        print("\nProbando tiempos de respuesta...")
        
        response_times = []
        for i in range(5):
            print(f"\nPrueba {i+1}:")
            
            # Seleccionar articulación aleatoria para prueba
            joint_id = np.random.randint(0, len(self.joint_info))
            target_pos = 0.5  # movimiento moderado
            
            # Medir tiempo de respuesta
            start_time = time.time()
            success = self.move_to_position_with_control(joint_id, target_pos, max_time=2.0)
            
            if success:
                response_time = time.time() - start_time
                response_times.append(response_time)
                print(f"Tiempo de respuesta: {response_time:.3f} segundos")
            else:
                print("Timeout - No se alcanzó la posición objetivo")
                self.diagnose_joint_issues(list(self.joint_info.keys())[joint_id])
        
        if response_times:
            avg_response_time = np.mean(response_times)
            std_response_time = np.std(response_times)
            
    def test_response_times(self):
        """Prueba mejorada de tiempos de respuesta"""
        print("\nProbando tiempos de respuesta...")
        
        response_times = []
        for i in range(5):
            print(f"\nPrueba {i+1}:")
            
            # Seleccionar articulación aleatoria para prueba
            joint_id = np.random.randint(0, len(self.joint_info))
            target_pos = 0.5  # movimiento moderado
            
            # Medir tiempo de respuesta
            start_time = time.time()
            success = self.move_to_position_with_control(joint_id, target_pos, max_time=2.0)
            
            if success:
                response_time = time.time() - start_time
                response_times.append(response_time)
                print(f"Tiempo de respuesta: {response_time:.3f} segundos")
            else:
                print("Timeout - No se alcanzó la posición objetivo")
                self.diagnose_joint_issues(list(self.joint_info.keys())[joint_id])
        
        if response_times:
            avg_response_time = np.mean(response_times)
            std_response_time = np.std(response_times)
            print(f"\nResumen de tiempos de respuesta:")
            print(f"Promedio: {avg_response_time:.3f} segundos")
            print(f"Desviación estándar: {std_response_time:.3f} segundos")
        else:
            print("\n¡Advertencia! No se pudieron medir tiempos de respuesta")

    def diagnose_joint_issues(self, joint_name):
        """Diagnóstico detallado de problemas en las articulaciones"""
        joint_id = self.joint_info[joint_name]['id']
        state = p.getJointState(self.robot_id, joint_id)
        dynamics_info = p.getDynamicsInfo(self.robot_id, joint_id)
        
        print(f"\nDiagnóstico de {joint_name}:")
        print(f"Estado actual:")
        print(f"  - Posición: {state[0]:.3f} rad")
        print(f"  - Velocidad: {state[1]:.3f} rad/s")
        print(f"  - Fuerza/Torque aplicado: {state[3]:.3f} N/Nm")
        
        print(f"Parámetros dinámicos:")
        print(f"  - Masa del eslabón: {dynamics_info[0]:.3f} kg")
        print(f"  - Fricción lateral: {dynamics_info[1]:.3f}")
        print(f"  - Inercia local: {[f'{x:.3f}' for x in dynamics_info[2]]}")
        print(f"  - Amortiguación: {self.joint_info[joint_name]['damping']:.3f}")
        
        # Verificar límites
        if state[0] <= self.joint_info[joint_name]['lowerLimit']:
            print("  ¡Advertencia! Articulación en límite inferior")
        elif state[0] >= self.joint_info[joint_name]['upperLimit']:
            print("  ¡Advertencia! Articulación en límite superior")
            
        # Verificar velocidad
        if abs(state[1]) >= self.joint_info[joint_name]['maxVelocity']:
            print("  ¡Advertencia! Velocidad cerca del límite máximo")

    def analyze_movement_smoothness(self, positions_log, velocities_log):
        """Analiza la suavidad del movimiento"""
        positions_array = np.array(positions_log)
        velocities_array = np.array(velocities_log)
        
        # Calcular jerk (derivada de la aceleración)
        velocity_diff = np.diff(velocities_array, axis=0)
        acceleration_diff = np.diff(velocity_diff, axis=0)
        jerk = np.diff(acceleration_diff, axis=0)
        
        print("\nAnálisis de suavidad del movimiento:")
        for joint_id in range(positions_array.shape[1]):
            joint_name = list(self.joint_info.keys())[joint_id]
            print(f"\nArticulación {joint_name}:")
            
            # Estadísticas de posición
            pos_range = np.ptp(positions_array[:, joint_id])
            print(f"  Rango de movimiento: {pos_range:.3f} rad")
            
            # Estadísticas de velocidad
            max_vel = np.max(np.abs(velocities_array[:, joint_id]))
            print(f"  Velocidad máxima: {max_vel:.3f} rad/s")
            
            # Análisis de jerk
            if len(jerk) > 0:
                mean_jerk = np.mean(np.abs(jerk[:, joint_id]))
                print(f"  Jerk promedio: {mean_jerk:.3f} rad/s³")
                if mean_jerk > 1.0:
                    print("  ¡Advertencia! Movimiento no suave detectado")

    def run_all_tests(self):
        """Ejecuta todas las pruebas con manejo de errores"""
        try:
            print("Iniciando pruebas del Modern Industrial Robot...")
            
            # Ejecutar pruebas en secuencia
            self.test_joint_limits()
            self.test_predefined_poses()
            self.test_smooth_movement()
            self.test_complex_trajectory()
            self.test_emergency_stop()
            self.test_response_times()
            
            print("\n¡Todas las pruebas completadas!")
            
        except Exception as e:
            print(f"\n¡Error durante las pruebas!: {str(e)}")
            import traceback
            print(traceback.format_exc())
        finally:
            # Limpiar y cerrar la simulación
            if hasattr(self, 'physics_client'):
                p.disconnect(self.physics_client)

if __name__ == "__main__":
    test = TestModernArm()
    test.run_all_tests()

