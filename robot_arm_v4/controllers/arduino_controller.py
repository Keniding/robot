import serial
import time

class ArduinoController:
    def __init__(self, port='COM3', baudrate=9600, simulate=True):
        self.simulate = simulate
        if not simulate:
            try:
                self.arduino = serial.Serial(port, baudrate)
                time.sleep(2)
                print(f"Arduino conectado en {port}")
            except:
                print("No se pudo conectar al Arduino - Modo simulación activado")
                self.simulate = True
    
    def move_joint(self, joint_id, angle):
        if self.simulate:
            print(f"Simulando: Mover articulación {joint_id} a {angle} grados")
        else:
            command = f"M{joint_id},{angle}\n"
            self.arduino.write(command.encode())
    
    def close(self):
        if not self.simulate:
            self.arduino.close()
