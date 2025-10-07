import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from keybrd import is_pressed, rising_edge

# --- Functions you can use (don't change) --- #
def get_sensor_values(threshold=0.2):
    """
    Read the 8 vision sensors and return a list of 0/1 values
    (1 = line detected, 0 = no line)
    Example: [0, 0, 1, 1, 0, 0, 0, 0]
    """

    states = []
    for h in cam_handles:
        img, res = sim.getVisionSensorImg(h)
        w, hres = int(res[0]), int(res[1])

        # Convert the buffer into a numpy array
        buf = np.frombuffer(img, dtype=np.uint8)

        # Reconstruct the image: RGB or grayscale
        frame = buf.reshape(hres, w, 3).astype(np.float32) / 255.0
        gray = frame.mean(axis=2)

        # Binary detection: 1 if black (line), 0 if white (background)
        detected = 1 if gray.mean() < threshold else 0
        states.append(detected)  # Store as integer, not string
    
    return states

def set_motor_speeds(norm_left, norm_right):
    """
    Set both wheel speeds using normalized values (-1 to 1)
    -1 = full speed backward, 0 = stop, 1 = full speed forward
    
    Args:
        norm_left: normalized speed for left wheel (-1 to 1)
        norm_right: normalized speed for right wheel (-1 to 1)
    """
    # Clamp to [-1, 1] range
    norm_left = max(-1.0, min(1.0, norm_left))
    norm_right = max(-1.0, min(1.0, norm_right))
    
    # Convert to linear velocities in m/s
    left_linear_velocity = norm_left * MAX_SPEED
    right_linear_velocity = norm_right * MAX_SPEED
    
    # Convert to angular velocities in rad/s
    left_angular_velocity = left_linear_velocity / (WHEEL_D / 2)
    right_angular_velocity = right_linear_velocity / (WHEEL_D / 2)
    
    # Set the motor velocities
    client.setStepping(True)
    sim.setJointTargetVelocity(left_mtr, left_angular_velocity)
    sim.setJointTargetVelocity(right_mtr, right_angular_velocity)
    client.setStepping(False)

# --- Initialization code (don't change) --- #
WHEEL_D = 0.05  # Wheel diameter in meters
MAX_SPEED = 1.2 # m/s
client = RemoteAPIClient('localhost', 23000)
sim = client.getObject('sim')
left_mtr = sim.getObject('/LineTracer/DynamicLeftJoint')
right_mtr = sim.getObject('/LineTracer/DynamicRightJoint')
cam_handles = [sim.getObject(f"/LineTracer/lineSensor[{i}]") for i in range(8)]

# --- Your code starts here --- #
# Example usage

if __name__ == "__main__":

    positions = [-3, -2, -1, -0.5, 0.5, 1, 2, 3]

    # PID gains
    Kp = 0.004 
    Ki = 0.0003 
    Kd = 0.005

    #https://towardinfinity.medium.com/pid-for-line-follower-11deb3a1a643 

    #Valores estimados
    # Kp = 0.001
    # Ki = 0
    # Kd = 0.05

    # Variables PID
    integral = 0
    last_error = 0

    # Velocidad base
    base_speed_max = 0.35
    base_speed_min = 0.15

    # Dirección del último lado de la línea (1 = derecha, -1 = izquierda)
    last_line_side = 1

    try:
        sim.startSimulation()

        while sim.getSimulationState() != sim.simulation_stopped:


            
                 
            sensors = get_sensor_values()
            sensors = sensors[::-1]  # solo si los sensores están invertidos
            num_active = sum(sensors) #NUMERO DE SENSORES ACTIVOS

          
            # ======================
            # Detección de línea y PID
            # ======================

            if num_active > 0: #SI HAY LINEA
                line_pos = sum(p * s for p, s in zip(positions, sensors)) / num_active
                error = line_pos
                # Guardar el lado de la línea
                if line_pos > 0:
                    last_line_side = 1
                elif line_pos < 0:
                    last_line_side = -1
                # Zona muerta
                # if abs(error) < 0.05:
                #     error = 0
                # Filtro suavizado
                # error = 0.6 * error + 0.4 * last_error
                integral += error
                derivative = error - last_error
                # derivative = max(-1.0, min(1.0, derivative))
                correction = Kp * error + Ki * integral + Kd * derivative
                correction = max(-0.4, min(0.4, correction))
                # Velocidad adaptativa

                base_speed = max(base_speed_min, base_speed_max - 0.1 * abs(error))
                left_motor = base_speed + correction
                right_motor = base_speed - correction
                lost_counter = 0  # resetear contador porque línea está visible
            else: #SI NO HAY LINEA
                # Línea perdida -> gira suavemente hacia el último lado de la línea
                error = 0
                integral += error
                derivative = 0
                correction = 0
                search_speed = 0.25
                left_motor = search_speed * last_line_side
                right_motor = -search_speed * last_line_side
                lost_counter += 1
                if lost_counter % 10 == 0:  # mostrar mensaje solo cada 10 ciclos
                    print(f"⚠ Línea perdida! Buscando activamente... (lost_counter={lost_counter})")

            # Limitar motores
            left_motor = max(-1, min(1, left_motor))
            right_motor = max(-1, min(1, right_motor))

            set_motor_speeds(left_motor, right_motor)

            # Depuración
            if num_active > 0:
                print(f"Sensors detected: {sensors} | line_pos={line_pos:.2f}")
                print(f"error={error:.2f} | PID -> P={Kp*error:.3f}, I={Ki*integral:.3f}, D={Kd*derivative:.3f}, correction={correction:.3f}")
                print(f"Motors -> Left={left_motor:.2f}, Right={right_motor:.2f}\n")

            last_error = error

    finally:
        sim.stopSimulation()