
import pygame
import math
import random

pygame.init()
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Drone Stabilization - Stable X Recovery")
clock = pygame.time.Clock()
font = pygame.font.SysFont("consolas", 18)

PIXELS_PER_METER = 100
gravity = 9.81
mass = 10.0
dt = 1 / 60

x, z, y = 4.0, 3.0, 2.0
vx, vz, vy = 0.0, 0.0, 0.0
angle = -math.pi / 2
angular_velocity = 0.0

z_target = 3.0
x_target = 4.0
angle_target = -math.pi / 2

z_integral = 0.0
z_prev_error = 0.0

x_integral = 0.0
x_prev_error = 0.0

angle_integral = 0.0
angle_prev_error = 0.0

z_kp, z_ki, z_kd = 60.0, 0.0, 40.0
x_kp, x_ki, x_kd = 30.0, 3.0, 20.0
a_kp, a_ki, a_kd = 150.0, 0.0, 80.0

fx_disturbance = 0.0
fz_disturbance = 0.0
torque_disturbance = 0.0

def draw_drone(surface, x, z, y, angle):
    base_size = 0.4 * PIXELS_PER_METER
    depth_scale = max(0.5, min(1.5, 3.0 / (y + 0.1)))
    size = base_size * depth_scale
    px, pz = x * PIXELS_PER_METER, z * PIXELS_PER_METER
    points = [
        (math.cos(angle) * size, math.sin(angle) * size),
        (math.cos(angle + 2.5) * size, math.sin(angle + 2.5) * size),
        (math.cos(angle - 2.5) * size, math.sin(angle - 2.5) * size),
    ]
    translated = [(px + dx, HEIGHT - (pz + dz)) for dx, dz in points]
    pygame.draw.polygon(surface, (0, 200, 255), translated)

def draw_thrust_vector(surface, x, z, y, angle, thrust_N):
    px, pz = x * PIXELS_PER_METER, z * PIXELS_PER_METER
    scale = max(0.5, min(1.5, 3.0 / (y + 0.1)))
    length = thrust_N * 0.1 * scale
    end_x = px - math.cos(angle) * length
    end_z = pz - math.sin(angle) * length
    pygame.draw.line(surface, (255, 100, 100), (px, HEIGHT - pz), (end_x, HEIGHT - end_z), 3)
    pygame.draw.circle(surface, (255, 100, 100), (int(end_x), int(HEIGHT - end_z)), 5)

def draw_telemetry(surface, angle, thrust, vx, vz, vy):
    angle_deg = math.degrees(angle + math.pi / 2)
    lines = [
        f"Angle: {angle_deg:+.2f}°",
        f"Thrust: {thrust:.2f} N",
        f"Vel X: {vx:.2f} m/s",
        f"Vel Z: {vz:.2f} m/s",
        f"Vel Y: {vy:.2f} m/s"
    ]
    for i, line in enumerate(lines):
        text = font.render(line, True, (255, 255, 255))
        surface.blit(text, (10, 10 + i * 20))

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_r:
            x, z, y = 4.0, 3.0, 2.0
            vx, vz, vy = 0.0, 0.0, 0.0
            angle = -math.pi / 2
            angle_target = angle
            angular_velocity = 0.0
            fx_disturbance = fz_disturbance = torque_disturbance = 0.0

    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT]:
        torque_disturbance += math.radians(200)
    if keys[pygame.K_RIGHT]:
        torque_disturbance -= math.radians(200)
    if keys[pygame.K_UP]:
        fz_disturbance += 50.0
    if keys[pygame.K_DOWN]:
        fz_disturbance -= 50.0
    if keys[pygame.K_a]:
        fx_disturbance -= 50.0
    if keys[pygame.K_d]:
        fx_disturbance += 50.0
    if keys[pygame.K_w]:
        vy -= 0.2
    if keys[pygame.K_s]:
        vy += 0.2

    # X PID to angle_target
    x_error = x_target - x
    x_integral += x_error * dt
    x_derivative = -vx
    if x_error * x_prev_error < 0:
        x_integral = 0  # anti-windup
    x_integral = max(min(x_integral, 1.0), -1.0)
    x_prev_error = x_error
    angle_offset = x_kp * x_error + x_kd * x_derivative + x_ki * x_integral
    max_angle_offset = math.radians(15)
    angle_target = -math.pi / 2 + max(min(angle_offset, max_angle_offset), -max_angle_offset)

    # Angle PID
    angle_error = angle_target - angle
    angle_integral += angle_error * dt
    angle_derivative = (angle_error - angle_prev_error) / dt
    angle_prev_error = angle_error
    angular_accel = a_kp * angle_error + a_kd * angle_derivative + a_ki * angle_integral
    angular_velocity += (angular_accel + torque_disturbance) * dt
    torque_disturbance *= 0.9
    angle += angular_velocity * dt

    # Z PID
    z_error = z_target - z
    z_integral += z_error * dt
    z_derivative = (z_error - z_prev_error) / dt
    z_prev_error = z_error
    z_pid = z_kp * z_error + z_kd * z_derivative + z_ki * z_integral
    thrust = mass * gravity + z_pid
    thrust = max(0, min(thrust, 2 * mass * gravity))

    fx = -math.cos(angle) * thrust + fx_disturbance
    fz = -math.sin(angle) * thrust + fz_disturbance - mass * gravity
    fx_disturbance *= 0.9
    fz_disturbance *= 0.9

    ax = fx / mass
    az = fz / mass
    vx += ax * dt
    vz += az * dt
    x += vx * dt
    z += vz * dt
    y += vy * dt
    y = max(0.5, min(5.0, y))

    vx *= 0.98
    vz *= 0.98
    vy *= 0.95
    if z < 0.1:
        z = 0.1
        vz = 0

    screen.fill((30, 30, 30))
    draw_drone(screen, x, z, y, angle)
    draw_thrust_vector(screen, x, z, y, angle, thrust)
    draw_telemetry(screen, angle, thrust, vx, vz, vy)
    ground_y = HEIGHT - int(0.1 * PIXELS_PER_METER)
    pygame.draw.line(screen, (100, 255, 100), (0, ground_y), (WIDTH, ground_y), 2)
    pygame.display.flip()
    clock.tick(60)

pygame.quit()
