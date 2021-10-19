import random
import time

import carla

actor_list = []
try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)

    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    v_bp = blueprint_library.filter("model3")[0]
    spawn_point = random.choice(world.get_map().get_spawn_points())
    vehicle = world.spawn_actor(v_bp, spawn_point)
    actor_list.append(vehicle)
    vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0))
    time.sleep(5)
finally:
    for actor in actor_list:
        actor.destroy()
    print("结束")