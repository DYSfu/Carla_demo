import random
import time

import carla
import cv2
import numpy as np

actor_list = []


def img_process(data):
    img = np.array(data.raw_data)
    img = img.reshape((1080, 1920, 4))
    img = img[:, :, :3]
    cv2.imwrite('car.png', img)
    # cv2.imshow('', img)
    # cv2.waitKey(1)
    pass

def callback(event):
    print("碰撞")

def callback2(event):
    print("穿越车道")


try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)

    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    v_bp = blueprint_library.filter("model3")[0]
    spawn_point = random.choice(world.get_map().get_spawn_points())
    vehicle = world.spawn_actor(v_bp, spawn_point)
    actor_list.append(vehicle)

    # Find the blueprint of the sensor.
    blueprint = blueprint_library.find('sensor.camera.rgb')
    # Modify the attributes of the blueprint to set image resolution and field of view.
    blueprint.set_attribute('image_size_x', '1920')
    blueprint.set_attribute('image_size_y', '1080')
    blueprint.set_attribute('fov', '110')
    # Set the time in seconds between sensor captures
    blueprint.set_attribute('sensor_tick', '1.0')
    transform = carla.Transform(carla.Location(x=0.8, z=1.7))
    sensor = world.spawn_actor(blueprint, transform, attach_to=vehicle)
    actor_list.append(sensor)
    sensor.listen(lambda data: img_process(data))

    blueprint_collision = blueprint_library.find('sensor.other.collision')
    transform = carla.Transform(carla.Location(x=0.8, z=1.7))
    sensor_collision = world.spawn_actor(blueprint_collision, transform, attach_to=vehicle)
    actor_list.append(sensor_collision)
    sensor_collision.listen(callback)

    blueprint_lane_invasion = blueprint_library.find('sensor.other.lane_invasion')
    transform = carla.Transform(carla.Location(x=0.8, z=1.7))
    sensor_lane_invasion = world.spawn_actor(blueprint_lane_invasion, transform, attach_to=vehicle)
    actor_list.append(sensor_lane_invasion)
    sensor_lane_invasion.listen(callback2)

    vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.3))
    time.sleep(10)
finally:
    for actor in actor_list:
        actor.destroy()
    print("结束")