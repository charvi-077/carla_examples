#!/usr/bin/env python

import glob
import os
import sys
import cv2

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse
from queue import Queue
from queue import Empty
from matplotlib import cm
import pygame
from pygame.locals import *
import random


try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

# To import a basic agent
from agents.navigation.basic_agent import BasicAgent


try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

try:
    from PIL import Image
except ImportError:
    raise RuntimeError('cannot import PIL, make sure "Pillow" package is installed')

VIRIDIS = np.array(cm.get_cmap('viridis').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
frame = 0

def sensor_callback(data, queue):
    """
    This simple callback just stores the data on a thread safe Python Queue
    to be retrieved from the "main thread".
    """
    queue.put(data)


def sync_auto_control(args):
    """
    This function is intended to be a Sync two sensor data i.e. Camera RGB and Semantic Seg
    with Automatic Basic Agent which goes from single point to destination.
    """
    # Connect to the server
    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)
    
    #client.load_world('Town01')
    world = client.get_world()

    bp_lib = world.get_blueprint_library()

    traffic_manager = client.get_trafficmanager(8000)
    traffic_manager.set_synchronous_mode(True)

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 3.0
    world.apply_settings(settings)
    m = world.get_map()

    vehicle = None
    camera = None
    camera_sem = None
    lidar = None
    resize_of_camera_view = 380

    # Pygame initialization
    pygame.init()
    image_w, image_h = 800, 400  # Adjust dimensions as needed
    screen = pygame.display.set_mode((image_w, image_h))
    pygame.display.set_caption('Lidar Points Visualization')

    try:
        # Search the desired blueprints
        vehicle_bp = bp_lib.filter("vehicle.lincoln.mkz2017")[0]
        camera_bp = bp_lib.filter("sensor.camera.rgb")[0]
        camera_sem_bp = bp_lib.filter("sensor.camera.semantic_segmentation")[0]
        lidar_bp = bp_lib.filter("sensor.lidar.ray_cast")[0]
        

        # Configure the blueprints
        camera_bp.set_attribute("image_size_x", str(args.width))
        camera_bp.set_attribute("image_size_y", str(args.height))

        camera_sem_bp.set_attribute("image_size_x", str(args.width))
        camera_sem_bp.set_attribute("image_size_y", str(args.height))

        if args.no_noise:
            lidar_bp.set_attribute('dropoff_general_rate', '0.0')
            lidar_bp.set_attribute('dropoff_intensity_limit', '1.0')
            lidar_bp.set_attribute('dropoff_zero_intensity', '0.0')
        lidar_bp.set_attribute('upper_fov', str(args.upper_fov))
        lidar_bp.set_attribute('lower_fov', str(args.lower_fov))
        lidar_bp.set_attribute('channels', str(args.channels))
        lidar_bp.set_attribute('range', str(args.range))
        lidar_bp.set_attribute('points_per_second', str(args.points_per_second))

        # Spawn the blueprints

        vehicle = world.spawn_actor(
            blueprint=vehicle_bp,
            transform=m.get_spawn_points()[0])

        # To start a basic agent
        agent = BasicAgent(vehicle)
        spawn_point = random.choice(m.get_spawn_points())
        agent.set_destination((spawn_point.location.x,
                                   spawn_point.location.y,
                                   spawn_point.location.z))

        # destination = spawn_points[0].location
        # agent.set_destination(destination)

        # vehicle.set_autopilot(True)

        camera = world.spawn_actor(
            blueprint=camera_bp,
            transform=carla.Transform(carla.Location(x=1.6, z=1.6)),
            attach_to=vehicle)
        lidar = world.spawn_actor(
            blueprint=lidar_bp,
            transform=carla.Transform(carla.Location(x=1.0, z=1.8)),
            attach_to=vehicle)

        camera_sem = world.spawn_actor(
            blueprint=camera_sem_bp,
            transform=carla.Transform(carla.Location(x=1.0, z=1.8)),
            attach_to=vehicle)

        image_w = camera_bp.get_attribute("image_size_x").as_int()
        image_h = camera_bp.get_attribute("image_size_y").as_int()
        fov = camera_bp.get_attribute("fov").as_float()
        focal = image_w / (2.0 * np.tan(fov * np.pi / 360.0))


        K = np.identity(3)
        K[0, 0] = K[1, 1] = focal
        K[0, 2] = image_w / 2.0
        K[1, 2] = image_h / 2.0

        # The sensor data will be saved in thread-safe Queues
        image_queue = Queue()
        image_sem_queue = Queue()
        lidar_queue = Queue()

        camera.listen(lambda data: sensor_callback(data, image_queue))
        camera_sem.listen(lambda data: sensor_callback(data, image_sem_queue))
        lidar.listen(lambda data: sensor_callback(data, lidar_queue))

        while True:
            global frame
            frame += 1
            if agent.done():
                print("The target has been reached, stopping the simulation")
                break

            vehicle.apply_control(agent.run_step())

        # for frame in range(args.frames):
            world.tick()
            world_frame = world.get_snapshot().frame

            try:
                # Get the data once it's received.
                image_data = image_queue.get(True, 1.0)
                image_sem_data = image_sem_queue.get(True, 1.0)
                image_sem_data.convert(carla.ColorConverter.CityScapesPalette)

                lidar_data = lidar_queue.get(True, 1.0)
            except Empty:
                print("[Warning] Some sensor data has been missed")
                continue

            assert image_data.frame == lidar_data.frame == world_frame
            # At this point, we have the synchronized information from the 2 sensors.
            sys.stdout.write("\r(%d/%d) Simulation: %d Camera: %d " %
                (frame, args.frames, world_frame, image_data.frame) + ' ')
            sys.stdout.flush()

            # Get the raw BGRA buffer and convert it to an array of RGB of
            # shape (image_data.height, image_data.width, 3).
            im_array = np.copy(np.frombuffer(image_data.raw_data, dtype=np.dtype("uint8")))
            im_array = np.reshape(im_array, (image_data.height, image_data.width, 4))
            im_array = im_array[:, :, :3][:, :, ::-1]
            rgb_image_np = im_array.copy()

            im_sem_array = np.copy(np.frombuffer(image_sem_data.raw_data, dtype=np.dtype("uint8")))
            im_sem_array = np.reshape(im_sem_array, (image_sem_data.height, image_sem_data.width, 4))
            im_sem_array = im_sem_array[:, :, :3][:, :, ::-1]
            sem_image_np = im_sem_array.copy()

            # Save the image using Pillow module.
            
            rg1 = cv2.resize(rgb_image_np, (resize_of_camera_view, resize_of_camera_view), interpolation = cv2.INTER_LINEAR)

            seg1 = cv2.resize(sem_image_np, (resize_of_camera_view, resize_of_camera_view), interpolation = cv2.INTER_LINEAR)

            total_array = np.concatenate((rg1, seg1 ), axis=1)
            image = Image.fromarray(total_array)
            image_surface = pygame.image.fromstring(image.tobytes(), image.size, image.mode)

            # Display the image on the screen
            screen.blit(image_surface, (0, 0))
            pygame.display.flip()

            
            # pygame.time.delay(10)  # Adjust delay as needed
            #image.save("_out/%08d.png" % image_data.frame)

    finally:
        # Apply the original settings when exiting.
        world.apply_settings(original_settings)

        # Destroy the actors in the scene.
        if camera:
            camera.destroy()
        if lidar:
            lidar.destroy()
        if vehicle:
            vehicle.destroy()


def main():
    """Start function"""
    argparser = argparse.ArgumentParser(
        description='CARLA Sensor sync and projection tutorial')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='680x420',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '-f', '--frames',
        metavar='N',
        default=500,
        type=int,
        help='number of frames to record (default: 500)')
    argparser.add_argument(
        '-d', '--dot-extent',
        metavar='SIZE',
        default=2,
        type=int,
        help='visualization dot extent in pixels (Recomended [1-4]) (default: 2)')
    argparser.add_argument(
        '--no-noise',
        action='store_true',
        help='remove the drop off and noise from the normal (non-semantic) lidar')
    argparser.add_argument(
        '--upper-fov',
        metavar='F',
        default=30.0,
        type=float,
        help='lidar\'s upper field of view in degrees (default: 15.0)')
    argparser.add_argument(
        '--lower-fov',
        metavar='F',
        default=-25.0,
        type=float,
        help='lidar\'s lower field of view in degrees (default: -25.0)')
    argparser.add_argument(
        '-c', '--channels',
        metavar='C',
        default=64.0,
        type=float,
        help='lidar\'s channel count (default: 64)')
    argparser.add_argument(
        '-r', '--range',
        metavar='R',
        default=100.0,
        type=float,
        help='lidar\'s maximum range in meters (default: 100.0)')
    argparser.add_argument(
        '--points-per-second',
        metavar='N',
        default='100000',
        type=int,
        help='lidar points per second (default: 100000)')
    args = argparser.parse_args()
    args.width, args.height = [int(x) for x in args.res.split('x')]
    args.dot_extent -= 1

    try:
        sync_auto_control(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
