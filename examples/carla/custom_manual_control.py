#!/usr/bin/env python3

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down
    CTRL + W     : toggle constant velocity mode at 60 km/h

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change sensor position
    C            : change weather (Shift+C reverse)
    Backspace    : change vehicle

    V            : Select next map layer (Shift+V reverse)
    B            : Load current selected map layer (Shift+B to unload)

    R            : toggle recording images to disk

    T             : restart fake sensors

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import os
import sys

try:
    # Assuming .egg at the same path
    this_script_path = os.path.dirname(os.path.realpath(__file__))
    egg_file_path =  'carla-0.9.11-py3.7-linux-x86_64.egg'

    sys.path.append(os.path.join(this_script_path, egg_file_path))
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

from functools import partial
from threading import Thread

import rclpy
from rclpy.node import Node

from object_model_msgs.msg import ObjectModel, Object, Track, Dimensions
from fusion_layer.srv import RegisterSensor, RemoveSensor

import carla
from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref
from copy import deepcopy

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_b
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_g
    from pygame.locals import K_h
    from pygame.locals import K_i
    from pygame.locals import K_l
    from pygame.locals import K_m
    from pygame.locals import K_n
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_t
    from pygame.locals import K_v
    from pygame.locals import K_w
    from pygame.locals import K_x
    from pygame.locals import K_z
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


# Simple time reference from since when this script is running
STARTED_TIME = None

SURROUND_SENSOR_RANGE = 50


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


def wrap_angle(theta):
    return (theta + np.pi) % (2*np.pi) - np.pi


def get_relative_obj(reference_obj: Object, obj: Object) -> Object:
    angle = reference_obj.track.state[Track.STATE_YAW_IDX]
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)

    rotation_reference = np.array([
        [cos_angle, -sin_angle, 0, 0, 0, 0, 0, 0],
        [sin_angle,  cos_angle, 0, 0, 0, 0, 0, 0],
        [0, 0, cos_angle, -sin_angle, 0, 0, 0, 0],
        [0, 0, sin_angle,  cos_angle, 0, 0, 0, 0],
        [0, 0, 0, 0, cos_angle, -sin_angle, 0, 0],
        [0, 0, 0, 0, sin_angle,  cos_angle, 0, 0],
        [0, 0, 0, 0,          0,         0, 1, 0],
        [0, 0, 0, 0,          0,         0, 0, 1],
    ], dtype='float64')

    rotated_reference = rotation_reference @ reference_obj.track.state

    transformation = np.array([
        [cos_angle, -sin_angle, 0, 0, 0, 0, 0, 0, -rotated_reference[Track.STATE_X_IDX]],
        [-sin_angle, -cos_angle, 0, 0, 0, 0, 0, 0, rotated_reference[Track.STATE_Y_IDX]],
        [0, 0, cos_angle, -sin_angle, 0, 0, 0, 0, -rotated_reference[Track.STATE_VELOCITY_X_IDX]],
        [0, 0, -sin_angle, -cos_angle, 0, 0, 0, 0, rotated_reference[Track.STATE_VELOCITY_Y_IDX]],
        [0, 0, 0, 0, cos_angle, -sin_angle, 0, 0, -rotated_reference[Track.STATE_ACCELERATION_X_IDX]],
        [0, 0, 0, 0, -sin_angle, -cos_angle, 0, 0, rotated_reference[Track.STATE_ACCELERATION_Y_IDX]],
        [0, 0, 0, 0,          0,         0, 1, 0, -rotated_reference[Track.STATE_YAW_IDX]],
        [0, 0, 0, 0,          0,         0, 0, 1, -rotated_reference[Track.STATE_YAW_RATE_IDX]],
        [0, 0, 0, 0,          0,         0, 0, 0,   1],
    ], dtype='float64')

    to_align = np.hstack((obj.track.state, 1)).T
    product = transformation @ to_align
    obj_state_aligned = np.delete(product, -1, 0).astype('float32')

    result = deepcopy(obj)
    result.track.state = obj_state_aligned
    result.track.state[Track.STATE_YAW_IDX] = wrap_angle(result.track.state[Track.STATE_YAW_IDX])

    # print('========>', list(np.round(result.track.state, 5)))

    return result


def actor_to_object_model(actor: carla.Actor) -> Object:
    pos = actor.get_location()
    rot = actor.get_transform().rotation
    vel = actor.get_velocity()
    ang_vel = actor.get_angular_velocity()
    acc = actor.get_acceleration()
    bb = actor.bounding_box

    obj = Object()

    obj.track.state[Track.STATE_X_IDX] = pos.x
    obj.track.state[Track.STATE_Y_IDX] = pos.y
    obj.track.state[Track.STATE_VELOCITY_X_IDX] = vel.x
    obj.track.state[Track.STATE_VELOCITY_Y_IDX] = vel.y
    obj.track.state[Track.STATE_ACCELERATION_X_IDX] = acc.x
    obj.track.state[Track.STATE_ACCELERATION_Y_IDX] = acc.y
    obj.track.state[Track.STATE_YAW_IDX] = -math.radians(rot.yaw)
    obj.track.state[Track.STATE_YAW_RATE_IDX] = -math.radians(ang_vel.z)

    obj.dimensions.values[Dimensions.DIMENSIONS_WIDTH_IDX] = bb.extent.y*2
    obj.dimensions.values[Dimensions.DIMENSIONS_LENGHT_IDX] = bb.extent.x*2

    return obj


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World:
    def __init__(self, carla_world, hud, ros_node, args):
        self._csv = None
        self._extra_objs_csv = None  # Extra information about the detected objects

        self.open_csv()
        self.open_extra_objs_csv()

        self.world = carla_world
        self.actor_role_name = args.rolename
        self.ros_node = ros_node

        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)

        self.hud = hud
        self.player = None
        self.surround_sensor = None
        self.surround_sensor2 = None
        self.imu_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._gamma = args.gamma
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0
        self.constant_velocity_enabled = False
        self.current_map_layer = 0
        self.map_layer_names = [
            carla.MapLayer.NONE,
            carla.MapLayer.Buildings,
            carla.MapLayer.Decals,
            carla.MapLayer.Foliage,
            carla.MapLayer.Ground,
            carla.MapLayer.ParkedVehicles,
            carla.MapLayer.Particles,
            carla.MapLayer.Props,
            carla.MapLayer.StreetLights,
            carla.MapLayer.Walls,
            carla.MapLayer.All
        ]

    def open_csv(self, name="sensor_layer.csv"):
        self._csv = open(name, "w")

    def open_extra_objs_csv(self, name="sensor_layer_extra_objs.csv"):
        self._extra_objs_csv = open(name, "w")

    def close_csv(self):
        self._csv.close()

    def close_extra_objs_csv(self):
        self._extra_objs_csv.close()

    def add_info_csv(self, surround_sensor, ego_obj, msg):
        time_ = self.ros_node.get_clock().now().from_msg(msg.header.stamp)

        list_ = [time_.nanoseconds, surround_sensor.name, len(msg.object_model)]
        list_.extend(list(np.round(ego_obj.track.state, 5)))

        self._csv.write(','.join(map(str, list_)) + '\n')
        self._csv.flush()

    def add_info_extra_objs_csv(self):
        actors = (list(self.world.get_actors().filter('vehicle.*'))
                  + list(self.world.get_actors().filter('walker.pedestrian.*')))

        player_location = self.player.get_location()
        time_ = self.ros_node.get_clock().now()

        def get_distance(a):
            location = a.get_location()
            return math.hypot(location.x - player_location.x, location.y - player_location.y)

        for actor in actors:
            distance = get_distance(actor)

            if distance <= SURROUND_SENSOR_RANGE and actor.id != self.player.id:
                location = actor.get_location()
                bb = actor.bounding_box

                attrs = [time_.nanoseconds, actor.id, actor.type_id, distance, bb.extent.x*2, bb.extent.y*2, location.x, location.y]
                line = ','.join(map(str, attrs)) + '\n'

                self._extra_objs_csv.write(line)

        self._extra_objs_csv.flush()

    def restart(self):
        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
        blueprint.set_attribute('role_name', self.actor_role_name)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')
        # set the max speed
        if blueprint.has_attribute('speed'):
            self.player_max_speed = float(blueprint.get_attribute('speed').recommended_values[1])
            self.player_max_speed_fast = float(blueprint.get_attribute('speed').recommended_values[2])
        else:
            print("No recommended values for 'speed' attribute")
        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.modify_vehicle_physics(self.player)
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()

            spawn_point = carla.Transform(
                carla.Location(-149, -80, 2),
                rotation=carla.Rotation(yaw=90)
            )

            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.modify_vehicle_physics(self.player)

        self.surround_sensor = SurroundSensor(self, 1.0, -2.0, np.pi/4, 'sensor1')
        self.surround_sensor2 = SurroundSensor(self, -1.0, 0.5, -np.pi/3, 'sensor2')

        self.imu_sensor = IMUSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)

        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def next_map_layer(self, reverse=False):
        self.current_map_layer += -1 if reverse else 1
        self.current_map_layer %= len(self.map_layer_names)
        selected = self.map_layer_names[self.current_map_layer]
        self.hud.notification('LayerMap selected: %s' % selected)

    def load_map_layer(self, unload=False):
        selected = self.map_layer_names[self.current_map_layer]
        if unload:
            self.hud.notification('Unloading map layer: %s' % selected)
            self.world.unload_map_layer(selected)
        else:
            self.hud.notification('Loading map layer: %s' % selected)
            self.world.load_map_layer(selected)

    def modify_vehicle_physics(self, vehicle):
        physics_control = vehicle.get_physics_control()
        physics_control.use_sweep_wheel_collision = True
        vehicle.apply_physics_control(physics_control)

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        sensors = [self.camera_manager.sensor, self.imu_sensor.sensor]

        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()

        self.close_csv()
        self.close_extra_objs_csv()


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl:
    """Class that handles keyboard input."""
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._lights = carla.VehicleLightState.NONE
            world.player.set_autopilot(self._autopilot_enabled)
            world.player.set_light_state(self._lights)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self, client, world, clock):
        if isinstance(self._control, carla.VehicleControl):
            current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    if self._autopilot_enabled:
                        world.player.set_autopilot(False)
                        world.restart()
                        world.player.set_autopilot(True)
                    else:
                        world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_v and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_map_layer(reverse=True)
                elif event.key == K_v:
                    world.next_map_layer()
                elif event.key == K_b and pygame.key.get_mods() & KMOD_SHIFT:
                    world.load_map_layer(unload=True)
                elif event.key == K_b:
                    world.load_map_layer()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key == K_n:
                    world.camera_manager.next_sensor()
                elif event.key == K_w and (pygame.key.get_mods() & KMOD_CTRL):
                    if world.constant_velocity_enabled:
                        world.player.disable_constant_velocity()
                        world.constant_velocity_enabled = False
                        world.hud.notification("Disabled Constant Velocity Mode")
                    else:
                        world.player.enable_constant_velocity(carla.Vector3D(17, 0, 0))
                        world.constant_velocity_enabled = True
                        world.hud.notification("Enabled Constant Velocity Mode at 60 km/h")
                elif event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
                    world.camera_manager.toggle_recording()
                elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
                    if (world.recording_enabled):
                        client.stop_recorder()
                        world.recording_enabled = False
                        world.hud.notification("Recorder is OFF")
                    else:
                        client.start_recorder("manual_recording.rec", True)
                        world.recording_enabled = True
                        world.hud.notification("Recorder is ON")
                elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
                    # stop recorder
                    client.stop_recorder()
                    world.recording_enabled = False
                    # work around to fix camera at start of replaying
                    current_index = world.camera_manager.index
                    world.destroy_sensors()
                    # disable autopilot
                    self._autopilot_enabled = False
                    world.player.set_autopilot(self._autopilot_enabled)
                    world.hud.notification("Replaying file 'manual_recording.rec'")
                    # replayer
                    client.replay_file("manual_recording.rec", world.recording_start, 0, 0)
                    world.camera_manager.set_sensor(current_index)
                elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start -= 10
                    else:
                        world.recording_start -= 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start += 10
                    else:
                        world.recording_start += 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))

                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = world.player.get_control().gear
                        world.hud.notification('%s Transmission' %
                                               ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p and not pygame.key.get_mods() & KMOD_CTRL:
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.player.set_autopilot(self._autopilot_enabled)
                        world.hud.notification(
                            'Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
                        current_lights ^= carla.VehicleLightState.Special1
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                        current_lights ^= carla.VehicleLightState.HighBeam
                    elif event.key == K_l:
                        # Use 'L' key to switch between lights:
                        # closed -> position -> low beam -> fog
                        if not self._lights & carla.VehicleLightState.Position:
                            world.hud.notification("Position lights")
                            current_lights |= carla.VehicleLightState.Position
                        else:
                            world.hud.notification("Low beam lights")
                            current_lights |= carla.VehicleLightState.LowBeam
                        if self._lights & carla.VehicleLightState.LowBeam:
                            world.hud.notification("Fog lights")
                            current_lights |= carla.VehicleLightState.Fog
                        if self._lights & carla.VehicleLightState.Fog:
                            world.hud.notification("Lights off")
                            current_lights ^= carla.VehicleLightState.Position
                            current_lights ^= carla.VehicleLightState.LowBeam
                            current_lights ^= carla.VehicleLightState.Fog
                    elif event.key == K_i:
                        current_lights ^= carla.VehicleLightState.Interior
                    elif event.key == K_z:
                        current_lights ^= carla.VehicleLightState.LeftBlinker
                    elif event.key == K_x:
                        current_lights ^= carla.VehicleLightState.RightBlinker
                    elif event.key == K_t:
                        world.hud.notification("Restarting Fake Sensors")

                        world.close_csv()
                        world.open_csv()

                        world.surround_sensor.destroy()
                        world.surround_sensor2.destroy()

                        world.surround_sensor = SurroundSensor(
                            world,
                            world.surround_sensor.relative_x,
                            world.surround_sensor.relative_y,
                            world.surround_sensor.relative_angle,
                            world.surround_sensor.name
                        )

                        world.surround_sensor2 = SurroundSensor(
                            world,
                            world.surround_sensor2.relative_x,
                            world.surround_sensor2.relative_y,
                            world.surround_sensor2.relative_angle,
                            world.surround_sensor2.name
                        )


        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._control.reverse = self._control.gear < 0
                # Set automatic control-related vehicle lights
                if self._control.brake:
                    current_lights |= carla.VehicleLightState.Brake
                else: # Remove the Brake flag
                    current_lights &= ~carla.VehicleLightState.Brake
                if self._control.reverse:
                    current_lights |= carla.VehicleLightState.Reverse
                else: # Remove the Reverse flag
                    current_lights &= ~carla.VehicleLightState.Reverse
                if current_lights != self._lights: # Change the light state only if necessary
                    self._lights = current_lights
                    world.player.set_light_state(carla.VehicleLightState(self._lights))
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time(), world)
            world.player.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        if keys[K_UP] or keys[K_w]:
            self._control.throttle = min(self._control.throttle + 0.01, 1)
        else:
            self._control.throttle = 0.0

        if keys[K_DOWN] or keys[K_s]:
            self._control.brake = min(self._control.brake + 0.2, 1)
        else:
            self._control.brake = 0

        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.hand_brake = keys[K_SPACE]

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD:
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 16), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        if world is None:
            return

        self._notifications.tick(world, clock)
        if not self._show_info:
            return

        t = world.player.get_transform()
        v = world.player.get_velocity()

        compass = world.imu_sensor.compass
        heading = 'N' if compass > 270.5 or compass < 89.5 else ''
        heading += 'S' if 90.5 < compass < 269.5 else ''
        heading += 'E' if 0.5 < compass < 179.5 else ''
        heading += 'W' if 180.5 < compass < 359.5 else ''

        ego_obj = actor_to_object_model(world.player)

        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.hypot(v.x, v.y)),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            u'Compass:% 17.5f\N{DEGREE SIGN} % 2s' % (compass, heading),
            '',
            'Object Model main attributes:',
            '  X:        % 7.5f m' % (ego_obj.track.state[Track.STATE_X_IDX]),
            '  Y:        % 7.5f m' % (ego_obj.track.state[Track.STATE_Y_IDX]),
            '  Vx:       % 7.5f m/s' % (ego_obj.track.state[Track.STATE_VELOCITY_X_IDX]),
            '  Vy:       % 7.5f m/s' % (ego_obj.track.state[Track.STATE_VELOCITY_Y_IDX]),
            '  Ax:       % 7.5f m/s^2' % (ego_obj.track.state[Track.STATE_ACCELERATION_X_IDX]),
            '  Ay:       % 7.5f m/s^2' % (ego_obj.track.state[Track.STATE_ACCELERATION_Y_IDX]),
            '  yaw:      % 7.5f rad' % (ego_obj.track.state[Track.STATE_YAW_IDX]),
            '  yaw_rate: % 7.5f rad/s' % (ego_obj.track.state[Track.STATE_YAW_RATE_IDX]),
            '  length:   % 7.5f m' % (ego_obj.dimensions.values[Dimensions.DIMENSIONS_LENGHT_IDX]),
            '  width:    % 7.5f m' % (ego_obj.dimensions.values[Dimensions.DIMENSIONS_WIDTH_IDX]),
            ''
        ]

        vehicles = world.world.get_actors().filter('vehicle.*')
        self._info_text += [
            '',
            'Number of vehicles: % 8d' % len(vehicles)
        ]
        # nearby_vehicles = world.surround_sensor.read(vehicles)

        actors = list(world.world.get_actors().filter('walker.pedestrian.*'))
        actors += list(vehicles)
        nearby_actors = world.surround_sensor.read(actors)

        if nearby_actors:
            self._info_text += ['', 'Surrounding actors:']

            for distance, actor in nearby_actors:
                actor_type = get_actor_display_name(actor, truncate=22)
                self._info_text.append('    % 4dm %s' % (distance, actor_type))

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((250, self.dim[1]))
            info_surface.set_alpha(150)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText:
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText:
    """Helper class to handle text output using pygame"""
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.line_space = 18
        self.dim = (780, len(lines) * self.line_space + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))

        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * self.line_space))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)



# ==============================================================================
# -- SurroundSensor ----------------------------------------------------------------
# ==============================================================================

class SurroundSensor:
    def __init__(self, world, x=0., y=0., angle=0., name="surround_sensor"):
        self.name = name
        self.world = world
        self.node = world.ros_node

        self.time_last_measurement = None
        self.last_measurement = None

        self.relative_x = x
        self.relative_y = y
        self.relative_angle = angle

        self.capable = [True] * Track.STATE_SIZE
        # self.measurement_noise_matrix = np.diag(
        #     [1.5**2, 1.5**2, 1**2, 0.5**2, 0.02**2, 0.1**2
        # ]).astype('float32')
        self.measurement_noise_matrix = np.diag(
            [0, 0, 0, 0, 0, 0]
        ).astype('float32')

        self.publisher = self.node.create_publisher(
            ObjectModel,
            'objectlevel_fusion/fusion_layer/fusion/submit',
            10
        )

        self.sensor_registration_client = self.node.create_client(
            RegisterSensor, 'fusion_layer/register_sensor'
        )
        while not self.sensor_registration_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error(
                'Failed to connect for sensor registration, trying again...'
            )

        self.sensor_remover_client = self.node.create_client(
            RemoveSensor, 'fusion_layer/remove_sensor'
        )
        while not self.sensor_remover_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error(
                'Failed to connect for sensor removing, trying again...'
            )

        self._register()

        # Nested to use 'self' from the context
        def _callback(_):
            # Not using WorldSnapshot because it doesn't have type_id information
            #   and ActorSnapshot doesn't have get_location()

            global STARTED_TIME

            actors = list(world.world.get_actors().filter('walker.pedestrian.*'))
            actors += list(self.world.world.get_actors().filter('vehicle.*'))

            nearby_actors = [v for d, v in self.read(actors)]

            if len(nearby_actors) == 0:
                return

            # Discard first seconds of simulation, they're noisy
            if STARTED_TIME is None:
                STARTED_TIME = datetime.datetime.now()
                return
            if datetime.datetime.now() - STARTED_TIME < datetime.timedelta(seconds=2):
                return

            self.publish(nearby_actors)

        self._callback_id = self.world.world.on_tick(_callback)

    @property
    def x(self):
        return self.world.player.get_location().x + self.relative_x

    @property
    def y(self):
        return self.world.player.get_location().y + self.relative_y

    @property
    def angle(self):
        return self.world.player.get_transform().rotation.yaw + self.relative_angle

    def _register(self):
        request = RegisterSensor.Request()

        request.name = self.name
        request.x = self.relative_x
        request.y = self.relative_y
        request.angle = self.relative_angle
        request.capable = self.capable
        request.measurement_noise_matrix = self.measurement_noise_matrix.reshape(-1)

        self.sensor_registration_client.call_async(request)
        self.node.get_logger().info(f"Sensor {self.name} registered successfully!")

    def destroy(self):
        self.world.world.remove_on_tick(self._callback_id)
        self._unregister()

    def _unregister(self):
        request = RemoveSensor.Request()
        request.name = self.name

        return self.sensor_remover_client.call_async(request)


    def read(self, actors=None, distance=SURROUND_SENSOR_RANGE):
        if self.world is None:
            return []

        def get_distance(location):
            return math.hypot(location.x - self.x, location.y - self.y)

        actors = actors or (list(self.world.world.get_actors().filter('vehicle.*'))
                            + list(self.world.world.get_actors().filter('walker.pedestrian.*')))

        distances = ((get_distance(a.get_location()), a)
                     for a in actors if a.id != self.world.player.id)

        with_distances = sorted([(d, a) for d, a in distances if d <= distance])

        self.time_last_measurement = self.node.get_clock().now()

        return with_distances

    def publish(self, measurement):
        ego_obj = actor_to_object_model(self.world.player)
        msg = self._measurement_to_msg(measurement, ego_obj)

        self.publisher.publish(msg)

        self.world.add_info_csv(self, ego_obj, msg)
        self.world.add_info_extra_objs_csv()

    def get_relative(self, obj: Object) -> Object:
        angle = self.relative_angle
        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)

        transformation = np.array([
            [cos_angle, -sin_angle, 0, 0, 0, 0, 0, 0,       self.relative_x],
            [sin_angle,  cos_angle, 0, 0, 0, 0, 0, 0,       self.relative_y],
            [0, 0, cos_angle, -sin_angle, 0, 0, 0, 0,                     0],
            [0, 0, sin_angle,  cos_angle, 0, 0, 0, 0,                     0],
            [0, 0, 0, 0, cos_angle, -sin_angle, 0, 0,                     0],
            [0, 0, 0, 0, sin_angle,  cos_angle, 0, 0,                     0],
            [0, 0, 0, 0,          0,           0, 1, 0, self.relative_angle],
            [0, 0, 0, 0,          0,           0, 0, 1,                   0],
            [0, 0, 0, 0,          0,           0, 0, 0,                   1],
        ], dtype='float32')

        result = deepcopy(obj)

        to_align = np.hstack((obj.track.state, 1)).T
        transformation = np.linalg.inv(transformation)

        product = transformation @ to_align
        result.track.state = np.delete(product, -1, 0).astype('float32')
        result.track.state[Track.STATE_YAW_IDX] = wrap_angle(result.track.state[Track.STATE_YAW_IDX])

        return result

    def _measurement_to_msg(self, measurement, ego_obj):
        msg = ObjectModel()
        msg.header.frame_id = self.name
        msg.header.stamp = self.time_last_measurement.to_msg()

        relative_to_ego = partial(get_relative_obj, ego_obj)

        object_model = map(actor_to_object_model, measurement)
        object_model = map(relative_to_ego, object_model)
        object_model = map(self.get_relative, object_model)

        msg.object_model = list(object_model)

        return msg


# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager:
    def __init__(self, parent_actor, hud, gamma_correction):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        Attachment = carla.AttachmentType
        self._camera_transforms = [
            (carla.Transform(carla.Location(x=-5.5, z=2.5), carla.Rotation(pitch=8.0)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
            (carla.Transform(carla.Location(x=5.5, y=1.5, z=1.5)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=-8.0, z=6.0), carla.Rotation(pitch=6.0)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=-1, y=-bound_y, z=0.5)), Attachment.Rigid)]

        self.transform_index = 1
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()

        self.sensors = [['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}]]

        for item in self.sensors:
            bp = bp_library.find(item[0])

            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)

            item.append(bp)
        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.camera.dvs'):
            # Example of converting the raw_data from a carla.DVSEventArray
            # sensor into a NumPy array and using it as an image
            dvs_events = np.frombuffer(image.raw_data, dtype=np.dtype([
                ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
            dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            # Blue is positive, red is negative
            dvs_img[dvs_events[:]['y'], dvs_events[:]['x'], dvs_events[:]['pol'] * 2] = 255
            self.surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)


# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor:
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)

# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    rclpy.init()

    pygame.init()
    pygame.font.init()
    world = None

    ros_node = Node('manual_control_node')
    ros_executor = rclpy.executors.MultiThreadedExecutor()
    ros_executor.add_node(ros_node)

    ros_thread = Thread(target=ros_executor.spin)
    ros_thread.start()

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        display.fill((0,0,0))
        pygame.display.flip()

        hud = HUD(args.width, args.height)
        world = World(client.get_world(), hud, ros_node, args)
        controller = KeyboardControl(world, args.autopilot)

        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(60)
            if controller.parse_events(client, world, clock):
                return
            world.tick(clock)
            world.render(display)
            pygame.display.flip()

    finally:
        print("Destroying things...")

        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
            world.surround_sensor.destroy()
            world.surround_sensor2.destroy()

            ros_node.destroy_node()
            rclpy.shutdown()

            world.destroy()
            world = None
        else:
            ros_node.destroy_node()
            rclpy.shutdown()

        pygame.quit()

        ros_thread.join()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
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
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.tesla.model3',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='ego_vehicle',
        help='actor role name (default: "ego_vehicle")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:
        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()
