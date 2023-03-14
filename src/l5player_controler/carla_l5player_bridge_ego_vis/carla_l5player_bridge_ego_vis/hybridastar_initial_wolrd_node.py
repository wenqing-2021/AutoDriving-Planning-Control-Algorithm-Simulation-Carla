#!/usr/bin/env python
import carla
import rclpy
from rclpy.node import Node

class InitialWorld(Node):
    def __init__(self):
        super().__init__('HybridAstar_InitialWorld')
        self.declare_parameter("map_max_x", 170.0)
        self.declare_parameter("map_min_x", 15.0)
        self.declare_parameter("map_max_y", -200)
        self.declare_parameter("map_min_y", -300)
        self.declare_parameter("resolution", 0.5)

        self.initial_town()
        
    def initial_town(self):
        # get client
        carla_client = carla.Client(host='localhost', port=2000)
        carla_client.set_timeout(2)
        # get world
        carla_world = carla_client.get_world()
        self.get_logger().info('Successfully get carla world')
        # clear objects for unstructure road
        env_objs = carla_world.get_environment_objects(carla.CityObjectLabel.Buildings)
        env_poles = carla_world.get_environment_objects(carla.CityObjectLabel.Poles)
        env_traffic = carla_world.get_environment_objects(carla.CityObjectLabel.TrafficSigns)
        env_TrafficLight = carla_world.get_environment_objects(carla.CityObjectLabel.TrafficLight)
        env_vegetation = carla_world.get_environment_objects(carla.CityObjectLabel.Vegetation)
        env_static = carla_world.get_environment_objects(carla.CityObjectLabel.Static)
        env_dynamic = carla_world.get_environment_objects(carla.CityObjectLabel.Dynamic)
        env_fences = carla_world.get_environment_objects(carla.CityObjectLabel.Fences)
        env_other = carla_world.get_environment_objects(carla.CityObjectLabel.Other)
        env_Terrain = carla_world.get_environment_objects(carla.CityObjectLabel.Terrain)
        env_wall = carla_world.get_environment_objects(carla.CityObjectLabel.Walls)
        env_item_list = [env_objs, env_poles, env_traffic, env_vegetation, env_static,\
                         env_fences, env_other, env_Terrain, env_wall, env_dynamic, env_TrafficLight]

        # disable all buildings/ poles/ 
        for env_item in env_item_list:
            for item_i in enumerate(env_item):
                if item_i[0] == 0:
                    objects_to_toggle = {item_i[1].id}
                else:
                    objects_to_toggle.update({item_i[1].id})
            
            carla_world.enable_environment_objects(objects_to_toggle, False)
        
        self.get_logger().info('Successfully clear all obstacles')

def main(args=None):
    rclpy.init(args=args)
    # clear all obstacles
    initial_world = InitialWorld()
    rclpy.spin_once(initial_world)
    # shutdown
    initial_world.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()