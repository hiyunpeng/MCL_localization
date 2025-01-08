import math
import random
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point
from . pf_base import PFLocaliserBase
from sklearn.cluster import DBSCAN
import numpy as np
from . util import rotateQuaternion, getHeading


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self, logger, clock):
        # ----- Call the superclass constructor
        super().__init__(logger, clock)
        
        # ----- Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0.1  # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0.1  # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0.1  # Odometry model y axis (side-to-side) noise
        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict
        
       
    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        particle_cloud = PoseArray()

        # Generate particles around the initialpose, adding random Gaussian noise
        for _ in range(self.NUMBER_PREDICTED_READINGS):
            particle = Pose()
            particle.position.x = initialpose.position.x + random.gauss(0, self.ODOM_TRANSLATION_NOISE)
            particle.position.y = initialpose.position.y + random.gauss(0, self.ODOM_DRIFT_NOISE)
            particle.orientation = rotateQuaternion(initialpose.orientation, random.gauss(0, self.ODOM_ROTATION_NOISE))

            particle_cloud.poses.append(particle)

        self.particlecloud = particle_cloud
        return particle_cloud

 
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        
        

        
        
        # Loop through each particle and update its weight based on sensor model
        particle_weights = []
        for particle in self.particlecloud.poses:
            weight = self.sensor_model.get_weight(scan, particle)
            particle_weights.append(weight)

        # Normalize weights
        total_weight = sum(particle_weights)
        if total_weight > 0:
            particle_weights = [w / total_weight for w in particle_weights]
        else:
            # If all weights are zero (unlikely), assign equal probability
            particle_weights = [1.0 / len(self.particlecloud.poses)] * len(self.particlecloud.poses)

        # Step 2: Resample particles using roulette-wheel selection
        new_particles = []
        for _ in range(len(self.particlecloud.poses)):
            # Select a particle based on weights
            selected_particle = random.choices(self.particlecloud.poses, weights=particle_weights, k=1)[0]

            # Step 3: Add noise to the resampled particle to maintain spread
            new_particle = Pose()
            new_particle.position.x = selected_particle.position.x + random.gauss(0, self.ODOM_TRANSLATION_NOISE)
            new_particle.position.y = selected_particle.position.y + random.gauss(0, self.ODOM_DRIFT_NOISE)

            # Add small rotation noise
            new_particle.orientation = rotateQuaternion(selected_particle.orientation,
                                                        random.gauss(0, self.ODOM_ROTATION_NOISE))
            new_particles.append(new_particle)

        # Step 4: Update particle cloud with new particles
        self.particlecloud.poses = new_particles

    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """
        num_particles = len(self.particlecloud.poses)
        if num_particles == 0:
            return None

        avg_x, avg_y, avg_cos, avg_sin = 0, 0, 0, 0

        for particle in self.particlecloud.poses:
            avg_x += particle.position.x
            avg_y += particle.position.y

            heading = getHeading(particle.orientation)
            avg_cos += math.cos(heading)
            avg_sin += math.sin(heading)

        # Compute the average position and orientation
        avg_x /= num_particles
        avg_y /= num_particles
        avg_theta = math.atan2(avg_sin, avg_cos)

        # Return the estimated pose as a Pose message
        estimated_pose = Pose()
        estimated_pose.position = Point(avg_x, avg_y, 0)
        estimated_pose.orientation = rotateQuaternion(Quaternion(), avg_theta)

        return estimated_pose