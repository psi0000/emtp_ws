#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import struct

class GlobalMapPublisher(Node):
    def __init__(self):
        super().__init__('global_map_publisher')

        self.publisher = self.create_publisher(
            PointCloud2,
            '/global_map_pcd',
            10
        )

        self.frame_id = 'map'
        self.pcd_path = '/root/emtp_ws/src/server_rviz/server_rviz/map_psi_downsampled.pcd'

        pcd = o3d.io.read_point_cloud(self.pcd_path)
        pts = np.asarray(pcd.points)

        if pts.size == 0:
            self.get_logger().error('PCD에 포인트 없음')
            return

        header = Header()
        header.frame_id = self.frame_id

        cloud_points = []
        for x, y, z in pts:
            # 🔹 높이 기준 색상 정의 (원하는 대로 수정)
            if z < 2.5:         #1층
                if z < 0.2:
                    r, g, b = 90, 140, 200     
                else:       
                    r, g, b = 0, 0, 0     
            elif z < 6.0:
                if z < 3.8:
                    r, g, b = 90, 200, 170       
                else:       
                    r, g, b = 0, 0, 0  
                             
            else:                
                r, g, b = 0, 0, 0    
            if y >27.8:
                r, g, b = 0, 0, 0   
            if x <-12.1:
                r, g, b = 0, 0, 0  


            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]
            cloud_points.append([x, y, z, rgb])

        fields = [
            pc2.PointField(name='x', offset=0,  datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4,  datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8,  datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.UINT32,  count=1),
        ]

        self.msg = pc2.create_cloud(header, fields, cloud_points)

        self.timer = self.create_timer(1.0, self.publish)
        self.get_logger().info(f'PCD loaded with RGB: {len(pts)} points')

    def publish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.msg)

def main():
    rclpy.init()
    node = GlobalMapPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
