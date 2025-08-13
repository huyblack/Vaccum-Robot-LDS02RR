#!/usr/bin/env python3

import os
import numpy as np
import json
import tempfile
from shapely.geometry import Polygon, Point
from math import *
import time
import subprocess
import yaml

from scipy.spatial.transform import Rotation

from path_coverage.list_helper import *
from path_coverage.trapezoidal_coverage import calc_path as trapezoid_calc_path
from path_coverage.border_drive import border_calc_path

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point as rosPoint

import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import ReliabilityPolicy, QoSProfile
from ament_index_python.packages import get_package_share_directory

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateThroughPoses
from nav2_msgs.action import NavigateToPose

from geometry_msgs.msg import PointStamped, PoseStamped
from action_msgs.msg import GoalStatus

INSCRIBED_INFLATED_OBSTACLE = 253

class MapDrive(Node): 
	def __init__(self):
		super().__init__('map_drive') 

		# Thêm các biến để quản lý waypoint tuần tự
		self.current_waypoint_index = 0
		self.saved_waypoints = []
		self.is_navigating = False

		# Thay đổi action client để sử dụng NavigateToPose thay vì NavigateThroughPoses
		self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

		self.x = None
		self.y = None
		
		self.rospack  = get_package_share_directory('path_coverage') 

		# Define the file name for the YAML file
		self.filename = "/home/huy/nav2_ws/Vaccum-Robot-LDS02RR/src/turtlebot3/ros2_explorer/path_coverage_ros2/scripts/full_path_cov_poses.yaml" # "pose_output.yaml"

		# Initialize 
		self.pose_output = {}
		self.last_points = {}
		self.last_path = None
		self.lClickPoints = []
		self.local_costmap = None
		self.global_costmap = None
		self.goal_handle = None
		self.result_future = None
		self.feedback = None
		
		self.declare_parameter("global_frame", "map")
		self.declare_parameter("robot_width", 0.15) 
		self.declare_parameter("costmap_max_non_lethal", 50)
		self.declare_parameter("boustrophedon_decomposition", True)
		self.declare_parameter("border_drive", False)
		self.declare_parameter("base_frame", "base_footprint")
		self.declare_parameter("num_points", 6) 
		self.declare_parameter("min_wp_dist", 0.1) 

		self.global_frame = self.get_parameter("global_frame").get_parameter_value().string_value 
		self.robot_width = self.get_parameter("robot_width").get_parameter_value().double_value
		self.costmap_max_non_lethal = self.get_parameter("costmap_max_non_lethal").get_parameter_value().integer_value
		self.boustrophedon_decomposition = self.get_parameter("boustrophedon_decomposition").get_parameter_value().bool_value #  False #
		self.border_drive = self.get_parameter("border_drive").get_parameter_value().bool_value
		self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
		self.num_points = self.get_parameter("num_points").get_parameter_value().integer_value
		self.min_wp_dist = self.get_parameter("min_wp_dist").get_parameter_value().double_value

		self.create_subscription(PointStamped, "/clicked_point", self.rvizPointReceived, 1)
		self.create_subscription(OccupancyGrid, f"/global_costmap/costmap", self.globalCostmapReceived, 10) 
		self.create_subscription(OccupancyGrid, f"/local_costmap/costmap", self.localCostmapReceived, 10)  

		self.pub_marker = self.create_publisher(Marker, 'path_coverage_marker', 16) 
		
		self.tfBuffer = Buffer()
		self.tf_listener = TransformListener(self.tfBuffer, self)

		self.get_logger().info('parameters::::global_frame::robot_width::costmap_max_non_lethal::boustrophedon_decomposition::border_drive::base_frame::num_points::min_wp_dist.')
		self.get_logger().info('::::::::::::::'+str(self.global_frame)+'::'+str(self.robot_width)+'::'+str(self.costmap_max_non_lethal)+'::'+str(self.boustrophedon_decomposition)+'::'+str(self.border_drive)+'::'+str(self.base_frame)+'::'+str(self.num_points)+'::'+str(self.min_wp_dist)+'.')
		self.get_logger().info("Path coverage node initialized successfully...")

	def localCostmapReceived(self, costmap):
		self.local_costmap = costmap
		self.local_costmap_width = costmap.info.width*costmap.info.resolution
		self.local_costmap_height = costmap.info.height*costmap.info.resolution

	def globalCostmapReceived(self, costmap):
		self.global_costmap = costmap
	
	def visualization_cleanup(self):
		for id, points in self.last_points.items():
			if points is not None:
				self.visualize_trapezoid(points, id=id, show=False)
			self.last_points = {}
		if self.last_path is not None:
			self.visualize_path(self.last_path, False)
			self.last_path = None

	def visualize_cell(self, points, show=True, close=True):
		self.visualize_trapezoid(points, show, close)

	def visualize_area(self, points, show=True, close=True):
		self.visualize_trapezoid(points, show, close, id=1, red=1.0, blue=0.0)

	def visualize_trapezoid(self, points, show=True, close=True, id=0, red=0.0, green=0.0, blue=1.0):
		if len(points) < 2: 
			return
		self.last_points[id] = points if show else None
		msg = Marker()
		msg.header.frame_id = self.global_frame
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.ns = "trapezoid"
		msg.id = id
		msg.type = Marker.LINE_STRIP
		msg.action = Marker.ADD if show else Marker.DELETE
		msg.pose.orientation.w = float(1)
		msg.pose.orientation.x = float(0)
		msg.pose.orientation.y = float(0)
		msg.pose.orientation.z = float(0)
		msg.scale.x = 0.02
		msg.color.r = red
		msg.color.g = green
		msg.color.b = blue
		msg.color.a = 1.0

		if close:
			points = points + [points[0]]
		for point in points:
			point_msg = rosPoint()
			point_msg.x = point[0]
			point_msg.y = point[1]
			msg.points.append(point_msg)

		self.pub_marker.publish(msg)
		time.sleep(0.3)
		self.get_logger().info("viz_trapezoid completed...")

	def visualize_path(self, path, show=True):
		i = 0
		self.last_path = path if show else None
		for pos_last,pos_cur in pairwise(path):
			msg = Marker()
			msg.header.frame_id = self.global_frame
			msg.header.stamp =  self.get_clock().now().to_msg()
			msg.ns = "path"
			msg.id = i
			msg.type = Marker.ARROW
			msg.action = Marker.ADD if show else Marker.DELETE
			msg.pose.orientation.w = float(1)
			msg.pose.orientation.x = float(0)
			msg.pose.orientation.y = float(0)
			msg.pose.orientation.z = float(0)
			msg.scale.x = 0.01 # shaft diameter
			msg.scale.y = 0.03 # head diameter
			# green
			msg.color.g = 1.0
			msg.color.a = 1.0

			point_msg_start = rosPoint()
			point_msg_start.x = pos_last[0]
			point_msg_start.y = pos_last[1]
			msg.points.append(point_msg_start)
			point_msg_end = rosPoint()
			point_msg_end.x = pos_cur[0]
			point_msg_end.y = pos_cur[1]
			msg.points.append(point_msg_end)

			i+=1
			self.pub_marker.publish(msg)
			time.sleep(0.3)
		self.get_logger().info("visualize_path completed...")

	def rvizPointReceived(self, point):
		self.lClickPoints.append(point)
		points = [(p.point.x, p.point.y) for p in self.lClickPoints]
		self.global_frame = point.header.frame_id
		if len(self.lClickPoints) > 2:
			if len(set([p.header.frame_id for p in self.lClickPoints])) != 1:
				raise ValueError()
			points_x = [p.point.x for p in self.lClickPoints]
			points_y = [p.point.y for p in self.lClickPoints]
			avg_x_dist = list_avg_dist(points_x)
			avg_y_dist = list_avg_dist(points_y)
			dist_x_first_last = abs(points_x[0] - points_x[-1])
			dist_y_first_last = abs(points_y[0] - points_y[-1])
			if dist_x_first_last < avg_x_dist/10.0 and dist_y_first_last < avg_y_dist/10.0:
				self.get_logger().info("Creating polygon %s" % (str(points)))
				self.visualize_area(points, close=True)
				
				try:
					# --- BƯỚC 1: TẠO DANH SÁCH NAV2 POSES ---
					# drive_polygon bây giờ sẽ gọi drive_path, và drive_path sẽ trả về 
					# một danh sách các PoseStamped đã sẵn sàng.
					if self.boustrophedon_decomposition:
						self.get_logger().info("do_boustrophedon initiated...")
						final_nav2_poses = self.do_boustrophedon(Polygon(points), self.global_costmap)
					else:
						self.get_logger().info("drive_polygon initiated...")
						final_nav2_poses = self.drive_polygon(Polygon(points))

					# --- BƯỚC 2: LƯU VÀ GỬI WAYPOINTS TUẦN TỰ ---
					if final_nav2_poses and len(final_nav2_poses) > 0:
						self.get_logger().info(f"Tạo thành công {len(final_nav2_poses)} waypoints")
						
						# Lưu waypoints vào file YAML
						waypoints_data = self.save_waypoints_to_yaml(final_nav2_poses)
						self.saved_waypoints = waypoints_data["waypoints"]
						self.current_waypoint_index = 0
						
						# Bắt đầu gửi waypoint đầu tiên
						self.send_next_waypoint()
					else:
						self.get_logger().warn('Không có đường đi nào được tạo ra.')

				except Exception as e:
					self.get_logger().error(f"Lỗi trong quá trình xử lý đa giác: {e}")

				self.visualize_area(points, close=True, show=False)
				self.lClickPoints = []
				# Tạm thời vô hiệu hóa việc ghi file YAML để tập trung vào chức năng tự động di chuyển
				# self.pose_output["updatetime"] = time.time_ns()
				# with open(self.filename, "w") as f:
				# 	yaml.dump(self.pose_output, f)
				# self.pose_output = {}	
				self.get_logger().info("Đã hoàn thành xử lý đa giác và gửi lệnh tới Nav2")
				return
		self.visualize_area(points, close=False)
		self.get_logger().info("finished successfully rvizPointReceived func.")

	def make_Polygons_shapely_polygons(self, Polygons):
		polygons = []
		polygon_area = []
		for polygon in Polygons:
			coords = [(x, y) for x, y in polygon]
			shapely_polygon = Polygon(coords)
			area = shapely_polygon.area
			polygons.append(shapely_polygon)
			polygon_area.append(area)
		return polygons, polygon_area

	def are_polygons_connected(self, poly1, poly2, threshold=2): 
		p1 = Polygon(poly1)
		p2 = Polygon(poly2)
		for c1 in p1.exterior.coords:
			for c2 in p2.exterior.coords:
				if math.sqrt((c1[0]-c2[0])**2 + (c1[1]-c2[1])**2) <= threshold:
					return True
		return False

	def are_polygons_connected_with_increased_thresh(self, poly1, poly2):
		return self.are_polygons_connected(poly1, poly2, threshold=4)

	def find_connected_polygons(self, Polygons, polygon_area_threshold=150):
		if len(Polygons) <= 2:
			return Polygons

		polygons, polygon_area = self.make_Polygons_shapely_polygons(Polygons)

		connected_polygons = []
		associated_polygons = set()

		for i, poly1 in enumerate(polygons):
			index1 = polygons.index(poly1)

			for j, poly2 in enumerate(polygons[i+1:], start=i+1):
				if self.are_polygons_connected(poly1, poly2):
					index2 = polygons.index(poly2)
					connected_polygons.append((index1, index2))
					associated_polygons.add(poly1)
					associated_polygons.add(poly2)

			if poly1 not in associated_polygons:
				for poly2 in polygons:
					if poly2 != poly1:
						if self.are_polygons_connected_with_increased_thresh(poly1, poly2):
							index2 = polygons.index(poly2)
							connected_polygons.append((index1, index2))
							associated_polygons.add(poly1)
							associated_polygons.add(poly2)
							break

			if poly1 not in associated_polygons:    
				connected_polygons.append((index1,))
				associated_polygons.add(poly1)

		if connected_polygons:
			order = []
			parent_children = {}

			for pair in connected_polygons:
				if isinstance(pair, tuple) and len(pair) == 2:
					parent, child = pair
					parent_children.setdefault(parent, []).append(child)

			for parent in range(len(connected_polygons)):
				if parent not in parent_children:
					continue
				order.append(parent)
				stack = parent_children[parent][::-1]
				while stack:
					node = stack.pop()
					if node not in parent_children:
						order.append(node)
						continue
					order.extend([node]+parent_children[node][::-1])			

			new_order = []
			for elem in order:
				if elem not in new_order:
					new_order.append(elem)

			ordered_polygons = [Polygons[i] for i in new_order]
			ordered_polygon_areas = [polygon_area[i] for i in new_order]

			filtered_polygons = []; 

			for polygon, area in zip(ordered_polygons, ordered_polygon_areas):
				if area >= polygon_area_threshold:
					filtered_polygons.append(polygon)

			parent_children = None; del parent_children
			order = None; del order
			new_order = None; del new_order
			ordered_polygons = None; del ordered_polygons
			ordered_polygon_areas = None; del ordered_polygon_areas

			return filtered_polygons
		else:
			print("No polygons are connected.")

	def do_boustrophedon(self, poly, costmap):
		(minx, miny, maxx, maxy) = poly.bounds
		# Convert to costmap coordinate
		minx = round((minx-costmap.info.origin.position.x)/costmap.info.resolution)
		maxx = round((maxx-costmap.info.origin.position.x)/costmap.info.resolution)
		miny = round((miny-costmap.info.origin.position.y)/costmap.info.resolution)
		maxy = round((maxy-costmap.info.origin.position.y)/costmap.info.resolution)
		# Check min/max limits
		if minx < 0: minx = 0
		if maxx > costmap.info.width: maxx = costmap.info.width
		if miny < 0: miny = 0
		if maxy > costmap.info.height: maxy = costmap.info.height
		# Transform costmap values to values expected by boustrophedon_decomposition script
		rows = []
		for ix in range(int(minx), int(maxx)):
			column = []
			for iy in range(int(miny), int(maxy)):
				x = ix*costmap.info.resolution+costmap.info.origin.position.x
				y = iy*costmap.info.resolution+costmap.info.origin.position.y
				data = costmap.data[int(iy*costmap.info.width+ix)]
				if data == -1 or not poly.contains(Point([x,y])):
					# Unknown or not inside polygon: Treat as obstacle
					column.append(0)
				elif data <= self.costmap_max_non_lethal:
					# Freespace (non-lethal)
					column.append(-1)
				else:
					# Obstacle
					column.append(0)
			rows.append(column)

		polygons = []
		with tempfile.NamedTemporaryFile(delete=False,mode='w') as ftmp:
			ftmp.write(json.dumps(rows))
			ftmp.flush()
			
			boustrophedon_script = os.path.join(self.rospack, "scripts/boustrophedon_decomposition.rb")

			try:
				result = subprocess.run(["ruby", boustrophedon_script, ftmp.name], capture_output=True, text=True)
				polygons = json.loads(result.stdout)
			except subprocess.CalledProcessError as e:
				print("**** Error: ", e)

		ordered_polygons = self.find_connected_polygons(polygons)

		# Gom tất cả poses từ các polygon con
		all_nav2_poses = []
		for poly in ordered_polygons:
			points = [
					(
					(point[0]+minx)*costmap.info.resolution+costmap.info.origin.position.x,
					(point[1]+miny)*costmap.info.resolution+costmap.info.origin.position.y
					) for point in poly]
			polygon_poses = self.drive_polygon(Polygon(points))
			if polygon_poses:
				all_nav2_poses.extend(polygon_poses)
		
		self.get_logger().info("Boustrophedon Decomposition completed...")
		return all_nav2_poses

	def euler_to_quaternion(self, yaw, pitch, roll):
		qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
		qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
		qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
		qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
		return [qx, qy, qz, qw]

	def save_waypoints_to_yaml(self, nav2_poses):
		"""Lưu các waypoints vào file YAML."""
		waypoints_data = {
			"waypoints": [],
			"total_count": len(nav2_poses),
			"updatetime": time.time_ns()
		}
		
		for i, pose in enumerate(nav2_poses):
			waypoint = {
				"id": i,
				"position": {
					"x": float(pose.pose.position.x),
					"y": float(pose.pose.position.y),
					"z": float(pose.pose.position.z)
				},
				"orientation": {
					"w": float(pose.pose.orientation.w),
					"x": float(pose.pose.orientation.x),
					"y": float(pose.pose.orientation.y),
					"z": float(pose.pose.orientation.z)
				}
			}
			waypoints_data["waypoints"].append(waypoint)
		
		with open(self.filename, "w") as f:
			yaml.dump(waypoints_data, f, default_flow_style=False)
		
		self.get_logger().info(f"Đã lưu {len(nav2_poses)} waypoints vào {self.filename}")
		return waypoints_data

	def send_next_waypoint(self):
		"""Gửi waypoint tiếp theo trong danh sách."""
		if self.is_navigating:
			self.get_logger().warn("Robot đang di chuyển, không thể gửi waypoint mới")
			return
			
		if self.current_waypoint_index >= len(self.saved_waypoints):
			self.get_logger().info("Đã hoàn thành tất cả waypoints!")
			self.current_waypoint_index = 0
			self.saved_waypoints = []
			return
			
		# Lấy waypoint hiện tại
		current_waypoint = self.saved_waypoints[self.current_waypoint_index]
		
		# Tạo PoseStamped từ dữ liệu đã lưu
		pose = PoseStamped()
		pose.header.frame_id = self.global_frame
		pose.header.stamp = self.get_clock().now().to_msg()
		pose.pose.position.x = current_waypoint["position"]["x"]
		pose.pose.position.y = current_waypoint["position"]["y"]
		pose.pose.position.z = current_waypoint["position"]["z"]
		pose.pose.orientation.w = current_waypoint["orientation"]["w"]
		pose.pose.orientation.x = current_waypoint["orientation"]["x"]
		pose.pose.orientation.y = current_waypoint["orientation"]["y"]
		pose.pose.orientation.z = current_waypoint["orientation"]["z"]
		
		self.send_single_waypoint_to_nav2(pose)

	def send_single_waypoint_to_nav2(self, pose):
		"""Gửi một waypoint đến Nav2."""
		while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
			self.get_logger().info("'navigate_to_pose' action server not available, waiting...")

		goal_msg = NavigateToPose.Goal()
		goal_msg.pose = pose

		self.is_navigating = True
		self.get_logger().info(f'Đang gửi waypoint {self.current_waypoint_index + 1}/{len(self.saved_waypoints)} tới Nav2...')
		
		send_goal_future = self.nav_to_pose_client.send_goal_async(
			goal_msg,
			feedback_callback=self.single_nav_feedback_callback
		)
		send_goal_future.add_done_callback(self.single_nav_goal_response_callback)

	def single_nav_feedback_callback(self, feedback_msg):
		"""Callback nhận feedback từ Nav2 NavigateToPose."""
		# Có thể thêm logic xử lý feedback nếu cần
		pass

	def single_nav_goal_response_callback(self, future):
		"""Callback xử lý response khi gửi goal tới Nav2."""
		goal_handle = future.result()
		if not goal_handle or not goal_handle.accepted:
			self.get_logger().error(f'NavigateToPose goal {self.current_waypoint_index + 1} was rejected!')
			self.is_navigating = False
			return

		self.get_logger().info(f'NavigateToPose goal {self.current_waypoint_index + 1} accepted, waiting for result...')
		result_future = goal_handle.get_result_async()
		result_future.add_done_callback(self.single_nav_result_callback)

	def single_nav_result_callback(self, future):
		"""Callback xử lý kết quả từ Nav2 và gửi waypoint tiếp theo."""
		try:
			result = future.result()
			status = result.status
			
			if status == GoalStatus.STATUS_SUCCEEDED:
				self.get_logger().info(f'Waypoint {self.current_waypoint_index + 1} completed successfully!')
				self.current_waypoint_index += 1
				self.is_navigating = False
				
				# Tự động gửi waypoint tiếp theo
				self.send_next_waypoint()
				
			else:
				self.get_logger().warn(f'Waypoint {self.current_waypoint_index + 1} failed with status: {status}')
				self.is_navigating = False
				# Có thể thêm logic retry hoặc dừng lại
				
		except Exception as e:
			self.get_logger().error(f'Error getting Nav2 result: {e}')
			self.is_navigating = False

	def drive_path(self, path):
		"""
		Hàm này bây giờ chỉ làm một việc:
		Chuyển đổi một đường đi thô [(x,y),...] thành một danh sách [PoseStamped].
		"""
		self.visualize_path(path)
		nav2_poses = []
		if not path:
			return nav2_poses

		# Thêm vị trí robot hiện tại làm điểm bắt đầu
		try:
			trans = self.tfBuffer.lookup_transform(self.global_frame, self.base_frame, rclpy.time.Time())
			path.insert(0, (trans.transform.translation.x, trans.transform.translation.y))
		except TransformException as ex:
			self.get_logger().error(f'Không thể lấy vị trí robot ban đầu, hủy bỏ nhiệm vụ: {ex}')
			return [] # Trả về danh sách rỗng nếu không lấy được vị trí

		for pos_last, pos_next in pairwise(path):
			if not rclpy.ok(): 
				return []
			
			pos_diff = np.array(pos_next) - np.array(pos_last)
			angle = atan2(pos_diff[1], pos_diff[0])
			angle_quat = self.euler_to_quaternion(angle, 0, 0)
			
			# Tạo một PoseStamped cho điểm đích của đoạn đường
			pose = PoseStamped()
			pose.header.frame_id = self.global_frame
			pose.header.stamp = self.get_clock().now().to_msg()
			pose.pose.position.x = float(pos_next[0])
			pose.pose.position.y = float(pos_next[1])
			pose.pose.orientation.w = angle_quat[3]
			pose.pose.orientation.x = angle_quat[0]
			pose.pose.orientation.y = angle_quat[1]
			pose.pose.orientation.z = angle_quat[2]
			nav2_poses.append(pose)

		self.get_logger().info(f"Đã tạo thành công {len(nav2_poses)} waypoints.")
		self.visualize_path(path, False)
		return nav2_poses # Trả về danh sách để hàm gọi nó xử lý

	def add_more_waypoints(self, x1, y1, x2, y2, angle_quat, num_waypoints):	
		increment_x = (x2 - x1) / (num_waypoints + 1)
		increment_y = (y2 - y1) / (num_waypoints + 1)
		for i in range(num_waypoints):
			new_x = x1 + (i + 1) * increment_x
			new_y = y1 + (i + 1) * increment_y
			index = len(self.pose_output) + 1
			self.pose_output[index] = {
									"position":
									{
										"x": float(new_x),
										"y": float(new_y),
										"z": 0.0
									},
									"orientation":
									{
										"w": angle_quat[3],
										"x": angle_quat[0],
										"y": angle_quat[1],
										"z": angle_quat[2]
									},
								} 

	def write_pose(self, x, y, angle):
		angle_quat = self.euler_to_quaternion(angle,0,0)

		if len(self.pose_output) >= 1:
			last_index = len(self.pose_output) 
			x1 = self.pose_output[last_index]["position"]["x"]
			y1 = self.pose_output[last_index]["position"]["y"]
			distance = math.sqrt((x - x1)**2 + (y - y1)**2)
			if distance >= self.min_wp_dist:
				num_waypoints = int(distance / self.min_wp_dist) * self.num_points	
				self.get_logger().info("--x-o-x-- including (%f) points." % (num_waypoints))
				self.add_more_waypoints(x1, y1, x, y, angle_quat, num_waypoints)
			
		index = len(self.pose_output) + 1
		self.pose_output[index] = {
                                "position":
                                {
                                    "x": x,
                                    "y": y,
                                    "z": 0.0
                                },
                                "orientation":
                                {
                                    "w": angle_quat[3],
                                    "x": angle_quat[0],
                                    "y": angle_quat[1],
                                    "z": angle_quat[2]
                                },
                            } 
		self.x = x
		self.y = y

	def drive_polygon(self, polygon):
		self.visualize_cell(polygon.exterior.coords[:])
		angle = get_angle_of_longest_side_to_horizontal(polygon)
		if angle == None:
			self.get_logger().warn("Can not return polygon")
			return []
		
		angle+=pi/2 # up/down instead of left/right
		poly_rotated = rotate_polygon(polygon, angle)

		self.get_logger().debug("Rotated polygon by %.0f: %s" % (angle*180/pi, str(poly_rotated.exterior.coords[:])))

		# Chạy thuật toán bao phủ để lấy đường đi thô
		path_rotated = trapezoid_calc_path(poly_rotated, self.robot_width)
		raw_path = rotate_points(path_rotated, -angle)
		
		# Gọi drive_path để chuyển đổi đường đi thô thành danh sách PoseStamped
		nav2_poses = self.drive_path(raw_path)

		self.visualize_cell(polygon.exterior.coords[:], False)
		self.get_logger().debug("Polygon processing done")
		return nav2_poses # Trả về danh sách poses

	def private_shutdown(self):
		self.visualization_cleanup()
		"""Cancel pending task request."""
		print('Canceling current task i.e. if any.')
		if self.result_future:
			future = self.goal_handle.cancel_goal_async()
			rclpy.spin_until_future_complete(self, future)
		return

def main(args=None):
	rclpy.init(args=args)
	p = MapDrive()
	try:
		rclpy.spin(p)
	except:
		p.private_shutdown()
		p.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
    main()
