import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Pose  # Pose 클래스를 가져옴
from std_msgs.msg import Header

class PublishPoses(Node):

    def __init__(self):
        super().__init__('publish_poses')

        # 콜백 그룹 생성
        self.create_timer(1, self.timer_callback)

        self.goal_pose_publisher = self.create_publisher(PoseArray, 'goal_poses', 1)
        self.header = Header()
        self.poses = []  # 여러 개의 목표 위치를 저장하는 리스트

        self.add_goal_pose(1.0, 2.0)
        self.add_goal_pose(3.0, 4.0)


    def add_goal_pose(self, x, y):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        self.poses.append(pose)

    def publish_goal_poses(self):
        goal_poses_msg = PoseArray()
        goal_poses_msg.header.frame_id = "map"
        goal_poses_msg.poses = self.poses
        self.goal_pose_publisher.publish(goal_poses_msg)

    def timer_callback(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    publish_poses_node = PublishPoses()
    rclpy.spin(publish_poses_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()