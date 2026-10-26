# 元のファイル: AI-Robot-Book-Humble/crane_plus（コミット 19c1d60）の crane_plus_gazebo/launch/crane_plus_with_table.launch.py
#   （Apache License 2.0。元のパッケージは変えず、このファイルだけを複製して変えた）
# 変えた点: ワールドの差し替え、手先への DetachableJoint の注入、ブリッジの追加、MoveIt を立てない選択肢（moveit:=0）。
# 使い方: ros2 launch /workspace/bombsort/bombsort.launch.py world:=/workspace/bombsort/bombsort_v6m.sdf gui:=1
import os
import re

from ament_index_python.packages import get_package_share_directory
from crane_plus_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription, LaunchContext
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

BOMBS = ['bomb_red', 'bomb_black']


def bombs_in(world):
    # 物体の一覧はワールドの bomb_ で始まるモデルから取る（bombsort_v6m は6個）。見つからなければ BOMBS の2個
    try:
        names = re.findall(r'<model name="(bomb_[^"]+)"', open(world, encoding='utf-8').read())
    except OSError:
        names = []
    return names or BOMBS


def detachable(child):
    # 固定先は手首（link4。固定側の指が付いている）。link_hand は動く側の指で、そこに固定すると
    # 開閉のたびに物体が指と一緒に動き、反対側の指を通り抜けた（開発の途中で目視で判明）
    return f'''
  <gazebo>
    <plugin filename="ignition-gazebo-detachable-joint-system" name="ignition::gazebo::systems::DetachableJoint">
      <parent_link>crane_plus_link4</parent_link>
      <child_model>{child}</child_model>
      <child_link>link</child_link>
      <attach_topic>/{child}/attach</attach_topic>
      <detach_topic>/{child}/detach</detach_topic>
      <output_topic>/{child}/attach_state</output_topic>
    </plugin>
  </gazebo>'''


def wrist_cam(pose, fov):
    # 手首（link4）に付けるカメラ（試した版で使った。提出の版では使わない）。link4 の +z が指先の向き（指先は z≈0.09）。
    # SDF のカメラは +x を向くので、pose の回転で指先のあたりへ向ける
    return f'''
  <gazebo reference="crane_plus_link4">
    <sensor name="wrist_cam" type="camera">
      <pose>{pose}</pose>
      <camera>
        <horizontal_fov>{fov}</horizontal_fov>
        <image><width>128</width><height>128</height><format>R8G8B8</format></image>
        <clip><near>0.01</near><far>3</far></clip>
      </camera>
      <always_on>1</always_on>
      <update_rate>15</update_rate>
      <topic>bs_wrist</topic>
    </sensor>
  </gazebo>'''


def setup(context):
    world = LaunchConfiguration('world').perform(context)
    bombs = bombs_in(world)
    gui = LaunchConfiguration('gui').perform(context) == '1'
    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.environ['LD_LIBRARY_PATH'],
           'IGN_GAZEBO_RESOURCE_PATH': os.path.dirname(
               get_package_share_directory('crane_plus_description'))}
    gui_config = os.path.join(get_package_share_directory('crane_plus_gazebo'), 'gui', 'gui.config')
    cmd = f'ign gazebo -r {world} --gui-config {gui_config}' if gui else f'ign gazebo -r -s {world}'
    ign_gazebo = ExecuteProcess(cmd=[cmd], output='screen', additional_env=env, shell=True)

    loader = RobotDescriptionLoader()
    loader.use_gazebo = 'true'
    loader.use_ignition = 'true'
    loader.gz_control_config_package = 'crane_plus_control'
    loader.gz_control_config_file_path = 'config/crane_plus_controllers.yaml'
    desc = loader.load().perform(context)
    # 環境変数 BS_WRIST（"x y z r p y"）があるときだけ手首カメラを付ける。BS_WRIST_FOV は水平の画角（rad）
    wrist = wrist_cam(os.environ['BS_WRIST'], os.environ.get('BS_WRIST_FOV', '1.2')) if os.environ.get('BS_WRIST') else ''
    desc = desc.replace('</robot>', ''.join(detachable(b) for b in bombs) + wrist + '\n</robot>')

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-string', desc, '-name', 'crane_plus', '-z', '1.015', '-allow_renaming', 'true'])
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('crane_plus_moveit_config'), '/launch/run_move_group.launch.py']),
        launch_arguments={'loaded_description': desc}.items())
    spawners = [ExecuteProcess(cmd=[f'ros2 run controller_manager spawner {c}'], shell=True, output='screen')
                for c in ['joint_state_broadcaster', 'crane_plus_arm_controller', 'crane_plus_gripper_controller']]
    bridge_args = ['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                   '/bs_camera@sensor_msgs/msg/Image[ignition.msgs.Image',
                   '/bs_camera2@sensor_msgs/msg/Image[ignition.msgs.Image',
                   '/bs_wrist@sensor_msgs/msg/Image[ignition.msgs.Image',
                   '/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose']
    for b in bombs:
        bridge_args += [f'/{b}/attach@std_msgs/msg/Empty]ignition.msgs.Empty',
                        f'/{b}/detach@std_msgs/msg/Empty]ignition.msgs.Empty',
                        f'/{b}/attach_state@std_msgs/msg/String[ignition.msgs.StringMsg',
                        f'/model/{b}/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose']
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=bridge_args, output='screen')
    # moveit:=0 で move_group（と rviz2）を立てない。評価の途中で CPU を取り合い、推論が 20 Hz に収まらなかったため
    # gz_ros2_control は robot_state_publisher を待つ（move_group の launch が立てていた）ので、そのときは単独で立てる
    moveit = LaunchConfiguration('moveit').perform(context) == '1'
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
               parameters=[{'robot_description': desc}])
    return [ign_gazebo, spawn, move_group if moveit else rsp, *spawners, bridge]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='/workspace/bombsort/bombsort.sdf'),
        DeclareLaunchArgument('gui', default_value='1'),
        DeclareLaunchArgument('moveit', default_value='1'),
        SetParameter(name='use_sim_time', value=True),
        OpaqueFunction(function=setup),
    ])
