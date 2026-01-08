from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
   # 修改后：显式指定加载 ompl 管线，这会自动绑定正确的 OMPL 插件
  moveit_config = (
    MoveItConfigsBuilder("fairino16_v6_robot", package_name="fairino16_v6_moveit2_config")
    .planning_pipelines(pipelines=["ompl"]) 
    .to_moveit_configs()
)
