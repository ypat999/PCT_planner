from .scene import ScenePCD, SceneMap, SceneTrav


class Scene1F():
    pcd = ScenePCD()
    pcd.file_name = '1f_down10.pcd'

    map = SceneMap()
    map.resolution = 0.5
    map.ground_h = -10.0
    map.slice_dh = 1.0

    trav = SceneTrav()
    trav.kernel_size = 1
    trav.interval_min = 0.50
    trav.interval_free = 0.65
    trav.slope_max = 0.70
    trav.step_max = 0.40
    trav.standable_ratio = 0.20
    trav.cost_barrier = 50.0
    trav.safe_margin = 0.3
    trav.inflation = 0.2
