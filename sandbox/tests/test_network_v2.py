"""Tests for the second network (sewer_network_v2) and for the generator's LiDAR-safety rule (no ROS, no Gazebo)."""
import os
import sys

import pytest
import yaml

import sim2d
from sewer_localization.map_utils import OccupancyMap
from sewer_planning.graph import Graph
from sewer_planning.route_cost import CostParams, RouteModel

CFG = os.path.join(sim2d.GAZEBO_PKG, 'config')
sys.path.insert(0, os.path.join(sim2d.GAZEBO_PKG, 'scripts'))
import gen_network  # noqa: E402


@pytest.fixture(scope='module')
def model2():
    graph = Graph.load(os.path.join(CFG, 'network_graph_v2.yaml'))
    prior = OccupancyMap.load(os.path.join(sim2d.GAZEBO_PKG, 'maps', 'sewer_map_v2'))
    return RouteModel(graph, prior, CostParams())


def status(*edges):
    return {e: dict(gap=0.0, blocked=True, s_min=3.5, s_max=4.0, offset=0.0, observed=1.0, cells=50) for e in edges}


def best(model, blocked):
    return model.plan(('node', 'A'), status(*blocked), 0.95, model.initial_covariance(), 'J4')[0]['nodes']


def test_v2_routes_follow_the_known_blockages(model2):
    assert best(model2, []) == ['A', 'J1', 'J4']
    assert best(model2, ['J1-J4']) == ['A', 'J1', 'J2', 'M', 'J3', 'J4']
    assert best(model2, ['J1-J4', 'M-J3']) == ['A', 'J1', 'J2', 'M', 'E', 'F', 'J3', 'J4']


def test_v2_adds_a_node_a_loop_and_a_blockage_and_leaves_classic_alone():
    classic = yaml.safe_load(open(os.path.join(CFG, 'network_graph.yaml')))
    v2 = yaml.safe_load(open(os.path.join(CFG, 'network_graph_v2.yaml')))
    assert 'F' not in classic['nodes'] and v2['nodes']['F']['x'] == 16.0 and v2['nodes']['F']['y'] == 10.0
    assert v2['nodes']['E']['type'] == 'bend'                 # no longer a dead end
    t_classic = yaml.safe_load(open(os.path.join(CFG, 'blockages_truth.yaml')))['blockages']
    t_v2 = yaml.safe_load(open(os.path.join(CFG, 'blockages_truth_v2.yaml')))['blockages']
    assert sorted(t_classic) == ['B1', 'B2'] and sorted(t_v2) == ['B1', 'B2', 'B3']
    assert t_v2['B3']['edge'] == ['M', 'J3'] and t_v2['B3']['blocks_route']


def test_v2_map_has_the_same_outline_as_classic():
    a = OccupancyMap.load(os.path.join(sim2d.GAZEBO_PKG, 'maps', 'sewer_map'))
    b = OccupancyMap.load(os.path.join(sim2d.GAZEBO_PKG, 'maps', 'sewer_map_v2'))
    assert (a.w, a.h, a.x0, a.y0) == (b.w, b.h, b.x0, b.y0)       # cameras and the RViz overview still fit


def test_decoration_may_not_reach_into_the_lidar_band_above_free_space():
    x0, y0, free, wall = gen_network.build_grid(gen_network.V2)
    d = gen_network.Decor(x0, y0, free)
    d.box('floor', 4.0, 0.0, 0.002, 1.0, 1.0, 0.004, (0.3, 0.3, 0.3, 1))          # below the band: fine
    d.box('ladder', 4.0, 0.0, 0.6, 0.03, 0.03, 0.6, (0.5, 0.5, 0.5, 1))           # above the band: fine
    with pytest.raises(ValueError):
        d.box('mast', 4.0, 0.0, 0.13, 0.05, 0.05, 0.2, (1, 0, 0, 1))              # in the corridor, across z = 0.13
    with pytest.raises(ValueError):
        d.cyl('pipe', 4.0, 0.0, 0.2, 0.1, 0.4, (1, 0, 0, 1), axis='y')
    d.box('in_wall', 4.0, -0.6, 0.13, 0.2, 0.1, 0.2, (0.2, 0.2, 0.2, 1))          # inside a wall, same height: hidden, fine


def test_generated_worlds_keep_the_world_name_and_have_no_decorative_collisions():
    for name in ('sewer_network', 'sewer_network_v2'):
        text = open(os.path.join(sim2d.GAZEBO_PKG, 'worlds', name + '.sdf')).read()
        assert '<world name="sewer_network">' in text            # the contact-sensor bridge depends on it
        decor = text.split('<model name="decor">')[1].split('</model>')[0]
        assert '<collision' not in decor and '<visual' in decor
