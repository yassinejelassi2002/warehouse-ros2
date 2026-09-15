import math
import unittest

from warehouse_marl.warehouse_graph import WarehouseGraph, build_demo_warehouse


class TestWarehouseGraph(unittest.TestCase):
    def test_world_layout_is_translated_into_map_frame(self):
        graph = build_demo_warehouse(map_origin_x=-2.0, map_origin_y=-1.0)

        self.assertEqual(len(graph.G.nodes), 18)
        self.assertEqual(len(graph.G.edges), 50)
        self.assertEqual(graph.G.nodes['shelf_0_0']['x'], 2.0)
        self.assertEqual(graph.G.nodes['shelf_0_0']['y'], 6.3)
        self.assertTrue(math.isclose(graph.G.nodes['charging_dock']['x'], -0.15))
        self.assertEqual(graph.G.nodes['charging_dock']['y'], 1.0)

    def test_every_target_is_connected_and_blocked_edges_are_avoided(self):
        graph = build_demo_warehouse()
        path, cost = graph.shortest_path('station_A', 'shelf_4_2')

        self.assertEqual(path[0], 'station_A')
        self.assertEqual(path[-1], 'shelf_4_2')
        self.assertTrue(math.isfinite(cost))

        graph.set_blocked('shelf_0_0', 'shelf_0_1', True)
        detour, detour_cost = graph.shortest_path('shelf_0_0', 'shelf_0_2')
        self.assertNotIn(('shelf_0_0', 'shelf_0_1'), zip(detour, detour[1:]))
        self.assertTrue(math.isfinite(detour_cost))
        self.assertGreater(detour_cost, 7.0)

    def test_json_round_trip_preserves_dynamic_state(self):
        graph = build_demo_warehouse()
        graph.set_node_occupancy('shelf_2_1', 'robot1', True)
        graph.update_congestion('shelf_2_1', 'shelf_2_2', 1.25)
        graph.set_blocked('shelf_3_0', 'shelf_3_1', True)

        restored = WarehouseGraph.from_json(graph.to_json())

        self.assertEqual(restored.G.nodes['shelf_2_1']['occupied_by'], ['robot1'])
        self.assertEqual(restored.G['shelf_2_1']['shelf_2_2']['congestion'], 1.25)
        self.assertIs(restored.G['shelf_3_0']['shelf_3_1']['blocked'], True)
