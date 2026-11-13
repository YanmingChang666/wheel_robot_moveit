import math
import unittest
from wheel_robot_stack.handling import (
    pickup_pose_ok, overlaps_station, relative_pose, downward_quaternion, orientation_distance,
)
from wheel_robot_stack.pattern import (
    BOX, pickup_center, pick_plan, pallet_slots, carry_pairs,
    route_distance, route_around, NoBasePose, _segment_clear,
)


class HandlingTest(unittest.TestCase):
    def test_both_hands_share_station_without_turning(self):
        left,right=pick_plan('left'),pick_plan('right')
        self.assertEqual(left.base,right.base)
        self.assertEqual(left.base,(0.,.18,0.))
        self.assertEqual((left.waist,right.waist),(0.,0.))
        self.assertAlmostEqual(left.distance,right.distance)

    def test_sixteen_boxes_are_eight_dual_trips(self):
        slots=pallet_slots()[:16]
        batches=carry_pairs(slots)
        self.assertEqual(len(batches),8)
        self.assertEqual(sorted(s.index for b in batches for s,p in b),list(range(16)))
        for batch in batches:
            self.assertEqual({p.arm for _,p in batch},{'left','right'})
            self.assertEqual(len({s.layer for s,p in batch}),1)
            self.assertGreaterEqual(batch[0][0].y,batch[1][0].y)
        self.assertEqual([b[0][0].layer for b in batches],sorted(b[0][0].layer for b in batches))

    def test_scaled_quaternion_does_not_reset_stationary_timer(self):
        a=(.55,.18,.4025,0.,0.,0.,.6601136)
        b=(.55,.18,.4025,0.,0.,0.,1.)
        self.assertAlmostEqual(orientation_distance(a,b),0.)
        self.assertAlmostEqual(orientation_distance(a,(*b[:3],0.,0.,0.,-1.)),0.)

    def test_skewed_or_unsettled_pickups_rejected(self):
        center=pickup_center()
        self.assertTrue(pickup_pose_ok((*center,0,0,0,1),center,BOX))
        for axis in ('roll','yaw'):
            a=math.radians(12)/2
            quat=(math.sin(a),0,0,math.cos(a)) if axis=='roll' else (0,0,math.sin(a),math.cos(a))
            self.assertFalse(pickup_pose_ok((*center,*quat),center,BOX))
        self.assertFalse(pickup_pose_ok((center[0]+.04,*center[1:],0,0,0,1),center,BOX))

    def test_previous_box_must_clear_spawn_volume(self):
        center=pickup_center()
        self.assertTrue(overlaps_station((*center,0,0,0,1),center,BOX))
        self.assertFalse(overlaps_station((.10,.54,.68,0,0,0,1),center,BOX))

    def test_measured_grasp_does_not_teleport_box(self):
        local=relative_pose((.54,.18,.50,1,0,0,0),(.55,.18,.4025,0,0,0,1))
        self.assertAlmostEqual(local[0],.01)
        self.assertAlmostEqual(local[2],.0975)
        self.assertAlmostEqual(local[3],-1.)

    def test_downward_rotation_compensates_chassis_heading(self):
        for yaw in (0,math.pi/2,math.pi,-math.pi/2):
            # A child with the returned local rotation must be Rx(pi) globally.
            world=(0,0,0,1,0,0,0)
            parent=(0,0,0,0,0,math.sin(yaw/2),math.cos(yaw/2))
            expected=relative_pose(parent,world)[3:]
            actual=downward_quaternion(yaw)
            self.assertAlmostEqual(abs(sum(a*b for a,b in zip(expected,actual))),1.)

    def test_route_edges_respect_obstacles(self):
        station=pick_plan('left').base
        for batch in carry_pairs(pallet_slots()[:16]):
            poses=[station,*[p.base for _,p in batch],station]
            for a,b in zip(poses,poses[1:]):
                route=route_around(a,b)
                self.assertTrue(math.isfinite(route_distance(a,b)))
                self.assertEqual(route[-1],b)
                for x,y in zip((a,*route),route):
                    self.assertTrue(_segment_clear(x,y))

    def test_side_parking_exits_outward_instead_of_returning_to_pickup(self):
        start=(-.925,1.583,-math.pi/2)
        goal=(-.159,1.943,0.)
        self.assertLess(route_distance(start,goal),1.8)
        self.assertTrue(all(point[1]>1.0 for point in route_around(start,goal)))

    def test_no_route_does_not_fall_back_to_driving_through_pallet(self):
        from wheel_robot_stack.pattern import pallet_center
        center=pallet_center()
        with self.assertRaises(NoBasePose):
            route_around(pick_plan('left').base,(center[0],center[1],0.))

    def test_pickup_station_cannot_turn_through_the_stand(self):
        station=pick_plan('left').base
        self.assertFalse(_segment_clear(station,(*station[:2],math.pi)))

    def test_odd_batch_never_duplicates_or_drops_boxes(self):
        batches=carry_pairs(pallet_slots()[:3])
        self.assertEqual(sorted(s.index for b in batches for s,p in b),[0,1,2])
        self.assertEqual(sorted(len(b) for b in batches),[1,2])


if __name__=='__main__':
    unittest.main()
