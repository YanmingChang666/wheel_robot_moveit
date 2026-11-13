"""ROS message-level regressions; source Humble and the workspace before running."""
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
from wheel_robot_stack.stack_boxes import StackBoxes
from wheel_robot_stack.pattern import pallet_slots, carry_pairs
from moveit_msgs.msg import PlanningScene, CollisionObject


class CycleTest(unittest.TestCase):
    def test_sixteen_picks_do_not_drive_between_hands(self):
        events=[]
        node=NS(
            get_parameter=lambda name:NS(value={'reach':.40,'max_boxes':16,'execute':True}[name]),
            get_logger=lambda:Mock(),wait_for_servers=lambda:True,_relax_start_tolerance=Mock(),
            _load_srdf_pairs=Mock(),_publish_collision_matrix=lambda:True,_use=Mock(),
            set_suction=Mock(),spawn=Mock(),sleep=Mock(),wait_scene=lambda timeout:0,
            _wait_base=lambda timeout:True,base_pose=(0.,0.,0.),_sync_scene=lambda:True,
            _pick_from_stand=lambda arm: events.append(('pick',arm)) or True,
            _place_held=lambda slot,plan: events.append(('place',slot.index)) or True,
            _spawn_box=lambda index: events.append(('spawn',index)) or True,
            _drive_base=lambda *base:events.append(('drive',base)) or True,
        )
        StackBoxes.run(node)
        picks=[i for i,e in enumerate(events) if e[0]=='pick']
        self.assertEqual(len(picks),16)
        for a,b in zip(picks[::2],picks[1::2]):
            self.assertEqual(events[a],('pick','left'))
            self.assertEqual(events[b],('pick','right'))
            self.assertFalse(any(e[0]=='drive' for e in events[a+1:b]))
            self.assertEqual([e[0] for e in events[a+1:b]],['spawn'])
        self.assertEqual(sum(e[0]=='place' for e in events),16)
        self.assertEqual([e[1] for e in events if e[0]=='spawn'],list(range(1,16)))
        self.assertEqual(events[-1][0],'drive')
        self.assertEqual({e[1] for e in events if e[0]=='drive'},{(0.,.18,0.)})

    def test_physical_box_id_survives_out_of_order_slot_assignment(self):
        placed=CollisionObject();placed.id='placed_3'
        scene=PlanningScene();scene.world.collision_objects=[placed]
        arm=NS(carried='carried_right',tool='tool0_R',box_index=3,holding=True)
        node=NS(_arm=lambda:arm,_into_base=lambda x,y:(x,y),
                _box=StackBoxes._box.__get__(NS()),_call_scene=Mock(return_value=True),
                get_scene=NS(wait_for_service=lambda **kw:True,
                             call_async=lambda req:NS(result=lambda:NS(scene=scene))),
                world_boxes=[],scene_ids=[],_publish_collision_matrix=lambda:True,
                get_logger=lambda:Mock())
        with patch('wheel_robot_stack.stack_boxes.rclpy.spin_until_future_complete'):
            self.assertTrue(StackBoxes._detach_at(node,7,(.1,.2,.3)))
        self.assertEqual(node.scene_ids,['placed_3'])
        self.assertEqual(node.world_boxes[0][0],'placed_3')
        self.assertIsNone(arm.box_index)
        self.assertFalse(arm.holding)

    def test_pick_method_refuses_to_reposition_between_hands(self):
        node=NS(_use=Mock(),pickup_base=(0.,.18,0.),base_pose=(1.,0.,0.),
                get_logger=lambda:Mock(),_drive_base=Mock())
        self.assertFalse(StackBoxes._pick_from_stand(node,'right'))
        node._drive_base.assert_not_called()

    def test_second_pick_uses_measured_parking_anchor(self):
        # 16 mm parking error plus 14 mm passive drift must not be mistaken
        # for a commanded move between the two hands.
        node=NS(_use=Mock(),pickup_base=(0.,.18,0.),pickup_anchor=(0.,.164,0.),
                base_pose=(0.,.150,0.),get_logger=lambda:Mock(),
                _wait_pickup_ready=Mock(return_value=False),ready_index=1)
        self.assertFalse(StackBoxes._pick_from_stand(node,'right'))
        node._wait_pickup_ready.assert_called_once_with(1)

    def test_tilted_fourth_box_is_not_accepted(self):
        import math
        from rclpy.duration import Duration
        class Time:
            def __init__(self,value): self.value=value
            def __add__(self,d): return Time(self.value+d.nanoseconds/1e9)
            def __sub__(self,other): return Duration(seconds=self.value-other.value)
            def __lt__(self,other): return self.value<other.value
        clock=NS(seconds=0.)
        pose=(.50,.18,.4025,math.sin(.15),0.,0.,math.cos(.15))
        logger=Mock()
        node=NS(get_clock=lambda:NS(now=lambda:Time(clock.seconds)),
                box_pose_serial=0,box_poses={3:pose},_poses_fresh=lambda:True,
                get_logger=lambda:logger)
        def sleep(seconds):
            clock.seconds+=seconds
            node.box_pose_serial+=1
        node.sleep=sleep
        with patch('wheel_robot_stack.stack_boxes.rclpy.ok',return_value=True):
            self.assertFalse(StackBoxes._wait_pickup_ready(node,3,timeout=.8))
        self.assertTrue(logger.error.called)
        # A level new box must remain stationary for at least half a second.
        node.box_poses[3]=(.50,.18,.4025,0.,0.,0.,.6603)
        before=clock.seconds
        with patch('wheel_robot_stack.stack_boxes.rclpy.ok',return_value=True):
            self.assertTrue(StackBoxes._wait_pickup_ready(node,3,timeout=1.))
        self.assertGreaterEqual(clock.seconds-before,.5)

    def test_occupied_station_does_not_publish_spawn(self):
        node=NS(_wait_station_clear=lambda:False,spawn=Mock())
        self.assertFalse(StackBoxes._spawn_box(node,3))
        node.spawn.publish.assert_not_called()

    def test_loaded_arm_bias_is_compensated_instead_of_repeating_target(self):
        state=NS(position=NS(x=0.,y=.5,z=.5))
        def solve(x,y,z,*args,**kwargs): return {'x':x,'y':y,'z':z}
        def follow(goal,speed):
            state.position.x=goal['x']-.022
            state.position.y=goal['y'];state.position.z=goal['z']
            return True
        node=NS(_lift_now=lambda:.1,_solve_arm=Mock(side_effect=solve),
                _follow_arm=Mock(side_effect=follow),_measured_tool=lambda:state,
                _current_arm=lambda:{},get_logger=lambda:Mock())
        self.assertTrue(StackBoxes._aim_arm(node,.19,.5,.5,.5,correct_bias=True))
        self.assertAlmostEqual(state.position.x,.19)
        self.assertEqual(node._follow_arm.call_count,2)

    def test_large_position_error_is_not_treated_as_small_load_bias(self):
        state=NS(position=NS(x=.11,y=.5,z=.5))
        node=NS(_lift_now=lambda:.1,_solve_arm=Mock(return_value={}),
                _follow_arm=Mock(return_value=True),_measured_tool=lambda:state,
                _current_arm=lambda:{},get_logger=lambda:Mock())
        self.assertTrue(StackBoxes._aim_arm(node,.19,.5,.5,.5,correct_bias=True))
        self.assertEqual(node._solve_arm.call_count,1)
        self.assertGreater(abs(state.position.x-.19),.04)

    def test_contact_servo_uses_actual_query_and_integrates_tracking_error(self):
        import math
        box=(.50,.18,.4025,0.,0.,0.,1.)
        point=[.50,.18,.450,math.sqrt(.5),0.,-math.sqrt(.5),0.]
        state=NS(lift=.1)
        def follow(target,speed):
            new=target-.011
            point[2]+=new-state.lift
            state.lift=new
            return True
        node=NS(side='left',pick_prim=0,box_poses={0:box,-1:point},
                _lift_now=lambda:state.lift,_follow_lift=Mock(side_effect=follow),
                _poses_fresh=lambda:True,get_logger=lambda:Mock(),
                _measured_tool=lambda:NS(position=NS(z=.7)))
        self.assertTrue(StackBoxes._align_suction_contact(node))
        self.assertAlmostEqual(point[2]-(box[2]+.0525),.006)
        self.assertEqual(node._follow_lift.call_count,2)

    def test_world_to_base_quaternion_has_correct_sign(self):
        import math
        node=NS(base_pose=(0.,0.,math.pi/2))
        result=StackBoxes._quat_into_base(node,(1.,0.,0.,0.))
        self.assertAlmostEqual(result[0],math.sqrt(.5))
        self.assertAlmostEqual(result[1],-math.sqrt(.5))


if __name__=='__main__':
    unittest.main()
