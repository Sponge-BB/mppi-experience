import copy
import unittest
from run_directed_low_speed import fixed_fixture_identity_proof


def rows():
    common=dict(raw_odom_missing=False,raw_odom_message_hash_algorithm='fnv1a64_ros1_serialized_standard',
        raw_odom_message_fnv64='123',raw_odom_header_seq=1,raw_odom_stamp_sec=1007,raw_odom_stamp_nsec=20,
        raw_odom_frame_id='odom',raw_odom_child_frame_id='base_link',raw_odom_finite=True,
        raw_odom_processed_finite=True,raw_odom_x=0.,raw_odom_y=0.,raw_odom_yaw=0.,raw_odom_body_vx=-.04,
        planner_p=[0.,0.],planner_v=[-.05,0.],planner_yaw=0.)
    records=[dict(copy.deepcopy(common),event='state_contract_'+stage) for stage in
        ('selected_boundary','desired_boundary','manager_input','frontend_init','backend_fixed')]
    records[-1]['initial_states']=[[[0.,-.05,0.,0.],[0.,0.,0.,0.]]]
    return records


class DirectedWitnessIdentity(unittest.TestCase):
    def test_complete_fixture_identity(self):self.assertTrue(fixed_fixture_identity_proof(rows()))

    def test_missing_or_duplicate_stage_rejected(self):
        data=rows()
        for index in range(len(data)):
            self.assertFalse(fixed_fixture_identity_proof(data[:index]+data[index+1:]))
        self.assertFalse(fixed_fixture_identity_proof(data+[data[-1]]))

    def test_swapped_or_missing_raw_identity_rejected(self):
        for key,value in [('raw_odom_message_fnv64','124'),('raw_odom_stamp_nsec',21),
                          ('raw_odom_frame_id','world'),('raw_odom_finite',False)]:
            data=rows();data[-1][key]=value
            self.assertFalse(fixed_fixture_identity_proof(data))
        data=rows();del data[-1]['raw_odom_message_fnv64']
        self.assertFalse(fixed_fixture_identity_proof(data))

    def test_substituted_pose_or_boundary_rejected(self):
        for key,value in [('planner_p',[.01,0.]),('planner_yaw',.01),('raw_odom_body_vx',-.03)]:
            data=rows();data[-1][key]=value
            self.assertFalse(fixed_fixture_identity_proof(data))
        data=rows();data[-1]['initial_states'][0][0][0]=.01
        self.assertFalse(fixed_fixture_identity_proof(data))


if __name__=='__main__':unittest.main()
