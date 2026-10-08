"""Physical lever-arm counterexamples using the recorded nominal mount model."""
import copy
import math
from types import SimpleNamespace as S
import pytest
from super_lio_vehicle_adapter.adapter_node import control_odometry_from_legacy, _qrotate
from super_lio_vehicle_adapter.control_reference_lock import (
    IMU_TO_CONTROL_TRANSLATION_M as R, CONTROL_CHILD_FRAME)


def legacy_at_control(x=2., y=-1., yaw=0., v=0., w=.35):
    q=(0.,0.,math.sin(yaw/2),math.cos(yaw/2))
    offset=_qrotate(q,R)
    cov=[.01 if i//6==i%6 else 0. for i in range(36)]
    return S(header=S(frame_id='odom',stamp=S(sec=1,nanosec=23)),child_frame_id='base_footprint',
        pose=S(pose=S(position=S(x=x-offset[0],y=y-offset[1],z=-offset[2]),
                      orientation=S(x=q[0],y=q[1],z=q[2],w=q[3])),covariance=cov),
        twist=S(twist=S(linear=S(x=v+w*R[1],y=-w*R[0],z=0.),
                        angular=S(x=0.,y=0.,z=w)),covariance=cov.copy()))


def test_pure_chassis_rotation_moves_imu_but_keeps_control_pivot_and_translation_zero():
    before=legacy_at_control(yaw=0.);after=legacy_at_control(yaw=math.pi/2)
    assert math.hypot(before.twist.twist.linear.x,before.twist.twist.linear.y)>.05
    assert math.dist((before.pose.pose.position.x,before.pose.pose.position.y),
                     (after.pose.pose.position.x,after.pose.pose.position.y))>.21
    original=copy.deepcopy(before)
    for msg in (before,after):
        control=control_odometry_from_legacy(msg)
        assert control.child_frame_id==CONTROL_CHILD_FRAME and control.header==msg.header
        assert (control.pose.pose.position.x,control.pose.pose.position.y,control.pose.pose.position.z)==pytest.approx((2.,-1.,0.))
        assert (control.twist.twist.linear.x,control.twist.twist.linear.y)==pytest.approx((0.,0.))
        assert control.twist.twist.angular.z==.35
    assert before==original


def test_control_pose_and_twist_retain_attitude_lever_covariance_cross_terms():
    msg=legacy_at_control(yaw=math.pi/2,v=.2)
    control=control_odometry_from_legacy(msg)
    # At yaw90, the world lever x=-r_y; yaw error displaces y by x*d_yaw.
    assert control.pose.covariance[11]==pytest.approx(-R[1]*.01)
    assert control.pose.covariance[31]==pytest.approx(control.pose.covariance[11])
    assert control.pose.covariance[7]>.01
    # Body-frame yaw uncertainty produces dv_x=-r_y*d_omega_z.
    assert control.twist.covariance[5]==pytest.approx(-R[1]*.01)
    assert control.twist.covariance[30]==pytest.approx(control.twist.covariance[5])
    assert control.twist.twist.linear.x==pytest.approx(.2)


def test_control_conversion_rejects_already_shifted_or_wrong_frame_state():
    msg=legacy_at_control();msg.child_frame_id=CONTROL_CHILD_FRAME
    with pytest.raises(ValueError):control_odometry_from_legacy(msg)
    msg=legacy_at_control();msg.header.frame_id='map'
    with pytest.raises(ValueError):control_odometry_from_legacy(msg)
