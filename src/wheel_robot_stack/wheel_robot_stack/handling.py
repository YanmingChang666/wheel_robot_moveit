"""Measured-box checks shared by pickup, transport and release."""
import math


def rotation_matrix(pose):
    if len(pose) != 7 or not all(math.isfinite(v) for v in pose):
        raise ValueError('箱子反馈缺少有效 XYZ 和四元数')
    x, y, z, w = pose[3:]
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if norm < 1e-6:
        raise ValueError('箱子四元数无效')
    x,y,z,w = (v/norm for v in (x,y,z,w))
    return ((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),
            (2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),
            (2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)))


def pose_angles(pose):
    matrix = rotation_matrix(pose)
    tilt = math.acos(max(-1., min(1., matrix[2][2])))
    yaw = math.atan2(matrix[1][0], matrix[0][0])
    # Rectangles have the same footprint after a half turn.
    yaw_error = abs((yaw+math.pi/2) % math.pi-math.pi/2)
    return tilt, yaw_error


def box_half_extents(pose, size):
    matrix = rotation_matrix(pose)
    return tuple(sum(abs(matrix[i][j])*size[j]/2 for j in range(3)) for i in range(3))


def overlaps_station(pose, station, size, margin=.04):
    half = box_half_extents(pose, size)
    return all(abs(pose[i]-station[i]) < half[i]+size[i]/2+margin for i in range(3))


def pickup_pose_ok(pose, station, size):
    tilt, yaw = pose_angles(pose)
    return (tilt <= math.radians(5) and yaw <= math.radians(5)
            and math.hypot(pose[0]-station[0], pose[1]-station[1]) <= .025
            and abs(pose[2]-station[2]) <= .015)


def relative_pose(parent, child):
    """Child xyz/xyzw in parent's frame, preserving the measured grasp."""
    matrix = rotation_matrix(parent)
    rotation_matrix(child)
    d = [child[i]-parent[i] for i in range(3)]
    xyz = tuple(sum(matrix[j][i]*d[j] for j in range(3)) for i in range(3))
    def unit(q):
        n=math.sqrt(sum(v*v for v in q))
        return tuple(v/n for v in q)
    px,py,pz,pw=unit(parent[3:])
    x,y,z,w=unit(child[3:])
    a,b,c,d=-px,-py,-pz,pw
    quat=(d*x+a*w+b*z-c*y, d*y-a*z+b*w+c*x,
          d*z+a*y-b*x+c*w, d*w-a*x-b*y-c*z)
    return xyz+quat


def downward_quaternion(base_yaw):
    """Base-frame tool rotation giving Rx(pi) in the world frame."""
    return (math.cos(base_yaw/2), -math.sin(base_yaw/2), 0., 0.)


def orientation_distance(a, b):
    """Angular difference independent of quaternion magnitude and sign."""
    rotation_matrix(a)
    rotation_matrix(b)
    qa,qb=a[3:],b[3:]
    divisor=math.sqrt(sum(v*v for v in qa)*sum(v*v for v in qb))
    cosine=abs(sum(x*y for x,y in zip(qa,qb)))/divisor
    return 2*math.acos(max(0.,min(1.,cosine)))
