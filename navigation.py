from Kmath import *
#默认弧度制
#春分点方向向量
def vernal_equinox_vector(conn,reference_frame):
    earth=conn.space_center.bodies['Earth']
    orbital_reference_frame=earth.orbital_reference_frame
    north_pole=Vector3.Tuple3(earth.direction(orbital_reference_frame))
    tmp=Vector3.Cross(Vector3(0.0,0.0,1.0),north_pole)
    q=Quaternion.Tuple4(earth.rotation(reference_frame))
    q=q*Quaternion.Tuple4(earth.rotation(orbital_reference_frame)).inverse()
    return q.rotate(tmp)

#轨道法向量
def target_normal_vector(conn,body,inc,lan,reference_frame):
    north_pole=Vector3.Tuple3(body.direction(reference_frame)).unit_vector()
    vernal_vector=vernal_equinox_vector(conn,reference_frame)
    q=Quaternion.PivotRad(north_pole,-lan)
    ascend_node=q.rotate(vernal_vector)
    tmp=Vector3.Cross(north_pole,ascend_node).unit_vector()
    if(abs(inc-math.pi/2)<1e-16):
        res=tmp
    else:
        res= north_pole+(tmp*math.tan(inc))
    return res.unit_vector()

#某倾角过某位置的轨道赤道点(升交点或降交点)经度
def equator_node_lon(lon,lat,inc,is_ascend=True):
    b=math.tan(lat)*math.tan(math.pi/2-inc)
    b=math.asin(min(1,max(-1,b)))
    if is_ascend:
        return lon-b
    else:
        return lon+b

#到轨道凌空的时间
def time_to_orbit_over(conn,body,lon,lat,inc,lan,heading_north=False):
    vev=vernal_equinox_vector(conn,body.reference_frame).tuple3()
    tarLon=math.radians(body.longitude_at_position(vev,body.reference_frame))+lan
    curLon=equator_node_lon(lon,lat,inc,heading_north)
    if not heading_north:
        tarLon=math.pi+tarLon
    dif=tarLon-curLon
    dif=normalized_rad(dif)
    if(dif<0):
        dif=dif+2*math.pi
    return dif/body.rotational_speed

#加速到发射
def warp_to_launch(conn,target_inc,target_lan,heading_north=False,advance=300):
    vessel = conn.space_center.active_vessel
    body=vessel.orbit.body
    reference_frame=body.non_rotating_reference_frame
    position=vessel.position(reference_frame)
    lon=math.radians(body.longitude_at_position(position, reference_frame))  
    lat=math.radians(body.latitude_at_position(position, reference_frame))
    deltaTime=time_to_orbit_over(conn,body,lon,lat,target_inc,target_lan,heading_north)
    conn.space_center.warp_to(conn.space_center.ut+deltaTime-advance)

def surface_velocity(vessel,reference_frame):
    bref=vessel.orbit.body.reference_frame
    q1=Quaternion.Tuple4(vessel.rotation(bref))
    q2=Quaternion.Tuple4(vessel.rotation(reference_frame))
    q1=q2*q1.inverse()
    v=Vector3.Tuple3(vessel.velocity(bref))
    v=q1.rotate(v)
    return v

def target_heading(vessel,target_normal_vector,reference_frame):
    body=vessel.orbit.body
    orbitV=target_normal_vector
    NorthNode=Vector3.Tuple3(body.direction(reference_frame))
    vesselPos=Vector3.Tuple3(vessel.position(reference_frame))
    front=Vector3.Cross(vesselPos-(orbitV*math.cos(Vector3.Angle(orbitV,vesselPos))),orbitV)
    east=Vector3.Cross(vesselPos,NorthNode) 
    res=Vector3.Angle(front,east)
    if Vector3.Dot(front,NorthNode)<0:
        res=-res
    return math.pi/2-res
