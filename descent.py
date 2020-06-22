import krpc
import time
import Ktimer
from peg import *
conn = krpc.connect()
vessel = conn.space_center.active_vessel
body=vessel.orbit.body
#reference_frame=vessel.orbit.body.non_rotating_reference_frame
reference_frame=vessel.orbit.body.reference_frame
north_pole=Vector3.Tuple3(body.direction(reference_frame)).unit_vector()
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')

#radiu of the orbit injection point ,180000 is the altitude
equatorial_radius=body.equatorial_radius
hover_height=100
target_altitude= 8154
target_radius =target_altitude+hover_height+equatorial_radius
target_lon=23.7
target_lat=89
target_position=Vector3.Tuple3(body.position_at_altitude(target_lat,target_lon,target_altitude+hover_height,reference_frame))

target_speed=3
target_angle=math.radians(-150)
init_pos=Vector3.Tuple3(vessel.position(reference_frame))

#the follow yaw can work for gravity turn
target_normal_vector=Vector3.Cross(init_pos,target_position).unit_vector()
yaw=math.degrees(target_heading(vessel,target_normal_vector,reference_frame))





#initialize the peg
peg=pegas(conn)
peg.set_reference_frame(reference_frame)
peg.set_target(0,0,target_radius,target_speed,target_angle)
peg.set_target_normal_vector(target_normal_vector)
#the function 'update_stages()' only work for those rocket with out any auxiliary booster
#it obtains the dry mass,total mass,specific ,impulse and thrust of each stages
#run it when initialize the peg or when the dry mass of the rock changes such as just droped fairings
#need run it when the acceleration exceeds 4.5g 
klim=0.05
thrust_lost=1.0-klim
peg.update_stages(thrust_lost)
peg.vehicle_info()


#the peg works like this
#In general the algorithm will diverge when tgo <3
pitch=0.0
yaw=0.0
angle_to_rd=0.0
vessel.auto_pilot.engage()
vessel.auto_pilot.stopping_time=(0.5,0.5,0.5)
vessel.control.rcs=True
vessel.auto_pilot.target_roll=0
difi=Ktimer.integrator()
difi.set_max(klim)
ignition=False

while altitude()>target_altitude+hover_height:
    py=peg.update()
    tgo=peg.time_to_go()
    if py!=None and tgo>3:
        pitch=math.degrees(py[0])
        yaw=math.degrees(py[1])

    angle_to_rd=Vector3.Angle(Vector3.Tuple3(vessel.position(reference_frame)),target_position)
    dis_to_rd=angle_to_rd*equatorial_radius
    rd=peg.rd_position()
    sn=sign(Vector3.Dot(Vector3.Cross(rd,target_position),target_normal_vector))
    dis=sn*Vector3.Angle(rd,target_position)*equatorial_radius
    
    if dis<0 and ignition==False:
        ignition=True
        vessel.control.activate_next_stage()

    p=dis*0.001
    i=difi.integral(0.1*p)
    pi=max(min(p+i,klim),-klim)
    if tgo<10:
        angle_to_rd=0.0
        pi=0.0
    vessel.control.throttle=thrust_lost-pi
    pitch=math.degrees(math.asin(min(math.sin(math.radians(pitch)+angle_to_rd)/(1.0+pi/thrust_lost),1.0))-angle_to_rd)
    vessel.auto_pilot.target_pitch_and_heading(pitch,yaw)

    print('tgo:%f pitch:%f yaw:%f dis: %f throttle:%f dis_to_rd:%f'%(tgo,pitch,yaw,dis,thrust_lost+pi,dis_to_rd),end='\r')

    #print('tgo:%f pitch:%f yaw:%f dis: %f throttle:%f dis_to_rd:%f'%(tgo,pitch,yaw,dis,thrust_lost+pi,dis_to_rd))

vessel.control.throttle=0.01
vessel.auto_pilot.target_pitch_and_heading(90,yaw)
vessel.auto_pilot.stopping_time=(5,5,5)

while vessel.flight().pitch<85:
    time.sleep(0.1)
#vessel.auto_pilot.target_roll=math.nan
#vessel.control.rcs=False
vessel.auto_pilot.stopping_time=(0.5,0.5,0.5)
thrust=vessel.available_thrust
g=body.surface_gravity
min_throttle=0.11
iy=Ktimer.integrator()
iz=Ktimer.integrator()
target_vertical_v=-3
while True:
    reference_frame=vessel.surface_reference_frame
    target_position=Vector3.Tuple3(body.position_at_altitude(target_lat,target_lon,target_altitude,reference_frame))
    horizon_pos=Vector3(0.0,target_position.y,target_position.z)
    print(horizon_pos.mag())

    surface_v=surface_velocity(vessel,reference_frame)
    horizon_v=Vector3(0.0,surface_v.y,surface_v.z)
    target_horizon_v=horizon_pos.unit_vector()*min(0.08*horizon_pos.mag(),3)
    p=target_horizon_v-horizon_v
    pid= 0.1*p.unit_vector()*min(math.sqrt(p.mag()),10)
    y=0.01*iy.integral(pid.y)
    z=0.01*iz.integral(pid.z)
    #pid=pid+Vector3(0.0,y,z)
    target_direction=Vector3(1.0,0.0,0.0)+pid
    py=target_direction.pitch_yaw()
    pitch=py[0]
    yaw=py[1]
    #print(py)
    vessel.auto_pilot.target_pitch_and_heading(pitch,yaw)

    if altitude()<target_altitude+20:
        target_vertical_v=-1
    
    if surface_v.x>-0.5:
        break

    a=g+min(max(10*(target_vertical_v-surface_v.x),-0.5),0.5)
    mass=vessel.mass
    vessel.control.throttle=(1.0-(1.0-mass*a/max(thrust,1e-3))/(1-min_throttle))*target_direction.mag()
    #time.sleep(0.1)
vessel.control.throttle=0
vessel.auto_pilot.disengage()

input()
