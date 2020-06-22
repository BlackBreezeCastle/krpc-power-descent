import krpc
import time
import mission
import SSautopilot
from peg import *

conn = krpc.connect(name='test')
vessel = conn.space_center.active_vessel
body=vessel.orbit.body
reference_frame=body.non_rotating_reference_frame


target_radius = mission.target_altitude+6371000
target_speed=mission.target_speed
target_lan=math.radians(mission.target_lan)
target_inc=math.radians(mission.target_inc)
target_normal_vector=target_normal_vector(conn,body,target_inc,target_lan,reference_frame)


position=conn.add_stream(vessel.position,reference_frame)
velocity=conn.add_stream(vessel.velocity,reference_frame)
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
speed=conn.add_stream(getattr, vessel.flight(reference_frame), 'speed')
horizontal_speed=conn.add_stream(getattr, vessel.flight(body.reference_frame), 'horizontal_speed')
vertical_speed=conn.add_stream(getattr, vessel.flight(body.reference_frame), 'vertical_speed')

peg=pegas(conn)
peg.set_target(target_inc,target_lan,target_radius,target_speed)
peg.update_stages()

pitch=90
yaw=math.degrees(target_heading(vessel,target_normal_vector,reference_frame))
vessel.auto_pilot.engage()
auto=SSautopilot.SSautopilot(vessel)
vessel.auto_pilot.target_pitch=pitch
''
#vessel.auto_pilot.target_pitch=90
#vessel.auto_pilot.attenuation_angle=(0.2,0.2,0.2)

fair_droped=False
stage_one_droped=False
#fair_droped=True
#stage_one_droped=True

warp_to_launch(conn,target_inc,target_lan,False)
print('5')
time.sleep(1)
print('4')
time.sleep(1)
print('ingition!')
vessel.control.throttle = 1.0
vessel.control.activate_next_stage()
time.sleep(1)
print('2')
time.sleep(1)
print('1')
time.sleep(1)
print('lift off')
vessel.control.activate_next_stage()

turn_start_altitude = altitude()+50
while altitude()<turn_start_altitude:
    time.sleep(0.1)

vessel.auto_pilot.target_roll=0
print('start gravity turn')
while altitude()<turn_start_altitude+500:
    ''
    pitch = 90-5.5*min((altitude() - turn_start_altitude) /300,1)
    yaw=math.degrees(target_heading(vessel,target_normal_vector,reference_frame))
    vessel.auto_pilot.target_pitch_and_heading(pitch,yaw)
    '''
    pitch = math.radians(90-3.5*min((altitude() - turn_start_altitude) /300,1))
    yaw=target_heading(vessel,target_normal_vector,reference_frame)
    auto.set_pitch_and_yaw(pitch,yaw)
    '''
    #time.sleep(0.1)

print('gravity turning')
while altitude()<30000:
    ''
    pitch=math.degrees(math.atan(vertical_speed()/horizontal_speed()))
    yaw=math.degrees(target_heading(vessel,target_normal_vector,reference_frame))
    vessel.auto_pilot.target_pitch_and_heading(pitch,yaw)

        #peg.update_stages()
    
    '''
    pitch=math.atan(vertical_speed()/horizontal_speed())
    yaw=target_heading(vessel,target_normal_vector,reference_frame)
    auto.set_pitch_and_yaw(pitch,yaw)
    '''
    #time.sleep(0.1)

''
print('pega running')

vessel.auto_pilot.disengage()
auto.set_pitch_and_yaw(math.radians(pitch),math.radians(yaw))
auto.engage()
peg.update_stages()
peg.vehicle_info()
for i in range(100):
    peg.update()
tgo=1e10  

while True:
    acc=vessel.thrust/max(vessel.mass,0.1)
    py=peg.update()
    if py!=None:
        pitch=py[0]
        yaw=py[1]
    tgo=peg.time_to_go()
    time_to_stage=peg.time_to_stage()
    rd=peg.rd_position()
    print('tgo:%f pitch:%f yaw:%f acc:%f time_to_stage:%f pos: %f %f'%(tgo,math.degrees(pitch),math.degrees(yaw),acc,time_to_stage,rd[0],rd[1]),end='\r')

    #print('tgo',tgo)
    #print('py',py)
    if tgo>5:   
        #vessel.auto_pilot.target_pitch_and_heading(math.degrees(py[0]), math.degrees(py[1]))
        auto.set_pitch_and_yaw(pitch,yaw)
    
    if fair_droped==False and altitude()>100000:
        fair_droped=True
        vessel.control.activate_next_stage()
        time.sleep(0.5)
        peg.update_stages()

    if stage_one_droped==False and peg.stages_num()==2 and time_to_stage<0.5:
        stage_one_droped=True
        #vessel.control.forward=1
        #time.sleep(2.0)
        vessel.control.activate_next_stage()
        time.sleep(1.0)
        vessel.control.activate_next_stage()
        #time.sleep(8.0)
        #vessel.control.forward=0
        for i in range(100):
            peg.update()

    if  speed()>target_speed-1:
        vessel.control.throttle = 0.0
        break

input()
