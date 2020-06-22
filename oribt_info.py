import krpc
import time
import math
conn = krpc.connect(name='clock')
vessel = conn.space_center.active_vessel
orbit=vessel.orbit
while True:
    print('倾角:%.4f'%(orbit.inclination*180/math.pi))
    print('升交点:%.4f'%(orbit.longitude_of_ascending_node*180/math.pi))
    print('远地点高度:%.4f'%orbit.apoapsis_altitude)
    print('近地点高度:%.4f'%orbit.periapsis_altitude)
    print('\n')
    time.sleep(1)
