import math

def sign(num):
    if num>1e-16:
        return 1.0
    if num<1e-16:
        return -1.0
    return 0.0

def normalized_angle(angle):
    angle=angle+180
    angle=angle%360
    angle=angle-180
    return angle

def normalized_rad(rad):
    rad=rad+math.pi
    rad=rad%(2*math.pi)
    rad=rad-math.pi
    return rad

#三维向量
class Vector3:
    def __init__(self,x,y,z):
        self.x=x
        self.y=y
        self.z=z
    def __str__(self):
        return "("+str(self.x)+" "+str(self.y)+" "+str(self.z)+")"

    def tuple3(self):
        return (self.x,self.y,self.z)

    def mag(self):
        return math.sqrt(Vector3.Dot(self,self))

    def unit_vector(self):
        if self.mag()<1e-16:
            return Vector3(0.0,0.0,0.0)
        return self/self.mag()
    #x轴:上 y轴：北 z轴：东
    def pitch_yaw(self):
        x=self.x
        y=self.y
        z=self.z
        v1=Vector3(x,y,z)
        pitch=90.0-Vector3.Angle(Vector3(1.0,0.0,0.0),v1)*180/math.pi
        v2=Vector3(0.0,y,z)
        if v2.mag()==0.0:
            return(0,0)
        yaw=Vector3.Angle(Vector3(0.0,1,0.0),v2)*180/math.pi
        yawsig=Vector3.Cross(Vector3(0.0,1.0,0.0),v2)
        if yawsig.x<0:
            yaw=360-yaw
        return(pitch,yaw)
        
    
    def __add__(self,b):
        return Vector3(self.x+b.x,
                  self.y+b.y,
                  self.z+b.z)

    def __radd__(self,b):
        return Vector3(self.x+b.x,
                  self.y+b.y,
                  self.z+b.z)
    
    def __sub__(self,b):
        return Vector3(self.x-b.x,
                  self.y-b.y,
                  self.z-b.z)
    
    def __rsub__(self,b):
        return Vector3(self.x-b.x,
                  self.y-b.y,
                  self.z-b.z)
    #数乘
    def __mul__(self,b):
        return Vector3(self.x*b,
                  self.y*b,
                  self.z*b)

    def __rmul__(self,b):
        return Vector3(self.x*b,
                  self.y*b,
                  self.z*b)
    #数除
    def __truediv__(self,b):
        return Vector3(self.x/b,
                  self.y/b,
                  self.z/b)
    @staticmethod
    def Tuple3(t3):
        return Vector3(t3[0],t3[1],t3[2])
  
    #点积
    @staticmethod
    def Dot(a,b):
        return (a.x*b.x+a.y*b.y+a.z*b.z)
    
    #叉积
    @staticmethod
    def Cross(a,b):
        x=a.y*b.z-a.z*b.y
        y=a.z*b.x-a.x*b.z
        z=a.x*b.y-a.y*b.x
        return Vector3(x,y,z)

    #夹角
    @staticmethod
    def Angle(a,b):
        return math.acos(max(min(Vector3.Dot(a,b)/(a.mag()*b.mag()),1),-1))

    

class Quaternion:
    def __init__(self, w, x, y, z):
        mag = x*x + y*y + z*z + w*w;
        self.w = w/mag
        self.x = x/mag
        self.y = y/mag
        self.z = z/mag

    def __str__(self):
        return "("+str(self.w)+" "+str(self.x)+"i "+str(self.y)+"j "+str(self.z)+"k )"        

    def __mul__(self, quater):
        w1 = self.w
        w2 = quater.w
        v1 = Vector3(self.x, self.y, self.z)
        v2 = Vector3(quater.x, quater.y, quater.z)
        w3 = w1*w2-Vector3.Dot(v1 ,v2)
        v3 = Vector3.Cross(v1,v2)+w1*v2+w2*v1
        return Quaternion(w3, v3.x, v3.y, v3.z)
    
    def tuple4(self):
        return (self.w,self.x,self.y,self.z)

    def inverse(self):
        return Quaternion(self.w,-self.x, -self.y, -self.z)
    #右手系轴角顺时针旋转
    def rotate(self,v):
        u=Vector3(self.x,self.y,self.z)
        s=self.w
        return 2.0*Vector3.Dot(u,v)*u+(s*s-Vector3.Dot(u,u))*v+2.0*s*Vector3.Cross(u,v)
    
    @staticmethod
    def PivotRad(v,rad):
        theta = rad/2
        if v.mag()==0.0:
            u=Vector3(1,0,0)
            theta=0.0
        else:
            u = v.unit_vector()     
        return Quaternion(math.cos(theta),math.sin(theta)*u.x,math.sin(theta)*u.y,math.sin(theta)*u.z)

    @staticmethod
    def PivotAngle(v,w):
        w=w*math.pi/180
        return Quaternion.PivotRad(v,w)

    @staticmethod
    def Tuple4(t4):
        mag = t4[0]*t4[0] + t4[1]*t4[1] + t4[2]*t4[2] + t4[3]*t4[3];
        return Quaternion(t4[3]/mag,t4[0]/mag,t4[1]/mag,t4[2]/mag)
        
'''       
q1=Quaternion.PivotRad(Vector3(1,0,0),0.1/1737100)
q2=Quaternion.PivotAngle(Vector3(0,1,0),90)
v=Vector3(0,0,1737100)
print(v.pitch_yaw())
print((q1.inverse()).rotate(v))
'''
