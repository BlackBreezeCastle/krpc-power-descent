import krpc
class Ktimer:
    conn=None
    def __init__(self):
        if Ktimer.conn==None:
            Ktimer.conn = krpc.connect(name='clock')
        self.old=0.0
    def delta_t(self):
        tmp=Ktimer.conn.space_center.ut
        res=tmp-self.old
        if self.old==0.0:
            res=0.0
        self.old=tmp
        #print('delta t',res)
        return res
    
    def time_t(self):
        return Ktimer.conn.space_center.ut

class derivator:
    def __init__(self,num=0.0):
        self.__timer=Ktimer()
        self.__last=num
        self.__last_state=num
    def derivative(self,num):
        deltaTime=self.__timer.delta_t()
        if(deltaTime>1e-6):
            self.__last_state=(num-self.__last)/deltaTime
        self.__last=num
        return self.__last_state
class integrator:
    def __init__(self,num=0.0):
        self.__timer=Ktimer()
        self.__integral=num
        self.__max=None
    def integral(self,num):
        deltaTime=self.__timer.delta_t()
        if(deltaTime>1e-6):
            self.__integral=self.__integral+num*deltaTime
        if self.__max!=None:
            self.__integral=max(min(self.__integral,self.__max),-self.__max)
        return self.__integral
    def clear(self):
        self.__integral=0.0
    def set(self,num):
        self.__integral=num
    def set_max(self,_max):
        self.__max=abs(_max)