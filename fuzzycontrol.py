from fuzzylogic.classes import Domain,Set,Rule
from fuzzylogic.functions import R, S, alpha,triangular
from matplotlib import pyplot
import math
import numpy as np

def cal_angle(input_angle):
        out_angle=input_angle
        while(abs(out_angle)>180):
            if(out_angle>0):
                out_angle=out_angle-360
            else:
                out_angle=out_angle+360
        return out_angle

class fuzzy_controler():
    def __init__(self,y,angle):
        self.y_pos=Domain("vertical_pos",-100,100,res=0.1)
        self.angle=Domain("angle_pos",-180,180,res=0.1)
        self.steer_ang=Domain("steering_angle",-30,30,res=0.1)
        self.rules=None
        self.start_vertical_pos=y
        self.start_angle=angle
        self.length=2.5
        self.sample_time=0.1
        self.speed=0.5
    def create_membership_function(self):
        self.y_pos.be=S(-40,-10) 
        self.y_pos.bc=triangular(-20,0)  
        self.y_pos.ce=triangular(-5,5)
        self.y_pos.ac=triangular(0,20) 
        self.y_pos.ab=R(10,40) 
        self.angle.bo=S(-125,-75)
        self.angle.br=triangular(-100,-25)
        self.angle.bh=triangular(-50,0)
        self.angle.hz=triangular(-25,25)
        self.angle.ar=triangular(25,100)
        self.angle.ah=triangular(0,50)
        self.angle.ao=R(75,125)
        self.steer_ang.nb=S(-30,-15)
        self.steer_ang.nm=triangular(-25,-5)
        self.steer_ang.ns=triangular(-12,0)
        self.steer_ang.ze=triangular(-5,5)
        self.steer_ang.pm=triangular(5,25)
        self.steer_ang.ps=triangular(0,12)
        self.steer_ang.pb=R(15,30)
    def create_rules(self):
        self.rules=Rule({(self.y_pos.be,self.angle.bo):self.steer_ang.pb,
                         (self.y_pos.be,self.angle.br):self.steer_ang.pb,
                         (self.y_pos.be,self.angle.bh):self.steer_ang.pb,
                         (self.y_pos.be,self.angle.hz):self.steer_ang.pm,
                         (self.y_pos.be,self.angle.ah):self.steer_ang.pm,
                         (self.y_pos.be,self.angle.ar):self.steer_ang.ps,
                         (self.y_pos.be,self.angle.ao):self.steer_ang.ns,
                         (self.y_pos.bc,self.angle.bo):self.steer_ang.pb,
                         (self.y_pos.bc,self.angle.br):self.steer_ang.pb,
                         (self.y_pos.bc,self.angle.bh):self.steer_ang.pm,
                         (self.y_pos.bc,self.angle.hz):self.steer_ang.pm,
                         (self.y_pos.bc,self.angle.ah):self.steer_ang.ps,
                         (self.y_pos.bc,self.angle.ar):self.steer_ang.ns,
                         (self.y_pos.bc,self.angle.ao):self.steer_ang.nm,
                         (self.y_pos.ce,self.angle.bo):self.steer_ang.pm,
                         (self.y_pos.ce,self.angle.br):self.steer_ang.pm,
                         (self.y_pos.ce,self.angle.bh):self.steer_ang.ps,
                         (self.y_pos.ce,self.angle.hz):self.steer_ang.ze,
                         (self.y_pos.ce,self.angle.ah):self.steer_ang.ns,
                         (self.y_pos.ce,self.angle.ar):self.steer_ang.nm,
                         (self.y_pos.ce,self.angle.ao):self.steer_ang.nm,
                         (self.y_pos.ac,self.angle.bo):self.steer_ang.pm,
                         (self.y_pos.ac,self.angle.br):self.steer_ang.ps,
                         (self.y_pos.ac,self.angle.bh):self.steer_ang.ns,
                         (self.y_pos.ac,self.angle.hz):self.steer_ang.nm,
                         (self.y_pos.ac,self.angle.ah):self.steer_ang.nm,
                         (self.y_pos.ac,self.angle.ar):self.steer_ang.nb,
                         (self.y_pos.ac,self.angle.ao):self.steer_ang.nb,
                         (self.y_pos.ab,self.angle.bo):self.steer_ang.ps,
                         (self.y_pos.ab,self.angle.br):self.steer_ang.ns,
                         (self.y_pos.ab,self.angle.bh):self.steer_ang.nm,
                         (self.y_pos.ab,self.angle.hz):self.steer_ang.nm,
                         (self.y_pos.ab,self.angle.ah):self.steer_ang.nb,
                         (self.y_pos.ab,self.angle.ar):self.steer_ang.nb,
                         (self.y_pos.ab,self.angle.ao):self.steer_ang.nb,
                  })


    def plot_track(self,y_cri,ang_cri):
        vt=self.speed*self.sample_time
        #initial state
        y=self.start_vertical_pos
        ang=cal_angle(self.start_angle)
        x=0
        y_track=[]
        x_track=[]
        y_track.append(y)
        x_track.append(0)
        num=0
        totaltime=0
        while(1):
            #avoid None value
            if(abs(y)>100):
                if(y>0):
                    u=-30
                else:
                    u=30
            else:
                 values = {self.y_pos: y, self.angle: ang}
                 u=self.rules(values)
            x=x+vt*math.cos(ang*math.pi/180)
            y=y+vt*math.sin(ang*math.pi/180)
            ang=ang+vt*math.tan(u*math.pi/180)/self.length
            ang=cal_angle(ang)
            y_track.append(y)
            x_track.append(x)
            totaltime +=self.sample_time
            num=num+1
            if(abs(y)<y_cri and abs(ang)<ang_cri):
                break
            if(num>1000000):
                break
        print('Total time: '+str(round(totaltime,2))+'s='+str(round(totaltime/60,2))+'min='+str(round(totaltime/3600,2))+'h')

        pyplot.plot(x_track,y_track)
        pyplot.xlabel('x')
        pyplot.ylabel('y')
        pyplot.title('the track of trunk')
        pyplot.show()
        

       


y_start=10
angle_Start=220
car_controller=fuzzy_controler(y_start,angle_Start)
car_controller.create_membership_function()
car_controller.create_rules()
y_end=1
angle_end=10
car_controller.plot_track(y_end,angle_end)




        
