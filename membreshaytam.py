import math
import numpy as np


class Membres:

    d5=-0.15 
    a5=-d5
    
    def __init__(self, group, DH, robot):
        self.group = group
        self.DH = DH
        self.robot = robot

    def matrix_transfo(self,n):
        mattransfo=np.zeros((4,4))
        
        for i in range (n):
            
            mattransfo[0,0]=math.cos(self.DH[i,2])
            mattransfo[0,1]=-math.sin(self.DH[i,2])
            mattransfo[0,2]=0.0
            mattransfo[0,3]=self.DH[i,1]
            mattransfo[1,0]=math.cos(self.DH[i,0])*math.sin(self.DH[i,2])
            mattransfo[1,1]=math.cos(self.DH[i,0])*math.cos(self.DH[i,2])
            mattransfo[1,2]=-math.sin(self.DH[i,0])
            mattransfo[1,3]=-self.DH[i,3]*math.sin(self.DH[i,0])
            mattransfo[2,0]=math.sin(self.DH[i,0])*math.sin(self.DH[i,2])
            mattransfo[2,1]=math.sin(self.DH[i,0])*math.cos(self.DH[i,2])
            mattransfo[2,2]=math.cos(self.DH[i,0])
            mattransfo[2,3]=self.DH[i,3]*math.cos(self.DH[i,0])
            mattransfo[3,0]=0
            mattransfo[3,1]=0
            mattransfo[3,2]=0
            mattransfo[3,3]=1
            if i==0:
                t=mattransfo
            if i==1:
                t=np.dot(t,mattransfo)
            if i==2:
                t=np.dot(t,mattransfo)
            if i==3:
                t=np.dot(t,mattransfo)
            if i==4:
                t=np.dot(t,mattransfo)

        #calculer la matrice de transformation 
        
        return t
                      

    
    def calc_mgd(self):
        o56=np.mat([[Membres.d5],[0],[0],[1]])
        mgd=np.dot(self.matrix_transfo(5),o56)
        mgd=mgd[0:3]
        
        #calculer le modèle géométrique direct 
        
        return mgd

    def jacobienne(self):
        t01=self.matrix_transfo(1)
        t02=self.matrix_transfo(2)
        t03=self.matrix_transfo(3)
        t04=self.matrix_transfo(4)
        t05=self.matrix_transfo(5)
        
        z01=t01[0:3,2]
        z02=t02[0:3,2]
        z03=t03[0:3,2]
        z04=t04[0:3,2]
        z05=t05[0:3,2]
        
        p01=t01[0:3,3]
        p02=t02[0:3,3]
        p03=t03[0:3,3]
        p04=t04[0:3,3]
        p05=t05[0:3,3]
        
        p15=p05-p01
        p25=p05-p02
        p35=p05-p03
        p45=p05-p04
        
        j00=np.cross(z01,p15)
        j01=np.cross(z02,p25)
        j02=np.cross(z03,p35)
        j03=np.cross(z04,p45)
        j04=np.zeros((3,1))
        
        J05=np.mat([[j00[0],j01[0],j02[0],j03[0],0],      #la matrice  inverse  
                   [j00[1],j01[1],j02[1],j03[1],0],
                   [j00[2],j01[2],j02[2],j03[2],0],
                   [z01[0],z02[0],z03[0],z04[0],z05[0]],
                   [z01[1],z02[1],z03[1],z04[1],z05[1]],
                   [z01[2],z02[2],z03[2],z04[2],z05[2]]])
        
        D=np.array([[0,Membres.a5*self.matrix_transfo(5)[2,0],-Membres.a5*self.matrix_transfo(5)[1,0]],
                    [-Membres.a5*self.matrix_transfo(5)[2,0],0,Membres.a5*self.matrix_transfo(5)[0,0]],
                    [Membres.a5*self.matrix_transfo(5)[1,0],-Membres.a5*self.matrix_transfo(5)[0,0],0]])
        
        M=np.mat([[1,0,0,D[0,0],D[0,1],D[0,2]],            #matrice M
             [0,1,0,D[1,0],D[1,1],D[1,2]],
             [0,0,1,D[2,0],D[2,1],D[2,2]],
             [0,0,0,1,0,0],
             [0,0,0,0,1,0],
             [0,0,0,0,0,1]])
       
        mat_jacob=np.dot(M,J05)          #prduit matriciel
        return mat_jacob 

        # #calculuer la matrice Jp^-1
        
        # return mat_jacob

    def get_min_max(self, robot_conf):
        """

        :return angle_limit: Valeur angulaire min et max  pour chaque moteur du group
  """      
        angle_limite = []
        for key, value in robot_conf.items():
            if key == 'motors':
                for i in range(len(self.group)):
                    # angle_limite.append(self.group[i]) # debug
                    angle_limite.append(value[self.group[i]]['angle_limit'])
        return angle_limite
    def gen_traj(self, qi, qf, tf):
               
        coeffs = np.array([])
        for i in range(len(self.group)):
            a0 = qi[i]
            a1 = 0
            a2 = (3 / (math.pow(tf, 2))) * (qf[i] - qi[i])
            a3 = -(2 / (math.pow(tf, 3))) * (qf[i] - qi[i])
            coeffs = np.append(coeffs, np.array([a0, a1, a2, a3]))
        return coeffs.reshape(len(self.group), 4)
        #génération de la trajectoire 
        #return résultat 

    def eval_traj(self, coeffs, vrep_time):
        
        vrep_time = float(vrep_time)
        qd, dqd = np.array([]), np.array([])
        pos = np.array([[1],
                        [vrep_time],
                        [(math.pow(vrep_time, 2))],
                        [(math.pow(vrep_time, 3))]]),
        speed = np.array([[0],
                          [1],
                          [2 * vrep_time],
                          [3 * math.pow(vrep_time, 2)]])
        qd = coeffs.dot(pos)
        dqd = coeffs.dot(speed)
        return qd

        #évaluation de la trajectoire
        #return résultat


poppy_head_config = {
    'controllers': {
        'my_dxl_controller': {
            'port': '/dev/ttyACM0',  # Depends on your OS
            'sync_read': False,
            'attached_motors': ['head'],  # You can mix motorgroups or individual motors
        },
    },

    'motorgroups': {
        'head': ['head_y'],
    },

    'motors': {
        'head_y': {
            'id': 1,
            'type': 'AX-12',
            'orientation': 'INdirect',
            'offset': 20.0,
            'angle_limit': (-45, 6),
        },
    },
}


