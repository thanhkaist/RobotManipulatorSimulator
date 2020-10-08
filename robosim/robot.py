# Copyright thanhnguyen@kaist.ac.kr 2020
#
import vpython as vp
import numpy as np


def vp_vec(vec):
    # In vpython the z axis point out the sceen so we change the other to map x axis point out the sceen 
    x , y , z  = vec[0],vec[1],vec[2]
    return vp.vec(y,z,x)



class Frame(object):
    ''' Frame object is the visualization of the robot frame.  
    The main parameters of a frame are:
    - trans: [x,y,z]
    - rotation: 3*3 rotation matrix in SO(3)
    - refer_frame: Another frame

    The mainly used functions:
    - update: update the internal parameters with new provided parameters and render the updated frame
    '''
    def __init__(self,trans,rotation, axis_long = 1,radius = 0.1, refer_frame = None, name ="frame"):

        self.trans = trans
        self.rotation = rotation
        self.axis_long = axis_long
        self.radius =radius
        self.name = name
        if refer_frame is None:
            self.refer_frame = FRAME_ORIGIN
        else:
            self.refer_frame = refer_frame

        self.draw()

    def draw(self):
        # Draw the axis
        d = self.axis_long
        R= self.radius
        h = 0.5


        rotation, trans = self.get_rot_trans_wrt_origin()

        ax = np.matmul(rotation,np.array([d,0,0]))
        ay = np.matmul(rotation,np.array([0,d,0])) 
        az = np.matmul(rotation,np.array([0,0,d])) 


        axc = np.matmul(rotation,np.array([R*2,0,0]))
        ayc = np.matmul(rotation,np.array([0,R*2,0])) 
        azc = np.matmul(rotation,np.array([0,0,R*2])) 

        txc = np.matmul(rotation,np.array([d,0,0])) +trans
        tyc = np.matmul(rotation,np.array([0,d,0])) +trans
        tzc = np.matmul(rotation,np.array([0,0,d])) +trans

        self.nameobj = vp.text(pos=vp_vec(trans), text=self.name, height=h, align='center', billboard=True, emissive=True)
        self.xaxis = vp.cylinder(pos=vp_vec(trans), axis=vp_vec(ax), radius=R, color=vp.color.red)
        self.xaxis_c = vp.cone(pos = vp_vec(txc), axis=vp_vec(axc),radius=R*2)
        self.yaxis = vp.cylinder(pos=vp_vec(trans), axis=vp_vec(ay), radius=R, color=vp.color.green)
        self.yaxis_c = vp.cone(pos = vp_vec(tyc), axis=vp_vec(ayc),radius=R*2)
        self.zaxis = vp.cylinder(pos=vp_vec(trans), axis=vp_vec(az), radius=R, color=vp.color.blue)
        self.zaxis_c = vp.cone(pos = vp_vec(tzc), axis=vp_vec(azc),radius=R*2)

        # Draw the rigid body
        self.draw_rigid_body()
    def draw_rigid_body(self):
        pass

    def __str__(self):
        out = "Frame:"+ str(self.name)\
           +"\n\tRelative to:" + self.refer_frame.name \
           +"\n\tTransformation:\n" \
           +"\t\tTranslation:"+str(self.trans) \
           +"\n\t\tRotation\n"+str(self.rotation) 
        return out

    def update(self,trans = None,rotation = None):
        d = self.axis_long
        R= self.radius
        if trans is not None:
            self.trans = trans
        
        if rotation is not None:
            self.rotation = rotation

        rotation, trans = self.get_rot_trans_wrt_origin()
   
        ax = np.matmul(rotation,np.array([d,0,0]))
        ay = np.matmul(rotation,np.array([0,d,0])) 
        az = np.matmul(rotation,np.array([0,0,d])) 


        axc = np.matmul(rotation,np.array([R*2,0,0]))
        ayc = np.matmul(rotation,np.array([0,R*2,0])) 
        azc = np.matmul(rotation,np.array([0,0,R*2])) 

        txc = np.matmul(rotation,np.array([d,0,0])) +trans
        tyc = np.matmul(rotation,np.array([0,d,0])) +trans
        tzc = np.matmul(rotation,np.array([0,0,d])) +trans

        self.nameobj.pos = vp_vec(trans)
        self.xaxis.pos,self.xaxis.axis=vp_vec(trans), vp_vec(ax)
        self.xaxis_c.pos,self.xaxis_c.axis = vp_vec(txc), vp_vec(axc)
        self.yaxis.pos,self.yaxis.axis=vp_vec(trans), vp_vec(ay)
        self.yaxis_c.pos,self.yaxis_c.axis = vp_vec(tyc), vp_vec(ayc)
        self.zaxis.pos,self.zaxis.axis=vp_vec(trans), vp_vec(az)
        self.zaxis_c.pos,self.zaxis_c.axis = vp_vec(tzc), vp_vec(azc)


    def visible(self,val):
        pass


    def get_rot_trans_wrt_origin(self):
        if not isinstance(self.refer_frame,Frame):
            # the frame is origin
            return self.rotation,self.trans
        
        #ref_T = Transformation(self.refer_frame.rotation,self.refer_frame.trans)
        ref_T = Transformation(*self.refer_frame.get_rot_trans_wrt_origin())
        T =  Transformation(self.rotation,self.trans)

        T_origin = ref_T*T
        return T_origin.rotation,T_origin.trans   



class Transformation():
    ''' Transformation matrix in SE(3). This object support multiply, inverse, get rotation, get translation
    
    Mainly used functions:
    - get_rotation: return rotation matrix
    - get_translation: return translation vector
    - matrix : return transformation matrix 

    '''
    def __init__(self,rotation = None,translation=None):
        if translation is None:
            translation = np.array([0,0,0])

        if rotation is None:
            rotation = np.eye(3)
        self.trans = translation
        self.rotation = rotation
        self.T = self.get_T(self.rotation,self.trans)

    def matrix(self):
        return T

    def get_T(self,rot,trans):
        
        out = np.concatenate([rot,trans.reshape(3,1)],axis = 1)
        out = np.concatenate([out,np.array([0,0,0,1]).reshape(1,4)],axis = 0)
        return out


    def get_translation(self,T):
        return T[:3,3].reshape(3)

    def get_rotation(self,T):
        return T[:3,:3].reshape(3,3)

    def __add__(self,o):
        raise("Cannot add two transformation")


    def __str__(self):
        out = "Transformation:\n" \
           +"\tTranslation:"+str(self.trans) \
           +"\n\tRotation\n"+str(self.rotation) 
        return out

    def __mul__(self,o):
        T = np.matmul(self.T,o.T)
        trans = self.get_translation(T)
        rotation = self.get_rotation(T)

        return Transformation(rotation,trans)

    def inverse(self):
        trans = -np.matmul(self.rotation.T,self.trans)
        rotation = self.rotation.T
        return Transformation(rotation,trans)


class Orientation():
    '''This class is used to manipulate the orientation with different representation: Rotation matrix, exponential coordinate, quadternions'''
    def __init__(self):
        self.rotation_matrix = np.eye(3)
        

    def matrix(self):
        return self.rotation_matrix

    def set_euler_xyz(self,roll,pitch,yaw):
        # roll, pitch, yaw in *C 
        # R = Rz*Ry*Rx
        
        gamma = roll*np.pi/180
        beta = pitch*np.pi/180
        alpha = yaw*np.pi/180

        self.rotation_matrix  = np.array([[np.cos(alpha)*np.cos(beta), 
            np.cos(alpha)*np.sin(beta)*np.sin(gamma)-np.sin(alpha)*np.cos(gamma),
            np.cos(alpha)*np.sin(beta)*np.cos(gamma)+np.sin(alpha)*np.sin(gamma)],
            [np.sin(alpha)*np.cos(beta),
            np.sin(alpha)*np.sin(beta)*np.sin(gamma)+np.cos(alpha)*np.cos(gamma),
            np.sin(alpha)*np.sin(beta)*np.cos(gamma)-np.cos(alpha)*np.sin(gamma)],
            [-np.sin(beta),np.cos(beta)*np.sin(gamma),np.cos(beta)*np.cos(gamma)]])



# Global configuration

# Draw object 
POS_ORIGIN = np.array([0,0,0])
FRAME_ORIGIN = Frame(POS_ORIGIN,np.eye(3),3,0.1,refer_frame='origin',name="ORIGIN") # This is the fixed origin frame or world frame



class Link():
    def __init__(self,frame1,frame2):
        self.frame1 =frame1
        self.frame2 = frame2


    def draw(self):
        pass

    def update(self):
        pass


if __name__=="__main__":

    L = 50

    # Set up the scene
    scene = vp.scene
    scene.title = "Robot transformation"


    # Set up the frame 
    orient = Orientation()
    orient.set_euler_xyz(0,0,0)
    pos = np.array([1,1,1])
    frame1 = Frame(pos,orient.rotation_matrix,2,0.1,name = "{1}")


    orient.set_euler_xyz(0,0,0)
    pos = np.array([1,1,1])
    frame2 = Frame(pos,orient.rotation_matrix,2,0.1,frame1,"{2}")

    orient.set_euler_xyz(0,0,0)
    pos = np.array([2,2,2])
    frame3 = Frame(pos,orient.rotation_matrix,2,0.1,frame2,"{3}")






    #T1 = Transformation()
    #T2 = Transformation(translation=np.array([1,1,1]),rotation=orient.rotation_matrix)
    #T3 = T1*T2

    #print(T2.inverse())


    #print(frame1)
    #print(frame2)

    input()

    # Ready to simulate 
    for i in range(200):
        vp.rate(30)
        orient = Orientation()
        orient.set_euler_xyz(i*5,0,i*10)
        frame1.update(pos,orient.rotation_matrix)
        frame2.update()
        frame3.update()




