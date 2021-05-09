#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np
import numpy.linalg as alg
import matplotlib.pyplot as plt
import pylab as pl
from pylab import *
from scipy.linalg import solve_continuous_are
import time
from nav_msgs.msg import Odometry
from Robot_pure_interface.msg import cmd_drive
from std_msgs.msg import Float64, String, Int32
from std_msgs.msg import Float64MultiArray #Rafael
from sensor_msgs.msg import Imu, NavSatFix
from scipy.linalg import sqrtm,expm,norm,block_diag
from mpl_toolkits.mplot3d import Axes3D
from numpy import mean,pi,cos,sin,sqrt,tan,arctan,arctan2,tanh,arcsin,\
          exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
          arange,reshape,concatenate,vstack,hstack,diag,median,sign,sum,meshgrid,cross,linspace,append,round
from numpy import mean,pi,cos,sin,sqrt,tan,arctan,arctan2,tanh,arcsin,\
          exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
          arange,reshape,concatenate,vstack,hstack,diag,median,sign,sum,meshgrid,cross,linspace,append,round
from matplotlib.pyplot import *
from numpy.random import randn,rand
from numpy.linalg import inv, det, norm, eig
from scipy.linalg import sqrtm,expm,norm,block_diag
from scipy.signal import *
from mpl_toolkits.mplot3d import Axes3D
from math import factorial, exp
import subprocess
import rospy
import numpy.linalg as alg
import pylab as pl
from pylab import *
from scipy import *
from scipy import spatial

import time
from matplotlib.patches import Ellipse,Rectangle,Circle, Wedge, Polygon, Arc
from matplotlib.collections import PatchCollection
import shapely.geometry as geom
from scipy import spatial
from scipy.signal import cont2discrete as c2d
from control.matlab import *
from functions import *

from qpsolvers import solve_qp,quadprog_solve_qp
from cvxopt import matrix
from cvxopt import solvers




'''
*** NEW IMPLEMENTATION ***
Modification of the old cold to add the Point cloud information
'''
from GenerateBezier import *

# Global variables
flag_obstacle     = 0
x_obstacle        = 0
y_obstacle        = 0
radius_obstacle   = 0
obstacle_info     = [None] * 4
ref               = []
ref_new           = []
ref_ND            = []
TrajX             = []
TrajY             = []
Path_done         = []
Path_ref          = []
Path_traj         = []
bezier_AllPoints  = np.zeros((7,2))
tracking_position = []
bezier_length     = 10000

'''
*** IMPLEMENTED BEFORE ***
Utilization of code already implemented for the control stage
'''
cmd = cmd_drive()
cmd_publisher= rospy.Publisher('cmd_drive', cmd_drive,queue_size=10)
# == Paramètres du modèle ======================================================
kkk=pi/180
Rt=6371000

latitude_init = 39.5080331 #39.5080322117 #sur gazebo
longitude_init = -0.4619816 #-0.46198057533;
Cf = 17000 #rigidité de dérive du train avant
Cr = 17000 # rigidité de dérive du train arrière
masse = 880
moment = 86.7
a = 0.85
b = 0.85
d = 0.5
Xp = 0
Yp = 0
Psip = 0
Psi = 0
X = 0
Y = 0
g = 9.81
s_vect = []
psi_ref = []
courbure = []
dc_ds = []
dpsi = 0
RR = []
FF = []
A = 0

Psiref =0.0
xref = 0.0
yref = 0.0
ey = 0.0
epsi = 0.0
line = geom.LineString()
t0 =  0.0
vit=0.0

def ImuCallback_F(imu_data):
    '''
    *** IMPLEMENTED BEFORE ***
    Utilization of code already implemented for the control stage
    '''
    global Qx, Qy, Qz, Qw, Xp,Yp, Psip,ay,ax,az,wx,wy,wz,Psi,theta,ph
    Qx=imu_data.orientation.x
    Qy=imu_data.orientation.y
    Qz=imu_data.orientation.z
    Qw=imu_data.orientation.w
    ax=imu_data.linear_acceleration.x
    ay=imu_data.linear_acceleration.y
    az=imu_data.linear_acceleration.z
    wx=imu_data.angular_velocity.x
    wy=imu_data.angular_velocity.y
    wz=imu_data.angular_velocity.z
    Psip=imu_data.angular_velocity.z
    Psi=math.atan2(2.0 * (Qw * Qz + Qx * Qy),1.0 - 2.0 * (Qy * Qy + Qz * Qz))
    phi=math.atan2(2.0 * (Qw * Qx + Qy * Qz),1.0 - 2.0 * (Qx * Qx + Qy * Qy))

    sinp = +2.0 * (Qw * Qy - Qz * Qx)
    if (fabs(sinp) >= 1):
        theta= math.copysign(pi / 2, sinp)
    else:
        theta= math.asin(sinp)

    return ax,ay,az,wx,wy,wz,Psi,theta,phi

def retour_gps (msg):
  '''
  *** IMPLEMENTED BEFORE ***
  Utilization of code already implemented for the control stage
  '''
  global longitude,latitude, X ,Y
  longitude= msg.longitude
  latitude= msg.latitude
  X=Rt*(longitude - longitude_init)*kkk*cos(kkk*latitude_init)
  Y=Rt*(latitude - latitude_init)*kkk

def receive_message (informa):
  """ Function that receives information from the detection module.
      Here, the message is received in an array with:
      - informa.data[0]: Flag indicating whether there's an obstacle (999) or not (1)
      - informa.data[1]: X-postion of the obstacle (Considering the Robot as reference)
      - informa.data[2]: Y-postion of the obstacle (Considering the Robot as reference)
      - informa.data[3]: Radius of the safety circle

  Args:
      informa (array): array with information about the obstacle
  """
  global x_med, y_med, flag_obstacle, obstacle_info
  flag_obstacle = informa.data[0]

  if flag_obstacle == 999:
    # Distance having as reference the frame of the robot
    obstacle_info[0] = informa.data[1]  # x_barycenter
    obstacle_info[1] = informa.data[2]  # y_barycenter
    obstacle_info[2] = informa.data[3]  # radius for the "safety zone"
    obstacle_info[3] = informa.data[4]  # radius of the circle around the obstacle

def NewPath():
  """ Function that creates the local path for the avoidance algorithm
  """
  global ref, ref_ND, TrajX, TrajY, s_vect, psi_ref, courbure, dc_ds, dpsi, RR, FF, A, Psiref, xref, yref, ey, epsi, line, t0, vit

  ref    = np.loadtxt ("Robot_ws/src/Robot_folder/mpc_primitives/scripts/Reference/Line.txt")
  ref_ND = np.loadtxt ("Robot_ws/src/Robot_folder/mpc_primitives/scripts/Reference/Line.txt")

  # Transformation ref path to the origin (0,0)
  ref[:,1] = ref[:,1] - ref[0,1]
  ref[:,2] = ref[:,2] - ref[0,2]

  # Curvature, abscisse curvilinear...
  TrajX = ref[:,1]
  TrajY = ref[:,2]

  '''
  *** IMPLEMENTED BEFORE ***
  Utilization of code already implemented for the control stage
  '''
  s_vect, psi_ref, courbure, dc_ds = TrajRefer(TrajX,TrajY)
  #  Transformation
  dpsi = Psi-psi_ref[0]
  psi_ref[:] = psi_ref[:]+dpsi
  RR = np.array([[cos(dpsi),-sin(dpsi),X],[sin(dpsi) ,cos(dpsi) ,Y],[0 ,0 ,1]])
  FF = np.array([np.transpose(ref[:,1]),np.transpose(ref[:,2]),np.ones((1,len(ref)))])
  A = RR.dot(FF)
  ref[:,1:2] = np.transpose(A[0])
  ref[:,2:3] = np.transpose(A[1])

  #  Psiref = ref[0,3]
  Psiref = psi_ref[0]
  Psipref = 0
  xref = ref[0,1]
  yref = ref[0,2]
  ey = -sin(Psiref)*(X-xref)+cos(Psiref)*(Y-yref)
  epsi = (Psi-Psiref)
  line = geom.LineString(ref[:,1:3])

def UpdatePath (index, X_prior, Y_prior, x_bezier, y_bezier, index_P3):
  """[summary]

  Args:
      index ([type]): [description]
      X_prior ([type]): [description]
      Y_prior ([type]): [description]
      x_bezier ([type]): [description]
      y_bezier ([type]): [description]
      index_P3 ([type]): [description]
  """
  #Call the global variables to change their values here
  global ref, TrajX, TrajY, s_vect, psi_ref, courbure, dc_ds, Psiref, xref,\
         yref, ey, epsi, cmd, cmd_publisher, vit, line,bezier_length

  # New_Path_bezier is a list that will append:
  # 1: The reference path followed by the Robot until the point of the detection of an obstacle
  # 2: The avoidance path generated with the two cubic bezier curves
  # 3: The original reference to be followed after the avoidance path
  New_Path_bezier = []

  for i in range (index):
    New_Path_bezier.append([ref[i,1], ref[i,2]])
  bezier_length = len(New_Path_bezier)

  for j in range (len(x_bezier)):
    New_Path_bezier.append([x_bezier[j], y_bezier[j]])
  bezier_length = len(New_Path_bezier)/2 + bezier_length

  for i in range (len(ref)):
    if i > index_P3:
      New_Path_bezier.append([ref[i,1], ref[i,2]])
  saveNewList(New_Path_bezier, "New_Path_bezier")

  # Clear the data from old variables that depend on the reference path
  del ref
  del TrajX
  del TrajY
  del s_vect
  del psi_ref
  del courbure
  del dc_ds
  del line

  # Reinitialize the variables that depend on the reference path
  ref = np.loadtxt("Robot_ws/src/Robot_folder/mpc_primitives/scripts/Reference/AvoidancePath/Error/New_Path_bezier.txt")
  TrajX = ref[:,1]
  TrajY = ref[:,2]
  s_vect, psi_ref, courbure, dc_ds = TrajRefer(TrajX,TrajY)
  line = geom.LineString(ref[:,1:3])

  # Saves the new path
  name_file = "Complete"
  saveNew (ref, len(ref), name_file)


'''
*** IMPLEMENTED BEFORE ***
Utilization of code already implemented for the control stage
'''
def main():
  data_pub = rospy.Publisher('floats', numpy_msg(Floats),queue_size=10)
  rospy.Subscriber("/IMU", Imu, ImuCallback_F)
  rospy.Subscriber("/GPS/fix",NavSatFix,retour_gps)
  rospy.init_node('LQR', anonymous=True)
  r = rospy.Rate(200)
  simul_time = rospy.get_param('~simulation_time', '2')

  # Sets the new trajectory path
  NewPath ()

  # Initializes the Robot
  vit = 5
  xhat = np.array([[0],[0],[0],[0]])
  Vxx=vit

  cmd.linear_speed = vit
  cmd.steering_angle_front = 0
  cmd.steering_angle_rear = 0
  cmd_publisher.publish(cmd)

  Cc = np.array([ [0, 1,0, 0],  [0,0, 1, 0],  [0,0,0, 1]])
  Bc = get_model_B_matrix(Cf,Cr)

  #####  kalman

  dt=1/1000  #periode d'échantionnage

  Tsamp=0.12
  Gammax=dt*eye(4,4)
  Gammaalpha=dt*0.0000001*eye(4,4)
  Gammabeta=dt**2*eye(3,3)*0.0000001
  uk_1=np.array([[0],[0]])
  Np = 8
  x_slope=np.array([[0],[0]])
  Gamma_slop=dt*np.eye(2,2)

  theta_r=x_slope[0,0]
  phi_r=x_slope[1,0]

  t0 =  rospy.get_time()

  '''
  *** NEW IMPLEMENTATION ***
  Modification of the old cold to add the Point cloud information
  '''
  # VARIABLES DESCRIPTIONS (Below, before the while)
  # • flag_new_path : Indicates that once the Robot detects one object, he can generate
  #                   an avoidance path. Initializes at one so just after the start of
  #                   the Robot, it can be prepared to create the alternative path
  # • Point i_prior : Used to Track the real path made by the Robot. (i = {x,y})
  # • obstacle_i    : Used to save the key-informations about the obstacle
  #                   detected. (i = {x, y, radius})

  X_prior = []
  Y_prior = []

  obstacle_x = 0
  obstacle_y = 0
  obstacle_radius = 0
  obstacle_radius_contour = 0

  flag_new_path = 1
  while ((rospy.get_time()-t0 <= simul_time)) :
    # This subscribes to the node that sends the information about the obstacle detection module
    rospy.Subscriber("/cloud_processing/object_information",Float64MultiArray,receive_message)

    # The step will tell the bezier generator the sampling wise for each curve
    step = 600
    new_path_test = []

    if flag_obstacle == 999.0:
      ### Converts the Obstacle coordinates from the Robot's reference to the ground reference
      #  1. Rotation Matrix
      ObsRR = np.array([[cos(Psi),-sin(Psi),X],[sin(Psi) ,cos(Psi) ,Y],[0 ,0 ,1]])
      #  2. Obs_data contains all the information needed, and will its reference converted
      Obs_data = np.array([obstacle_info[0]+0.5, obstacle_info[1]+0.5, 1])
      Obs_data.shape = (3,1)

      #  3. Transposes the obstacle data
      TrasposedObstacle = ObsRR.dot(Obs_data)
      obstacle_x= float(TrasposedObstacle[0])
      obstacle_y= float(TrasposedObstacle[1])
      obstacle_radius = obstacle_info[2]
      obstacle_radius_contour = obstacle_info[3]

      obstacle_info[0] = obstacle_x
      obstacle_info[1] = obstacle_y

      # This if boucle was done to allow the blocking of the creation of a new path
      # every time the flag_obstacle is activated. Once the flag_new_path is
      # detected (==1), it is deactivated (==0)
      Max_curb_obstacle = []

      if flag_new_path == 1:
        flag_new_path=0

        cmd.linear_speed = 0
        cmd_publisher.publish(cmd)

        # Here the Point obstacle_bezier was created to proceed to the bezier generation
        obstacle_bezier_x = obstacle_x
        obstacle_bezier_y = obstacle_y


        ## FINDING THE FINAL POINT FOR THE BEZIER
        #   - Loop to find the bezier final point. This point is located whithin the reference
        #   path, at the point Pf = (xf,yf) such that the distace Pf and the obstacle
        #   coordinates is greater than the safety circle radius.

        #   The index_finder is a variable that increments throughout the reference path,
        #   starting at the detection point coordinate of the reference path. At each time
        #   it increases its value, the distance between the index_finder coordinate and
        #   the object coordinates is computed. The loop stops when the distance is superior
        #   compared to the obstace safety radius
        index_finder = index

        # middle_path is a "cheking variable", that means, it was used to see if the portion
        # detected corresponds to the path found within the obstacle safety radius
        middle_path=[]
        for i_obs in range(len(ref)-index):
          inside_check = (((obstacle_bezier_x)-ref[index_finder,1])**2 + ((obstacle_bezier_y)-ref[index_finder,2])**2)

          middle_path.append([ref[index_finder,1],ref[index_finder,2]])
          # The last (x,y) of the local path will be considered the (X,Y) where the robot will be outside of
          # the safety_circle of the object detected here as an obstacle by the robot
          if (inside_check-obstacle_radius**2) > obstacle_bezier_x:
            index_P3 = index_finder
            break
          index_finder += 1

        # Giving the Pi and Pf, and knowing the auxiliary point that will help to determine
        # the tangent at P0 and at P3, we  can determine the other middle points: P1, P2
        # P3 = P4, P5 and P6
        P3x_bezier, P3y_bezier, P3_x_aux, P3_y_aux, P4_x_aux, P4_y_aux = findCenterBezier(X, Y, ref[index_P3,1], ref[index_P3,2], obstacle_info, middle_path)

        ## FINDING THE BEZIER CURVES
        #  A cubic Bezier curve needs 4 control points:
        #  - First curve control points: P0, P1, P2 and P3
        #  - First curve control points: P4, P5, P6 and P7
        #  The auxiliary points P0_aux and P7_aux help to make sure the P1 and P6 control points
        #  will be located in a point over the tangent of P0 and P6 on the ref_path (to ensure
        #  continuity)

        # Control point within the ref_path, a couple of index before the start of the
        # local path
        P0_x_aux = ref[index-5,1]
        P0_y_aux = ref[index-5,2]
        # Value of (X,Y) at the current moment
        P0_x = X
        P0_y = Y
        # Last control point of the first Bezier is at the middle_path point
        P3_x = P3x_bezier
        P3_y = P3y_bezier

        # First control point of the second Bezier curve. It is the same point as the last
        # control point of the first Bezier curve
        P4_x = P3_x
        P4_y = P3_y

        # The 4th control point of the second Bezier curve is the (X,Y) index_P3
        P7_x = ref[index_P3,1]
        P7_y = ref[index_P3,2]
        # Control point within the ref_path, a couple of index after the start of the
        # local path
        P7_x_aux = ref[index_P3 + 5,1]
        P7_y_aux = ref[index_P3 + 5,2]

        # Find the helping points for the control points. The tangent here refers to the
        # tangent that ensures the continuity between the ref_path and the local path to be
        # constructed: the FindFirstTangent finds the tangent for the first Bezier curve and
        # the FindSecondTangent finds the tangent for the second Bezier curve
        Px_end1, Py_end1, Px_end2, Py_end2 = FindFirstTangent(P0_x_aux, P0_y_aux, P0_x, P0_y, P3_x_aux, P3_y_aux, P3_x, P3_y)
        Px_end3, Py_end3 = FindSecondTangent(P4_x_aux, P4_y_aux, P4_x, P4_y, P7_x_aux, P7_y_aux, P7_x, P7_y)

        # Finds the best combination of points that will give create a Bezier curve with the appropriate
        # curvature
        P1_x, P1_y, P2_x, P2_y = SearchFirstOptimalBezier(P0_x_aux, P0_y_aux, P0_x, P0_y, P3_x, P3_y, P3_x_aux, P3_y_aux, Px_end1, Py_end1, Px_end2, Py_end2, index)
        P5_x, P5_y, P6_x, P6_y = SearchSecondOptimalBezier(P2_x, P2_y,P3_x, P3_y, P4_x_aux, P4_y_aux, P4_x, P4_y, P7_x, P7_y, P7_x_aux, P7_y_aux, Px_end3, Py_end3, index, ref)

        x_bezier = []
        y_bezier = []
        # After finding all the control points for the first Bezier curve, this function computes the curve
        # and adds it to the (x,y)_bezier list;
        x, y = ComputeBezier(P0_x, P0_y, P1_x, P1_y, P2_x, P2_y, P3_x, P3_y)

        for k in range (len(x)):
          x_bezier.append(x[k])
          y_bezier.append(y[k])

        del x
        del y
        # After finding all the control points for the second Bezier curve, this function computes the curve
        # and adds it to the (x,y)_bezier list;
        x,y = ComputeBezier(P4_x_aux, P4_y_aux, P4_x, P4_y, P5_x, P5_y, P6_x, P6_y, P7_x, P7_y, P7_x_aux, P7_y_aux)
        for k in range (len(x)):
          if k>0:
            x_bezier.append(x[k])
            y_bezier.append(y[k])

        ### Saves the avoidance Path generated into a file
        saveBezier(x_bezier,y_bezier)

        ### Updates the reference path to the new one
        UpdatePath(index, X_prior, Y_prior, x_bezier, y_bezier, index_P3)

    '''
    *** IMPLEMENTED BEFORE ***
    Utilization of code already implemented for the control stage
    '''
    ### Command part
    point = geom.Point(X,Y)
    nearest_pt=line.interpolate(line.project(point))
    qw=np.column_stack((ref[:,1:2],psi_ref[:]))
    distance,index = spatial.KDTree(ref[:,1:3]).query(nearest_pt)


    xref=ref[index,1]
    yref=ref[index,2]

    #Saves the Path done by the Robot
    Path_done.append([X,Y])

    #Flag to avoid 2 obstacles
    if (index-bezier_length > 0):
      flag_new_path = 1


    Psiref=psi_ref[index]
    k=courbure[index]

    acc_y=k*vit**2
    vyref=0#mat[index,5]
    Psipref=0#vit*k#mat[index,6]
    ey=-sin(Psiref)*(X-xref)+cos(Psiref)*(Y-yref)
    epsi=(Psi-Psiref)

    vy=xhat[0,0]
    eyy=xhat[2,0]
    epsih=xhat[3,0]
    vpsih=xhat[1,0]

    x=np.array([[vy],[Psip],[ey],[epsi]])

    yd=np.array([[vyref],[Psipref],[0]])

    y=Cc.dot(x)

    Ac = get_model_A_matrix(k,Cf,Cr,Vxx)


    ##########################################################################
    ############    Slope angles estimation    ###############################
    ##########################################################################

    #x_slope=Luenberger(vy,vpsih,ax,ay,az,vit,x_slope,wx,wy,wz,Psi,phi,theta,uk_1)
    # theta_r=x_slope[0,0]
    # phi_r=x_slope[1,0]

    # print "theta_r",theta_r*180/pi




    ## Steady state vectors

    #         Vyss=-(k*vit*(Ac[(0,1)]*Bc[(1,0)] - Ac[(1,1)]*Bc[(0,0)] - Ac[(0,1)]*Bc[(1,1)] + Ac[(1,1)]*Bc[(0,1)]))/(Ac[(0,0)]*Bc[(1,0)] - Ac[(1,0)]*Bc[(0,0)] - Ac[(0,0)]*Bc[(1,1)] +  Ac[(1,0)]*Bc[(0,1)])
    #         Vpsiss=vit*k
    #         eyss=0;
    #         epsiss=-Vyss/vit;
    #
    #         bfss=-(k*vit*(Ac[(0,0)]*Ac[(1,1)] - Ac[(0,1)]*Ac[(1,0)]))/(Ac[(0,0)]*Bc[(1,0)] - Ac[(1,0)]*Bc[(0,0)] - Ac[(0,0)]*Bc[(1,1)] +  Ac[(1,0)]*Bc[(0,1)])
    #         brss=-bfss;
    #
    #         xsss=np.array([[Vyss],[Vpsiss],[eyss],[epsiss]])
    #         usss=np.array([[bfss],[brss]])





    ##########################################################################
    ############    Steady state     ##########################################
    ##########################################################################


    theta_r=0
    phi_r=0

    Vyss=(k*Vxx*(Ac[(1,1)]*(Bc[(0,0)]-Bc[(0,1)])-Ac[(0,1)]*(Bc[(1,0)]-Bc[(1,1)]))-g*cos(phi_r)*sin(theta_r)*(Bc[(1,0)]-Bc[(1,1)]))/(Ac[(0,0)]*(Bc[(1,0)]-Bc[(1,1)])-Ac[(1,0)]*(Bc[(0,0)]-Bc[(0,1)]))
    Vpsiss=Vxx*k
    eyss=0
    epsiss=-Vyss/Vxx

    bfss=(k*Vxx*(Ac[(0,0)]*Ac[(1,1)]-Ac[(1,0)]*Ac[(0,1)]) + Ac[(1,0)]*g*cos(phi_r)*sin(theta_r))/(-Ac[(0,0)]*(Bc[(1,0)]-Bc[(1,1)])+Ac[(1,0)]*(Bc[(0,0)]-Bc[(0,1)]))
    brss=-bfss

    xsss=np.array([[Vyss],[Vpsiss],[eyss],[epsiss]])
    usss=np.array([[bfss],[brss]])







    ##########################################################################
    ############    MPC     ##################################################
    ##########################################################################


    Phi,Gamma,Cd = SysDiscretise(Ac,Bc,Cc,0.3)

    [Pxi,Pu] = Defin_matrice_Pxi_Pu(Phi,Gamma,Cd,Np)

    Yref = reff(Np,vpsih,ey,epsi)

    u,Erms = QP_matrices(Pxi,Pu,Yref,xhat,Vxx,Phi,Gamma,Cd,Np,usss,xsss,uk_1)

    Vxx=vit#
    Vxxx=0#LongControl(Erms,vit,k)
    #
    #         print 'vxxx',Vxxx

    #         print "vit",Vxx


    uu=u+usss
    uk_1=u+usss#np.array([[u[0]],[u[1]]])

    T = np.array([[1/vit, a/vit, 0, 0],[1/vit, -b/vit, 0, 0]])

    betaf,betar=SlipAngles(Vxx,vy,vpsih,u[0],u[1])+T.dot(xsss)-usss
    beta=np.array([betaf,betaf])

    # #999 functions too
    #print "u", 180/pi*uu
    #print "beta", 180/pi*beta

    Cff=Cf
    Crr=Cr
    Fyf_obs=betaf*Cf
    Fyr_obs=betar*Cr


    xhat,Gammax=kalman(xhat,Gammax,dt*Bc.dot(u),y,Gammaalpha,Gammabeta,eye(4,4)+dt*Ac,Cc)

    cmd.linear_speed = Vxx

    cmd.steering_angle_front = uu[0]
    cmd.steering_angle_rear = uu[1]

    betaM = 6*(pi/180)
    betam = -6*(pi/180)
    deltaM = 10*(pi/180)
    deltam = -10*(pi/180)

    if uu[0]>deltaM :
      uu[0]=deltaM
    if uu[0]<deltam :
      uu[0]=deltam

    if uu[1]>deltaM :
     uu[1]=deltaM
    if uu[1]<deltam :
     uu[1]=deltam

    if betaf>betaM :
      betaf=betaM
    if betaf<betam :
      betaf=betam

    if betar>betaM :
      betar=betaM
    if betar<betam :
      betar=betam


    cmd_publisher.publish(cmd)

    #posture = np.array([X,Y,Psi,ey,epsi,vy,Psip,u[0,0],u[1,0],xref,yref,Psiref,vpsih,eyy,epsih], dtype=np.float32)



    posture = np.array([X,Y,Psi,ey,epsi,vy,Psip,u[0],u[1],xref,yref,Psiref,vpsih,eyy,epsih,Cff,Crr,betaf,betar,Fyf_obs,Fyr_obs,acc_y,Vxxx,theta_r,phi_r,uu[0],uu[1]], dtype=np.float32)
    data_pub.publish(posture)

    #         Vxx=LongControl(Erms,vit,k)
    #
    #         print "vit",Vxx

    r.sleep()

  cmd.steering_angle_front = 0
  cmd.steering_angle_rear = 0
  cmd.linear_speed = 0
  cmd_publisher.publish(cmd)


  '''
  *** NEW IMPLEMENTATION ***
  Modification of the old cold to add the Point cloud information
  '''
  saveNewList(Path_done, "Path_done")

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException: pass
