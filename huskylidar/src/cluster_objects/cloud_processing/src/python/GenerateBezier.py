#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import numpy as np
from math import cos,sin, atan, sqrt
from lmpc import bezier_AllPoints
from functions import *
from numpy import *

def saveBezier(x_bezier,y_bezier):
  """ Function used to save the local path created using the two Bezier curves

  Args:
      (x,y)_bezier : List containing the (x,y) points of the local path
  """
  file = open("Robot_ws/src/Robot_folder/mpc_primitives/scripts/Reference/BezierAvoidancePath.txt","w")
  i = 0
  while (i < len(x_bezier)):
    file.write('  '.join((str("%8.5f" % 0.0), str("%8.5f" % x_bezier[i]), str("%8.5f" % y_bezier[i]))) + '\n')
    i = i + 1
  file.close()

def saveNew(ref, name_file):
  """ Function used to save path done by the robot during simulation, for analysis

  Args:
      ref       : List with (x,y) points
      name_file : name of the file that will be saved
  """
  file = open("Robot_ws/src/Robot_folder/mpc_primitives/scripts/Reference/AvoidancePath/" + name_file + ".txt","w")
  i = 0
  while (i < len(ref)):
    file.write('  '.join((str("%8.5f" % 0.0), str("%8.5f" % ref[i,1]), str("%8.5f" % ref[i,2]))) + '\n')
    i = i + 1
  file.close()

def saveNewList(ref, name_file):
  """Function used to save a list at the end of the simulation, for analysis

  Args:
      ref       : List with (x,y) points
      name_file : name of the file that will be saved
  """
  file = open("Robot_ws/src/Robot_folder/mpc_primitives/scripts/Reference/AvoidancePath/Error/" + name_file + ".txt","w")
  i = 0
  while (i < len(ref)):
    file.write('  '.join((str("%8.5f" % 0.0), str("%8.5f" % ref[i][0]),str("%8.5f" % ref[i][1]))) +'\n')
    i = i + 1
  file.close()

def findCenterBezier(x_init, y_init, x_end, y_end, obstacle_info, middle_path):
  """Finds the (x,y) coordinates of the connetion point for the two Bezier curves.

  Args:
      x_init        : x coordinate where the local path will start
      y_init        : y coordinate where the local path will start
      x_end         : x coordinate where the local path will end
      y_end         : y coordinate where the local path will end
      obstacle_info : Information about the obstacle coordinates and size.
      middle_path   : list with all the ref_path points between the init and end points

  Returns:
      The points P3 and P4 of the algorithm and its auxiliary points
  """
  # (Cx, Cy) is the (x,y) coordinates of the barycenter of the obstacle
  Cx     = obstacle_info[0]
  Cy     = obstacle_info[1]
  radius = obstacle_info[3] + 2

  # Line "r" connecting start end end points
  a1 = (y_end-y_init)/(x_end - x_init)
  b1 = y_init - a1*x_init

  # Line "s" perpendicular to the line r
  a2 = -1/a1
  b2 = Cy - a2*Cx

  # Finds the intersection of the line r with the bezier circle
  a = 1 + a2**2
  b = -2*Cx + 2*a2*b2 - 2*a2*Cy
  c = Cx**2 + Cy**2 + b2**2 -2*b2*Cy - radius**2

  Det = b**2 - 4*a*c

  # if Det < 0, there's no intersection lines. But this is not the case.
  # This is the only possible case (two intersections)
  if Det > 1e-4:
    u = (-b + sqrt(Det))/(2*a)
    #Point 1: canditate to be the middle point
    Px_1 = u
    Py_1 = a2*Px_1 + b2

    u = (-b - sqrt(Det))/(2*a)

    #Point 2: canditate to be the middle point
    Px_2 = u
    Py_2 = a2*Px_2 + b2

  # Finds the coordinate in the reference path, the closest to the insterction with the line r
  error = []
  for i in range (len(middle_path)):
    error.append(abs(middle_path[i][1] - (a2*middle_path[i][0] + b2)))
  path_index = error.index(min(error))
  path_perp_intersec = [middle_path[path_index][0], middle_path[path_index][1]]

  #Distance between each point and the tan line
  d1 = ((path_perp_intersec[0]-Px_1)**2 + (path_perp_intersec[1]-Py_1)**2)**0.5
  d2 = ((path_perp_intersec[0]-Px_2)**2 + (path_perp_intersec[1]-Py_2)**2)**0.5

  if d1 < d2:
    # Linear coeficient for the line tangent to the middle bezier point
    b3 = Py_1 - a1*Px_1      # Tangent line  at the point P2
    b4 = y_end - a2*x_end  # Perpendicular line  at the end point

    ## P_aux1: Point aux for the vector P3->P3' the first Bezier Curve
    #  - projection of the end point at the tangent line:
    Px_aux1 = (b4 - b3)/(a1 - a2)
    Py_aux1 = Px_aux1*a1 + b3

    b4 = y_init - a2*x_init # Perpendicular line  at the initial point

    #P_aux2: Point aux for the vector P0->P0' the second Bezier Curve
    #Projection of the initial point at the tangent line
    Px_aux2 = (b4 - b3)/(a1 - a2)
    Py_aux2 = Px_aux2*a1 + b3

    return Px_1, Py_1, Px_aux1, Py_aux1, Px_aux2, Py_aux2

  else:
    # Linear coeficient for the line tangent to the middle bezier point
    b3 = Py_2 - a1*Px_2      # Tangent line  at the point P2
    b4 = y_end - a2*x_end  # Perpendicular line  at the end point

    #Projection of the end point at the tangent line
    Px_aux1 = (b4 - b3)/(a1 - a2)
    Py_aux1 = Px_aux1*a1 + b3

    b4 = y_init - a2 * x_init

    #P_aux2: Point aux for the vector P0->P1 the second Bezier Curve
    Px_aux2 = (b4 - b3)/(a1 - a2)
    Py_aux2 = Px_aux2*a1 + b3

    return Px_2, Py_2, Px_aux1, Py_aux1, Px_aux2, Py_aux2

def FindFirstTangent(P0_x_aux, P0_y_aux, P0_x, P0_y, P3_x_aux, P3_y_aux, P3_x, P3_y):
  """ Finds the tangent to the ref_path curve at the point P0 using P0_aux, and finds the
      tangent to the ref_path curve at the point P3 using P3_aux.

  Args:
      P0_(x,y)_aux : coordinates of a point in the ref_path a couple of indices before the point P0
      P0_(x,y)     : coordinates of the robot at the moment it identifies a obstacle
      P3_(x,y)_aux : coordinates of a point in the ref_path a couple of indices after the point P3
      P3_(x,y)     : coordinates of the point of connection between the two Bezier curves

  Returns:
      Returns the limits of the vector that will be used find the optimal control points P1 and P2
  """
  # P_end1 and P_end2 are points that vary the length of the vector in order to to find the control points

  # Line coefficients of the line tangent to the ref_path at the P0 point
  a1 = (P0_y_aux-P0_y)/(P0_x_aux-P0_x)
  b1 = P0_y - a1*P0_x

  # Line coefficients of the line tangent to the bezier_circle at the P3 point
  a2 = (P3_y-P3_y_aux)/(P3_x-P3_x_aux)
  b2 = P3_y - a2*P3_x

  ## Finds the point that will limit the vector to find the optimal point control P1
  # Finds line perpendicular with the tangent at P0, but at the point P3
  a_perp = -1/a1
  b_perp = P3_y - a_perp*P3_x
  # Point of intersection of the line tangent at P0 and the perpendicular above
  Px_end1 = (b_perp - b1)/(a1-a_perp)
  Py_end1 = Px_end1*a1 + b1

  ## Finds the point that will limit the vector to find the optimal point control P2
  # Finds line perpendicular with the tangent at P3, but at the point P1
  a_perp = -1/a2
  b_perp = P0_y - a_perp*P0_x
  # Point of intersection of the tangent at P3 and the perpendicular above
  Px_end2 = (b_perp - b2)/(a2-a_perp)
  Py_end2 = Px_end2*a2 + b2

  return Px_end1, Py_end1, Px_end2, Py_end2

def  FindSecondTangent(P4_x_aux, P4_y_aux, P4_x, P4_y, P7_x_aux, P7_y_aux, P7_x, P7_y):
  """ Finds the tangent to the ref_path curve at the point P4 using P4_aux, and finds the
      tangent to the ref_path curve at the point P7 using P7_aux.

  Args:
      P4_(x,y)_aux : coordinates of a point in the line tangent to the bezier_circle a
                     couple of indices before the point P4
      P4_(x,y)     : coordinates of the point of connection between the two Bezier curves
      P7_(x,y)_aux : coordinates of a point in the ref_path a couple of indices after the point P7
      P7_(x,y)     : coordinates of the final point where we want the robot to finish the local path and
                     go back to the ref_path

  Returns:
      Returns the limits of the vector that will be used find the optimal control point P6.
  """
  # OBS: There's no need to find P4 here. Due to properties of the connection of two Bezier curves,
  # we have that P3 = P4. The control point P3 was find already.

  # Line coefficients of the line tangent to the ref_path at the P7 point
  a1 = (P7_y_aux-P7_y)/(P7_x_aux-P7_x)
  b1 = P7_y - a1*P7_x

  ## Finds the point that will limit the vector to find the optimal point control P6
  # Finds line perpendicular with the tangent at P7, but at the point P4
  a_perp = -1/a1
  b_perp = P4_y - a_perp*P4_x
  # Point of intersection of the line tangent at P7 and the perpendicular above
  Px_end3 = (b_perp - b1)/(a1-a_perp)
  Py_end3 = Px_end3*a1 + b1

  return Px_end3, Py_end3

def TrajRefer(trajX, trajY):
    '''
    *** IMPLEMENTED BEFORE ***
    Utilization of code already implemented for the control stage
    '''
    cpt     = trajX.size
    s_vect  = np.zeros(cpt)
    theta_s = np.zeros(cpt)
    c       = np.zeros(cpt)
    dc_ds   = np.zeros(cpt)

    # Curvilinear Abscissa s
    for i in range(1, cpt):
        s_vect[i] = s_vect[i-1] + hypot(trajX[i] - trajX[i-1], trajY[i] - trajY[i-1])

    # theta_s
    for i in range(cpt):
        if i<cpt-1:
            angle = atan2(trajY[i+1]-trajY[i], trajX[i+1]-trajX[i])
            theta_s[i] = angle
        else:
            angle = theta_s[cpt-2]
            theta_s[i] = angle
    # c
    for i in range(cpt):
        if i<cpt-1:
            c[i] = (theta_s[i+1]-theta_s[i])/(s_vect[i+1]-s_vect[i])
        elif i == cpt-1:
            c[i] = c[cpt-2]

    # dc/ds
    for i in range(cpt):
        if i<cpt-1:
            dc_ds[i] = (c[i+1]-c[i])/(s_vect[i+1]-s_vect[i])
        else:
            dc_ds[i] = dc_ds[cpt-2]
    return s_vect, theta_s, c, dc_ds

def SearchFirstOptimalBezier(P0_x_aux, P0_y_aux, P0_x, P0_y, P3_x, P3_y, P3_x_aux, P3_y_aux, Px_end1, Py_end1, Px_end2, Py_end2, index):
  """ Function to search for the optmal values of the control points P1 and P2.

  Args:
      P0_(x,y)_aux : coordinates of a point in the ref_path a couple of indices before the point P0
      P0_(x,y)     : First control point of the Bezier curve
      P3_(x,y)     : coordinates of the point of connection between the two Bezier curves
      P3_(x,y)_aux : coordinates of a point in the ref_path a couple of indices after the point P3
      P(x,y)_end1  : point that limits the vector that used find the optimal control point P1
      P(x,y)_end2  : point that limits the vector that used find the optimal control point P2
      index        : index points (x,y) from the ref_path

  Returns:
      Returns the optmal values for the control points P1 and P2
  """
  t1               = 0
  t2               = 0
  step_optmization = 10
  i                = 0
  j                = 0

  bezier_candidates = []

  for i in range (step_optmization):
    # First control point varies from P0 to Px_end1.
    P1_x = (Px_end1 - P0_x)*t1 + P0_x
    P1_y = (Py_end1 - P0_y)*t1 + P0_y

    for j in range (step_optmization):
      # First control point varies from P3 to Px_end2.
      P2_x = (Px_end2 - P3_x)*t2 + P3_x
      P2_y = (Py_end2 - P3_y)*t2 + P3_y

      # Verifies curvature conditions for the current configuration of control points P0, P1, P2 and P3.
      x, y, max_courbure = ComputeFirstMaxCourb(P0_x_aux, P0_y_aux, P0_x, P0_y, P1_x, P1_y, P2_x, P2_y, P3_x, P3_y, P3_x_aux, P3_y_aux, index)

      # If curvature is under limit set, add control points to candidates list
      if max_courbure < 0.246:
        bezier_candidates.append([P1_x,P1_y, P2_x, P2_y, max_courbure])
      t2 = t2 + 1/(float(step_optmization) - 1)

    t1 = t1 + 1/(float(step_optmization) - 1)
    t2 = 0

  best_bezier = np.array(bezier_candidates)
  # From candidate points, select the one with the least curvature
  index_best_bezier =  np.argmin(best_bezier[:,4])


  P1_x = bezier_candidates[index_best_bezier][0]
  P1_y = bezier_candidates[index_best_bezier][1]
  P2_x = bezier_candidates[index_best_bezier][2]
  P2_y = bezier_candidates[index_best_bezier][3]

  return P1_x, P1_y, P2_x, P2_y

def ComputeFirstMaxCourb(P0_x_aux, P0_y_aux, P0_x, P0_y, P1_x, P1_y, P2_x, P2_y, P3_x, P3_y, P3_x_aux, P3_y_aux, index):
  """[summary]

  Args:
      P0_(x,y)_aux : coordinates of a point in the ref_path a couple of indices before the point P0
      P0_(x,y)     : coordinates of the robot at the moment it identifies a obstacle
      P1_(x,y)     : coordinates of the control point P1
      P2_(x,y)     : coordinates of the control point P2
      P3_(x,y)     : coordinates of the point of connection between the two Bezier curves
      P3_(x,y)_aux : coordinates of a point in the ref_path a couple of indices after the point P3
      index        : index points (x,y) from the ref_path

  Returns:
      Parametric values of the curve (x,y) and the max value of curvature for the curve
  """
  step      = 100
  t         = 0
  x         = []
  y         = []
  x_current = P0_x_aux
  y_current = P0_y_aux
  s_current = 0
  x.append(x_current)
  y.append(y_current)

  for i in range(10) :
    # Parametrization of the curve from P0_aux to P0
    x_line = (P0_x - P0_x_aux)*t + P0_x_aux
    y_line = (P0_y - P0_y_aux)*t + P0_y_aux
    s_current = ((x_current-x_line)**2+(y_current-y_line)**2)**0.5
    if s_current > 0.11:
      x_current = x_line
      y_current = y_line
      x.append(x_line)
      y.append(y_line)
    t += 1.0/(step-1)

  t = 0
  i = 0
  for i in range(step) :
    # Parametrization of the Bezier curve with the control points P0, P1, P2 and P3
    xt_bezier = P0_x*((1-t)**3) + P1_x*3*t*((1-t)**2) + P2_x*3*(t**2)*(1-t) + P3_x*(t**3)
    yt_bezier = P0_y*((1-t)**3) + P1_y*3*t*((1-t)**2) + P2_y*3*(t**2)*(1-t) + P3_y*(t**3)
    s_current = ((x_current-xt_bezier)**2+(y_current-yt_bezier)**2)**0.5
    if s_current > 0.13:
      x_current = xt_bezier
      y_current = yt_bezier
      x.append(xt_bezier)
      y.append(yt_bezier)
    t += 1.0/(step-1)
  t = 0
  i = 0

  for i in range(10) :
    # Parametrization of the curve from P3 to P3_aux
    x_line = (P3_x_aux - P3_x)*t + P3_x
    y_line = (P3_y_aux - P3_y)*t + P3_y
    s_current = ((x_current-x_line)**2 + (y_current-y_line)**2)**0.5
    if s_current > 0.11:
      x_current = x_line
      y_current = y_line
      x.append(x_line)
      y.append(y_line)
    t += 1.0/(step-1)

  # (x,y)_array is the curve going from P0_aux to P3_aux, passing through the Bezier curve
  x_array = np.array(x)
  y_array = np.array(y)
  # Finds the curvature for each point of the curve
  s_vect, psi_ref, courbure, dc_ds = TrajRefer(x_array,y_array)

  # Finds maximum absolute value of curvature for the curve parameterized
  min_courbure = min(courbure)
  max_courbure = max(courbure)
  max_courbure = max(abs(min_courbure),abs(max_courbure))
  return x, y, max_courbure

def SearchSecondOptimalBezier(P2_x, P2_y,P3_x, P3_y, P4_x_aux, P4_y_aux, P4_x, P4_y, P7_x, P7_y, P7_x_aux, P7_y_aux, Px_end3, Py_end3, index, ref):
  """ Function to search for the optmal values of the control points P6

  Args:
      P2_(x,y)     : coordinates of the control point P2
      P3_(x,y)     : coordinates of the control point P3
      P4_(x,y)_aux :coordinates of a point in the ref_path a couple of indices before the point P4
      P4_(x,y)     : coordinates of the control point P4
      P7_(x,y)     : coordinates of the control point P7
      P7_(x,y)_aux :coordinates of a point in the ref_path a couple of indices after the point P7
      P(x,y)_end3  : point that limits the vector that used find the optimal control point P6
      index        : index points (x,y) from the ref_path
      ref          : is the ref_path

  Returns:
      Returns the optmal values for the control points P6
  """
  t1               = 0
  t2               = 0
  i                = 0
  j                = 0
  P5_x             = (P3_x - P2_x)*2 + P2_x
  P5_y             = (P3_y - P2_y)*2 + P2_y
  step_optmization = 10

  bezier_candidates = []
  for j in range (step_optmization):
    # First control point varies from P7 to P_end3.
    P6_x = (Px_end3 - P7_x)*t1 + P7_x
    P6_y = (Py_end3 - P7_y)*t1 + P7_y
    x,y,max_courbure = ComputeSecondMaxCourb(P4_x_aux, P4_y_aux, P4_x, P4_y, P5_x, P5_y, P6_x, P6_y, P7_x, P7_y, P7_x_aux, P7_y_aux, index, ref)

    # If curvature is under limit set, add control points to candidates list
    if max_courbure < 0.246:
      bezier_candidates.append([P6_x, P6_y, max_courbure])
    t1 = t1+1/(float(step_optmization)-1)

  best_bezier = np.array(bezier_candidates)
  # From candidate points, select the one with the least curvature
  index_best_bezier =  np.argmin(best_bezier[:,2])

  P6_x = bezier_candidates[index_best_bezier][0]
  P6_y = bezier_candidates[index_best_bezier][1]

  return P5_x, P5_y, P6_x, P6_y

def ComputeSecondMaxCourb(P4_x_aux, P4_y_aux, P4_x, P4_y, P5_x, P5_y, P6_x, P6_y, P7_x, P7_y, P7_x_aux, P7_y_aux, index, ref):
  """[summary]

  Args:
      P4_(x,y)_aux :coordinates of a point in the ref_path a couple of indices before the point P4
      P4_(x,y)     : coordinates of the robot at middle point of the local path
      P5_(x,y)     : coordinates of the control point P5
      P6_(x,y)     : coordinates of the control point P6
      P7_(x,y)     : coordinates of the final point of the local path
      P7_(x,y)_aux : coordinates of a point in the ref_path a couple of indices after the point P7
      index        : index points (x,y) from the ref_path
      ref          : is the ref_path

  Returns:
      Parametric values of the curve (x,y) and the max value of curvature for the curve
  """
  step      = 100
  t         = 0
  x         = []
  y         = []
  x_current = P4_x_aux
  y_current = P4_y_aux
  s_current = 0

  i = 0
  for i in range(step) :
    # Parametrization of the Bezier curve with the control points P4, P5, P6 and P7
    xt_bezier = P4_x*((1-t)**3) + P5_x*3*t*((1-t)**2) + P6_x*3*(t**2)*(1-t) + P7_x*(t**3)
    yt_bezier = P4_y*((1-t)**3) + P5_y*3*t*((1-t)**2) + P6_y*3*(t**2)*(1-t) + P7_y*(t**3)
    s_current = ((x_current - xt_bezier)**2 + (y_current - yt_bezier)**2)**0.5
    if s_current > 0.13:
      x_current = xt_bezier
      y_current = yt_bezier
      x.append(xt_bezier)
      y.append(yt_bezier)
    t += 1.0/(step-1)

  t = 0
  i = 0
  for i in range(len(ref)) :
    # Parametrization of the curve from P7 to P7_aux
    x_line = (P7_x_aux - P7_x)*t + P7_x
    y_line = (P7_y_aux - P7_y)*t + P7_y
    s_current = ((x_current-x_line)**2 + (y_current - y_line)**2)**0.5
    if s_current > 0.11:
      x_current = x_line
      y_current = y_line
      x.append(x_line)
      y.append(y_line)
    t += 1.0/(step-1)

  # (x,y)_array is the curve going from P4 to P7_aux, passing through the Bezier curve
  x_array = np.array(x)
  y_array = np.array(y)

  # Finds the curvature for each point of the curve
  s_vect, psi_ref, courbure, dc_ds= TrajRefer(x_array,y_array)

  # Finds maximum absolute value of curvature for the curve parameterized
  min_courbure = min(courbure)
  max_courbure = max(courbure)
  max_courbure = max(abs(min_courbure),abs(max_courbure))
  return x, y, max_courbure

def ComputeBezier(x_current, y_current, P0_x, P0_y, P1_x, P1_y, P2_x, P2_y, P3_x, P3_y):
  """ Do the parametrization of a Bezier curve based on the current position of the robot
      and the control points of of the curve (P0, P1, P2 and P3)

  Args:
      (x,y)_current : current position of the robot
      P0_(x,y)      : First control point of the Bezier curve
      P1_(x,y)      : Second control point of the Bezier curve
      P2_(x,y)      : Third control point of the Bezier curve
      P3_(x,y)      : Last control point of the Bezier curve

  Returns:
      The (x,y) of the Bezier curve parameterized from P0 to P3.
  """
  step      = 100
  t         = 0
  x         = []
  y         = []
  s_current = 0

  i = 0
  # Parametric equation of the Bezier curve
  for i in range(step) :
    xt_bezier = P0_x*((1-t)**3) + P1_x*3*t*((1-t)**2) + P2_x*3*(t**2)*(1-t) + P3_x*(t**3)
    yt_bezier = P0_y*((1-t)**3) + P1_y*3*t*((1-t)**2) + P2_y*3*(t**2)*(1-t) + P3_y*(t**3)
    s_current = ((x_current - xt_bezier)**2 + (y_current - yt_bezier)**2)**0.5
    if s_current > 0.11:
      x_current = xt_bezier
      y_current = yt_bezier
      x.append(xt_bezier)
      y.append(yt_bezier)
    t += 1.0/(step-1)
  return x, y
