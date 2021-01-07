#!/usr/bin/python
'''
This code is provided for internal research and development purposes by Huawei solely,
in accordance with the terms and conditions of the research collaboration agreement of May 7, 2020.
Any further use for commercial purposes is subject to a written agreement.
'''
"""
Zurich Eye
"""

import numpy as np
import matplotlib.pyplot as plt
import ze_py.plot_utils as plot_utils
import ze_py.transformations as tf
from pylab import setp
import ze_imu.imu_trajectory_simulation as imu_sim
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import rc

# tell matplotlib to use latex font
rc('font',**{'family':'serif','serif':['Cardo']})
rc('text', usetex=True)

gravity = 9.81
g = np.array([0, 0, -gravity])

def renormalize_quaternion_in_state(state):
    state[:4] /= np.linalg.norm(state[:4])
    return state
       
def integrate_expmap(t_W_B0, v_W_B0, R_W_B0, B_a_W_B, B_w_W_B, dt):
    n = np.shape(B_a_W_B)[0]
    t_W_B = np.zeros((n,3))
    t_W_B[0,:] = t_W_B0
    v_W_B = np.zeros((n,3))
    v_W_B[0,:] = v_W_B0
    R_W_B = np.zeros((n, 9))
    R_W_B[0,:] = R_W_B0
    for i in range(1,n):
        R = np.reshape(R_W_B[i-1,:], (3,3))
        a = B_a_W_B[i-1,:]
        w = B_w_W_B[i-1,:]
        t_W_B[i,:] = t_W_B[i-1,:] + v_W_B[i-1,:] * dt  + g * (dt**2) * 0.5  + np.dot(R, a * (dt**2) * 0.5)
        v_W_B[i,:] = v_W_B[i-1,:] + g * dt + np.dot(R, a * dt)
        R_W_B[i,:] = np.reshape(np.dot(R, tf.expmap_so3(w * dt)), (9,))
      
    return R_W_B, v_W_B, t_W_B
     
def integrate_quaternion_zero_order_hold(t_W_B0, v_W_B0, R_W_B0, B_a_W_B, B_w_W_B, dt):
    n = np.shape(B_a_W_B)[0]
    t_W_B = np.zeros((n,3))
    t_W_B[0,:] = t_W_B0
    v_W_B = np.zeros((n,3))
    v_W_B[0,:] = v_W_B0
    R_W_B = np.zeros((n, 9))
    R_W_B[0,:] = R_W_B0
    for i in range(1,n):
        # Last state:
        R_W_Bk = np.reshape(R_W_B[i-1,:], (3,3))
        #q_Bk_W = tf.quaternion_from_matrix(tf.convert_3x3_to_4x4(np.transpose(R_W_Bk)))
        q_Bk_W = tf.quaternion_from_matrix(tf.convert_3x3_to_4x4(R_W_Bk))
        a = B_a_W_B[i-1,:]
        w = B_w_W_B[i-1,:]
 
        Theta = np.eye(4) + 0.5 * dt * tf.quat_Omega(w) # Eq. 124 Quat. Tutorial
        q_Bkp1_W = np.dot(Theta, q_Bk_W)                # Eq. 111 Quat. Tutorial
        #R_W_B[i,:] = np.reshape(np.transpose(tf.matrix_from_quaternion(q_Bkp1_W)[:3,:3]), (9,))
        R_W_B[i,:] = np.reshape(tf.matrix_from_quaternion(q_Bkp1_W)[:3,:3], (9,))
        v_W_B[i,:] = v_W_B[i-1,:] + g * dt + np.dot(R_W_Bk, a) * dt
        t_W_B[i,:] = t_W_B[i-1,:] + v_W_B[i-1,:] * dt
    
    return R_W_B, v_W_B, t_W_B
   
def integrate_quaternion_runge_kutta_4(t_W_B0, v_W_B0, R_W_B0, B_a_W_B, B_w_W_B, dt):
    n = np.shape(B_a_W_B)[0]
    t_W_B = np.zeros((n,3))from pylab import setp
    t_W_B[0,:] = t_W_B0
    v_W_B = np.zeros((n,3))
    v_W_B[0,:] = v_W_B0
    R_W_B = np.zeros((n, 9))
    R_W_B[0,:] = R_W_B0
    
    def state_derivative(imu_acc, imu_gyr, state, dt):
        q_W_B = state[:4]
        v     = state[4:7]
        R_W_B = tf.matrix_from_quaternion(q_W_B)[:3,:3]
        Omega = tf.quat_Omega(imu_gyr)
        
        # Quaternion derivative according to MARS-Lab Quaternion Tutorial.
        q_dot = 0.5 * np.dot(Omega, q_W_B)
        v_dot = np.array([0.0, 0.0, -gravity]) + np.dot(R_W_B, imu_acc)
        p_dot = v # + dt * np.array([0.0, 0.0, -gravity]) + dt * np.dot(R_W_B, imu_acc)
        state_dot = np.concatenate((q_dot, v_dot, p_dot))
        return state_dot
    
    for i in range(1,n):
        # Last state:
        R = np.reshape(R_W_B[i-1,:], (3,3))
        q_W_B = tf.quaternion_from_matrix(tf.convert_3x3_to_4x4(R))
        v = v_W_B[i-1,:]
        p = t_W_B[i-1,:]
      
        # Get imu measurements from last, current timestamp and interpolate in between.
        imu_acc_1 = B_a_W_B[i-1,:]
        imu_acc_3 = B_a_W_B[i,:]
        imu_acc_2 = 0.5 * (imu_acc_1 + imu_acc_3)
        imu_gyr_1 = B_w_W_B[i-1,:]
        imu_gyr_3 = B_w_W_B[i,:]
        imu_gyr_2 = 0.5 * (imu_gyr_1 + imu_gyr_3)
      
        # Runge-Kutta Integration:
        state_1 = np.concatenate((q_W_B, v, p))
        state_1_dot = state_derivative(imu_acc_1, imu_gyr_1, state_1, 0.0)
        
        state_2 = state_1 + 0.5 * dt * state_1_dot
        state_2 = renormalize_quaternion_in_state(state_2)
        state_2_dot = state_derivative(imu_acc_2, imu_gyr_2, state_2, 0.5 * dt)
        
        state_3 = state_1 + 0.5 * dt * state_2_dot
        state_3 = renormalize_quaternion_in_state(state_3)
        state_3_dot = state_derivative(imu_acc_2, imu_gyr_2, state_3, 0.5 * dt)
        
        state_4 = state_1 + 1.0 * dt * state_3_dot
        state_4 = renormalize_quaternion_in_state(state_4)
        state_4_dot = state_derivative(imu_acc_3, imu_gyr_3, state_4, dt)
        
        integrated_state = \
            state_1 + 1.0 / 6.0 * dt * (state_1_dot + 2.0 * state_2_dot + 2.0 * state_3_dot + state_4_dot) 
        integrated_state = renormalize_quaternion_in_state(integrated_state)

        # Save next state:
        R = tf.matrix_from_quaternion(integrated_state[:4])[:3,:3]
        R_W_B[i,:] = np.reshape(R, (9,))
        v_W_B[i,:] = integrated_state[4:7]
        t_W_B[i,:] = integrated_state[7:10]
    
    return R_W_B, v_W_B, t_W_B
    
def rotation_estimation_error(R_W_Bgt, R_W_Bes):
    n = np.shape(R_W_Bgt)[0]
    errors = np.zeros(n)
    for i in range(0,n):
        R_gt = np.reshape(R_W_Bgt[i,:], (3,3))
        R_es = np.reshape(R_W_Bes[i,:], (3,3))
        R_err = np.dot(np.transpose(R_gt), R_es)
        errors[i] = np.linalg.norm(tf.logmap_so3(R_err))
    errors = errors * 180.0 / np.pi
    return errors
    
def translation_estimation_errors(t_W_Bgt, t_W_Bes):
    errors = np.sqrt(np.sum((t_W_Bgt - t_W_Bes)**2, 1))
    return errors
    
def experiment1():
    
    t_max = 0.25
    dt = 1.0/800.0
    n = int(t_max / dt)
    
    # -----------------------------------------------------------------------------
    # Generate measurements and ground-truth data.
    
    t_W_B, v_W_B, R_W_B, B_a_W_B, B_w_W_B = imu_sim.get_simulated_imu_data(t_max, dt)
    
    ax = imu_sim.plot_trajectory(t_W_B, R_W_B)
    
    # -------------------------------------------------------------------------
    # Verification: Repeat integration to see how big the integration error is
    
    R_expm, v_expm, t_expm = integrate_expmap(
        t_W_B[0,:], v_W_B[0,:], R_W_B[0,:], B_a_W_B, B_w_W_B, dt)
        
    R_qrk4, v_qrk4, t_qrk4 = integrate_quaternion_runge_kutta_4(
        t_W_B[0,:], v_W_B[0,:], R_W_B[0,:], B_a_W_B, B_w_W_B, dt)
        
    ax.plot(t_expm[:,0], t_expm[:,1], t_expm[:,2], '-r', label='Proposed')
    ax.plot(t_qrk4[:,0], t_qrk4[:,1], t_qrk4[:,2], '-g', label='Runge-Kutta 4th order')
    ax.legend()
    plot_utils.axis_equal_3d(ax)
    
    
    # Compute rotation error:
    R_err_expm = rotation_estimation_error(R_W_B, R_expm)
    R_err_qrk4 = rotation_estimation_error(R_W_B, R_qrk4)
    t_err_expm = translation_estimation_errors(t_W_B, t_expm)
    t_err_qrk4 = translation_estimation_errors(t_W_B, t_qrk4)
    
    fig = plt.figure(figsize=(8,4))
    ax = fig.add_subplot(211, title='Rotation Error', ylabel='Rotation error [deg]')
    times = np.arange(0, n*dt*1000, dt*1000)
    ax.plot(times, R_err_expm, '-b', label='Proposed', lw=3)
    ax.plot(times, R_err_qrk4, '-g', label='Runge-Kutta 4th order', lw=3)
    
    ax = fig.add_subplot(212, title='Translation Error', ylabel='Translation error [m]', xlabel='Time [ms]')
    ax.plot(times, t_err_expm, '-b', lw=3)
    ax.plot(times, t_err_qrk4, '-g', lw=3)
    ax.legend()

def experiment2():
    
    N = 100
    t_interval = 0.25 
    t_max = (N+1) * t_interval
    dt = 1.0 / 200.0
    n_per_interval = t_interval / dt
    
    # -----------------------------------------------------------------------------
    # Generate measurements and ground-truth data.
    
    t_W_B, v_W_B, R_W_B, B_a_W_B, B_w_W_B = imu_sim.get_simulated_imu_data(t_max, dt)
    
    ax, fig = imu_sim.plot_trajectory(t_W_B, R_W_B, interval=10)
    fig.savefig('imu_sim_trajectory.pdf')
    
    # -------------------------------------------------------------------------
    # Verification: Repeat integration to see how big the integration error is
    R_err_expm = np.zeros(N)
    R_err_qrk4 = np.zeros(N)
    t_err_expm = np.zeros(N)
    t_err_qrk4 = np.zeros(N)
    for i in np.arange(N):
        s = i * n_per_interval
        e = (i+1) * n_per_interval        

        # Integrate Segment        
        R_expm, v_expm, t_expm = integrate_expmap(
            t_W_B[s,:], v_W_B[s,:], R_W_B[s,:], B_a_W_B[s:e,:], B_w_W_B[s:e,:], dt)
            
        R_qrk4, v_qrk4, t_qrk4 = integrate_quaternion_runge_kutta_4(
            t_W_B[s,:], v_W_B[s,:], R_W_B[s,:], B_a_W_B[s:e,:], B_w_W_B[s:e,:], dt)
        
        
        # Compute rotation error:
        R_err_expm[i] = rotation_estimation_error(R_W_B[s:e,:], R_expm)[-1]
        R_err_qrk4[i] = rotation_estimation_error(R_W_B[s:e,:], R_qrk4)[-1]
        t_err_expm[i] = translation_estimation_errors(t_W_B[s:e,:], t_expm)[-1]
        t_err_qrk4[i] = translation_estimation_errors(t_W_B[s:e,:], t_qrk4)[-1]
    
    fig = plt.figure(figsize=(8,8))
    bins = np.linspace(0,np.max(R_err_expm),20)
    ax = fig.add_subplot(211, xlabel='Rotation error [deg]')
    ax.hist(R_err_expm, bins=bins, color='b', label='Proposed')
    ax.hist(R_err_qrk4, bins=bins, color='g', label='Runge-Kutta 4th order')
    ax.legend()
    
    bins = np.linspace(0,np.max(t_err_expm),20)
    ax = fig.add_subplot(212, xlabel='Translation error [m]')
    ax.hist(t_err_expm, bins=bins, color='b')
    ax.hist(t_err_qrk4, bins=bins, color='g')
    fig.savefig('imu_sim_error_hist.png')
    
    
def experiment3():
    
    N = 50
    t_interval = 0.25 
    t_max = (N+1) * t_interval
    hz_range = np.arange(100, 1000, 200) 
    
    R_err_expm_all = []
    R_err_qrk4_all = []
    t_err_expm_all = []
    t_err_qrk4_all = []
    
    for hz in hz_range:
        print('Computing errors for hz = ' + str(hz))
        
        dt = 1.0 / hz
        n_per_interval = t_interval / dt
        
        # -----------------------------------------------------------------------------
        # Generate measurements and ground-truth data.
        
        t_W_B, v_W_B, R_W_B, B_a_W_B, B_w_W_B = imu_sim.get_simulated_imu_data(t_max, dt)
        
        #ax, fig = imu_sim.plot_trajectory(t_W_B, R_W_B, interval=10)
        #fig.savefig('imu_sim_trajectory.png')
        
        # -------------------------------------------------------------------------
        # Verification: Repeat integration to see how big the integration error is
        R_err_expm = np.zeros(N)
        R_err_qrk4 = np.zeros(N)
        t_err_expm = np.zeros(N)
        t_err_qrk4 = np.zeros(N)
        for i in np.arange(N):
            s = i * n_per_interval
            e = (i+1) * n_per_interval        
    
            # Integrate Segment        
            R_expm, v_expm, t_expm = integrate_expmap(
                t_W_B[s,:], v_W_B[s,:], R_W_B[s,:], B_a_W_B[s:e,:], B_w_W_B[s:e,:], dt)
                
            R_qrk4, v_qrk4, t_qrk4 = integrate_quaternion_runge_kutta_4(
                t_W_B[s,:], v_W_B[s,:], R_W_B[s,:], B_a_W_B[s:e,:], B_w_W_B[s:e,:], dt)
            
            
            # Compute rotation error:
            R_err_expm[i] = rotation_estimation_error(R_W_B[s:e,:], R_expm)[-1]
            R_err_qrk4[i] = rotation_estimation_error(R_W_B[s:e,:], R_qrk4)[-1]
            t_err_expm[i] = translation_estimation_errors(t_W_B[s:e,:], t_expm)[-1]
            t_err_qrk4[i] = translation_estimation_errors(t_W_B[s:e,:], t_qrk4)[-1]
        
        R_err_expm_all.append(R_err_expm)
        R_err_qrk4_all.append(R_err_qrk4)
        t_err_expm_all.append(t_err_expm)
        t_err_qrk4_all.append(t_err_qrk4)
        
        
        
    def set_boxplot_colors(pb, color):
        setp(pb['boxes'][0], color=color)
        setp(pb['caps'][0], color=color)
        setp(pb['caps'][1], color=color)
        setp(pb['whiskers'][0], color=color)
        setp(pb['whiskers'][1], color=color)
        #setp(pb['fliers'][0], color=color)
        #setp(pb['fliers'][1], color=color)
        setp(pb['medians'][0], color=color)
        
        
    n_exp = 2
    n_dist = len(hz_range)
    spacing = 2
    pos = np.arange(0, n_dist*(n_exp+spacing), (n_exp+spacing))

    fig = plt.figure(figsize=(6,5))
    ax_rot = fig.add_subplot(211, ylabel='Rotation error [deg]')
    ax_pos = fig.add_subplot(212, ylabel='Translation error [m]', xlabel='IMU Rate [Hz]')
    
    dummy_plots_rot = []
    dummy_plot_rot = ax_rot.plot([1,1], '-', color='b', label="Proposed")
    dummy_plots_rot.append(dummy_plot_rot[0])    
    dummy_plot_rot = ax_rot.plot([1,1], '-', color='g', label="Runge-Kutta 4th order")
    dummy_plots_rot.append(dummy_plot_rot[0])

    for idx_hz, hz in enumerate(hz_range):     
        pb = ax_rot.boxplot(R_err_expm_all[idx_hz], False, '', positions=[pos[idx_hz]+0], widths=0.8)
        set_boxplot_colors(pb, 'b')
    
        pb = ax_pos.boxplot(t_err_expm_all[idx_hz], False, '', positions=[pos[idx_hz]+0], widths=0.8)
        set_boxplot_colors(pb, 'b')
        
        pb = ax_rot.boxplot(R_err_qrk4_all[idx_hz], False, '', positions=[pos[idx_hz]+1], widths=0.8)
        set_boxplot_colors(pb, 'g')
    
        pb = ax_pos.boxplot(t_err_qrk4_all[idx_hz], False, '', positions=[pos[idx_hz]+1], widths=0.8)
        set_boxplot_colors(pb, 'g')
           
    
        
    ax_rot.set_xticks(pos+0.5*n_exp-0.5)
    ax_rot.set_xticklabels(hz_range)
    ax_rot.set_xlim(xmin=pos[0]-2, xmax=pos[-1]+5)
    ax_rot.set_ylim([-0.001, np.max(R_err_expm_all[0])])
    
    ax_pos.set_xticks(pos+0.5*n_exp-0.5)
    ax_pos.set_xticklabels(hz_range)
    ax_pos.set_xlim(xmin=pos[0]-2, xmax=pos[-1]+5)
    ax_pos.set_ylim([-0.00001, np.max(t_err_expm_all[0])])

    
    
    # create legend
    ax_rot.legend(dummy_plots_rot, ['Proposed','Runge-Kutta 4th order'], loc='upper right')
    for p in dummy_plots_rot:
        p.set_visible(False)
        

    fig.tight_layout()
    fig.savefig('imu_sim_errors.pdf', bbox_inches="tight")
    #
    #fig_yaw.tight_layout()
    #fig_yaw.savefig('vicon_yaw_error_comparison'+FORMAT, bbox_inches="tight")
    #
    #fig_g.tight_layout()
    #fig_g.savefig('vicon_gravity_error_comparison'+FORMAT, bbox_inches="tight")