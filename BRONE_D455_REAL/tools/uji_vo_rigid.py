#!/usr/bin/env python3
"""Uji sintetis estimator gerak VO (rigid 2D RANSAC) vs metode lama. Pakai: python3 tools/uji_vo_rigid.py"""
import sys, math, numpy as np
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'rtabmap_rgbd', 'src'))
from rgbd_visual_odometry import estimate_rigid_2d_ransac
rng = np.random.default_rng(1)
dt = 1/15
for name, (vx, vy, wz) in {'putar murni 0.6rad/s': (0,0,0.6), 'maju 0.22': (0.22,0,0),
                           'geser 0.2': (0,0.2,0), 'maju+putar': (0.2,0.05,0.5)}.items():
    errs=[]
    for k in range(50):
        # landmark di frame base sebelumnya (b), robot bergerak (R,t): b = R a + t
        n=60; ang=rng.uniform(-0.7,0.7,n); d=rng.uniform(0.4,4.5,n)
        b=np.stack([d*np.cos(ang)+0.08, d*np.sin(ang)],1)
        th=wz*dt; t=np.array([vx*dt,vy*dt]); R=np.array([[math.cos(th),-math.sin(th)],[math.sin(th),math.cos(th)]])
        a=(b-t)@R  # R^T (b - t)
        a+=rng.normal(0,1,a.shape)*(0.003*np.linalg.norm(a,axis=1,keepdims=True)**2)
        a[:6]+=rng.uniform(-0.5,0.5,(6,2))  # outlier
        dth,te,inl,rms=estimate_rigid_2d_ransac(a,b,rng)
        errs.append([te[0]/dt-vx, te[1]/dt-vy, dth/dt-wz])
    e=np.array(errs)
    print(f'{name:22s} bias v=({e[:,0].mean():+.3f},{e[:,1].mean():+.3f}) m/s  std=({e[:,0].std():.3f},{e[:,1].std():.3f})  wz bias={e[:,2].mean():+.3f}')
# versi lama: delta median saat putar murni
b=np.stack([2*np.ones(60)+0.08, np.linspace(-1.5,1.5,60)],1); th=0.6*dt
R=np.array([[math.cos(th),-math.sin(th)],[math.sin(th),math.cos(th)]]); a=b@R
print('metode LAMA saat putar murni: vy palsu =', np.median(-(a-b)[:,1])/dt, 'm/s')
