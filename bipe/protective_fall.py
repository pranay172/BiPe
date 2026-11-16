"""Estimator-only protective crouch; SI settings are supplied by robot.json.

This supervises finite actuator commands. It never changes a physical pose or
velocity. The portable C implementation receives the same estimate/settings.
"""
import math
import numpy as np
from .kinematics import equilibrium, posture

REASONS = ['none', 'pitch', 'roll', 'freefall', 'invalid']
SETTING_KEYS = ['pitch_rad', 'hard_pitch_rad', 'roll_rad', 'outward_rate_rad_s', 'persist_s',
                'freefall_m_s2', 'freefall_persist_s', 'posture_deg', 'reference_rate_rad_s',
                'crouch_s', 'ramp_s', 'wheel_ramp_s', 'joint_cap_Nm']

class ProtectiveFall:
    def __init__(self,p,gains):
        self.p,self.gains=p,gains
        self.settings=p['physics']['protective_fall']
        self.latched=self.torque_off=False
        self.exposure_s=self.freefall_s=self.age_s=0.
        self.reason=0
        self.qref=np.zeros(4);self.qdref=np.zeros(4);self.wheel=np.zeros(2)

    def leg_torques(self,q,qd):
        if not self.latched or self.torque_off:return np.zeros(4)
        c=self.settings
        scale=np.clip((c['crouch_s']+c['ramp_s']-self.age_s)/c['ramp_s'],0,1)
        return scale*np.clip(self.gains['joint_kp']*(self.qref-q)+
            self.gains['joint_kd']*(self.qdref-qd),-c['joint_cap_Nm'],c['joint_cap_Nm'])

    def step(self,e,previous,dt,valid=True):
        if self.torque_off:return np.zeros(6)
        c=self.settings
        observed=[e['pitch'],e['pitch_rate'],e['roll'],e['velocity'],e.get('accel_norm',9.81)]
        valid=valid and np.isfinite(observed).all() and observed[-1]>=0 and \
            np.isfinite(e['q']).all() and np.isfinite(e['qd']).all() and np.isfinite(previous).all()
        try:eq=equilibrium(self.p,e['q']) if valid else 0
        except ValueError:valid=False;eq=0
        if not valid:
            self.latched=self.torque_off=True;self.reason=4;return np.zeros(6)
        if not self.latched:
            error=e['pitch']-eq
            pitch=abs(error)>=c['pitch_rad'] and error*e['pitch_rate']>abs(error)*c['outward_rate_rad_s']
            roll=abs(e['roll'])>=c['roll_rad']
            self.exposure_s=self.exposure_s+dt if pitch or roll else 0.
            self.freefall_s=self.freefall_s+dt if observed[-1]<c['freefall_m_s2'] else 0.
            hard=abs(error)>=c['hard_pitch_rad']
            if not hard and self.exposure_s+1e-12<c['persist_s'] and \
                    self.freefall_s+1e-12<c['freefall_persist_s']:return np.zeros(6)
            self.latched=True;self.age_s=0.
            self.reason=3 if self.freefall_s+1e-12>=c['freefall_persist_s'] else (2 if roll else 1)
            self.qref=e['q'].copy();self.wheel=previous[4:].copy()
        else:self.age_s+=dt
        if self.age_s+1e-12>=c['crouch_s']+c['ramp_s']:
            self.torque_off=True;self.qdref[:]=0;return np.zeros(6)
        change=np.clip(posture(c['posture_deg'],self.p)-self.qref,
            -c['reference_rate_rad_s']*dt,c['reference_rate_rad_s']*dt)
        self.qref+=change;self.qdref=change/dt
        return np.r_[self.leg_torques(e['q'],e['qd']),self.wheel*max(0,1-self.age_s/c['wheel_ramp_s'])]
