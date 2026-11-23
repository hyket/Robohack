import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import math
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from core import Attitudes, Estimator, project, valid_quaternion
from metrics import evaluate


def scan(position, rot, rng, noise=.01):
    n=720; angles=-math.pi+np.arange(n)*2*math.pi/n
    origin=np.r_[position, .4]+rot.apply([.22,0,.11])
    directions=rot.apply(np.column_stack((np.cos(angles),np.sin(angles),np.zeros(n))))
    with np.errstate(divide='ignore',invalid='ignore'):
        tx=np.where(directions[:,0]>0,(5-origin[0])/directions[:,0],(-3-origin[0])/directions[:,0])
        ty=np.where(directions[:,1]>0,(5-origin[1])/directions[:,1],(-3-origin[1])/directions[:,1])
    ranges=np.minimum(tx,ty)+rng.normal(0,noise,n)
    return project(ranges,-math.pi,2*math.pi/n,.12,20.,rot,[.22,0,.11])


class TestOdometry(unittest.TestCase):
    def test_noisy_square_with_body_tilt_and_drift(self):
        rng=np.random.default_rng(42); est=Estimator(); xy=np.zeros(2); yaw=0.; t=0.
        gt=[]; od=[]; accepted=0; total=0
        def tick(moving,turning):
            nonlocal t,yaw,xy,accepted,total
            t+=.05; yaw+=turning*.05
            xy+=moving*.05*np.array([math.cos(yaw),math.sin(yaw)])
            tilt=.035 if moving or turning else .001
            rot=Rotation.from_euler('xyz',[tilt*math.sin(t*5),tilt*math.cos(t*7),yaw])
            pts,idx,a=scan(xy,rot,rng)
            result=est.update(pts,idx,a,t)
            accepted+=int(result.accepted); total+=1
            gt.append([t,*xy,0.,*rot.as_quat()]); od.append([t,*est.position,0.,*rot.as_quat()])
        for _ in range(20): tick(0.,0.)
        for _ in range(4):
            for _ in range(110): tick(.2,0.)
            for _ in range(105): tick(0.,.3)
        # Avoid changing the keyframe while drifting: 11 stationary seconds.
        stationary_start=t+.05
        for _ in range(221): tick(0.,0.)
        report=evaluate(np.array(gt),np.array(od),stationary_start)
        print('\nSynthetic square:',report,'accepted',accepted,'/',total)
        self.assertGreaterEqual(report['segments'],4)
        self.assertLess(report['mean_rpe'],.05)
        self.assertLess(report['drift_m'],.02)
        self.assertGreater(accepted/total,.98)

    def test_invalid_measurements(self):
        self.assertIsNone(valid_quaternion([0,0,0,0]))
        self.assertIsNone(valid_quaternion([0,0,float('nan'),1]))
        e=Estimator()
        self.assertFalse(e.update(np.zeros((3,2)),np.arange(3),0.,1.).accepted)
        np.testing.assert_array_equal(e.position,[0.,0.])

    def test_attitude_interpolates_wrap(self):
        a=Attitudes()
        a.add(1.,Rotation.from_euler('z',math.radians(179)).as_quat())
        a.add(1.05,Rotation.from_euler('z',math.radians(-179)).as_quat())
        self.assertAlmostEqual(a.at(1.025).as_euler('xyz')[2],math.radians(1),places=6)
        self.assertIsNone(a.at(1.1))
        self.assertFalse(a.add(.5,[0,0,0,1]))

    def test_degenerate_corridor_is_rejected(self):
        x=np.linspace(-4,4,200); pts=np.c_[x,np.full(200,3.)]; e=Estimator()
        self.assertTrue(e.update(pts,np.arange(200),0.,1.).accepted)
        self.assertEqual(e.update(pts-.01,np.arange(200),0.,1.05).reason,'degenerate_geometry')

    def test_metrics_detect_bad_odometry(self):
        t=np.arange(0,20,.05); q=np.tile([0,0,0,1],(len(t),1))
        truth=np.c_[t,t*.2,np.zeros((len(t),2)),q]; estimate=truth.copy()
        estimate[:,1]=0.
        self.assertAlmostEqual(evaluate(truth,estimate)['mean_rpe'],1.)

if __name__=='__main__': unittest.main()
