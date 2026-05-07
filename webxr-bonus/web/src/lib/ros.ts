// @ts-expect-error roslib has no types
import ROSLIB from 'roslib';
import { useStore, type Particle } from './store';
import { quatToYaw } from './utils';

let ros: any = null;

export function connectRos(url: string) {
  if (ros) try { ros.close(); } catch {}
  const store = useStore.getState();
  ros = new ROSLIB.Ros({ url });
  ros.on('connection', () => store.setConnected(true));
  ros.on('error', () => store.setConnected(false));
  ros.on('close', () => store.setConnected(false));

  const sub = (name: string, type: string, cb: (m: any) => void) =>
    new ROSLIB.Topic({ ros, name, messageType: type }).subscribe(cb);

  const onPose = (which: 'KF' | 'EKF' | 'PF') => (m: any) => {
    const p = m.pose.pose.position;
    const cov = m.pose.covariance as number[];
    const sample = {
      x: p.x, y: p.y,
      yaw: quatToYaw(m.pose.pose.orientation),
      covXY: cov?.[0],
      covYaw: cov?.[35],
    };
    if (which === 'KF') store.setKF(sample);
    else if (which === 'EKF') store.setEKF(sample);
    else store.setPF(sample);
  };

  sub('/kf/pose', 'geometry_msgs/msg/PoseWithCovarianceStamped', onPose('KF'));
  sub('/ekf/pose', 'geometry_msgs/msg/PoseWithCovarianceStamped', onPose('EKF'));
  sub('/pf/pose', 'geometry_msgs/msg/PoseWithCovarianceStamped', onPose('PF'));
  sub('/webxr/particles', 'geometry_msgs/msg/PoseArray', (m: any) => {
    const arr: Particle[] = m.poses.map((pp: any) => ({
      x: pp.position.x,
      y: pp.position.y,
      yaw: quatToYaw(pp.orientation),
    }));
    store.setParticles(arr);
  });

  return ros;
}
