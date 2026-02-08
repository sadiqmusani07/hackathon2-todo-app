import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'ROS 2 Fundamentals',
    description: (
      <>
        Learn about nodes, topics, and services in ROS 2 - the backbone of robotic communication systems.
      </>
    ),
  },
  {
    title: 'AI Integration',
    description: (
      <>
        Connect AI decision-making systems to humanoid robot bodies using Python and rclpy.
      </>
    ),
  },
  {
    title: 'Humanoid Modeling',
    description: (
      <>
        Design and model humanoid robots using URDF for simulation and real-world deployment.
      </>
    ),
  },
];

function Feature({title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}