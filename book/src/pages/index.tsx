import React, { JSX } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import styles from './index.module.css';

export default function Home(): JSX.Element {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="A complete interactive learning book about Embodied Intelligence, Robotics, and AI in the physical world."
    >
      <header className={styles.heroBanner}>
        <div className={styles.container}>
          <h1 className={styles.heroTitle}>Physical AI & Humanoid Robotics</h1>
          <p className={styles.heroSubtitle}>
            Bridging the gap between the digital brain and the physical body.
          </p>

          <div className={styles.buttons}>
            <Link className="button button--primary button--lg" to="/docs/intro">
              Start Learning 🚀
            </Link>

            <Link
              className="button button--secondary button--lg"
              to="https://github.com/Ayishaakhan786/Physical-AI-Humanoid-Robotics"
            >
              View on GitHub ⭐
            </Link>
          </div>
        </div>
      </header>

      <main>
        <section className={styles.section}>
          <div className={styles.container}>
            <h2>What You Will Learn</h2>

            <div className={styles.features}>
              <div className={styles.featureCard}>
                <h3>🤖 Embodied Intelligence</h3>
                <p>
                  Understand how AI interacts with the real world through sensors,
                  actuators, and robot bodies.
                </p>
              </div>

              <div className={styles.featureCard}>
                <h3>🦿 Humanoid Robotics</h3>
                <p>
                  Explore locomotion, manipulation, balance control, and humanoid robot
                  behaviors.
                </p>
              </div>

              <div className={styles.featureCard}>
                <h3>⚙️ Simulation Labs</h3>
                <p>
                  Hands-on labs using ROS, Gazebo, and Isaac Sim for real robot control
                  experience.
                </p>
              </div>

              <div className={styles.featureCard}>
                <h3>📡 Perception & Sensors</h3>
                <p>
                  Learn camera vision, depth sensing, LiDAR, IMU fusion, and SLAM.
                </p>
              </div>

              <div className={styles.featureCard}>
                <h3>🧠 AI Planning & Control</h3>
                <p>
                  Implement task planning, motion planning, and control loops using AI
                  algorithms.
                </p>
              </div>

              <div className={styles.featureCard}>
                <h3>💡 Projects & Case Studies</h3>
                <p>
                  Build real-world robotics projects with guided steps and code samples.
                </p>
              </div>
            </div>
          </div>
        </section>

        <section className={styles.sectionDark}>
          <div className={styles.container}>
            <h2>Why This Book?</h2>
            <p>
              Robotics is the future — and Physical AI is the next revolution. This book
              makes robotics accessible, practical, and exciting by combining AI theory
              with real-world robot interaction.
            </p>
          </div>
        </section>
      </main>
    </Layout>
  );
}