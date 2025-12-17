import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          Physical AI & Humanoid Robotics
        </Heading>
        <p className="hero__subtitle">AI-Native Capstone Textbook</p>
        <div className={styles.buttons}>
          <Link
            className="button button--primary button--lg"
            to="/docs/quarter-overview">
            Start Reading
          </Link>
          <button
            className="button button--secondary button--lg"
            onClick={() => document.getElementById('modules-section')?.scrollIntoView({ behavior: 'smooth' })}
            style={{ marginLeft: '1rem' }}
          >
            View Modules
          </button>
        </div>
      </div>
    </header>
  );
}

function HomepageContent() {
  return (
    <section className={styles.homepageContent}>
      <div className="container">
        {/* Modules Section */}
        <div id="modules-section">
          <Heading as="h2" className={styles.sectionTitle}>Modules</Heading>
          <div className={clsx('row', styles.moduleCards)}>
            {[
              {
                title: 'Module 1 — ROS 2',
                description: 'Learn the fundamentals of ROS 2 for robotic control and communication.',
              },
              {
                title: 'Module 2 — Digital Twin',
                description: 'Explore Gazebo for simulation and Unity for advanced visualization.',
              },
              {
                title: 'Module 3 — NVIDIA Isaac',
                description: 'Dive into Isaac Sim for AI robotics, synthetic data, and VSLAM.',
              },
              {
                title: 'Module 4 — VLA',
                description: 'Understand Vision-Language-Action systems for intelligent humanoid control.',
              },
            ].map((props, idx) => (
              <div key={idx} className={clsx('col col--3', styles.moduleCard)}>
                <Heading as="h3">{props.title}</Heading>
                <p>{props.description}</p>
              </div>
            ))}
          </div>
        </div>

        {/* Quarter Roadmap Section */}
        <Heading as="h2" className={styles.sectionTitle}>Quarter Roadmap</Heading>
        <div className={styles.roadmap}>
          <div className={styles.roadmapSteps}>
            <div className={styles.roadmapStep}>
              <div className={styles.stepNumber}>1</div>
              <div className={styles.stepContent}>
                <h3>ROS 2 fundamentals</h3>
                <p>Learn the Robotic Operating System 2 for robot communication and control.</p>
              </div>
            </div>
            <div className={styles.roadmapStep}>
              <div className={styles.stepNumber}>2</div>
              <div className={styles.stepContent}>
                <h3>Simulation (Gazebo + Unity)</h3>
                <p>Build digital twins with physics simulation and advanced visualization.</p>
              </div>
            </div>
            <div className={styles.roadmapStep}>
              <div className={styles.stepNumber}>3</div>
              <div className={styles.stepContent}>
                <h3>NVIDIA Isaac (AI Robot Brain)</h3>
                <p>Implement AI perception and planning with Isaac Sim and ROS packages.</p>
              </div>
            </div>
            <div className={styles.roadmapStep}>
              <div className={styles.stepNumber}>4</div>
              <div className={styles.stepContent}>
                <h3>Vision-Language-Action & Capstone</h3>
                <p>Design intelligent humanoid behaviors with VLA systems and complete your project.</p>
              </div>
            </div>
          </div>
        </div>

        {/* Learning Outcomes Section */}
        <Heading as="h2" className={styles.sectionTitle}>Learning Outcomes</Heading>
        <ul className={styles.learningOutcomesGrid}>
          <li>Master ROS 2 for robot control</li>
          <li>Build digital twin simulations</li>
          <li>Use NVIDIA Isaac for AI robotics</li>
          <li>Design VLA-based humanoid behaviors</li>
          <li>Understand Physical AI & ethics</li>
          <li>Develop embodied AI systems</li>
        </ul>

        {/* Coming Soon Buttons Row */}
        <div className={styles.comingSoonSection}>
          <div className={styles.comingSoonButtons}>
            <button className="button button--secondary" onClick={() => console.log('Ask AI about this book clicked')} disabled>
              ChatBot
            </button>
            <button className="button button--secondary" onClick={() => console.log('Personalize later clicked')} disabled>
              Personalize
            </button>
            <button className="button button--secondary" onClick={() => console.log('Translate to Urdu later clicked')} disabled>
              Translate to Urdu
            </button>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): React.ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="AI-Native Capstone Textbook for Physical AI & Humanoid Robotics">
      <HomepageHeader />
      <main>
        <HomepageContent />
      </main>
    </Layout>
  );
}
