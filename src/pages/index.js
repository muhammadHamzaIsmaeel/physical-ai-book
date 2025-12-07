import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

const FeatureList = [
  {
    title: 'Comprehensive Curriculum',
    description: (
      <>
        Dive deep into ROS 2 fundamentals, advanced NVIDIA Isaac Sim, Vision-Language-Action (VLA) models, URDF, and Dexterous Manipulation.
      </>
    ),
  },
  {
    title: 'Hands-on Learning',
    description: (
      <>
        Reinforce your knowledge with practical code examples, immersive labs, and step-by-step tutorials designed for real-world application.
      </>
    ),
  },
  {
    title: 'Community-Driven & Open-Source',
    description: (
      <>
        Benefit from a living document that's continuously updated, improved, and expanded through vibrant community contributions.
      </>
    ),
  },
  {
    title: 'Interactive Quizzes & Challenges',
    description: (
      <>
        Test your understanding at the end of each chapter with interactive quizzes and engaging challenges that solidify your learning.
      </>
    ),
  },
];

function Feature({title, description}) {
  return (
    <div className={clsx('col col--6 margin-bottom--lg')}>
      <div className="card shadow--md">
        <div className="card__header">
          <h3>{title}</h3>
        </div>
        <div className="card__body">
          <p>{description}</p>
        </div>
      </div>
    </div>
  );
}

function HomepageHero() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">Mastering Physical AI: Build the Future of Robotics</p>
        <p className={styles.heroDescription}>
          An open-source guide to embodied intelligence with ROS 2, NVIDIA Isaac Sim, and cutting-edge Vision-Language-Action models.
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Your Journey 🚀
          </Link>
        </div>
      </div>
    </header>
  );
}

function HomepageFeatures() {
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

function WhatYoullLearn() {
  return (
    <section className={styles.whatYoullLearn}>
      <div className="container">
        <h2 className="text--center">What You'll Learn</h2>
        <div className="row">
          <div className={clsx('col col--4')}>
            <ul>
              <li>ROS 2 Navigation & Perception</li>
              <li>NVIDIA Isaac Sim for Robotics Simulation</li>
              <li>URDF Modeling & Digital Twins</li>
            </ul>
          </div>
          <div className={clsx('col col--4')}>
            <ul>
              <li>Vision-Language-Action (VLA) Models</li>
              <li>Dexterous Manipulation Techniques</li>
              <li>Sim-to-Real Transfer Strategies</li>
            </ul>
          </div>
          <div className={clsx('col col--4')}>
            <ul>
              <li>Voice-to-Action Pipelines</li>
              <li>Building Embodied Agents</li>
              <li>Ethical AI in Robotics</li>
            </ul>
          </div>
        </div>
      </div>
    </section>
  );
}

function WhoIsThisFor() {
  return (
    <section className={styles.whoIsThisFor}>
      <div className="container text--center">
        <h2>Who Is This Book For?</h2>
        <p className={styles.whoIsThisForDescription}>
          This comprehensive guide is crafted for robotics enthusiasts, AI/ML engineers, academic researchers, and students eager to
          explore and build at the forefront of Physical AI and humanoid robotics.
        </p>
        <div className="row">
          <div className="col col--4">
            <div className="card shadow--md padding--md margin-bottom--md">
              <h3>Robotics Enthusiasts</h3>
              <p>Hobbyists and makers looking to deepen their understanding and practical skills.</p>
            </div>
          </div>
          <div className="col col--4">
            <div className="card shadow--md padding--md margin-bottom--md">
              <h3>AI/ML Engineers</h3>
              <p>Professionals transitioning into robotics or specializing in embodied intelligence.</p>
            </div>
          </div>
          <div className="col col--4">
            <div className="card shadow--md padding--md margin-bottom--md">
              <h3>Researchers & Students</h3>
              <p>Academics and students seeking a cutting-edge resource for study and projects.</p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function FinalCallToAction() {
  return (
    <section className={styles.finalCallToAction}>
      <div className="container text--center">
        <h2>Ready to Build the Future?</h2>
        <p className={styles.finalCallToActionDescription}>
          Join a thriving community, contribute to the open-source movement, and shape the next generation of intelligent robots.
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Explore Chapters 📖
          </Link>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Mastering Physical AI: Build the Future of Robotics with ROS 2, NVIDIA Isaac Sim, and Vision-Language-Action models.">
      <HomepageHero />
      <main>
        <HomepageFeatures />
        <WhatYoullLearn />
        <WhoIsThisFor />
        <FinalCallToAction />
      </main>
    </Layout>
  );
}
