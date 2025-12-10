import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

const FeatureList = [
  {
    title: 'Comprehensive Curriculum',
    icon: '🎓',
    gradient: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
    description: (
      <>
        Dive deep into ROS 2 fundamentals, advanced NVIDIA Isaac Sim, Vision-Language-Action (VLA) models, URDF, and Dexterous Manipulation.
      </>
    ),
  },
  {
    title: 'Hands-on Learning',
    icon: '🛠️',
    gradient: 'linear-gradient(135deg, #f093fb 0%, #f5576c 100%)',
    description: (
      <>
        Reinforce your knowledge with practical code examples, immersive labs, and step-by-step tutorials designed for real-world application.
      </>
    ),
  },
  {
    title: 'Community-Driven & Open-Source',
    icon: '🌐',
    gradient: 'linear-gradient(135deg, #4facfe 0%, #00f2fe 100%)',
    description: (
      <>
        Benefit from a living document that's continuously updated, improved, and expanded through vibrant community contributions.
      </>
    ),
  },
  {
    title: 'Interactive Quizzes & Challenges',
    icon: '🎯',
    gradient: 'linear-gradient(135deg, #43e97b 0%, #38f9d7 100%)',
    description: (
      <>
        Test your understanding at the end of each chapter with interactive quizzes and engaging challenges that solidify your learning.
      </>
    ),
  },
];

function Feature({title, description, icon, gradient}) {
  return (
    <div className={clsx('col col--6 margin-bottom--lg')}>
      <div className={styles.featureCard}>
        <div className={styles.featureIconWrapper} style={{background: gradient}}>
          <div className={styles.featureIcon}>{icon}</div>
        </div>
        <div className={styles.featureContent}>
          <h3 className={styles.featureTitle}>{title}</h3>
          <p className={styles.featureDescription}>{description}</p>
        </div>
      </div>
    </div>
  );
}

function HomepageHero() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className={clsx('container', styles.heroContainer)}>
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className={styles.heroDescription}>
          An open-source guide to embodied intelligence.
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
        <div className={styles.sectionHeader}>
          <h2 className={styles.sectionTitle}>Why Choose This Book?</h2>
          <p className={styles.sectionSubtitle}>
            Everything you need to master Physical AI and Humanoid Robotics
          </p>
        </div>
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
  const topics = [
    { emoji: '🧭', text: 'ROS 2 Navigation & Perception' },
    { emoji: '🎮', text: 'NVIDIA Isaac Sim for Robotics Simulation' },
    { emoji: '🤖', text: 'URDF Modeling & Digital Twins' },
    { emoji: '👁️', text: 'Vision-Language-Action (VLA) Models' },
    { emoji: '🤲', text: 'Dexterous Manipulation Techniques' },
    { emoji: '🔄', text: 'Sim-to-Real Transfer Strategies' },
    { emoji: '🎤', text: 'Voice-to-Action Pipelines' },
    { emoji: '🧠', text: 'Building Embodied Agents' },
    { emoji: '⚖️', text: 'Ethical AI in Robotics' },
  ];

  return (
    <section className={styles.whatYoullLearn}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <h2 className={styles.sectionTitleLight}>What You'll Master</h2>
          <p className={styles.sectionSubtitleLight}>
            A comprehensive curriculum covering cutting-edge robotics and AI
          </p>
        </div>
        <div className={styles.topicsGrid}>
          {topics.map((topic, idx) => (
            <div key={idx} className={styles.topicCard}>
              <span className={styles.topicEmoji}>{topic.emoji}</span>
              <span className={styles.topicText}>{topic.text}</span>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function WhoIsThisFor() {
  const audience = [
    {
      icon: '🔧',
      title: 'Robotics Enthusiasts',
      description: 'Hobbyists and makers looking to deepen their understanding and practical skills.',
      color: '#667eea'
    },
    {
      icon: '💻',
      title: 'AI/ML Engineers',
      description: 'Professionals transitioning into robotics or specializing in embodied intelligence.',
      color: '#f5576c'
    },
    {
      icon: '🎓',
      title: 'Researchers & Students',
      description: 'Academics and students seeking a cutting-edge resource for study and projects.',
      color: '#00f2fe'
    }
  ];

  return (
    <section className={styles.whoIsThisFor}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <h2 className={styles.sectionTitle}>Who Is This Book For?</h2>
          <p className={styles.whoIsThisForDescription}>
            This comprehensive guide is crafted for robotics enthusiasts, AI/ML engineers, academic researchers, and students eager to
            explore and build at the forefront of Physical AI and humanoid robotics.
          </p>
        </div>
        <div className="row">
          {audience.map((item, idx) => (
            <div key={idx} className="col col--4">
              <div className={styles.audienceCard}>
                <div className={styles.audienceIcon} style={{backgroundColor: item.color}}>
                  {item.icon}
                </div>
                <h3 className={styles.audienceTitle}>{item.title}</h3>
                <p className={styles.audienceDescription}>{item.description}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function FinalCallToAction() {
  return (
    <section className={styles.finalCallToAction}>
      <div className="container text--center">
        <div className={styles.ctaContent}>
          <h2 className={styles.ctaTitle}>Ready to Build the Future?</h2>
          <p className={styles.finalCallToActionDescription}>
            Join a thriving community, contribute to the open-source movement, and shape the next generation of intelligent robots.
          </p>
          <div className={styles.ctaButtons}>
            <Link
              className={clsx('button button--lg', styles.ctaPrimaryButton)}
              to="/docs/intro">
              Start Learning Now 🚀
            </Link>
            <Link
              className={clsx('button button--lg', styles.ctaSecondaryButton)}
              to="https://github.com/muhammadHamzaIsmaeel/physical-ai-book">
              View on GitHub ⭐
            </Link>
          </div>
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
