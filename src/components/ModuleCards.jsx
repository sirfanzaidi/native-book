import React from 'react';
import '@site/src/css/module-cards.css';

const modules = [
  {
    id: 1,
    title: 'ROS 2 Fundamentals',
    description: 'Master the core concepts of Robot Operating System 2, including nodes, topics, services, and advanced node architecture.',
    icon: '🤖',
    color: 'module-blue',
    link: '/native-book/module-1-ros2/introduction',
    chapters: 3
  },
  {
    id: 2,
    title: 'Digital Twin & Simulation',
    description: 'Learn Gazebo fundamentals and integrate ROS 2 with Gazebo for realistic robot simulation and testing.',
    icon: '🎮',
    color: 'module-purple',
    link: '/native-book/module-2-digital-twin/gazebo-fundamentals',
    chapters: 2
  },
  {
    id: 3,
    title: 'AI & Perception',
    description: 'Explore AI perception pipelines using Isaac Lab for advanced computer vision and autonomous decision-making.',
    icon: '🧠',
    color: 'module-pink',
    link: '/native-book/module-3-ai-perception/isaac-fundamentals',
    chapters: 2
  },
  {
    id: 4,
    title: 'Voice Control & Autonomy',
    description: 'Integrate LLMs and vision-language models for intelligent voice control and autonomous action planning.',
    icon: '🎤',
    color: 'module-orange',
    link: '/native-book/module-4-voice-control/llm-integration',
    chapters: 2
  }
];

export default function ModuleCards() {
  return (
    <div className="module-cards-container">
      <div className="modules-grid">
        {modules.map((module) => (
          <a key={module.id} href={module.link} className={`module-card ${module.color}`}>
            <div className="smoke-effect"></div>
            <div className="card-content">
              <div className="card-icon">{module.icon}</div>
              <h3 className="card-title">{module.title}</h3>
              <p className="card-description">{module.description}</p>
              <div className="card-footer">
                <span className="chapter-count">{module.chapters} chapters</span>
                <span className="arrow">→</span>
              </div>
            </div>
            <div className="gradient-border"></div>
          </a>
        ))}
      </div>
    </div>
  );
}
