import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/native-book/',
    component: ComponentCreator('/native-book/', 'df1'),
    routes: [
      {
        path: '/native-book/',
        component: ComponentCreator('/native-book/', '98f'),
        routes: [
          {
            path: '/native-book/',
            component: ComponentCreator('/native-book/', 'f8f'),
            routes: [
              {
                path: '/native-book/capstone-project/10-capstone-project',
                component: ComponentCreator('/native-book/capstone-project/10-capstone-project', 'f9c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/native-book/capstone-project/11-capstone-implementation',
                component: ComponentCreator('/native-book/capstone-project/11-capstone-implementation', 'bd3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/native-book/module-1-ros2/01-introduction',
                component: ComponentCreator('/native-book/module-1-ros2/01-introduction', '3e8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/native-book/module-1-ros2/02-services',
                component: ComponentCreator('/native-book/module-1-ros2/02-services', '66f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/native-book/module-1-ros2/03-advanced-nodes',
                component: ComponentCreator('/native-book/module-1-ros2/03-advanced-nodes', '535'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/native-book/module-2-digital-twin/04-gazebo-fundamentals',
                component: ComponentCreator('/native-book/module-2-digital-twin/04-gazebo-fundamentals', 'cc6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/native-book/module-2-digital-twin/05-ros2-gazebo-bridge',
                component: ComponentCreator('/native-book/module-2-digital-twin/05-ros2-gazebo-bridge', '91d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/native-book/module-3-ai-perception/06-isaac-fundamentals',
                component: ComponentCreator('/native-book/module-3-ai-perception/06-isaac-fundamentals', '0f4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/native-book/module-3-ai-perception/07-perception-pipelines',
                component: ComponentCreator('/native-book/module-3-ai-perception/07-perception-pipelines', 'c27'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/native-book/module-4-voice-control/08-llm-integration',
                component: ComponentCreator('/native-book/module-4-voice-control/08-llm-integration', 'eef'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/native-book/module-4-voice-control/09-vision-language-action',
                component: ComponentCreator('/native-book/module-4-voice-control/09-vision-language-action', '3eb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/native-book/',
                component: ComponentCreator('/native-book/', '6f3'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
