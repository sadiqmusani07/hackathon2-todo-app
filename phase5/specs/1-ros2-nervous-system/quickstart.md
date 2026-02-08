# Quickstart: ROS 2 Nervous System Module

## Prerequisites

- Node.js (v16 or higher)
- npm or yarn package manager
- Python 3.8+ for ROS 2 examples (if running locally)
- Basic understanding of Python and AI concepts

## Setup Instructions

### 1. Initialize Docusaurus Site

```bash
# Create a new Docusaurus site
npx create-docusaurus@latest my-ros2-book classic

# Navigate to the project directory
cd my-ros2-book
```

### 2. Configure Documentation Structure

Update `docusaurus.config.js`:

```javascript
// docusaurus.config.js
module.exports = {
  title: 'ROS 2 Nervous System for Physical AI & Humanoid Robotics',
  tagline: 'Connecting AI Decision-Making to Humanoid Robot Bodies',
  // ... other config
  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/your-org/your-repo/edit/main/',
        },
        // ... other presets
      },
    ],
  ],
};
```

### 3. Create Sidebar Navigation

Update `sidebars.js`:

```javascript
// sidebars.js
module.exports = {
  docs: [
    {
      type: 'category',
      label: 'ROS 2 Nervous System',
      items: [
        'ros2-fundamentals',
        'python-ai-integration',
        'urdf-modeling',
      ],
    },
  ],
};
```

### 4. Create Documentation Files

Create the three required chapters in the `docs` folder:

```bash
# Create the documentation files
mkdir -p docs
touch docs/ros2-fundamentals.md
touch docs/python-ai-integration.md
touch docs/urdf-modeling.md
```

### 5. Start Development Server

```bash
# Install dependencies
npm install

# Start the development server
npm start
```

## Chapter Creation Guide

### Chapter 1: ROS 2 Fundamentals

Create `docs/ros2-fundamentals.md`:

```markdown
---
sidebar_position: 1
title: "ROS 2 Fundamentals — Nodes, Topics, and Services"
---

# ROS 2 Fundamentals — Nodes, Topics, and Services

## Introduction to ROS 2 as Robotic Middleware

[Content about ROS 2's purpose as middleware...]

## Nodes: Independent Computational Units

[Content about ROS 2 nodes...]

## Topics and Publish/Subscribe Communication

[Content about topics and pub/sub pattern...]

## Services for Request/Response Interactions

[Content about services...]

## DDS-Based Architecture and Reliability

[Content about DDS...]

## How Components Form a Robotic Nervous System

[Content about the nervous system analogy...]
```

### Chapter 2: Python AI Integration

Create `docs/python-ai-integration.md`:

```markdown
---
sidebar_position: 2
title: "Connecting Python AI Agents with rclpy"
---

# Connecting Python AI Agents with rclpy

## Overview of rclpy and Its Role in ROS 2

[Content about rclpy...]

## Structure of a Python-based ROS 2 Node

[Content about node structure with code examples...]

## How AI Agents Send Commands and Receive Feedback

[Content about AI-robot communication...]

## Mapping High-Level AI Decisions to Robot Actions

[Content about decision mapping...]

## Observability, Safety, and Control Flow Considerations

[Content about safety and observability...]
```

### Chapter 3: URDF Modeling

Create `docs/urdf-modeling.md`:

```markdown
---
sidebar_position: 3
title: "Defining the Robot Body with URDF"
---

# Defining the Robot Body with URDF

## Role of URDF in Robot Description

[Content about URDF...]

## Links, Joints, and Kinematic Chains

[Content about links and joints...]

## Humanoid-Specific Design Considerations

[Content about humanoid robots...]

## How URDF Connects Physical Structure to ROS 2 Controllers

[Content about URDF-ROS2 integration...]

## Preparing Robot Models for Simulation and Control

[Content about simulation preparation...]
```

## Running Examples

To run Python examples with rclpy, ensure you have ROS 2 installed:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash  # or your ROS 2 distribution

# Run Python examples
python3 your_ros2_node.py
```

## Building for Production

```bash
# Build the static site
npm run build

# Serve the built site locally for testing
npm run serve
```

## Deployment

The Docusaurus site can be deployed to:
- GitHub Pages
- Netlify
- Vercel
- Any static hosting service

For GitHub Pages deployment:

```bash
# Deploy to GitHub Pages
GIT_USER=<Your GitHub username> USE_SSH=true npm run deploy
```