---
sidebar_position: 1
title: "Module 1, Chapter 1: ROS 2 Core Concepts - Nodes, Topics, and Services"
description: "Learn about ROS 2 as robotic middleware, nodes, topics, and services"
---

# ROS 2 Core Concepts - Nodes, Topics, and Services

![ROS 2 Architecture](/img/robot.jpg)

## Introduction to ROS 2 as Robotic Middleware

ROS 2 (Robot Operating System 2) serves as the foundational middleware for robotic applications. It provides a flexible framework for developing robot software by handling communication between different software components, device drivers, and libraries. Unlike traditional middleware, ROS 2 is specifically designed for robotics applications with real-time constraints, safety considerations, and distributed computing requirements.

ROS 2 provides a collection of libraries, tools, and conventions that aim to simplify the development of complex robotic systems. It handles the "plumbing" of robotics software, allowing developers to focus on the actual robot behaviors and applications rather than low-level communication details.

## Nodes: Independent Computational Units

Nodes are the fundamental building blocks of any ROS 2 system. A node is an executable that uses ROS 2 to communicate with other nodes. Each node runs independently and typically performs a specific task or function within the robotic system. Nodes can be written in different programming languages (C++, Python, etc.) and can run on different machines.

### Key Characteristics of Nodes:
- **Isolation**: Each node runs in its own process, providing fault isolation
- **Specialization**: Each node typically performs a specific function (e.g., sensor processing, control, planning)
- **Communication**: Nodes communicate with each other through topics, services, and actions
- **Distributed**: Nodes can run on the same machine or distributed across multiple machines

### Creating a Node
A node must be initialized with the ROS 2 client library, which provides the necessary interfaces to participate in the ROS 2 network. The initialization process connects the node to the ROS 2 communication infrastructure.

## Topics and Publish/Subscribe Communication

Topics enable one-way, asynchronous communication between nodes using a publish/subscribe pattern. In this model, nodes called publishers send messages to a topic, while nodes called subscribers receive messages from the topic. Multiple publishers can send to the same topic, and multiple subscribers can receive from the same topic.

### How Publish/Subscribe Works:
1. Publishers send messages to topics without knowing who (if anyone) is subscribed
2. Subscribers receive messages from topics without knowing who (if anyone) is publishing
3. The ROS 2 communication layer handles the message routing between publishers and subscribers
4. This creates a decoupled system where publishers and subscribers don't need to know about each other

### Message Types:
All messages published to a topic must be of the same type. ROS 2 defines standard message types for common data structures like sensor data, coordinates, and commands. Custom message types can also be defined for specific applications.

## Services for Request/Response Interactions

Services provide a request/response communication pattern where one node sends a request to another node and waits for a response. This is different from topics, which provide asynchronous, one-way communication. Services are synchronous and establish a direct connection between a client (the requester) and a server (the responder).

### Service Characteristics:
- **Synchronous**: The client waits for the response before continuing
- **Direct**: There's a direct request/response connection between client and server
- **Reliable**: Services typically guarantee delivery and response
- **Stateless**: Each service call is independent of previous calls

### When to Use Services:
- Configuration operations
- Actions that must complete before continuing
- Operations that return a result immediately
- Synchronous tasks that require confirmation

## DDS-Based Architecture and Reliability Concepts

ROS 2 uses Data Distribution Service (DDS) as its underlying communication middleware. DDS is a peer-to-peer, real-time data distribution standard that provides Quality of Service (QoS) policies for different communication needs.

### DDS Quality of Service (QoS) Policies:
- **Reliability**: Ensures message delivery (reliable vs. best-effort)
- **Durability**: Whether late-joining subscribers receive previous messages (transient-local vs. volatile)
- **History**: How many messages to store (keep-all vs. keep-last)
- **Deadline**: Maximum time between consecutive messages
- **Liveliness**: How to detect if a publisher is alive

### Reliability in ROS 2:
- **Reliable**: Every message sent is guaranteed to be received
- **Best-effort**: Messages may be lost, but the system will try to deliver them
- The choice depends on the application requirements (e.g., sensor data might use best-effort, while commands might use reliable)

## How Components Form a Robotic Nervous System

The combination of nodes, topics, and services creates a robotic nervous system that mimics biological nervous systems:

### Sensory System (Subscribers):
- Nodes that receive sensor data (camera, lidar, IMU, etc.)
- Process and interpret environmental information
- Analogous to sensory neurons in biological systems

### Motor System (Publishers):
- Nodes that send commands to actuators (motors, servos, etc.)
- Execute physical actions based on decisions
- Analogous to motor neurons in biological systems

### Processing Centers (Processing Nodes):
- Nodes that process information and make decisions
- Similar to neural processing centers in the brain
- May implement AI algorithms, control systems, or planning logic

### Communication Pathways (Topics/Services):
- The communication infrastructure that connects all components
- Enables information flow throughout the system
- Provides the pathways for sensorimotor coordination

This architecture allows for a distributed, fault-tolerant system where different components can be developed, tested, and maintained independently while working together as a cohesive robotic system.

## Learning Objectives

After completing this chapter, you should be able to:
1. Explain the role of ROS 2 as robotic middleware
2. Describe the purpose and characteristics of nodes
3. Understand publish/subscribe communication patterns
4. Explain request/response interactions with services
5. Describe how DDS provides reliability in ROS 2
6. Understand how these components form a robotic nervous system

## Next Steps

Now that you understand ROS 2 core concepts, continue to the next chapter to learn about [connecting Python AI agents with rclpy](./python-ai-integration).