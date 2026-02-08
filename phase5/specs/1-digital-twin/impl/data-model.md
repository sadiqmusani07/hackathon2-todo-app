# Data Model: Digital Twin (Gazebo & Unity) Module

## Content Entities

### Chapter 1: Physics Simulation in Gazebo
**Entity**: GazeboSimulationConfig
- **Fields**:
  - `physics_engine` (string): Name of the physics engine (e.g., "ignition-physics", "bullet", "ode")
  - `gravity` (object): Gravity vector (x, y, z components)
  - `time_step` (number): Physics simulation time step in seconds
  - `real_time_factor` (number): Real-time update rate multiplier
  - `robot_models` (array): List of URDF/SDF robot model paths
  - `environment_objects` (array): List of SDF environment object paths
  - `sensor_configurations` (array): List of sensor plugin configurations
- **Relationships**:
  - Connects to ROS2Nodes via sensor data topics
  - References URDF models for robot configuration
- **Validation**:
  - physics_engine must be supported by Gazebo version
  - gravity vector must have reasonable magnitude (typically 9.81 m/s²)
  - time_step must be positive and within stable range
  - robot_models must be valid URDF/SDF files

### Chapter 2: High-Fidelity Rendering in Unity
**Entity**: UnityEnvironmentConfig
- **Fields**:
  - `lighting_config` (object): Lighting setup (ambient, directional, etc.)
  - `materials` (array): List of material configurations
  - `textures` (array): List of texture paths
  - `rendering_settings` (object): Quality and performance settings
  - `robot_import_config` (object): URDF/Gazebo import parameters
  - `interaction_scenarios` (array): List of human-robot interaction setups
- **Relationships**:
  - Imports from URDF/Gazebo models
  - Connects to ROS2Nodes for visualization updates
- **Validation**:
  - All material and texture paths must exist
  - Rendering settings must be compatible with target hardware
  - Robot import config must reference valid model files

### Chapter 3: Sensor and Environment Integration
**Entity**: DigitalTwinConnection
- **Fields**:
  - `gazebo_bridge_config` (object): Configuration for Gazebo-ROS bridge
  - `unity_bridge_config` (object): Configuration for Unity-ROS bridge
  - `synchronization_protocol` (string): Time synchronization method
  - `data_streams` (array): List of synchronized data streams
  - `ai_agent_interface` (object): Configuration for AI agent connections
- **Relationships**:
  - Connects physics and visualization systems
  - Interfaces with ROS2Nodes for data flow
  - Connects to AI agents for perception/navigation
- **Validation**:
  - Both bridge configurations must be valid
  - Data streams must have consistent formats between systems
  - Synchronization must maintain temporal consistency

## Navigation Entities

### Module Navigation
**Entity**: ModuleNavigation
- **Fields**:
  - `module_id` (string): Unique identifier for the module ("digital-twin")
  - `module_title` (string): Display title ("Digital Twin (Gazebo & Unity)")
  - `chapters` (array): List of chapter configurations
  - `sidebar_position` (number): Position in sidebar navigation
- **Relationships**:
  - Contains multiple Chapter entities
  - Connects to main documentation navigation
- **Validation**:
  - module_id must be unique across all modules
  - sidebar_position must not conflict with existing positions
  - All referenced chapters must exist

### Chapter Navigation
**Entity**: Chapter
- **Fields**:
  - `chapter_id` (string): Unique identifier for the chapter
  - `chapter_title` (string): Display title
  - `sidebar_position` (number): Position within module
  - `prerequisites` (array): List of required knowledge
  - `learning_objectives` (array): List of learning goals
  - `content_path` (string): File path relative to docs/
- **Relationships**:
  - Belongs to a ModuleNavigation entity
  - References other chapters for cross-links
- **Validation**:
  - chapter_id must be unique within module
  - content_path must point to existing file
  - sidebar_position must be unique within module

## Integration Entities

### ROS 2 Interface
**Entity**: ROS2Interface
- **Fields**:
  - `node_name` (string): Name of the ROS 2 node
  - `topics` (array): List of subscribed/published topics
  - `services` (array): List of provided/called services
  - `actions` (array): List of action servers/clients
  - `message_types` (array): List of used message types
- **Relationships**:
  - Connects to simulation environments
  - Interfaces with AI agents
- **Validation**:
  - All message types must be valid ROS 2 types
  - Topic names must follow ROS naming conventions
  - Node name must be unique in namespace

## Validation Rules

### Content Validation
- All code examples must be syntactically correct
- All file paths must exist within the project
- All cross-references must point to existing content
- All learning objectives must be measurable

### Navigation Validation
- All sidebar entries must have corresponding content files
- Navigation hierarchy must be consistent
- All links must be valid and accessible
- Content must be organized in logical progression

### Integration Validation
- All ROS 2 interfaces must be properly documented
- Sensor data flows must be clearly explained
- Synchronization mechanisms must be described in detail
- Troubleshooting information must be provided for common issues