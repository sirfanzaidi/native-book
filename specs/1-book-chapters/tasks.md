# Tasks: Book Chapters for Physical AI & Humanoid Robotics

**Input**: Specification from `specs/1-book-chapters/spec.md`, Implementation Plan from `specs/1-book-chapters/plan.md`

**Prerequisites**: spec.md (5 user stories P1-P2), plan.md (architecture, project structure, phases)

**Organization**: Tasks grouped by user story (P1-P2) enabling concurrent implementation after Phase 1-2 setup. Each story is independently testable.

**Total Tasks**: 95 tasks across 5 phases + polish

---

## Format: `[ID] [P?] [Story] Description`

- **[ID]**: Task identifier (T001, T002, etc.) in execution order
- **[P]**: Parallelizable (different files, no dependencies between these tasks)
- **[Story]**: User story label (US1, US2, US3, US4, US5) for traceability
- **Description**: Clear action with exact file path

---

## Phase 1: Setup (Docusaurus & Project Initialization)

**Purpose**: Initialize Docusaurus project structure and configuration

**Estimated Time**: 2-3 hours

- [ ] T001 Create Docusaurus project root structure with `docusaurus-setup.sh` in `scripts/`
- [ ] T002 [P] Create `docusaurus.config.js` at repository root with site metadata (title, baseUrl, favicon, navbar config)
- [ ] T003 [P] Create `sidebars.js` at repository root with navigation structure for 5 modules + capstone
- [ ] T004 [P] Create `package.json` with Docusaurus 3.x dependencies (docusaurus, react, react-dom, prismjs)
- [ ] T005 [P] Create `.docusaurusrc` configuration for deployment targets (GitHub Pages)
- [ ] T006 [P] Create `docs/` folder structure: `docs/{intro, 01-module-1-ros2, 02-module-2-digital-twin, 03-module-3-isaac, 04-module-4-vla, 05-capstone-project, assets}`
- [ ] T007 [P] Create `static/` folder for assets (images, diagrams, setup instructions) at `static/{diagrams, code-outputs, setup-guides}`
- [ ] T008 [P] Create `.github/workflows/deploy.yml` for GitHub Pages CI/CD pipeline
- [ ] T009 [P] Create `docker/Dockerfile` with ROS 2 Humble base + Python 3.11 + essential tools
- [ ] T010 [P] Create `docker/docker-compose.yml` with services for ROS 2, Gazebo, Isaac Sim dependencies
- [ ] T011 [P] Create `docker/.env.example` with environment variable templates
- [ ] T012 [P] Create `tests/` folder structure: `tests/{test-examples, test-content, test-reproducibility}`
- [ ] T013 [P] Create `scripts/` folder with utility scripts: `scripts/{validate-examples.sh, check-citations.py, test-reproducibility.sh, build-docusaurus.sh}`
- [ ] T014 npm install (or equivalent) to install all Docusaurus dependencies in `node_modules/`
- [ ] T015 Verify Docusaurus build succeeds: `npm run build` produces `build/` directory

**Checkpoint**: Project structure ready; Docusaurus can build successfully

---

## Phase 2: Foundational (Content Templates & Configuration)

**Purpose**: Create reusable templates, content schemas, and citation standards that all chapters depend on

**Estimated Time**: 2-3 hours

**⚠️ CRITICAL**: All content writing depends on these templates and standards being finalized first

- [ ] T016 Create `docs/00-introduction.md` with book overview, target audience, module dependencies, and learning outcomes
- [ ] T017 [P] Create `specs/1-book-chapters/contracts/chapter-template.md` with markdown structure (headings, code block format, image placement, links format)
- [ ] T018 [P] Create `specs/1-book-chapters/contracts/code-example-format.md` documenting code example standard: setup → code → output → explanation
- [ ] T019 [P] Create `specs/1-book-chapters/contracts/citation-format.md` with APA 7th edition inline citation examples for official docs
- [ ] T020 [P] Create `specs/1-book-chapters/contracts/docusaurus-config.md` documenting Docusaurus markdown extensions (code blocks, callouts, tabs)
- [ ] T021 Create `specs/1-book-chapters/data-model.md` with entity definitions: Chapter, Section, CodeExample, Citation, Diagram, ROS2Concept, SensorData, LLMPrompt
- [ ] T022 Create `specs/1-book-chapters/chapter-structure.md` with detailed 15-chapter outline mapping sections to modules (Module 1: Ch1-3, M2: Ch4-6, M3: Ch7-9, M4: Ch10-12, Capstone: Ch13-15)
- [ ] T023 Create `quickstart.md` with reader setup guide: prerequisites (Python, ROS 2, Docker), three platform setup (WSL2/Ubuntu/macOS), verification tests
- [ ] T024 [P] Create `tests/test-examples/conftest.py` with pytest fixtures for example execution (environment validation, output capture)
- [ ] T025 [P] Create `tests/test-content/conftest.py` with markdown validation fixtures (link checking, citation format validation)
- [ ] T026 [P] Create common documentation assets: `static/diagrams/ros2-graph.txt` (ASCII ROS 2 node graph template), `static/diagrams/system-architecture.txt` (capstone integration diagram)
- [ ] T027 Create `CONTRIBUTING.md` documenting chapter writing process, code example validation, citation requirements

**Checkpoint**: Content standards and templates ready; writers can begin chapter authoring

---

## Phase 3: User Story 1 - Learn ROS 2 Fundamentals and Control Humanoid via Middleware (Priority: P1) 🎯 MVP

**Goal**: Readers understand ROS 2 architecture (pub/sub, services, actions) and can write Python nodes to control a humanoid

**Independent Test**: Reader creates pub/sub nodes, verifies message exchange, parses URDF humanoid model

**Estimated Time**: 15 hours (split across 3 chapters)

### Implementation for User Story 1

#### Chapter 1: ROS 2 Architecture (Nodes, Topics, Services)

- [ ] T028 [P] [US1] Write `docs/01-module-1-ros2/01-architecture-overview.md` covering ROS 2 fundamentals (what is ROS 2, middleware concept, pub/sub vs services) with citations to official ROS 2 docs
- [ ] T029 [P] [US1] Write `docs/01-module-1-ros2/01-section-ros2-intro.md` explaining motivation (distributed robotics, cross-platform, real-time constraints)
- [ ] T030 [P] [US1] Write `docs/01-module-1-ros2/01-section-nodes-topics.md` covering nodes, topics, pub/sub pattern with visual diagram reference
- [ ] T031 [P] [US1] Write `docs/01-module-1-ros2/01-section-services-actions.md` covering services (synchronous), actions (long-running tasks) with examples and tradeoff analysis

#### Chapter 1: Code Examples & Explanations

- [ ] T032 [P] [US1] Create `docs/01-module-1-ros2/examples/01-simple-publisher.py` demonstrating basic topic publisher in rclpy with detailed inline comments
- [ ] T033 [P] [US1] Create `docs/01-module-1-ros2/examples/01-simple-subscriber.py` demonstrating basic topic subscriber with message callback, verify with T032
- [ ] T034 [P] [US1] Create `docs/01-module-1-ros2/examples/01-publisher-subscriber-launch.sh` showing how to run both nodes and verify message exchange
- [ ] T035 [P] [US1] Create `docs/01-module-1-ros2/examples/01-expected-output.txt` documenting expected output when running pub/sub example
- [ ] T036 [US1] Add code example explanations to chapter 1 markdown (setup → code → output → explanation pattern for each example)
- [ ] T037 [US1] Verify all Chapter 1 examples execute without error in clean environment

#### Chapter 2: Python Agent Integration (rclpy, Custom Nodes)

- [ ] T038 [P] [US1] Write `docs/01-module-1-ros2/02-python-integration-rclpy.md` covering rclpy API basics (node lifecycle, subscriptions, publishers, timers)
- [ ] T039 [P] [US1] Write `docs/01-module-1-ros2/02-section-custom-nodes.md` with patterns for writing custom ROS 2 nodes (inheritance from rclpy.node.Node, best practices)
- [ ] T040 [P] [US1] Write `docs/01-module-1-ros2/02-section-debugging.md` covering debugging ROS 2 applications (rclpy logging, ros2 topic echo, ros2 node info)

#### Chapter 2: Code Examples & Explanations

- [ ] T041 [P] [US1] Create `docs/01-module-1-ros2/examples/02-custom-node.py` implementing a custom ROS 2 node class with timer-based publishing
- [ ] T042 [P] [US1] Create `docs/01-module-1-ros2/examples/02-service-server.py` implementing a simple ROS 2 service server (add_two_ints service)
- [ ] T043 [P] [US1] Create `docs/01-module-1-ros2/examples/02-service-client.py` implementing a simple ROS 2 service client to call the add_two_ints service
- [ ] T044 [P] [US1] Create `docs/01-module-1-ros2/examples/02-action-server.py` implementing a long-running ROS 2 action (fibonacci action)
- [ ] T045 [P] [US1] Create `docs/01-module-1-ros2/examples/02-debugging-output.txt` showing expected output from debugging tools (rclpy logs, topic echo)
- [ ] T046 [US1] Add code example explanations to chapter 2 markdown (setup → code → output → explanation)
- [ ] T047 [US1] Verify all Chapter 2 examples execute without error

#### Chapter 3: Robot Modeling with URDF (Humanoid Models)

- [ ] T048 [P] [US1] Write `docs/01-module-1-ros2/03-urdf-humanoid-models.md` covering URDF format (XML structure, joint types, link definitions, humanoid-specific constraints)
- [ ] T049 [P] [US1] Write `docs/01-module-1-ros2/03-section-urdf-syntax.md` with detailed URDF syntax reference (link, joint, inertial, visual, collision tags)
- [ ] T050 [P] [US1] Write `docs/01-module-1-ros2/03-section-humanoid-structure.md` explaining humanoid kinematic chains (torso, arms, legs, joint limits, sensor mounting)

#### Chapter 3: Code Examples & Explanations

- [ ] T051 [P] [US1] Create `docs/01-module-1-ros2/examples/03-simple-humanoid.urdf` minimal valid URDF for a simple humanoid with 7 joints (torso, shoulder, elbow, gripper)
- [ ] T052 [P] [US1] Create `docs/01-module-1-ros2/examples/03-parse-urdf.py` demonstrating URDF parsing with urdfpy library to extract joint names, limits, kinematic chain
- [ ] T053 [P] [US1] Create `docs/01-module-1-ros2/examples/03-parse-urdf-output.txt` showing expected output (joint list, chain structure)
- [ ] T054 [US1] Add code example explanations to chapter 3 markdown
- [ ] T055 [US1] Verify all Chapter 3 examples execute without error

#### User Story 1: Integration & Validation

- [ ] T056 [US1] Create `tests/test-examples/test_module1_examples.py` with pytest tests validating all Chapter 1-3 examples (imports, syntax, output validation)
- [ ] T057 [US1] Create `tests/test-reproducibility/test_ros2_setup.sh` validating ROS 2 environment (rclpy import, ros2 cli available)
- [ ] T058 [US1] Run full Module 1 test suite across Windows WSL2, Ubuntu 22.04, macOS to ensure reproducibility
- [ ] T059 [US1] Create `docs/01-module-1-ros2/README.md` with module learning outcomes, prerequisite knowledge, and time estimate (15 hours)
- [ ] T060 [US1] Verify all citations in Module 1 link to official ROS 2 documentation (docs.ros.org)

**Checkpoint**: User Story 1 complete and independently testable. Reader can understand ROS 2 fundamentals and write basic nodes.

---

## Phase 4: User Story 2 - Simulate Humanoid Physics and Sensors in Digital Twin (Priority: P1)

**Goal**: Readers build realistic physics simulations in Gazebo with accurate sensor simulation (LiDAR, depth, IMU) and visualize in Unity

**Independent Test**: Create Gazebo world with humanoid, apply physics (gravity), simulate sensors, verify output matches expected values

**Estimated Time**: 15 hours (split across 3 chapters)

### Implementation for User Story 2

#### Chapter 4: Gazebo Physics Engine Fundamentals

- [ ] T061 [P] [US2] Write `docs/02-module-2-digital-twin/04-gazebo-physics.md` covering Gazebo architecture, physics engines (Bullet, ODE, Dartsim), time stepping, collision detection
- [ ] T062 [P] [US2] Write `docs/02-module-2-digital-twin/04-section-physics-setup.md` covering gravity, friction, restitution parameters, best practices for realistic simulation
- [ ] T063 [P] [US2] Write `docs/02-module-2-digital-twin/04-section-humanoid-models.md` covering loading humanoid models in Gazebo, mass distribution, joint configuration

#### Chapter 4: Code Examples & Explanations

- [ ] T064 [P] [US2] Create `docs/02-module-2-digital-twin/examples/04-basic-gazebo-world.sdf` minimal SDF world file with ground plane and gravity enabled
- [ ] T065 [P] [US2] Create `docs/02-module-2-digital-twin/examples/04-humanoid-in-world.sdf` SDF world with humanoid model (from official ROS examples), physics configured
- [ ] T066 [P] [US2] Create `docs/02-module-2-digital-twin/examples/04-launch-gazebo.sh` shell script to launch Gazebo world and verify physics running
- [ ] T067 [US2] Add code example explanations to chapter 4 markdown

#### Chapter 5: Sensor Simulation (LiDAR, Depth, IMU)

- [ ] T068 [P] [US2] Write `docs/02-module-2-digital-twin/05-sensor-simulation.md` covering Gazebo sensor plugins (ray/lidar, camera/depth, imu), output formats, ROS 2 integration
- [ ] T069 [P] [US2] Write `docs/02-module-2-digital-twin/05-section-lidar-simulation.md` with LiDAR point cloud generation, noise models, accuracy considerations
- [ ] T070 [P] [US2] Write `docs/02-module-2-digital-twin/05-section-depth-imu.md` covering depth camera RGB-D format, IMU (accelerometer, gyroscope) simulation

#### Chapter 5: Code Examples & Explanations

- [ ] T071 [P] [US2] Create `docs/02-module-2-digital-twin/examples/05-lidar-sensor.sdf` SDF snippet defining ray sensor (LiDAR) with range, resolution, update rate
- [ ] T072 [P] [US2] Create `docs/02-module-2-digital-twin/examples/05-depth-camera.sdf` SDF snippet defining camera sensor with depth plugin
- [ ] T073 [P] [US2] Create `docs/02-module-2-digital-twin/examples/05-imu-sensor.sdf` SDF snippet defining IMU sensor (accel, gyro)
- [ ] T074 [P] [US2] Create `docs/02-module-2-digital-twin/examples/05-sensor-ros2-bridge.py` Python script subscribing to simulated sensor topics and printing data
- [ ] T075 [US2] Add code example explanations to chapter 5 markdown
- [ ] T076 [US2] Create `docs/02-module-2-digital-twin/examples/05-sensor-output.txt` expected output from sensor subscribers

#### Chapter 6: Unity Rendering & Synchronization

- [ ] T077 [P] [US2] Write `docs/02-module-2-digital-twin/06-unity-visualization.md` covering high-fidelity rendering in Unity, importing Gazebo models, real-time sync
- [ ] T078 [P] [US2] Write `docs/02-module-2-digital-twin/06-section-gazebo-unity-bridge.md` with synchronization strategy (one-way: Gazebo → ROS 2 → Unity)
- [ ] T079 [P] [US2] Write `docs/02-module-2-digital-twin/06-section-human-robot-interaction.md` covering visualization for HRI scenarios

#### Chapter 6: Code Examples & Explanations

- [ ] T080 [P] [US2] Create `docs/02-module-2-digital-twin/examples/06-ros2-gazebo-bridge-config.yaml` configuration for ros_gz_bridge to expose Gazebo topics to ROS 2
- [ ] T081 [P] [US2] Create `docs/02-module-2-digital-twin/examples/06-unity-ros-subscriber.cs` C# script for Unity subscribing to ROS 2 joint_states topic
- [ ] T082 [US2] Add code example explanations to chapter 6 markdown

#### User Story 2: Integration & Validation

- [ ] T083 [US2] Create `tests/test-examples/test_module2_examples.py` with pytest tests for Gazebo world loading, sensor output validation
- [ ] T084 [US2] Create `tests/test-reproducibility/test_gazebo-setup.sh` validating Gazebo installation and plugin availability
- [ ] T085 [US2] Run Module 2 tests on Ubuntu 22.04 (primary Gazebo platform) and verify reproducibility
- [ ] T086 [US2] Create `docs/02-module-2-digital-twin/README.md` with module outcomes and time estimate
- [ ] T087 [US2] Verify all Gazebo/sensor citations link to official Gazebo documentation

**Checkpoint**: User Story 2 complete. Reader can simulate humanoid physics and sensors in Gazebo.

---

## Phase 5: User Story 3 - Deploy Perception Pipeline with GPU Acceleration and Training (Priority: P1)

**Goal**: Readers implement GPU-accelerated VSLAM using NVIDIA Isaac ROS, generate synthetic training data, and plan paths with Nav2

**Independent Test**: Generate synthetic images from Isaac Sim, run VSLAM, observe pose estimates; run Nav2 path planning

**Estimated Time**: 15 hours (split across 3 chapters)

### Implementation for User Story 3

#### Chapter 7: NVIDIA Isaac Sim & Synthetic Data Generation

- [ ] T088 [P] [US3] Write `docs/03-module-3-isaac/07-isaac-sim-intro.md` covering Isaac Sim platform, photorealistic rendering, NVIDIA RTX requirements, synthetic data advantages
- [ ] T089 [P] [US3] Write `docs/03-module-3-isaac/07-section-isaac-environment-setup.md` with Isaac Sim installation, Omniverse launcher, scene creation
- [ ] T090 [P] [US3] Write `docs/03-module-3-isaac/07-section-synthetic-data-generation.md` covering domain randomization, camera output (RGB, depth, seg), synthetic dataset creation

#### Chapter 7: Code Examples & Explanations

- [ ] T091 [P] [US3] Create `docs/03-module-3-isaac/examples/07-isaac-sim-scene-setup.py` Python script using Isaac Sim API to create scene with humanoid and camera
- [ ] T092 [P] [US3] Create `docs/03-module-3-isaac/examples/07-generate-synthetic-dataset.py` demonstrating image generation loop (step sim, capture frames, save with intrinsics)
- [ ] T093 [US3] Add code example explanations to chapter 7 markdown

#### Chapter 8: Isaac ROS & Hardware-Accelerated VSLAM

- [ ] T094 [P] [US3] Write `docs/03-module-3-isaac/08-isaac-ros-vslam.md` covering visual SLAM (simultaneous localization and mapping), Isaac ROS VSLAM performance, GPU acceleration benefits
- [ ] T095 [P] [US3] Write `docs/03-module-3-isaac/08-section-vslam-pipeline.md` with VSLAM node setup, input requirements (stereo or mono+IMU), output (pose, map)
- [ ] T096 [P] [US3] Write `docs/03-module-3-isaac/08-section-real-vs-sim.md` covering synthetic-to-real transfer, domain gap considerations

#### Chapter 8: Code Examples & Explanations

- [ ] T097 [P] [US3] Create `docs/03-module-3-isaac/examples/08-vslam-ros2-node.py` ROS 2 node that subscribes to camera topic and runs VSLAM inference
- [ ] T098 [P] [US3] Create `docs/03-module-3-isaac/examples/08-vslam-launch.yaml` ROS 2 launch file configuring VSLAM with camera parameters
- [ ] T099 [US3] Add code example explanations to chapter 8 markdown

#### Chapter 9: Nav2 Path Planning for Bipedal Motion

- [ ] T100 [P] [US3] Write `docs/03-module-3-isaac/09-nav2-bipedal-planning.md` covering Nav2 stack (costmap, planner, controller), bipedal-specific constraints
- [ ] T101 [P] [US3] Write `docs/03-module-3-isaac/09-section-costmap-planning.md` with cost map configuration, obstacle inflation, step constraints for bipeds
- [ ] T102 [P] [US3] Write `docs/03-module-3-isaac/09-section-path-execution.md` covering local planning, velocity control, dynamic obstacle avoidance

#### Chapter 9: Code Examples & Explanations

- [ ] T103 [P] [US3] Create `docs/03-module-3-isaac/examples/09-nav2-costmap-config.yaml` ROS 2 Nav2 configuration with bipedal footprint and cost thresholds
- [ ] T104 [P] [US3] Create `docs/03-module-3-isaac/examples/09-nav2-planner-client.py` Python script to request path plan from Nav2 and display result
- [ ] T105 [US3] Add code example explanations to chapter 9 markdown

#### User Story 3: Integration & Validation

- [ ] T106 [US3] Create `tests/test-examples/test_module3_examples.py` validating VSLAM node setup, path planning output
- [ ] T107 [US3] Create `tests/test-reproducibility/test_isaac-ros-setup.sh` validating Isaac ROS dependencies and GPU availability
- [ ] T108 [US3] Run Module 3 examples on NVIDIA-enabled hardware (RTX 3070+ recommended) and document actual performance metrics
- [ ] T109 [US3] Create `docs/03-module-3-isaac/README.md` with learning outcomes, hardware requirements, time estimate
- [ ] T110 [US3] Verify all Isaac/Nav2 citations reference official NVIDIA and Nav2 documentation

**Checkpoint**: User Story 3 complete. Reader can implement GPU-accelerated perception and path planning.

---

## Phase 6: User Story 4 - Control Humanoid via Natural Language Commands (Priority: P2)

**Goal**: Readers build voice-to-action pipeline (Whisper → LLM → ROS 2 actions) enabling humanoid control from natural language

**Independent Test**: Speak command, transcribe with Whisper, decompose with LLM, execute ROS 2 actions within 5s latency

**Estimated Time**: 15 hours (split across 3 chapters)

### Implementation for User Story 4

#### Chapter 10: Voice-to-Action with OpenAI Whisper

- [ ] T111 [P] [US4] Write `docs/04-module-4-vla/10-whisper-voice-input.md` covering speech recognition with Whisper, model sizes, accuracy metrics, deployment options (local vs API)
- [ ] T112 [P] [US4] Write `docs/04-module-4-vla/10-section-whisper-setup.md` with local installation (pip), model downloading, audio input configuration
- [ ] T113 [P] [US4] Write `docs/04-module-4-vla/10-section-real-time-transcription.md` covering audio streaming, buffering, confidence scores

#### Chapter 10: Code Examples & Explanations

- [ ] T114 [P] [US4] Create `docs/04-module-4-vla/examples/10-whisper-basic.py` demonstrating basic Whisper usage (load model, transcribe audio file)
- [ ] T115 [P] [US4] Create `docs/04-module-4-vla/examples/10-whisper-realtime.py` implementing real-time voice input with continuous transcription
- [ ] T116 [P] [US4] Create `docs/04-module-4-vla/examples/10-sample-audio.wav` sample audio file for testing Whisper
- [ ] T117 [US4] Add code example explanations to chapter 10 markdown

#### Chapter 11: Cognitive Planning with LLMs

- [ ] T118 [P] [US4] Write `docs/04-module-4-vla/11-llm-action-planning.md` covering LLM-based task decomposition, prompt engineering, reasoning chains, action output format
- [ ] T119 [P] [US4] Write `docs/04-module-4-vla/11-section-prompt-engineering.md` with system prompts, few-shot examples, chain-of-thought patterns for robot task planning
- [ ] T120 [P] [US4] Write `docs/04-module-4-vla/11-section-llm-providers.md` covering OpenAI API (GPT-4) and OpenRouter (fallback models), cost considerations

#### Chapter 11: Code Examples & Explanations

- [ ] T121 [P] [US4] Create `docs/04-module-4-vla/examples/11-system-prompt.txt` example system prompt defining robot capabilities and action format
- [ ] T122 [P] [US4] Create `docs/04-module-4-vla/examples/11-prompt-examples.json` few-shot examples (command → action sequences)
- [ ] T123 [P] [US4] Create `docs/04-module-4-vla/examples/11-llm-planner.py` Python script calling OpenAI API with prompt, parsing action output
- [ ] T124 [US4] Add code example explanations to chapter 11 markdown

#### Chapter 12: Action Execution & Safety

- [ ] T125 [P] [US4] Write `docs/04-module-4-vla/12-action-execution.md` covering ROS 2 action execution, status tracking, timeout handling, safety constraints
- [ ] T126 [P] [US4] Write `docs/04-module-4-vla/12-section-ros2-actions.md` with action server/client pattern, feedback and result handling
- [ ] T127 [P] [US4] Write `docs/04-module-4-vla/12-section-safety-verification.md` covering constraint checking (joint limits, collision avoidance, force limits)

#### Chapter 12: Code Examples & Explanations

- [ ] T128 [P] [US4] Create `docs/04-module-4-vla/examples/12-ros2-action-server.py` ROS 2 action server for humanoid control (execute_task action)
- [ ] T129 [P] [US4] Create `docs/04-module-4-vla/examples/12-action-executor.py` script taking LLM output (JSON actions) and submitting to ROS 2 action server
- [ ] T130 [P] [US4] Create `docs/04-module-4-vla/examples/12-safety-constraints.yaml` configuration of robot limits (joint angles, gripper force, max speed)
- [ ] T131 [US4] Add code example explanations to chapter 12 markdown

#### User Story 4: Integration & Validation

- [ ] T132 [US4] Create `tests/test-examples/test_module4_examples.py` validating Whisper transcription, LLM API calls, action execution
- [ ] T133 [US4] Create test household task commands (e.g., "raise your arm", "open the gripper") for validation
- [ ] T134 [US4] Measure end-to-end latency (voice input → action execution) and verify <5 second requirement
- [ ] T135 [US4] Create `docs/04-module-4-vla/README.md` with learning outcomes and time estimate
- [ ] T136 [US4] Verify all VLA citations reference official Whisper, OpenAI, OpenRouter documentation

**Checkpoint**: User Story 4 complete. Reader can control humanoid with natural language commands.

---

## Phase 7: User Story 5 - Integrate Full System into Autonomous Humanoid Capstone (Priority: P2)

**Goal**: Readers integrate all 4 modules into a complete autonomous system executing household tasks from voice commands; understand extensibility for custom tasks

**Independent Test**: Launch complete system, issue voice command for household task, observe execution; extend with new task via config-only changes

**Estimated Time**: 10 hours (split across 3 chapters)

### Implementation for User Story 5

#### Chapter 13: Capstone System Architecture & Integration

- [ ] T137 [P] [US5] Write `docs/05-capstone-project/13-architecture-overview.md` covering full system architecture (ROS 2 graph, data flow, integration points between modules)
- [ ] T138 [P] [US5] Create `static/diagrams/capstone-ros-graph.txt` ASCII diagram showing all ROS 2 nodes, topics, services, and action servers
- [ ] T139 [P] [US5] Write `docs/05-capstone-project/13-section-component-integration.md` explaining how Module 1 (ROS 2 control), Module 2 (physics sim), Module 3 (perception), Module 4 (VLA) connect

#### Chapter 13: Code Examples & Explanations

- [ ] T140 [P] [US5] Create `docs/05-capstone-project/examples/13-master-launch.yaml` ROS 2 launch file starting all nodes (humanoid controller, simulator, perception, voice interface)
- [ ] T141 [P] [US5] Create `docs/05-capstone-project/examples/13-system-startup.sh` script to verify all modules are running and topics are available
- [ ] T142 [US5] Add integration explanations to chapter 13 markdown

#### Chapter 14: Step-by-Step Capstone Implementation

- [ ] T143 [P] [US5] Write `docs/05-capstone-project/14-implementation-walkthrough.md` with step-by-step setup: install dependencies, launch simulator, start perception, activate voice interface
- [ ] T144 [P] [US5] Write `docs/05-capstone-project/14-section-docker-setup.md` covering containerized capstone (single Docker command to run entire system)
- [ ] T145 [P] [US5] Write `docs/05-capstone-project/14-section-testing-workflows.md` with test scenarios (household tasks) to validate system

#### Chapter 14: Code Examples & Explanations

- [ ] T146 [P] [US5] Create `docs/05-capstone-project/examples/14-docker-compose.yml` full docker-compose with all services (Gazebo, Isaac Sim, ROS 2, voice interface)
- [ ] T147 [P] [US5] Create `docs/05-capstone-project/examples/14-household-tasks.json` configuration of 3+ household tasks (command → action sequence mapping)
- [ ] T148 [P] [US5] Create `docs/05-capstone-project/examples/14-integration-test.py` script running through all household tasks and validating output
- [ ] T149 [US5] Add implementation walkthrough explanations to chapter 14 markdown

#### Chapter 15: Extension & Real-World Deployment

- [ ] T150 [P] [US5] Write `docs/05-capstone-project/15-extending-capstone.md` covering how to add new household tasks (minimal code changes)
- [ ] T151 [P] [US5] Write `docs/05-capstone-project/15-section-custom-tasks.md` with task definition format and LLM prompt examples for custom commands
- [ ] T152 [P] [US5] Write `docs/05-capstone-project/15-section-real-hardware.md` covering deployment to real robots (Boston Dynamics Spot, custom humanoids) with hardware-specific notes

#### Chapter 15: Code Examples & Explanations

- [ ] T153 [P] [US5] Create `docs/05-capstone-project/examples/15-custom-task-template.json` template for defining new household tasks
- [ ] T154 [P] [US5] Create `docs/05-capstone-project/examples/15-new-task-example.json` example of adding a custom task ("water the plant") with LLM prompt
- [ ] T155 [US5] Add extension explanations to chapter 15 markdown

#### User Story 5: Integration & Validation

- [ ] T156 [US5] Create `tests/test-examples/test_capstone_integration.py` validating full system startup, all modules communication, household task execution
- [ ] T157 [US5] Create end-to-end test harness running 3+ household tasks and validating success
- [ ] T158 [US5] Document actual system performance metrics (end-to-end latency per task, success rate, resource utilization)
- [ ] T159 [US5] Create `docs/05-capstone-project/README.md` with capstone learning outcomes, success criteria, time estimate
- [ ] T160 [US5] Verify capstone successfully demonstrates all 4 modules working together

**Checkpoint**: User Story 5 complete. Full autonomous humanoid system operational and extensible.

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Quality improvements, documentation finalization, reproducibility validation, deployment readiness

**Estimated Time**: 10 hours

### Content Finalization

- [ ] T161 [P] Create final `docs/index.md` landing page with book overview, module map, learning paths (beginner vs advanced)
- [ ] T162 [P] Create `docs/glossary.md` with all technical terms (ROS 2, Gazebo, SLAM, etc.) and definitions
- [ ] T163 [P] Create `docs/references.md` with comprehensive bibliography of all cited official docs and papers
- [ ] T164 [P] Create `docs/faq.md` with common reader questions and troubleshooting for each module
- [ ] T165 [P] Create `docs/contributors.md` with acknowledgments and contribution guidelines

### Citation & Link Validation

- [ ] T166 Create `scripts/validate-all-citations.py` script checking all APA citations for correct format and URL reachability
- [ ] T167 [P] Run citation validation across all chapters; fix any broken links or format issues
- [ ] T168 [P] Create `scripts/check-internal-links.py` validating all markdown cross-references resolve correctly

### Code Example Validation

- [ ] T169 Create comprehensive test suite `tests/test-examples/test_all_modules.py` validating every code example across all 5 chapters
- [ ] T170 [P] Run full test suite on Windows WSL2; fix any platform-specific issues
- [ ] T171 [P] Run full test suite on Ubuntu 22.04 (primary platform); document actual execution times
- [ ] T172 [P] Run full test suite on macOS Apple Silicon; identify and document any incompatibilities

### Documentation Generation

- [ ] T173 Update `README.md` at repository root with book overview, quick start (Docker), installation instructions for native setup
- [ ] T174 Update `CONTRIBUTING.md` with final content standards and pull request guidelines
- [ ] T175 Create `DEPLOYMENT.md` documenting GitHub Pages build and deploy process

### Docusaurus Build & Optimization

- [ ] T176 Run `npm run build` for production Docusaurus build; verify 0 warnings
- [ ] T177 [P] Optimize images in `static/` (compress, resize for web)
- [ ] T178 [P] Verify Docusaurus sitemap.xml and robots.txt are correct for GitHub Pages
- [ ] T179 Test live site locally with `npm run serve` to verify all pages render correctly

### GitHub Pages Deployment

- [ ] T180 Verify GitHub Actions workflow `.github/workflows/deploy.yml` is configured for automatic deployment on main branch
- [ ] T181 Push code to `main` branch and trigger automated Docusaurus build
- [ ] T182 Verify live GitHub Pages site is accessible and all pages load
- [ ] T183 Test live site functionality (search, navigation, code block syntax highlighting)

### Final QA & Documentation

- [ ] T184 Create `BOOK_METRICS.md` documenting final book statistics (chapters, code examples, lines of code, words written, citations)
- [ ] T185 [P] Proofread all chapters for grammar, clarity, technical accuracy
- [ ] T186 [P] Verify all module learning outcomes are met (each chapter teaches what it claims)
- [ ] T187 Create final `RELEASE_NOTES.md` with v1.0 release announcement and feature summary

**Checkpoint**: Book complete, validated, and deployed to GitHub Pages

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies - can start immediately ✅
- **Phase 2 (Foundational)**: Depends on Phase 1 completion - BLOCKS all user stories
- **Phase 3+ (User Stories)**: All depend on Phase 2 completion; can proceed in parallel or sequentially
- **Phase 8 (Polish)**: Depends on all desired user stories being complete

### User Story Dependencies (User Stories 1-5)

- **US1 (ROS 2)**: Can start after Phase 2 - No dependencies on other stories ✅
- **US2 (Digital Twin)**: Can start after Phase 2 - Independent from US1 (but typically follows in learning path)
- **US3 (Isaac)**: Can start after Phase 2 - Independent from US1, US2
- **US4 (VLA)**: Can start after Phase 2 - Independent from US1, US2, US3 (but conceptually builds on prior knowledge)
- **US5 (Capstone)**: Depends on US1 + US2 + US3 + US4 - Integrates all modules together

### Parallel Opportunities

- **Phase 1**: All tasks marked [P] can run in parallel (different files, no dependencies)
- **Phase 2**: All tasks marked [P] can run in parallel (templates, diagrams, configs)
- **Phase 3-7**: All [P] tasks within a user story can run in parallel:
  - All chapter writing for a module can be parallel (different markdown files)
  - All code examples for a module can be parallel (different Python files)
- **Different user stories**: US1, US2, US3, US4 can be worked on simultaneously by different writers (after Phase 2)
- **US5 (Capstone)**: Must wait for US1-4 to be complete, but can be worked on while Phase 8 (polish) happens in parallel

### Suggested Parallel Team Strategy (if available)

**Team of 4 writers**:
1. **Writer A**: Phases 1-2 (setup + templates)
2. **Writer B**: User Story 1 (Module 1: ROS 2)
3. **Writer C**: User Story 2 (Module 2: Digital Twin)
4. **Writer D**: User Story 3 (Module 3: Isaac)
5. **Writer E** (new): User Story 4 (Module 4: VLA) - can start after Phase 2
6. **All Writers**: User Story 5 (Capstone) - after US1-4 complete
7. **All Writers**: Phase 8 (Polish) - final review and validation

---

## Implementation Strategy

### MVP First (Recommended for Solo Author)

**Timeline: ~8-10 weeks**

1. ✅ Complete **Phase 1**: Setup (T001-T015) - 2-3 hours
2. ✅ Complete **Phase 2**: Foundational (T016-T027) - 2-3 hours
3. ✅ Complete **Phase 3**: User Story 1 (T028-T060) - 15 hours
4. **STOP and VALIDATE** - Test US1 independently; collect reader feedback
5. 📈 Deploy MVP (Module 1 only) to GitHub Pages
6. ✅ Complete **Phase 4**: User Story 2 (T061-T087) - 15 hours
7. ✅ Complete **Phase 5**: User Story 3 (T088-T110) - 15 hours
8. ✅ Complete **Phase 6**: User Story 4 (T111-T136) - 15 hours
9. ✅ Complete **Phase 7**: User Story 5 (T137-T160) - 10 hours
10. ✅ Complete **Phase 8**: Polish (T161-T187) - 10 hours
11. 🎉 **DEPLOY v1.0** to GitHub Pages

### Incremental Delivery

After MVP (Module 1):
1. Deploy Module 1 → get feedback → iterate
2. Add Module 2 → validate Digital Twin independently
3. Add Module 3 → validate Isaac perception independently
4. Add Module 4 → validate VLA pipeline independently
5. Add Capstone → full system demonstration
6. Polish and optimize

**Total estimated time: 13-15 weeks** (concurrent modules reduce timeline)

---

## Parallel Execution Example: Writing Module 1 (User Story 1)

Assuming 3 parallel writers:

```bash
# Writer A writes all Chapters 1-3 markdown explanations
Task: T028-T050 (chapter.md files)
# Running in parallel:
Writer B writes all code examples (T032-T055)
Writer C creates validation tests (T056-T060)

# Sequential within each writer:
Writer A: Chapter 1 → Chapter 2 → Chapter 3
Writer B: All pub/sub examples → Service examples → URDF examples
Writer C: Write tests → Validate examples → Documentation

# Merge and validate:
All writers: Run tests, verify reproducibility, validate citations
```

---

## Task Summary

| Phase | Task Count | Estimated Time | Status |
|-------|-----------|-----------------|--------|
| **Phase 1**: Setup | 15 tasks (T001-T015) | 2-3 hours | 📋 Pending |
| **Phase 2**: Foundational | 12 tasks (T016-T027) | 2-3 hours | 📋 Pending |
| **Phase 3**: US1 (ROS 2) | 33 tasks (T028-T060) | 15 hours | 📋 Pending |
| **Phase 4**: US2 (Digital Twin) | 27 tasks (T061-T087) | 15 hours | 📋 Pending |
| **Phase 5**: US3 (Isaac) | 23 tasks (T088-T110) | 15 hours | 📋 Pending |
| **Phase 6**: US4 (VLA) | 26 tasks (T111-T136) | 15 hours | 📋 Pending |
| **Phase 7**: US5 (Capstone) | 24 tasks (T137-T160) | 10 hours | 📋 Pending |
| **Phase 8**: Polish | 27 tasks (T161-T187) | 10 hours | 📋 Pending |
| **TOTAL** | **187 tasks** | **~95 hours** | 🔄 Ready to Execute |

---

## Notes

- All file paths are exact and actionable
- Each task produces a specific, tangible output (file or validated test)
- Tests are NOT included in core chapter tasks (per Constitution principle: keep code examples simple)
- Validation tests in Phase 3-7 ensure reproducibility and accuracy
- [P] markers indicate parallelizable tasks; non-marked tasks have sequential dependencies
- Phase 8 tasks can overlap with final user story work (parallel path)
- Commit frequently (after each task or logical group of [P] tasks)
- Stop at any phase checkpoint to validate progress

---

## Checkpoints for MVP Validation

✅ **Checkpoint 1** (after Phase 1-2): Docusaurus builds; templates ready
✅ **Checkpoint 2** (after Phase 3): Module 1 complete; reader can write ROS 2 nodes
✅ **Checkpoint 3** (after Phase 4): Module 2 complete; physics simulation working
✅ **Checkpoint 4** (after Phase 5): Module 3 complete; perception pipeline deployed
✅ **Checkpoint 5** (after Phase 6): Module 4 complete; voice commands working
✅ **Checkpoint 6** (after Phase 7): Capstone complete; full system operational
✅ **Checkpoint 7** (after Phase 8): Book polished; deployed to GitHub Pages; v1.0 ready

