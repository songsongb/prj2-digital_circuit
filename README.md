# FPGA-Based Autonomous Driving Perception–Decision–Planning Pipeline System

**Valet Parking Scenario | DE2 FPGA + HuskyLens + OV7670 + TRDB LCM**

## 1. Project Overview

### 1.1 Project Title

**FPGA-Based Autonomous Driving Perception–Decision–Planning Pipeline System**
**Applied to a Valet Parking Scenario**

### 1.2 Core Concept

This project implements the essential autonomous driving pipeline:

```text
Perception → Decision → Planning
```

on an FPGA/RISC-V-based system.

The final **Control** stage is replaced by a PC dashboard animation. This approach is similar to a Hardware-in-the-Loop (HIL) simulation method commonly used in real autonomous driving development, where hardware perception and decision logic are verified through a simulated vehicle motion environment.

### 1.3 Demonstration Strategy

| Component                          | Role                                                                                                  | Limitation and Compensation                                                           |
| ---------------------------------- | ----------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------- |
| RC Car, manually driven by a human | Performs real-world environment recognition using HuskyLens and captures real-time video using OV7670 | The RC car itself is not autonomous, so autonomous motion is represented by animation |
| PC Dashboard Animation             | Visualizes the autonomous driving path based on RISC-V computation results                            | Pure simulation alone may lack realism, so the RC car provides real-world sensing     |
| DE2 Hardware Output                | Displays driving direction and system status in real time through HEX, LED, and LCD                   | Allows the audience to intuitively verify system operation                            |

---

## 2. Modules and Roles

| Module                      | Type               | Role                                                                                 | Interface                 |
| --------------------------- | ------------------ | ------------------------------------------------------------------------------------ | ------------------------- |
| Intel Altera DE2 FPGA Board | FPGA board         | Main system platform containing the RISC-V core and RTL modules                      | Cyclone II EP2C35         |
| HuskyLens SEN0305           | Main vision sensor | Core environment recognition: tag recognition, object recognition, and line tracking | UART 115200 bps           |
| OV7670 Camera Module        | Sub camera module  | Real-time image capture and auxiliary wall detection                                 | Parallel 8-bit + SCCB/I2C |
| TRDB LCM                    | LCD display        | Displays OV7670 real-time camera video to improve demo realism                       | Serial 8-bit RGB/YUV      |
| PC Python Application       | Software platform  | LLM API integration and dashboard visualization                                      | UART USB-Serial           |

---

## 3. HuskyLens Recognition Design: Perception Layer

### 3.1 Three-Mode Cyclic Polling

HuskyLens can operate in only one mode at a time. Therefore, the FPGA uses a 25 ms timer interrupt to cyclically switch between three modes and recognize the environment.

| Polling Order                       | Mode               | Detection Target                | Processing Result                                                                     |
| ----------------------------------- | ------------------ | ------------------------------- | ------------------------------------------------------------------------------------- |
| Tick 0, 1, 3, 4: 25 ms × 4 = 100 ms | Line Tracking      | Driving lane/path line          | Generates direction commands: GO, LEFT, RIGHT, STOP, BACK                             |
| Tick 2: 25 ms                       | Tag Recognition    | AprilTag parking section marker | Determines current position and whether the cell is a parking space                   |
| Tick 5: 25 ms                       | Object Recognition | Car object, fixed category      | If both tag and car are detected, marks the corresponding parking section as OCCUPIED |

### 3.2 Parking Map Construction Rules

* Map size: 6 × 6 grid, total 36 cells.
* Entrance: fixed at coordinate `(1, 1)`.
* Exit: fixed at coordinate `(6, 6)`.
* Parking space detection:

  * If an AprilTag is detected, the cell is considered a parking space.
* Empty/occupied classification:

  * AprilTag only → `EMPTY`
  * AprilTag + car detected together → `OCCUPIED`
* Cells without tags are treated as driving paths:

  * `CELL_PATH`
* The RC car is manually driven around the parking lot once to complete the map.

### 3.3 OV7670 Usage

* Displays real-time camera video on the TRDB LCM so that the audience can visually confirm the environment.
* Performs auxiliary front-wall detection using pixel brightness changes.
* Wall detection is processed at the FPGA hardware level.
* HuskyLens acts as the main vision sensor.
* OV7670 acts as a sub-sensor.

---

## 4. DE2 Hardware Output Design

### 4.1 Line Tracking Direction Display

The direction command generated from the HuskyLens Line Tracking result is displayed simultaneously through multiple DE2 board outputs.

| Output Device | Display Content         | Details                                                    |
| ------------- | ----------------------- | ---------------------------------------------------------- |
| HEX0          | Direction character     | GO → `G`, LEFT → `L`, RIGHT → `r`, STOP → `S`, BACK → `b`  |
| HEX1          | Current FSM state       | `0 = IDLE`, `1 = SCAN`, `2 = ANLZ`, `3 = WAIT`, `4 = DONE` |
| HEX2          | Current X coordinate    | Displays number 1–6                                        |
| HEX3          | Current Y coordinate    | Displays number 1–6                                        |
| LEDR[0]       | GO indicator            | ON while moving forward                                    |
| LEDR[1]       | LEFT indicator          | ON while turning left                                      |
| LEDR[2]       | RIGHT indicator         | ON while turning right                                     |
| LEDR[3]       | STOP indicator          | ON while stopped                                           |
| LEDR[4]       | BACK indicator          | ON while moving backward                                   |
| LEDR[7]       | Scan complete indicator | ON when map construction is complete                       |
| TRDB LCM      | OV7670 real-time video  | Displays the camera image directly                         |
| KEY[0]        | System reset            | Press to reset the system                                  |
| KEY[1]        | Scan start              | Press to enter the SCANNING state                          |
| SW[0]         | Manual mode switch      | ON: manual mode, OFF: automatic polling mode               |

---

## 5. FPGA RTL Module Design: Hardware Layer

| Module             | Role                                                           | Interface                 |
| ------------------ | -------------------------------------------------------------- | ------------------------- |
| `UART_RX.v`        | Receives UART data from HuskyLens or PC using 16× oversampling | 115200 bps                |
| `UART_TX.v`        | Sends HuskyLens mode commands and PC data                      | 115200 bps                |
| `HL_PARSER.v`      | Parses HuskyLens packets and verifies header/checksum          | Internal bus              |
| `TIMER.v`          | Generates 25 ms periodic interrupt for RISC-V polling control  | Based on 50 MHz clock     |
| `OV7670_CAPTURE.v` | Captures camera pixel data and processes VSYNC/HREF/PCLK       | Parallel 8-bit            |
| `LCD_CTRL.v`       | Controls TRDB LCM output and displays real-time video          | Serial RGB                |
| `WALL_DETECT.v`    | Analyzes OV7670 pixels and detects front wall                  | Frame buffer              |
| `DMEM_MMIO.v`      | Data memory and MMIO bus connecting RISC-V to peripherals      | MMIO address map          |
| `TOP.v`            | Top-level module connecting all system modules                 | Direct DE2 pin connection |

### 5.1 MMIO Address Map

| Address      | Direction | Description                                                           |
| ------------ | --------- | --------------------------------------------------------------------- |
| `0x10000000` | RO        | HuskyLens result: `[31:24] ALGO`, `[23:16] ID`, `[15:8] X`, `[7:0] Y` |
| `0x10000004` | RO        | HuskyLens bounding box: `[31:16] W`, `[15:0] H`                       |
| `0x10000008` | WO        | HuskyLens mode switch: `0x01 = Tag`, `0x02 = Object`, `0x03 = Line`   |
| `0x1000000C` | RW        | HuskyLens status: `[1] NO_RESULT`, `[0] DATA_VALID`, write = clear    |
| `0x10000010` | RO        | Number of detected objects                                            |
| `0x10000014` | RO        | Wall detection flag from OV7670                                       |
| `0x10000020` | WO        | 7-segment HEX output                                                  |
| `0x10000024` | WO        | LED output                                                            |
| `0x10000040` | WO        | PC UART transmit data                                                 |
| `0x10000044` | WO        | PC UART transmit trigger                                              |
| `0x10000048` | RO        | PC UART receive data                                                  |
| `0x1000004C` | RW        | PC UART receive valid flag, write = clear                             |
| `0x10000050` | RO        | KEY input `[1:0]`                                                     |
| `0x10000054` | RO        | SW input `[0]`                                                        |
| `0x10000060` | RW        | Timer flag, write = clear                                             |

---

## 6. RISC-V Software Design: Planning Layer

### 6.1 Data Structures

* Parking lot bitmap:

```c
uint8_t map[6][6];
```

Cell encoding:

| Value | Meaning                |
| ----- | ---------------------- |
| 0     | Unknown                |
| 1     | Path                   |
| 2     | Empty parking space    |
| 3     | Occupied parking space |
| 4     | Entrance               |
| 5     | Exit                   |

* Current vehicle position:

```c
Pos car_pos {x, y};
```

Coordinates are 1-based.

* Vehicle heading:

| Value | Direction |
| ----- | --------- |
| 0     | North     |
| 1     | East      |
| 2     | South     |
| 3     | West      |

* A* nodes:

  * Static array-based Min-Heap.
  * No `malloc`.
  * Maximum of 36 nodes.

* Path buffer:

```c
uint8_t path_x[36];
uint8_t path_y[36];
uint8_t path_len;
```

* Score array:

```c
uint8_t score[6][6];
```

The score is based on exit distance and obstacle adjacency.

### 6.2 FSM State Definition

| State      | Entry Condition          | Operation                                                                                                   | Transition Condition  |
| ---------- | ------------------------ | ----------------------------------------------------------------------------------------------------------- | --------------------- |
| IDLE       | System start or reset    | Wait for KEY[1]                                                                                             | KEY[1] pressed        |
| SCANNING   | KEY[1] pressed           | Cyclically polls HuskyLens in 3 modes, updates the map in real time, and displays direction through HEX/LED | Map scan complete     |
| ANALYZING  | Scan complete            | Extracts empty parking candidates, calculates score, serializes JSON, and sends it to PC                    | Transmission complete |
| WAITING_AI | PC transmission complete | Waits for LLM response and displays `HEX1 = 3`                                                              | JSON received         |
| EXECUTING  | JSON received            | Performs A* path search, sends path sequence to PC, and displays target parking section on HEX              | Transmission complete |
| DONE       | Transmission complete    | Turns on LEDR[7] and maintains the result                                                                   | KEY[0] reset          |

### 6.3 A* Algorithm

* Uses Manhattan distance heuristic.
* Uses Q4 fixed-point integer arithmetic.
* Uses a static array-based Min-Heap.
* No dynamic allocation.
* Maximum map size is 6 × 6 = 36 nodes.
* No memory overflow is expected.
* If path search fails, the system reports the deadlock situation to the Agent AI as `Path Not Found`.

### 6.4 Line Tracking Direction Calculation

* Direction is determined based on the line center X coordinate.
* X coordinate range: 0–320.
* Center point: 160.
* Center ±30 → `GO`.
* If the line center is to the right of the center, output `LEFT`.

  * The vehicle must turn left to follow the line.
* If the line center is to the left of the center, output `RIGHT`.
* If no line is detected, output `STOP`.
* The direction result is immediately displayed on HEX/LED and transmitted to the PC.

---

## 7. Agent AI Design: Decision Layer

### 7.1 VLA Structure

The Agent AI follows a VLA-style structure:

```text
Vision + Language + Action
```

* Vision:

  * HuskyLens visual recognition.
* Language:

  * LLM-based natural language reasoning.
* Action:

  * JSON action command output.

This combination allows the system to handle complex contextual situations that are difficult to process using simple if-else logic.

### 7.2 Three Scenarios Where Agent AI Is Meaningful

| Scenario                                                  | Limitation of if-else Logic                             | Role of Agent AI                                                                   |
| --------------------------------------------------------- | ------------------------------------------------------- | ---------------------------------------------------------------------------------- |
| Tie between candidate parking spaces                      | If scores are equal, simple logic cannot decide clearly | Considers vehicle size, exit distance, and congestion together                     |
| Path deadlock: `Path Not Found`                           | RISC-V alone cannot resolve the situation               | Analyzes the entire map and assigns an alternative destination                     |
| Emergency vehicle entry detected by object classification | Simple thresholds cannot determine priority properly    | Cancels current assignment, secures a dedicated section, and reallocates the route |

### 7.3 LLM Input/Output JSON Structure

#### Input: RISC-V → PC → LLM

```json
{
  "map": "map[6][6] full map state",
  "vehicle": {
    "position": "vehicle position",
    "type": "normal or emergency"
  },
  "candidates": "candidate parking sections with scores",
  "path_found": "whether A* path search succeeded",
  "state": "current FSM state"
}
```

Input fields:

* `map[6][6]`: full map state.
* `vehicle`: vehicle position and vehicle type, either normal or emergency.
* `candidates`: list of candidate parking sections and their scores.
* `path_found`: whether A* path search succeeded.
* `state`: current FSM state.

#### Output: LLM → PC → RISC-V

```json
{
  "action": "ASSIGN_SLOT / WAIT / REROUTE",
  "target_slot": "target parking section ID",
  "target_pos": [x, y],
  "reason": "1–2 Korean sentences explaining the decision"
}
```

Output fields:

* `action`: `ASSIGN_SLOT`, `WAIT`, or `REROUTE`.
* `target_slot`: target parking section ID.
* `target_pos`: target coordinates `[x, y]`.
* `reason`: one or two Korean sentences explaining the decision, shown on the dashboard.

---

## 8. PC Dashboard Design

### 8.1 Screen Layout

| Area                                 | Content                                                                                                                                                                    |
| ------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Left: 6 × 6 parking grid             | Cell colors: empty = green, occupied = red, path = gray, target = blue. Displays the current RC car position icon and updates in real time based on HuskyLens scan results |
| Center: autonomous driving animation | Shows A* path using yellow arrows. A virtual vehicle moves smoothly along the path, visualizing autonomous driving that the RC car itself cannot physically perform        |
| Upper right: line tracking panel     | Displays the current direction command, such as GO, LEFT, RIGHT, STOP, or BACK, in large text. Also shows a real-time graph of the line center coordinate                  |
| Middle right: Agent AI panel         | Displays the decision reason text in real time and shows timestamped logs                                                                                                  |
| Lower right: system status           | Displays the current FSM state, map scan progress, and connection status                                                                                                   |

---

## 9. Development Schedule: Two-Week Plan

| Day      | Task                                                                                                        | Completion Criteria                                      |
| -------- | ----------------------------------------------------------------------------------------------------------- | -------------------------------------------------------- |
| Day 1–2  | Install the RISC-V toolchain and complete HuskyLens UART communication test. Verify received data on the PC | HuskyLens data is received through the PC serial monitor |
| Day 3–4  | Write basic RISC-V C code structure, implement bitmap data structure, and connect MMIO interface            | RISC-V can read HuskyLens data                           |
| Day 5    | Implement three-mode cyclic polling, timer interrupt-based mode switching, and HEX/LED direction display    | Direction command is displayed on HEX/LED                |
| Day 6–7  | Implement A* algorithm, static Min-Heap, and 6 × 6 test cases                                               | Path search works correctly                              |
| Day 8    | Implement 6-state FSM, scoring computation, JSON serialization, and PC transmission                         | PC receives JSON successfully                            |
| Day 9–10 | Implement OV7670 capture module, connect TRDB LCM output, and display real-time video                       | Camera video is displayed on LCD                         |
| Day 11   | Implement PC Python UART receive, asynchronous LLM API integration, JSON parsing, and command transmission  | Agent AI judgment loop is completed                      |
| Day 12   | Implement PC dashboard with grid, animation, AI panel, and line tracking display panel                      | Entire dashboard operation is verified                   |
| Day 13   | Perform full integration test, verify three scenarios, and fix bugs                                         | End-to-end operation is verified                         |
| Day 14   | Conduct demo rehearsal and prepare presentation materials                                                   | Final demo is ready                                      |

---

## 10. Demo Scenario: 10 Minutes

| Stage                         | Time     | RC Car Operation                                                                                                                     | Board/PC Display                                                                                                           |
| ----------------------------- | -------- | ------------------------------------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------- |
| Environment Scan              | 0–3 min  | Manually drive the RC car around the parking lot once. HuskyLens recognizes tags and objects. OV7670 displays real-time video on LCD | HEX displays direction, LED indicates direction, and PC grid fills in real time                                            |
| AI Decision                   | 3–6 min  | RC car stops after scanning is complete                                                                                              | HEX1 shows WAIT state. PC displays Agent AI decision reason and highlights the recommended parking section in blue         |
| Autonomous Driving Simulation | 6–9 min  | RC car waits. Actual autonomous driving is replaced with simulation                                                                  | PC shows a virtual vehicle moving along the A* path. HEX displays the target section ID. LEDR[7] turns on after completion |
| Emergency Vehicle Scenario    | 9–10 min | RC car recognizes an emergency vehicle model through custom object classification                                                    | Agent AI re-evaluates the situation, updates the route, and changes the displayed decision reason                          |

---

## 11. Evaluation Criteria Response Strategy

| Evaluation Item                           |     Score | Response Strategy                                                                                                                                            |
| ----------------------------------------- | --------: | ------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Demo preference and creativity            | 12 points | Run RC car hardware, OV7670 LCD video, and dashboard animation simultaneously. Use the emergency vehicle scenario to create a dramatic transition            |
| RISC-V utilization                        |  6 points | Execute A* Min-Heap, FSM, scoring, and JSON serialization entirely in RISC-V C code. Clearly separate FPGA and software roles using MMIO                     |
| Presentation clarity                      |  5 points | Explain the system clearly within 5 minutes using the Perception → Decision → Planning pipeline structure. HEX/LED outputs visually show real-time operation |
| Vision sensor utilization                 |  5 points | Demonstrate actual HuskyLens three-mode operation and OV7670 + TRDB LCM real-time video display                                                              |
| Agent AI utilization                      |  4 points | Demonstrate three scenarios where if-else logic is insufficient and display the AI decision reason on the dashboard in real time                             |
| Implementation completeness and stability |  3 points | Minimize hardware failure risk by replacing physical autonomous driving with simulation. Allow manual demo flow control using KEY/SW                         |
| PC integration bonus                      | +2 points | Implement bidirectional UART, LLM API integration, and dashboard visualization                                                                               |

---

## 12. Resume and Application Connection Points

| Company / Role                                                  | Project Connection Keywords                                                                   |
| --------------------------------------------------------------- | --------------------------------------------------------------------------------------------- |
| Hyundai Mobis — SoC Development                                 | RISC-V core design, FPGA prototyping using Altera DE2, MMIO interface design                  |
| Hyundai Mobis — Automotive Multimodal LLM                       | Vision-Language-Action pipeline, real-time vehicle-environment LLM decision-making            |
| Hyundai Motor — Autonomous Driving Development                  | Camera perception, object detection, A* path planning, autonomous parking decision system     |
| Hyundai Motor — Software/Hardware Architecture                  | Hardware-software co-design, separation of FPGA RTL and RISC-V software roles                 |
| Samsung Electronics DX — AI Development                         | Agentic AI, context-based complex decision-making, explainable AI with decision-reason output |
| LG Innotek — Autonomous Driving Sensors                         | Object detection, vision sensor data pipeline, sensor fusion concept                          |
| Samsung Electro-Mechanics — Hardware-Based Software Development | AI agent, image processing, physical AI, embedded system design                               |

---

## Note

This project plan may be modified during the development process.
