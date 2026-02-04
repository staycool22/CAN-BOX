import base64
import requests
import sys
import os

def generate_flowchart():
    """
    Generates a PNG image of the system framework flowchart using the Mermaid.ink API.
    This approach is used to avoid requiring a local Graphviz installation.
    """
    
    # Define the Mermaid graph definition
    mermaid_graph = """
    graph TD
        %% Styles
        classDef input fill:#e1f5fe,stroke:#01579b,stroke-width:2px;
        classDef core fill:#fff9c4,stroke:#fbc02d,stroke-width:2px;
        classDef logic fill:#e8f5e9,stroke:#2e7d32,stroke-width:2px;
        classDef actuator fill:#f3e5f5,stroke:#7b1fa2,stroke-width:2px;
        classDef hardware fill:#eceff1,stroke:#455a64,stroke-width:2px;

        subgraph InputLayer [输入层-ui/driver]
            KB[键盘输入<br>test_steer_control]:::input
            JOY[手柄输入<br>JoystickController]:::input
            WEB[Web 仪表盘<br>DashboardClient]:::input
        end

        subgraph CoreLayer [核心层-core]
            STATE[(ChassisState<br>单例数据中心)]:::core
        end

        subgraph ActuatorLayer [执行器层-actuators]
            SYSTEM[ChassisSystem<br>中央调度器 100Hz Loop]:::actuator
            CTRL[SteerController<br>运动控制器]:::actuator
            MON[VESCMonitor<br>CAN 总线管理]:::actuator
            MOD[SwerveModule<br>FL/FR/RL/RR 模组]:::actuator
        end

        subgraph LogicLayer [算法层-algorithm]
            KIN[逆运动学解算<br>ChassisKinematics]:::logic
            OPT[Swerve 路径优化<br>最短路径/反转逻辑]:::logic
        end

        subgraph HardwareLayer [物理硬件]
            CAN{{CAN Bus}}:::hardware
            MOTOR((电机/VESC)):::hardware
        end

        %% Data Flow
        KB -->|写入命令| STATE
        JOY -->|写入命令| STATE
        WEB -->|写入命令| STATE

        STATE <-->|读取命令 / 更新状态| SYSTEM
        
        SYSTEM -->|1. 目标速度/角度| KIN
        KIN -->|2. 原始轮速/轮角| OPT
        OPT -->|3. 优化后状态| CTRL
        
        CTRL -->|4. 电机指令| MON
        MON -->|5. 分发| MOD
        MOD -->|6. CAN 帧| CAN
        CAN <--> MOTOR

        %% Feedback Flow
        MOTOR -.->|反馈 RPM/角度| CAN
        CAN -.->|解析| MON
        MON -.->|更新| MOD
        MOD -.->|汇总| STATE
        STATE -.->|广播状态| WEB
    """

    print("Generating flowchart...")
    
    # Encode the graph definition
    graphbytes = mermaid_graph.encode("utf8")
    base64_bytes = base64.urlsafe_b64encode(graphbytes)
    base64_string = base64_bytes.decode("ascii")
    
    # Construct the URL (using styling and higher quality)
    # bgColor=!white allows for a white background
    url = f"https://mermaid.ink/img/{base64_string}?bgColor=!white"
    
    try:
        print(f"Requesting image from: {url[:50]}...")
        response = requests.get(url, timeout=10)
        
        if response.status_code == 200:
            output_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "system_framework_flowchart.png")
            with open(output_file, "wb") as f:
                f.write(response.content)
            print(f"✅ Successfully generated: {output_file}")
        else:
            print(f"❌ Error: Failed to generate image. Status code: {response.status_code}")
            print(response.text)
            
    except Exception as e:
        print(f"❌ Exception occurred: {e}")

if __name__ == "__main__":
    generate_flowchart()
