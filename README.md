# This is a work in progress / help for myself to know what to add to readme
## stuff to add here

- Diagrams showing data flow in program
- Section outlining how to create new components and stuff, like if you add something to sensor_tasks you need to add in cmakelists stuff. 

```mermaid
flowchart TB
    subgraph Vehicle["Vehicle MCU (ESP32-S3)"]
        direction TB
        
        %% Data Pool in center
        DataPool[("Data Pool\n(Thread-safe singleton)")]
        
        %% Individual Sensor Tasks with their components
        subgraph IMUTask["IMU Task"]
            ICM["ICM20948\nDriver"]
        end
        
        subgraph GPSTask["GPS Task"]
            TinyGPS["TinyGPS++\nESP-IDF Layer"]
        end
        
        subgraph SBUSTask["SBUS Task"]
            SBUSDriver["SBUS\nDecoder"]
        end
        
        %% Hardware Control Tasks
        subgraph DSHOTTask["DSHOT Task"]
            DShotEnc["DSHOT ESC\nEncoder"]
        end
        
        subgraph ServoTask["Servo Task"]
            ServoDrv["Servo\nDriver"]
        end
        
        %% Vehicle Controller
        VehicleCtrl["Vehicle Dynamics\nController"]
        
        %% Telemetry System with clear ownership
        subgraph TelemetryMgr["Telemetry Manager"]
            direction TB
            MsgQueue[("Message Queue\n(Commands/Sensor/Text/Heartbeat)")]
            DataFetcher["Data Fetcher\nTask"]
            MsgLogger["Message Logger"]
            QueueConsumer["Queue Consumer\nTask"]
        end
        
        %% Connect sensors to DataPool
        IMUTask -->|Write| DataPool
        GPSTask -->|Write| DataPool
        SBUSTask -->|Write| DataPool
        DSHOTTask -->|"Write\n(ESC Telemetry)"| DataPool
        
        %% Control flow
        DataPool -->|Read| VehicleCtrl
        VehicleCtrl -->|Control| DSHOTTask
        VehicleCtrl -->|Control| ServoTask
        
        %% Telemetry Flow
        DataPool -->|Read| DataFetcher
        DataFetcher -->|Queue Data| MsgQueue
        MsgLogger -->|Queue Messages| MsgQueue
        MsgQueue -->|Consume| QueueConsumer
    end
    
    %% External Components
    Motors["4x BLDC Motors\n(with ESC)"] 
    ServoHW["Steering Servo"]
    BaseStation["Base Station\n(Data Processing & Visualization)"]
    
    %% External Connections
    DSHOTTask -->|"DSHOT300\nProtocol"| Motors
    ServoTask -->|"PWM\n(50Hz)"| ServoHW
    QueueConsumer -->|"ESP-NOW\nProtocol"| BaseStation

    %% Styling
    classDef default fill:#f9f9f9,stroke:#333,stroke-width:2px;
    classDef system fill:#e1f5fe,stroke:#01579b,stroke-width:2px;
    classDef task fill:#fff3e0,stroke:#e65100,stroke-width:2px;
    classDef hardware fill:#f3e5f5,stroke:#4a148c,stroke-width:2px;
    classDef datastore fill:#fbe9e7,stroke:#bf360c,stroke-width:2px;
    classDef component fill:#f1f8e9,stroke:#33691e,stroke-width:1px;
    
    class Vehicle,BaseStation system;
    class IMUTask,GPSTask,SBUSTask,DSHOTTask,ServoTask,VehicleCtrl,DataFetcher,QueueConsumer task;
    class Motors,ServoHW hardware;
    class DataPool,MsgQueue datastore;
    class ICM,TinyGPS,SBUSDriver,DShotEnc,ServoDrv component;
    ```