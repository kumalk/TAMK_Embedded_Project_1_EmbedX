# EmbedX Robot Main Loop Flowchart

This flowchart describes the main execution loop (`loop()`) of the `EmbedX_Robot.ino` program.

```mermaid
flowchart TD
    Start([Start Loop]) --> CheckFollowFlag{is followFlag true?}

    %% Follow Logic
    CheckFollowFlag -- Yes --> CheckDistFollow{Check Distance}
    CheckDistFollow -- "Dist > Gap + MaxErr" --> MoveForwardFollow[Move Forward]
    CheckDistFollow -- "Dist < Gap - MaxErr" --> MoveBackwardFollow[Move Backward]
    CheckDistFollow -- "In range" --> StopMotorsFollow[Stop Motors]
    
    MoveForwardFollow --> CheckPathFlag
    MoveBackwardFollow --> CheckPathFlag
    StopMotorsFollow --> CheckPathFlag
    CheckFollowFlag -- No --> CheckPathFlag

    %% Path Plan Logic
    CheckPathFlag{is pathFlag true?}
    CheckPathFlag -- Yes --> CalculateBearingError[Calculate Bearing Error]
    CalculateBearingError --> CheckBearingError{Abs(Bearing Error) > 3.0?}
    CheckBearingError -- Yes --> FixBearing[Turn to correct bearing]
    CheckBearingError -- No --> ManageDistancePath{Manage Distance}
    
    ManageDistancePath -- "Dist > Gap + MaxErr" --> MoveForwardPath[Drive Straight]
    ManageDistancePath -- "Dist < Gap - MaxErr" --> MoveBackwardPath[Back Up]
    ManageDistancePath -- "In range" --> StopMotorsPath[Stop Motors & Advance Step]
    
    FixBearing --> CheckModeButton1
    MoveForwardPath --> CheckModeButton1
    MoveBackwardPath --> CheckModeButton1
    StopMotorsPath --> CheckModeButton1
    CheckPathFlag -- No --> CheckModeButton1

    %% Mode Change Button
    CheckModeButton1{is joyPressedFlag true?}
    CheckModeButton1 -- Yes --> ResetFlags1[Stop Motors, log mode switch]
    ResetFlags1 --> ReadCompass
    CheckModeButton1 -- No --> ReadCompass

    %% Common Sensor Read
    ReadCompass[Read Compass: bearingDegrees] --> ControlModeLogic

    %% Control Mode Logic
    ControlModeLogic{Control Mode?}
    ControlModeLogic -- "ESP" --> ReadSerial[Process ESP/Serial Commands]
    ControlModeLogic -- "JOY" --> ReadJoy[Read Joystick & Run Motors]
    ControlModeLogic -- "Other" --> CheckModeButton2
    
    ReadSerial --> CheckModeButton2
    ReadJoy --> CheckModeButton2

    %% Travel Plan Setup
    CheckModeButton2{is TravelFlag true?}
    CheckModeButton2 -- Yes --> StartTravelPlan[Setup & Start Travel Plan]
    StartTravelPlan --> MotorControl
    CheckModeButton2 -- No --> MotorControl

    %% Motor Movement Control
    MotorControl{motorRunning & Target Reached?}
    MotorControl -- Yes --> AdvancePlan[Stop Motors & Advance Travel Plan]
    MotorControl -- No --> LCDUpdate
    AdvancePlan --> LCDUpdate

    %% Update Screen & Telemetry
    LCDUpdate[Update LCD Screen] --> SendTelemetry{500ms elapsed?}
    SendTelemetry -- Yes --> SendESP[Send LIDAR & Compass to ESP]
    SendESP --> Delay[Delay 100ms]
    SendTelemetry -- No --> Delay
    Delay --> End([End Loop])
```
