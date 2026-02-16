# **Hardware IO Architecture Refactor**  
### *Team 1507 Warlocks — feature/hardware-io-architecture*

This branch introduces a **full‑system architectural overhaul** of the robot codebase, replacing legacy WPILib command patterns and vendor‑bound subsystems with a **clean, IO‑based, subsystem‑atomic architecture**.  
The goal is long‑term maintainability, testability, and clarity for both students and mentors.

---

# 🚀 **Overview**

The new architecture is built around three core principles:

### **1. IO‑Based Subsystems**  
Every subsystem now has:
- A **thin Subsystem class** (no vendor code)
- A **SwerveIO / ShooterIO / FeederIO / …** interface
- A **Real** implementation (hardware)
- A **Sim** implementation (simulation)

This isolates hardware access and makes subsystems:
- deterministic  
- testable  
- simulation‑ready  
- vendor‑agnostic  

### **2. Atomic Commands**  
All old WPILib `Command` subclasses have been replaced with **stateless, zero‑allocation atomic commands** using `CommandBuilder`.

Each subsystem has a corresponding `*Commands.java` file:
```
AgitatorCommands
FeederCommands
HopperCommands
IntakeArmCommands
IntakeRollerCommands
ShooterCommands
ClimberCommands
DriveCommands
```

These commands:
- do exactly one thing  
- never store state  
- never allocate objects during execution  
- are easy to read and easy to teach  

### **3. Coordinator Commands**  
Complex behaviors (shooting, multi‑subsystem actions) are implemented in:
```
ShooterCoordinator.java
```

This keeps subsystem commands simple and reusable.

---

# 🧠 **Why This Architecture?**

This refactor solves long‑standing issues:

### ❌ Before  
- Subsystems contained vendor code  
- Commands were stateful and duplicated logic  
- Autos used old command classes  
- Dashboard buttons referenced legacy commands  
- Hard to test, hard to simulate, hard to teach  

### ✔ After  
- Subsystems are pure logic + IO  
- Commands are atomic and stateless  
- Autos use a clean builder API  
- Dashboard uses new atomic commands  
- Architecture is consistent across all subsystems  
- Easy to onboard new students  
- Easy to extend for future seasons  

---

# 🏗️ **Major Changes**

### **Subsystems**
- All subsystems rewritten to use IO interfaces  
- Hardware moved to `frc.lib.hardware`  
- Simulation support standardized  
- SwerveSubsystem now wraps SwerveIO instead of CTRE’s vendor subsystem  

### **Commands**
- Removed all legacy commands:
  - CmdShoot, CmdMoveToPose, CmdIntakeArmDown, CmdFeederFeed, etc.
- Added atomic command files for each subsystem  
- Added DriveCommands (maintain heading, move to pose, move through pose)  
- Added ClimberCommands (new)  
- Added ShooterCoordinator  

### **RobotContainer**
- Fully rewritten  
- Uses new IO‑based subsystems  
- Uses DriveCommands + ShooterCoordinator  
- Uses subsystem atomic commands for all driver controls  
- Removed all vendor‑bound command usage  
- Restored “maintain heading to target” using DriveCommands  

### **DashboardManager**
- Replaced all old manual commands  
- Added manual hopper control  
- Added manual climber control  
- Retained CmdShooterPIDTuner (diagnostic only)  
- Cleaned up NT publishers  

### **AutoSequence**
- Updated to use DriveCommands + atomic commands  
- Removed all old auto commands  
- AutoSample updated accordingly  

### **Constants**
- Cleaned up kMoveToPose and kMoveThroughPose  
- Updated kSwerve speed constants  
- Ensured all subsystems pull tuning from Constants + Hardware  

---

# 📁 **New Folder Structure**

```
frc/
 ├─ lib/
 │   ├─ hardware/        # CAN IDs, transforms, gear ratios
 │   ├─ io/              # IO interfaces + Real/Sim implementations
 │   ├─ util/            # CommandBuilder, math, logging
 │   └─ shooterML/       # ML model + ShotTrainer
 │
 ├─ robot/
 │   ├─ commands/
 │   │   ├─ AgitatorCommands.java
 │   │   ├─ DriveCommands.java
 │   │   ├─ FeederCommands.java
 │   │   ├─ HopperCommands.java
 │   │   ├─ IntakeArmCommands.java
 │   │   ├─ IntakeRollerCommands.java
 │   │   ├─ ShooterCommands.java
 │   │   ├─ ShooterCoordinator.java
 │   │   ├─ tuning/
 │   │   │   └─ CmdShooterPIDTuner.java
 │   │   │
 │   │   ├─ auto/
 │   │   │   ├─ AutoSequence.java
 │   │   │   └─ routines/
 │   │   │       └─ AutoSample.java
 │   │
 │   ├─ subsystems/      # Thin IO-based subsystems
 │   ├─ utilities/       # SubsystemsRecord, MotorConfig
 │   ├─ localization/    # PV + QuestNav
 │   ├─ DashboardManager.java
 │   ├─ RobotContainer.java
 │   └─ Constants.java
```

---

# 🧪 **Testing & Validation**

This architecture supports:

### ✔ Simulation  
All subsystems have IOSim implementations.

### ✔ Unit testing  
Subsystems are deterministic and mockable.

### ✔ Runtime safety  
Commands are stateless and cannot leak state between runs.

### ✔ Telemetry  
All subsystems publish through the centralized Telemetry system.

---

# 🛠️ **Migration Notes**

If you are writing new code:

### **DO**
- Add new subsystem logic to IO interfaces  
- Add new commands to `*Commands.java`  
- Add new multi‑subsystem behaviors to coordinators  
- Use `CommandBuilder` for all commands  
- Use `SubsystemsRecord` to pass subsystems into autos  

### **DO NOT**
- Put vendor code in subsystems  
- Create new WPILib Command subclasses  
- Store state inside commands  
- Access hardware directly from RobotContainer  
- Add logic to Robot.java  

---

# 📌 **Known Remaining Tasks**
- Add DriveCoordinator for multi‑step drive routines  
- Add more autonomous routines  
- Add shooter idle logic  
- Add vision‑aligned shooting mode  

---

# 🎉 **Conclusion**

The `feature/hardware-io-architecture` branch represents a **major modernization** of Team 1507’s robot codebase.  
It brings the project in line with industry‑grade robotics architecture and sets the foundation for:

- cleaner code  
- easier debugging  
- better student onboarding  
- more powerful autonomous routines  
- safer and more predictable robot behavior  

This is the architecture the team will build on for years.
