# Robot Programming Instructions

This guide explains how to modify, compile, deploy, and run C code on the robot.

## Development Steps

1. Launch Eclipse IDE
2. Edit the C Code:
   - Modify the desired C source files
   - Save your changes

3. Build the Project:
   - Go to `Project → Build Project`
   - Ensure the compiled binary is created in `/Debug/` directory

## Deployment Instructions

### Prerequisites
- Ensure your computer is connected to "Serverrobotik" WiFi network

### Deploying the Compiled File

Use the following SCP command to transfer the binary to the robot:

```bash
scp /home/laboras/eclipse-workspace/another/Debug/another root@192.168.1.100:/home/root
```

> Note: The general SCP command format is:
> ```bash
> scp local_file destination host_address:destination_path
> ```

## Running the Program

1. Connect to the robot via SSH:
```bash
ssh root@192.168.1.100
```
> Note: The general SSH command format is:
> ```bash
> ssh user@robot_address
> ```

2. Execute the program using the command:
```bash
./another

> Note: The general command to execute a binary file:
> ```bash
> ./binary_filename
> ```
