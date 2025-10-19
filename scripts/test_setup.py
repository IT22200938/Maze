#!/usr/bin/env python3

"""
Test script to verify ROS 2 setup and dependencies
Run this to check if everything is properly installed
"""

import sys
import subprocess
import os

def print_header(text):
    print("\n" + "="*50)
    print(f"  {text}")
    print("="*50)

def check_command(cmd, description):
    """Check if a command exists"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)
        return True, result.stdout.strip()
    except Exception as e:
        return False, str(e)

def check_package(package_name):
    """Check if a ROS 2 package exists"""
    try:
        result = subprocess.run(
            f"ros2 pkg list | grep {package_name}",
            shell=True,
            capture_output=True,
            text=True,
            timeout=5
        )
        return package_name in result.stdout
    except:
        return False

def main():
    print_header("TurtleBot3 Maze Navigation - Setup Verification")
    
    all_checks_passed = True
    
    # Check 1: ROS 2 Installation
    print_header("1. Checking ROS 2 Installation")
    success, output = check_command("ros2 --version", "ROS 2 Version")
    if success:
        print(f"✓ ROS 2 installed: {output}")
    else:
        print("✗ ROS 2 not found!")
        all_checks_passed = False
    
    # Check 2: Gazebo Installation
    print_header("2. Checking Gazebo Installation")
    success, output = check_command("gz sim --version", "Gazebo Version")
    if success:
        print(f"✓ Gazebo installed")
        print(f"  Version info: {output.split()[0] if output else 'Unknown'}")
    else:
        print("✗ Gazebo not found!")
        print("  Install with: sudo apt install gz-harmonic")
        all_checks_passed = False
    
    # Check 3: TurtleBot3 Packages
    print_header("3. Checking TurtleBot3 Packages")
    tb3_packages = [
        "turtlebot3",
        "turtlebot3_gazebo",
        "turtlebot3_msgs",
        "turtlebot3_teleop"
    ]
    
    for pkg in tb3_packages:
        if check_package(pkg):
            print(f"✓ {pkg} found")
        else:
            print(f"✗ {pkg} not found!")
            all_checks_passed = False
    
    # Check 4: Navigation Packages
    print_header("4. Checking Navigation Packages")
    nav_packages = [
        "nav2_bringup",
        "nav2_map_server",
        "cartographer_ros"
    ]
    
    for pkg in nav_packages:
        if check_package(pkg):
            print(f"✓ {pkg} found")
        else:
            print(f"✗ {pkg} not found!")
            all_checks_passed = False
    
    # Check 5: Python Dependencies
    print_header("5. Checking Python Dependencies")
    python_packages = [
        "numpy",
        "torch"
    ]
    
    for pkg in python_packages:
        try:
            __import__(pkg)
            print(f"✓ {pkg} installed")
        except ImportError:
            if pkg == "torch":
                print(f"⚠ {pkg} not found (required only for DQL)")
                print(f"  Install with: pip3 install torch")
            else:
                print(f"✗ {pkg} not found!")
                print(f"  Install with: pip3 install {pkg}")
                all_checks_passed = False
    
    # Check 6: Environment Variables
    print_header("6. Checking Environment Variables")
    
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', None)
    if turtlebot3_model == 'burger':
        print(f"✓ TURTLEBOT3_MODEL set to: {turtlebot3_model}")
    else:
        print(f"⚠ TURTLEBOT3_MODEL not set or incorrect")
        print(f"  Current value: {turtlebot3_model}")
        print(f"  Set with: export TURTLEBOT3_MODEL=burger")
    
    ros_domain_id = os.environ.get('ROS_DOMAIN_ID', None)
    if ros_domain_id:
        print(f"✓ ROS_DOMAIN_ID set to: {ros_domain_id}")
    else:
        print(f"⚠ ROS_DOMAIN_ID not set")
        print(f"  Set with: export ROS_DOMAIN_ID=30")
    
    # Check 7: Workspace
    print_header("7. Checking Workspace")
    
    ws_path = os.path.expanduser("~/turtlebot3_ws")
    if os.path.exists(ws_path):
        print(f"✓ Workspace found at: {ws_path}")
        
        # Check if our package is in the workspace
        pkg_path = os.path.join(ws_path, "src", "turtlebot3_maze_navigation")
        if os.path.exists(pkg_path):
            print(f"✓ Package found at: {pkg_path}")
        else:
            print(f"⚠ Package not found in workspace")
            print(f"  Expected at: {pkg_path}")
    else:
        print(f"✗ Workspace not found at: {ws_path}")
        all_checks_passed = False
    
    # Check 8: Build Status
    print_header("8. Checking Build Status")
    
    install_path = os.path.join(ws_path, "install", "turtlebot3_maze_navigation")
    if os.path.exists(install_path):
        print(f"✓ Package built successfully")
    else:
        print(f"⚠ Package not built")
        print(f"  Build with: cd ~/turtlebot3_ws && colcon build --packages-select turtlebot3_maze_navigation")
    
    # Final Summary
    print_header("Verification Summary")
    
    if all_checks_passed:
        print("✓ All critical checks passed!")
        print("\nYou're ready to start! Try:")
        print("  ros2 launch turtlebot3_maze_navigation gazebo_maze.launch.py")
    else:
        print("✗ Some checks failed. Please address the issues above.")
        print("\nFor detailed setup instructions, see:")
        print("  - README.md")
        print("  - QUICK_START.md")
    
    print("\nFor help, visit:")
    print("  - https://emanual.robotis.com/docs/en/platform/turtlebot3/")
    print("  - https://navigation.ros.org/")
    print("="*50 + "\n")
    
    return 0 if all_checks_passed else 1

if __name__ == "__main__":
    sys.exit(main())

