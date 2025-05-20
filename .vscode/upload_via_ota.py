import os
import sys
import subprocess

Import("env")

def ota_upload(source, target, env):
    print("Starting OTA upload...")
    
    # Get the firmware.bin path from the environment
    firmware_path = str(source[0])
    
    # ESP32 IP address and port
    esp_ip = "192.168.1.252"
    esp_port = "3232"
    
    # Path to espota.py
    espota_path = os.path.join(
        env.PioPlatform().get_package_dir("framework-arduinoespressif32"),
        "tools",
        "espota.py"
    )
    
    # Command to run espota.py with the correct parameters
    cmd = [
        "python3",
        espota_path,
        "-i", esp_ip,
        "-p", esp_port,
        "-f", firmware_path,
        "-d"  # Debug flag
    ]
    
    print(f"Running command: {' '.join(cmd)}")
    
    # Run the command
    try:
        result = subprocess.run(cmd, check=True, capture_output=True, text=True)
        print(f"OTA Upload result: {result.stdout}")
        if result.stderr:
            print(f"Errors: {result.stderr}")
    except subprocess.CalledProcessError as e:
        print(f"OTA Upload failed: {e}")
        print(f"Output: {e.stdout}")
        print(f"Error: {e.stderr}")
        sys.exit(1)
    
    print("OTA upload completed successfully!")

# Replace the default upload method
env.Replace(UPLOADCMD=ota_upload) 