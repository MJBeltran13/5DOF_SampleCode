from robot_control import app

# Static Raspberry Pi IP address
RASPBERRY_PI_IP = '192.168.101.252'

if __name__ == '__main__':
    print(f"Access the server at: http://{RASPBERRY_PI_IP}:5000")
    print("Running server on all interfaces...")
    # Run the server on all interfaces (0.0.0.0) but display Raspberry Pi IP for access
    app.run(host='0.0.0.0', port=5000, debug=True)